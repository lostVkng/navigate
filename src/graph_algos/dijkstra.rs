//! Dijkstra's algorithm (1959).
//!
//! From a start state, expand the reachable state with the smallest path cost
//! until the goal test passes or the component is exhausted. Edge weights must
//! be non-negative ([`SearchCost`]).
//!
//! This is A* with a zero heuristic, including the same tie break and the same
//! optional [`SearchLimits`].

use std::fmt::Debug;
use std::hash::Hash;

use super::search::{path_from_map, search_identity};
use super::{GraphNodeMap, SearchCost, SearchLimits};
use crate::errors::PathPlannerError;

/// Shortest paths on a non-negatively weighted graph.
#[derive(Clone, Debug)]
pub struct Dijkstra {
    limits: SearchLimits,
}

impl Default for Dijkstra {
    fn default() -> Self {
        Self::new()
    }
}

impl Dijkstra {
    /// Unlimited search.
    pub fn new() -> Self {
        Self {
            limits: SearchLimits::none(),
        }
    }

    /// Replace the expansion and time budgets.
    pub fn with_limits(mut self, limits: SearchLimits) -> Self {
        self.limits = limits;
        self
    }

    /// Budgets currently attached to this search.
    pub fn limits(&self) -> &SearchLimits {
        &self.limits
    }

    /// States from `start` through the first state that satisfies `goal_fn`.
    ///
    /// `neighbors` returns the successors of a state and the non-negative cost
    /// of each edge. The goal test is applied when a state is about to be
    /// expanded, which is the first time it is the cheapest open state.
    pub fn plan<N, C, IT, NN, G>(
        &self,
        start: N,
        neighbors: NN,
        goal_fn: G,
    ) -> Result<Vec<N>, PathPlannerError>
    where
        N: Eq + Hash + Clone + Debug,
        C: SearchCost,
        NN: Fn(&N) -> IT,
        IT: IntoIterator<Item = (N, C)>,
        G: FnMut(&N) -> bool,
    {
        let outcome = search_identity(
            self.limits.clone(),
            start,
            neighbors,
            |_| C::zero(),
            goal_fn,
        )?;
        match outcome.goal {
            Some(index) => path_from_map(&outcome.nodes, index),
            None => Err(PathPlannerError::NoPathFound),
        }
    }

    /// Closed set at the moment the goal is expanded, including states that
    /// were generated but not yet expanded.
    ///
    /// When the goal is never reached the map contains every reachable state.
    pub fn get_visited_nodes<N, C, IT, NN, G>(
        &self,
        start: N,
        neighbors: NN,
        goal_fn: G,
    ) -> Result<GraphNodeMap<N, C>, PathPlannerError>
    where
        N: Eq + Hash + Clone + Debug,
        C: SearchCost,
        NN: Fn(&N) -> IT,
        IT: IntoIterator<Item = (N, C)>,
        G: FnMut(&N) -> bool,
    {
        Ok(search_identity(
            self.limits.clone(),
            start,
            neighbors,
            |_| C::zero(),
            goal_fn,
        )?
        .nodes)
    }

    /// Every state reachable from `start`, with its cheapest path cost.
    ///
    /// Equivalent to [`Self::get_visited_nodes`] with a goal test that never
    /// matches. Set [`SearchLimits`] if the graph may be unbounded.
    pub fn get_all_nodes<N, C, IT, NN>(
        &self,
        start: N,
        neighbors: NN,
    ) -> Result<GraphNodeMap<N, C>, PathPlannerError>
    where
        N: Eq + Hash + Clone + Debug,
        C: SearchCost,
        NN: Fn(&N) -> IT,
        IT: IntoIterator<Item = (N, C)>,
    {
        self.get_visited_nodes(start, neighbors, |_| false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::errors::SearchLimitKind;
    use std::collections::HashMap;

    fn neighbors_of(
        graph: &HashMap<String, Vec<(String, u32)>>,
    ) -> impl Fn(&String) -> Vec<(String, u32)> + '_ {
        move |node: &String| graph.get(node).cloned().unwrap_or_default()
    }

    #[test]
    fn cheapest_path_on_a_diamond() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1), ("C".into(), 3)]);
        graph.insert("B".into(), vec![("D".into(), 5)]);
        graph.insert("C".into(), vec![("D".into(), 1)]);
        graph.insert("D".into(), vec![]);

        let path = Dijkstra::new()
            .plan("A".into(), neighbors_of(&graph), |node| node == "D")
            .unwrap();
        assert_eq!(path, vec!["A", "C", "D"]);
    }

    #[test]
    fn unreachable_goal() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1)]);
        graph.insert("B".into(), vec![("C".into(), 1)]);
        graph.insert("C".into(), vec![]);
        graph.insert("D".into(), vec![]);

        let result = Dijkstra::new().plan("A".into(), neighbors_of(&graph), |node| node == "D");
        assert!(matches!(result, Err(PathPlannerError::NoPathFound)));
    }

    #[test]
    fn cycle_does_not_prevent_the_shortest_costs() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1)]);
        graph.insert("B".into(), vec![("C".into(), 1)]);
        graph.insert("C".into(), vec![("A".into(), 1), ("D".into(), 2)]);
        graph.insert("D".into(), vec![]);

        let nodes = Dijkstra::new()
            .get_visited_nodes("A".into(), neighbors_of(&graph), |node| node == "D")
            .unwrap();
        let cost = |name: &str| nodes.get(name).unwrap().1;
        assert_eq!(cost("A"), 0);
        assert_eq!(cost("B"), 1);
        assert_eq!(cost("C"), 2);
        assert_eq!(cost("D"), 4);
    }

    #[test]
    fn float_costs_choose_the_shorter_real_path() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1.5_f64), ("C".into(), 2.0)]);
        graph.insert("B".into(), vec![("D".into(), 1.5)]);
        graph.insert("C".into(), vec![("D".into(), 0.5)]);
        graph.insert("D".into(), vec![]);

        let path = Dijkstra::new()
            .plan("A".into(), neighbors_of_f64(&graph), |node| node == "D")
            .unwrap();
        assert_eq!(path, vec!["A", "C", "D"]);
    }

    fn neighbors_of_f64(
        graph: &HashMap<String, Vec<(String, f64)>>,
    ) -> impl Fn(&String) -> Vec<(String, f64)> + '_ {
        move |node: &String| graph.get(node).cloned().unwrap_or_default()
    }

    #[test]
    fn equal_costs_follow_insertion_order() {
        // Both branches cost 2. Neighbors are emitted A then B, so A is
        // inserted first and must be the parent of G.
        let mut graph = HashMap::new();
        graph.insert("S".into(), vec![("A".into(), 1.0_f64), ("B".into(), 1.0)]);
        graph.insert("A".into(), vec![("G".into(), 1.0)]);
        graph.insert("B".into(), vec![("G".into(), 1.0)]);
        graph.insert("G".into(), vec![]);

        let path = Dijkstra::new()
            .plan("S".into(), neighbors_of_f64(&graph), |node| node == "G")
            .unwrap();
        assert_eq!(path, vec!["S", "A", "G"]);
    }

    #[test]
    fn negative_edge_is_rejected() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), -1_i32)]);
        graph.insert("B".into(), vec![]);
        let result = Dijkstra::new().plan("A".into(), neighbors_of_i32(&graph), |node| node == "B");
        assert!(matches!(result, Err(PathPlannerError::NegativeCost)));
    }

    fn neighbors_of_i32(
        graph: &HashMap<String, Vec<(String, i32)>>,
    ) -> impl Fn(&String) -> Vec<(String, i32)> + '_ {
        move |node: &String| graph.get(node).cloned().unwrap_or_default()
    }

    #[test]
    fn expansion_budget_stops_a_long_chain() {
        let mut graph = HashMap::new();
        for i in 0..50 {
            graph.insert(format!("n{i}"), vec![(format!("n{}", i + 1), 1_u32)]);
        }
        graph.insert("n50".into(), vec![]);
        let result = Dijkstra::new()
            .with_limits(SearchLimits::expansions(3))
            .plan("n0".into(), neighbors_of(&graph), |node| node == "n50");
        match result {
            Err(PathPlannerError::SearchLimitExceeded {
                kind: SearchLimitKind::Expansions,
                ..
            }) => {}
            other => panic!("expected expansion limit, got {other:?}"),
        }
    }

    #[test]
    fn start_is_a_zero_length_path_when_it_is_the_goal() {
        let graph: HashMap<String, Vec<(String, u32)>> = HashMap::new();
        let path = Dijkstra::new()
            .with_limits(SearchLimits::expansions(0))
            .plan("A".into(), neighbors_of(&graph), |node| node == "A")
            .unwrap();
        assert_eq!(path, vec!["A"]);
    }

    #[test]
    fn integer_overflow_is_an_invalid_cost() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), u32::MAX)]);
        graph.insert("B".into(), vec![("C".into(), 1)]);
        graph.insert("C".into(), vec![]);
        let result = Dijkstra::new().plan("A".into(), neighbors_of(&graph), |node| node == "C");
        assert!(matches!(result, Err(PathPlannerError::InvalidCost)));
    }
}
