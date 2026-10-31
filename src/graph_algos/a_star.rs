//! A* search (Hart, Nilsson, and Raphael, 1968).
//!
//! A* expands the open state with the smallest `f = g + h`, where `g` is the
//! path cost from the start and `h` estimates the cost still to go. The
//! estimate must be admissible (never greater than the true remaining cost)
//! for the returned path to be optimal. It must be consistent
//! (`h(n) ≤ c(n, n') + h(n')`) to guarantee each state is expanded at most
//! once; a cheaper rediscovery is still accepted if it happens.
//!
//! When several states share an `f` value, the larger `g` is expanded first
//! (the state with the smaller heuristic, which is usually closer to the
//! goal). A monotonic insertion counter breaks whatever remains, so two runs
//! on the same graph produce the same path.
//!
//! [`AStar::plan_keyed`] is the same search when the closed-set identity is a
//! function of the state rather than the state itself. Hybrid-A* uses it to
//! key continuous poses by discretized cell and heading while remembering the
//! pose that was actually expanded.

use std::fmt::Debug;
use std::hash::Hash;

use super::search::{KeyedSearchResult, path_from_map, search_identity, search_keyed};
use super::{SearchCost, SearchLimits};
use crate::errors::PathPlannerError;

/// Heuristic graph search.
#[derive(Clone, Debug)]
pub struct AStar {
    limits: SearchLimits,
}

impl Default for AStar {
    fn default() -> Self {
        Self::new()
    }
}

impl AStar {
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

    /// Path from `start` to the first expanded state that satisfies `goal_fn`.
    ///
    /// `neighbors` yields successors and non-negative edge costs. `heuristic`
    /// estimates the remaining cost of a state. A zero heuristic makes this
    /// Dijkstra's algorithm.
    pub fn plan<N, C, IT, NN, H, G>(
        &self,
        start: N,
        neighbors: NN,
        heuristic: H,
        goal_fn: G,
    ) -> Result<Vec<N>, PathPlannerError>
    where
        N: Eq + Hash + Clone + Debug,
        C: SearchCost,
        NN: Fn(&N) -> IT,
        IT: IntoIterator<Item = (N, C)>,
        H: FnMut(&N) -> C,
        G: FnMut(&N) -> bool,
    {
        let outcome = search_identity(self.limits.clone(), start, neighbors, heuristic, goal_fn)?;
        match outcome.goal {
            Some(index) => path_from_map(&outcome.nodes, index),
            None => Err(PathPlannerError::NoPathFound),
        }
    }

    /// A* where the closed set is keyed by `key_of(state)` instead of `state`.
    ///
    /// A strictly cheaper path to an existing key appends a new record. The
    /// record that was expanded is left untouched, so a reconstructed path is
    /// the exact sequence of states the successor function returned. `N` does
    /// not need to implement [`Eq`] or [`Hash`]; `K` does.
    pub fn plan_keyed<N, K, C, IT, NN, H, G, KF>(
        &self,
        start: N,
        key_of: KF,
        neighbors: NN,
        heuristic: H,
        goal_fn: G,
    ) -> Result<KeyedSearchResult<N, C>, PathPlannerError>
    where
        N: Clone,
        K: Eq + Hash,
        C: SearchCost,
        NN: Fn(&N) -> IT,
        IT: IntoIterator<Item = (N, C)>,
        H: FnMut(&N) -> C,
        G: FnMut(&N) -> bool,
        KF: Fn(&N) -> K,
    {
        search_keyed(
            self.limits.clone(),
            start,
            key_of,
            neighbors,
            heuristic,
            goal_fn,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    fn neighbors_of(
        graph: &HashMap<String, Vec<(String, u32)>>,
    ) -> impl Fn(&String) -> Vec<(String, u32)> + '_ {
        move |node: &String| graph.get(node).cloned().unwrap_or_default()
    }

    #[test]
    fn zero_heuristic_matches_dijkstra() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1), ("C".into(), 3)]);
        graph.insert("B".into(), vec![("D".into(), 5)]);
        graph.insert("C".into(), vec![("D".into(), 1)]);
        graph.insert("D".into(), vec![]);

        let path = AStar::new()
            .plan("A".into(), neighbors_of(&graph), |_| 0, |node| node == "D")
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

        let result = AStar::new().plan("A".into(), neighbors_of(&graph), |_| 0, |node| node == "D");
        assert!(matches!(result, Err(PathPlannerError::NoPathFound)));
    }

    #[test]
    fn heuristic_guides_a_unique_optimal_path() {
        let mut graph = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1), ("C".into(), 1)]);
        graph.insert("B".into(), vec![("D".into(), 1)]);
        graph.insert("C".into(), vec![("D".into(), 2)]);
        graph.insert("D".into(), vec![]);
        let coords = HashMap::from([
            ("A".to_string(), (0_i32, 0_i32)),
            ("B".into(), (1, 0)),
            ("C".into(), (0, 1)),
            ("D".into(), (2, 0)),
        ]);
        let heuristic = |node: &String| {
            let (x, y) = coords[node];
            let (gx, gy) = coords["D"];
            ((x - gx).abs() + (y - gy).abs()) as u32
        };
        let path = AStar::new()
            .plan("A".into(), neighbors_of(&graph), heuristic, |node| {
                node == "D"
            })
            .unwrap();
        assert_eq!(path, vec!["A", "B", "D"]);
    }

    #[test]
    fn equal_f_prefers_the_larger_g() {
        // f(A) = g 1 + h 1 = 2, f(B) = g 2 + h 0 = 2.
        // The higher-g tie break expands B first, and B's edge into G costs 0,
        // so the optimal path that is discovered first is S-B-G.
        let mut graph = HashMap::new();
        graph.insert("S".into(), vec![("A".into(), 1_u32), ("B".into(), 2)]);
        graph.insert("A".into(), vec![("G".into(), 1)]);
        graph.insert("B".into(), vec![("G".into(), 0)]);
        graph.insert("G".into(), vec![]);
        let heuristic = |node: &String| match node.as_str() {
            "S" => 2,
            "A" => 1,
            "B" | "G" => 0,
            _ => 0,
        };
        let path = AStar::new()
            .plan("S".into(), neighbors_of(&graph), heuristic, |n| n == "G")
            .unwrap();
        assert_eq!(path, vec!["S", "B", "G"]);
    }

    #[test]
    fn float_heuristic_and_float_edges() {
        let mut graph: HashMap<String, Vec<(String, f64)>> = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1.0), ("C".into(), 2.5)]);
        graph.insert("B".into(), vec![("D".into(), 1.0)]);
        graph.insert("C".into(), vec![("D".into(), 0.25)]);
        graph.insert("D".into(), vec![]);
        let h = |node: &String| if node == "D" { 0.0 } else { 0.5 };
        let path = AStar::new()
            .plan(
                "A".into(),
                move |n: &String| graph.get(n).cloned().unwrap_or_default(),
                h,
                |n| n == "D",
            )
            .unwrap();
        // A-B-D costs 2.0, A-C-D costs 2.75.
        assert_eq!(path, vec!["A", "B", "D"]);
    }

    #[test]
    fn nan_heuristic_is_invalid() {
        let mut graph: HashMap<String, Vec<(String, f64)>> = HashMap::new();
        graph.insert("A".into(), vec![("B".into(), 1.0)]);
        graph.insert("B".into(), vec![]);
        let result = AStar::new().plan(
            "A".into(),
            move |n: &String| graph.get(n).cloned().unwrap_or_default(),
            |_| f64::NAN,
            |n| n == "B",
        );
        assert!(matches!(result, Err(PathPlannerError::InvalidCost)));
    }

    #[test]
    fn keyed_search_reopens_a_discretization_bucket() {
        // Two continuous states share a key. The expensive one is generated
        // first; the cheap one must replace it.
        #[derive(Clone, Debug)]
        struct State {
            name: &'static str,
            key: u8,
        }
        let result = AStar::new()
            .plan_keyed(
                State { name: "S", key: 0 },
                |s: &State| s.key,
                |s: &State| -> Vec<(State, f64)> {
                    match s.name {
                        "S" => vec![
                            (
                                State {
                                    name: "expensive",
                                    key: 1,
                                },
                                5.0,
                            ),
                            (
                                State {
                                    name: "cheap",
                                    key: 1,
                                },
                                1.0,
                            ),
                        ],
                        "cheap" => vec![(State { name: "G", key: 2 }, 1.0)],
                        "expensive" => vec![(State { name: "G", key: 2 }, 1.0)],
                        _ => vec![],
                    }
                },
                |_| 0.0,
                |s: &State| s.name == "G",
            )
            .unwrap();
        let names: Vec<_> = result.path.iter().map(|s| s.name).collect();
        assert_eq!(names, vec!["S", "cheap", "G"]);
        assert!((result.cost - 2.0).abs() < 1e-9);
    }
}
