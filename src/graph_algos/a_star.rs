use crate::errors::PathPlannerError;
use crate::collections::FxIndexMap;
use super::shortest_path;

use std::{
    collections::BinaryHeap, 
    hash::Hash, 
    fmt::Debug,
    cmp::Ordering
};
use num_traits::Zero;
use indexmap::map::Entry::{Occupied, Vacant};



/// Node on A* graph
#[derive(Debug)]
struct Node<T> {
    index: usize, // index in the closed_list - maps to the Id of the node
    cost: T, // Cost to reach this node
    f_cost: T, // Total cost = cost + h(n) aka estimated cost
}

impl<T: Ord> Ord for Node<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.f_cost.cmp(&self.f_cost)
    }
}
impl<T: Ord> PartialOrd for Node<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<T: PartialEq> PartialEq for Node<T> {
    fn eq(&self, other: &Self) -> bool {
        self.f_cost == other.f_cost
    }
}
impl<T: PartialEq> Eq for Node<T> {}

/// A* Algorithm
/// https://en.wikipedia.org/wiki/A*_search_algorithm
pub struct AStar {}

impl AStar{

    /// From start Node, traverse through graph until node meets goal criteria
    /// The Approach has 2 requirements:
    /// 1. The heuristic function must be admissible (never overestimates the true cost to reach the goal)
    /// 2. A path actually exists between the start and goal nodes
    pub fn plan<N, C, IT, NN, H, G>(&self, start: N, neighbors: NN, heuristic_fn: H, goal_fn: G) -> Result<Vec<N>, PathPlannerError>
    where 
        N: Eq + Hash + Clone + Debug,
        NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
        H: Fn(&N) -> C, // heuristic function
        IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
        C: Zero + Ord + Copy + Debug,
        G: Fn(&N) -> bool, // node qualifier for goal
        {

        // build_a_star_graph
        let (node_map, goal_index) = self.build_graph(start, neighbors, heuristic_fn, goal_fn)?;

        // Return the shortest path
        match goal_index {
            Some(goal_index) => {
                let path = shortest_path(&node_map, goal_index)?;
                Ok(path)
            }
            None => Err(PathPlannerError::NoPathFound)
        }
    }


    /// Traverses the graph using A* algorithm 
    /// Returns a map of nodes with their smallest costs along with the index of the goal node
    fn build_graph<N, C, IT, NN, H, G>(&self, start: N, neighbors: NN, heuristic_fn: H, goal_fn: G) -> Result<(FxIndexMap<N, (usize, C)>, Option<usize>), PathPlannerError>
    where 
        N: Eq + Hash + Clone + Debug,
        NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
        IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
        C: Zero + Ord + Copy + Debug,
        H: Fn(&N) -> C, // heuristic function
        G: Fn(&N) -> bool // Returns true if goal is met
    {
        // Open List
        // Nodes that need to be evaluated, implemented as priority queue
        // Sorting is done by f_cost (cost + heuristic)
        let mut open_list: BinaryHeap<Node<C>> = BinaryHeap::new();

        // visited nodes - cost is known, no longer need to visit
        // Evaluated nodes, avoids re-evaluating nodes, used to find the final path
        // The tuple contains (parent_index, cost) where parent_index is the index of the parent node in the closed_list
        // for the start node, parent_index is set to usize::MAX to indicate it has no parent
        let mut closed_list: FxIndexMap<N, (usize, C)> = FxIndexMap::default();

        // Add the start node to both open & closed list
        // Only this node needs to be duplicated so we can lookup its value in closed_list
        // and also retrieve neighbors from it
        let start_index = closed_list.insert_full(start.clone(), (usize::MAX, Zero::zero())).0;
        open_list.push(Node{
            index: start_index,
            cost: Zero::zero(), // This is the cost from the start node
            f_cost: Zero::zero(), // cost + heuristic
        });

        while let Some(Node{index, cost, ..}) = open_list.pop() {

            // fetch current best cost for node
            let (node, &(_, c)) = closed_list.get_index(index).unwrap();

            // If cost of new node from BinaryHeap is higher than the best cost, skip it
            // This implies we've already found a better path to this node
            if cost > c {
                continue;
            }

            // Check if we've reached the goal
            if goal_fn(&node) {
                return Ok((closed_list, Some(index)));
            }

            // loop over neighbors
            for (neighbor, edge_cost) in neighbors(&node).into_iter() {

                // new cost to reach this node = edge cost + node cost
                // This is confirmed cost, not heuristic
                let new_cost = edge_cost + c;

                let neighbor_index: usize;
                // calculate heuristic cost
                let h_cost: C = heuristic_fn(&neighbor);

                match closed_list.entry(neighbor) {
                    Vacant(e) => {
                        // This is the first time we're seeing this neighbor
                        neighbor_index = e.index();
                        e.insert((index, new_cost));
                    }
                    Occupied(mut e) => {
                        if e.get().1 > new_cost {
                            // We've found a better path to this neighbor
                            neighbor_index = e.index();
                            e.insert((index, new_cost));
                        } else {
                            // The existing path is better, do nothing
                            continue;
                        }
                    }
                }
                
                // Only add to the queue if we've found a better path
                open_list.push(Node {
                    index: neighbor_index,
                    cost: new_cost,
                    f_cost: new_cost + h_cost,
                });
            }
        }
        Ok((closed_list, None))
    }
}



#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    // Helper function to create a neighbor function from a graph
    // Assumes data stored as: HashMap<String, Vec<(String, u32)>>
    fn create_neighbor_fn(graph: &HashMap<String, Vec<(String, u32)>>) -> impl Fn(&String) -> Vec<(String, u32)> + '_ {
        move |node: &String| {
            graph.get(node).unwrap_or(&vec![]).clone()
        }
    }

    /// A* algorithm test
    #[test]
    fn test_a_star() {
        // Diamond-shaped graph: A -> B -> D and A -> C -> D
        let mut graph = HashMap::new();
        graph.insert("A".to_string(), vec![("B".to_string(), 1), ("C".to_string(), 3)]);
        graph.insert("B".to_string(), vec![("D".to_string(), 5)]);
        graph.insert("C".to_string(), vec![("D".to_string(), 1)]);
        graph.insert("D".to_string(), vec![]);

        let neighbors = create_neighbor_fn(&graph);
        
        // Simple zero heuristic (makes A* behave like Dijkstra)
        let heuristic = |_node: &String| 0;

        // Run A* algorithm from node A to node D
        let a_star = AStar{};
        let path = a_star.plan(
            "A".to_string(),
            neighbors,
            heuristic,
            |node| node == "D"
        ).unwrap();
        
        // The expected path is A -> C -> D (the cheapest path)
        assert_eq!(path, vec!["A", "C", "D"].into_iter().map(String::from).collect::<Vec<_>>());
    }

    #[test]
    fn test_a_star_handles_unreachable_goal() {
        // Create a graph with no path to the goal
        let mut graph = HashMap::new();
        graph.insert("A".to_string(), vec![("B".to_string(), 1)]);
        graph.insert("B".to_string(), vec![("C".to_string(), 1)]);
        graph.insert("C".to_string(), vec![]);
        graph.insert("D".to_string(), vec![]); // D is not connected
        
        let neighbors = create_neighbor_fn(&graph);
        
        // Simple zero heuristic
        let heuristic = |_node: &String| 0;
        
        // Try to find a path from A to D (which doesn't exist)
        let a_star = AStar{};
        let result = a_star.plan("A".to_string(), neighbors, heuristic, |node| node == "D");
        
        // Expect a NoPathFound error
        assert!(matches!(result, Err(PathPlannerError::NoPathFound)));
    }

    #[test]
    fn test_build_a_star_graph_with_cycle() {
        // Create a graph with a cycle: A -> B -> C -> A
        let mut graph = HashMap::new();
        
        graph.insert("A".to_string(), vec![("B".to_string(), 1)]);
        graph.insert("B".to_string(), vec![("C".to_string(), 1)]);
        graph.insert("C".to_string(), vec![("A".to_string(), 1), ("D".to_string(), 2)]);
        graph.insert("D".to_string(), vec![]);
        
        let neighbors = create_neighbor_fn(&graph);
        
        // Simple zero heuristic
        let heuristic = |_node: &String| 0;
        
        // Run A* algorithm from node A
        let a_star = AStar{};
        let (result, _) = a_star.build_graph(
            "A".to_string(),
            neighbors,
            heuristic,
            |node| node == "D"
        ).unwrap();
        
        // Verify costs
        let costs: HashMap<_, _> = result.iter().map(|(node, (_, cost))| (node.clone(), *cost)).collect();
        
        assert_eq!(costs.get("A").unwrap(), &0);
        assert_eq!(costs.get("B").unwrap(), &1);
        assert_eq!(costs.get("C").unwrap(), &2);
        assert_eq!(costs.get("D").unwrap(), &4);
    }

    #[test]
    fn test_a_star_with_heuristic() {
        // Create a simple grid-like graph where nodes are represented as (x, y) coordinates
        // A(0,0) -> B(1,0) -> D(2,0)
        //   |
        //   v
        // C(0,1) ------> D(2,0)
        //
        // Direct path C->D should be chosen with a good heuristic
        
        let mut graph = HashMap::new();
        graph.insert("A".to_string(), vec![("B".to_string(), 1), ("C".to_string(), 1)]);
        graph.insert("B".to_string(), vec![("D".to_string(), 1)]);
        graph.insert("C".to_string(), vec![("D".to_string(), 2)]);
        graph.insert("D".to_string(), vec![]);
        
        // Coordinates for each node
        let coords = HashMap::from([
            ("A".to_string(), (0i32, 0i32)),
            ("B".to_string(), (1i32, 0i32)),
            ("C".to_string(), (0i32, 1i32)),
            ("D".to_string(), (2i32, 0i32)),
        ]);
        
        let neighbors = create_neighbor_fn(&graph);
        
        // Manhattan distance heuristic
        let heuristic = |node: &String| {
            let (nx, ny) = coords.get(node).unwrap();
            let (gx, gy) = coords.get("D").unwrap(); // Goal is D
            ((nx - gx).abs() + (ny - gy).abs()) as u32
        };
        
        // Run A* algorithm from node A to node D
        let a_star = AStar{};
        let path = a_star.plan(
            "A".to_string(),
            neighbors,
            heuristic,
            |node| node == "D"
        ).unwrap();
        
        // The expected path is A -> B -> D (the path guided by heuristic)
        assert_eq!(path, vec!["A", "B", "D"].into_iter().map(String::from).collect::<Vec<_>>());
    }
}