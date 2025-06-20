use crate::errors::PathPlannerError;
use super::{shortest_path, GraphNodeMap};

use std::{
    collections::BinaryHeap, 
    hash::Hash, 
    fmt::Debug,
    cmp::Ordering
};
use num_traits::Zero;
use indexmap::map::Entry::{Occupied, Vacant};


/// Node identifier 
/// - for ordering we only need cost and a way to identify the node
/// - Nodes can contain additional data, but we only need to identify them
#[derive(Debug)]
pub struct Node<T> {
    pub index: usize,
    pub cost: T
}

impl<T: Ord> Ord for Node<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}
impl<T: Ord> PartialOrd for Node<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<T: PartialEq> PartialEq for Node<T> {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}
impl<T: PartialEq> Eq for Node<T> {}


/// Dijkstra Algorithm
/// https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
pub struct Dijkstra {}


impl Dijkstra {
    
    /// From start Node, traverse through graph until node meets goal criteria
    pub fn plan<N, C, IT, NN, G>(&self, start: N, neighbors: NN, goal_fn: G) -> Result<Vec<N>, PathPlannerError>
    where 
        N: Eq + Hash + Clone + Debug,
        NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
        IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
        C: Zero + Ord + Copy + Debug,
        G: Fn(&N) -> bool, // node qualifier for goal
    {
        // Build the graph - terminates when the goal is met
        let (node_map, goal_index) = self.build_graph(start, neighbors, goal_fn)?;

        // Return the shortest path
        match goal_index {
            Some(goal_index) => {
                let path = shortest_path(&node_map, goal_index)?;
                Ok(path)
            }
            None => Err(PathPlannerError::NoPathFound)
        }
    }

    /// Return a partial map of the graph up to the goal node
    /// Nodes with lower cost than the goal node will be included
    pub fn get_visited_nodes<N, C, IT, NN, G>(&self, start: N, neighbors: NN, goal: G) -> Result<GraphNodeMap<N, C>, PathPlannerError>
    where 
        N: Eq + Hash + Clone + Debug,
        NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
        IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
        C: Zero + Ord + Copy + Debug,
        G: Fn(&N) -> bool,
        {

        // Build the graph - terminates when the goal is met
        let (node_map, _) = self.build_graph(start, neighbors, goal)?;
        
        Ok(node_map)
    }


    /// Returns a full map of the graph, includes all (reachable) nodes and costs
    pub fn get_all_nodes<N, C, IT, NN>(&self, start: N, neighbors: NN) -> Result<GraphNodeMap<N, C>, PathPlannerError>
    where 
        N: Eq + Hash + Clone + Debug,
        NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
        IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
        C: Zero + Ord + Copy + Debug,
        {

        // Build the graph - terminates when the goal is met
        let (node_map, _) = self.build_graph(start, neighbors, |_| false)?;
        
        Ok(node_map)
    }


    /// Traverses the graph using Dijkstra's algorithm 
    /// Returns a map of nodes with their smallest costs along with the index of the goal node
    fn build_graph<N, C, IT, NN, G>(&self, start: N, neighbors: NN, goal_fn: G) -> Result<(GraphNodeMap<N, C>, Option<usize>), PathPlannerError>
    where 
        N: Eq + Hash + Clone + Debug,
        NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
        IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
        C: Zero + Ord + Copy + Debug,
        G: Fn(&N) -> bool // Returns true if goal is met
        {

        // Nodes to visit - binary heap sorts Biggest to Smallest
        // Dijkstra's algorithm uses a priority queue to always expand the least costly node first
        // We store the cost from the starting node
        let mut nodes_to_visit: BinaryHeap<Node<C>> = BinaryHeap::new();

        // visited nodes - cost is known, no longer need to visit
        // usize is the index in the nodes_map
        // The tuple contains (parent_index, cost) where parent_index is the index of the parent node in the map
        // for the start node, parent_index is set to usize::MAX to indicate it has no parent
        let mut nodes_map: GraphNodeMap<N, C> = GraphNodeMap::default();
        
        // Add start node to the map and queue
        let start_index = nodes_map.insert_full(start.clone(), (usize::MAX, Zero::zero())).0;
        nodes_to_visit.push(Node{
            index: start_index,
            cost: Zero::zero(), // This is the cost from the start node
        });

        // Loop over each node to visit, removing the smallest node
        while let Some(Node {cost, index}) = nodes_to_visit.pop() {

            // fetch current best cost for node
            let (node, &(_, c)) = nodes_map.get_index(index).unwrap();

            // If cost of new node from BinaryHeap is higher than the best cost, skip it
            // This implies we've already found a better path to this node
            if cost > c {
                continue;
            }

            // Check if we've reached the goal
            if goal_fn(&node) {
                return Ok((nodes_map, Some(index)));
            }
            
            // loop over neighbors
            for (neighbor, edge_cost) in neighbors(&node).into_iter() {

                // new cost to reach this node = edge cost + node cost
                let new_cost = edge_cost + c;

                // Check if we've found a better path to this neighbor
                let neighbor_index;
                
                match nodes_map.entry(neighbor) {
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
                nodes_to_visit.push(Node {
                    index: neighbor_index,
                    cost: new_cost,
                });
            }
        }
        
        Ok((nodes_map, None))
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

    /// Dijkstra's algorithm test
    #[test]
    fn test_dijkstra() {

        // Diamond-shaped graph: A -> B -> D and A -> C -> D
        let mut graph = HashMap::new();
        graph.insert("A".to_string(), vec![("B".to_string(), 1), ("C".to_string(), 3)]);
        graph.insert("B".to_string(), vec![("D".to_string(), 5)]);
        graph.insert("C".to_string(), vec![("D".to_string(), 1)]);
        graph.insert("D".to_string(), vec![]);

        let neighbors = create_neighbor_fn(&graph);

        // Run Dijkstra's algorithm from node A to node D
        let dijkstra = Dijkstra{};
        let path = dijkstra.plan(
            "A".to_string(),
            neighbors,
            |node| node == "D"
        ).unwrap();
        
        // The expected path is A -> C -> D (the cheapest path)
        assert_eq!(path, vec!["A", "C", "D"].into_iter().map(String::from).collect::<Vec<_>>());
    }

    #[test]
    fn test_dijkstra_handles_unreachable_goal() {
        // Create a graph with no path to the goal
        let mut graph = HashMap::new();
        graph.insert("A".to_string(), vec![("B".to_string(), 1)]);
        graph.insert("B".to_string(), vec![("C".to_string(), 1)]);
        graph.insert("C".to_string(), vec![]);
        graph.insert("D".to_string(), vec![]); // D is not connected
        
        let neighbors = create_neighbor_fn(&graph);
        
        // Try to find a path from A to D (which doesn't exist)
        let dijkstra = Dijkstra{};
        let result = dijkstra.plan("A".to_string(), neighbors, |node| node == "D");
        
        // Expect a NoPathFound error
        assert!(matches!(result, Err(PathPlannerError::NoPathFound)));
    }

    #[test]
    fn test_build_dijkstra_graph_with_cycle() {
        // Create a graph with a cycle: A -> B -> C -> A
        let mut graph = HashMap::new();
        
        graph.insert("A".to_string(), vec![("B".to_string(), 1)]);
        graph.insert("B".to_string(), vec![("C".to_string(), 1)]);
        graph.insert("C".to_string(), vec![("A".to_string(), 1), ("D".to_string(), 2)]);
        graph.insert("D".to_string(), vec![]);
        
        let neighbors = create_neighbor_fn(&graph);
        
        // Run Dijkstra's algorithm from node A
        let dijkstra = Dijkstra{};
        let (result, _) = dijkstra.build_graph(
            "A".to_string(),
            neighbors,
            |node| node == "D"
        ).unwrap();
        
        // Verify costs
        let costs: HashMap<_, _> = result.iter().map(|(node, (_, cost))| (node.clone(), *cost)).collect();
        
        assert_eq!(costs.get("A").unwrap(), &0);
        assert_eq!(costs.get("B").unwrap(), &1);
        assert_eq!(costs.get("C").unwrap(), &2);
        assert_eq!(costs.get("D").unwrap(), &4);
    }
}