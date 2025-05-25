use crate::errors::PathPlannerError;
use crate::collections::FxIndexMap;

use std::{collections::BinaryHeap, hash::Hash, cmp::Ordering, fmt::Debug};
use num_traits::Zero;
use indexmap::map::Entry::{Occupied, Vacant};




/// Identify the shortest path using Dijkstra's Algorithm
/// https://en.wikipedia.org/wiki/Dijkstra%27s_algorithm
/// From start Node, traverse through graph until node meets goal criteria
pub fn dijkstra<N, C, IT, NN, G>(start: N, neighbors: NN, goal: G) -> Result<Vec<N>, PathPlannerError>
where 
    N: Eq + Hash + Clone + Debug,
    NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
    IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
    C: Zero + Ord + Copy + Debug,
    G: Fn(&N) -> bool, // node qualifier for goal
    {

    // Build the graph - terminates when the goal is met
    let (node_map, goal_index) = build_dijkstra_graph(start, neighbors, goal)?;

    if let Some(goal_index) = goal_index {
        // find the path
        let path = dijkstra_path(&node_map, goal_index)?;
        Ok(path)
    } else {
        Err(PathPlannerError::NoPathFound)
    }
}


/// Return a partial map of the graph up to the goal node
/// Nodes with lower cost than the goal node will be included
pub fn dijkstra_nodes_partial<N, C, IT, NN, G>(start: N, neighbors: NN, goal: G) -> Result<FxIndexMap<N, (usize, C)>, PathPlannerError>
where 
    N: Eq + Hash + Clone + Debug,
    NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
    IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
    C: Zero + Ord + Copy + Debug,
    G: Fn(&N) -> bool,
    {

    // Build the graph - terminates when the goal is met
    let (node_map, _) = build_dijkstra_graph(start, neighbors, goal)?;
    
    Ok(node_map)
}

/// Returns a full map of the graph, includes all (reachable) nodes and costs
pub fn dijkstra_nodes_full<N, C, IT, NN>(start: N, neighbors: NN) -> Result<FxIndexMap<N, (usize, C)>, PathPlannerError>
where 
    N: Eq + Hash + Clone + Debug,
    NN: Fn(&N) -> IT, // returns iterator of neighbors + costs
    IT: IntoIterator<Item = (N, C)>, // Iterator of neighbors + edge cost to neighbor node
    C: Zero + Ord + Copy + Debug,
    {

    // Build the graph - terminates when the goal is met
    let (node_map, _) = build_dijkstra_graph(start, neighbors, |_| false)?;
    
    Ok(node_map)
}


/// Traverses the graph using Dijkstra's algorithm 
/// Returns a map of nodes with their smallest costs along with the index of the goal node
fn build_dijkstra_graph<N, C, IT, NN, G>(start: N, neighbors: NN, goal_fn: G) -> Result<(FxIndexMap<N, (usize, C)>, Option<usize>), PathPlannerError>
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
    let mut nodes_to_visit: BinaryHeap<NodeId<C>> = BinaryHeap::new();
    nodes_to_visit.push(NodeId{
        index: 0,
        cost: Zero::zero(), // This is the cost from the start node
    });

    // visited nodes - cost is known, no longer need to visit
    // usize is the index in the nodes_map
    // The tuple contains (parent_index, cost) where parent_index is the index of the parent node in the map
    // for the start node, parent_index is set to usize::MAX to indicate it has no parent
    let mut nodes_map: FxIndexMap<N, (usize, C)> = FxIndexMap::default();
    
    // Add start node to the map and queue
    let start_index = nodes_map.insert_full(start.clone(), (usize::MAX, Zero::zero())).0;
    nodes_to_visit.push(NodeId{
        index: start_index,
        cost: Zero::zero(), // This is the cost from the start node
    });

    // Loop over each node to visit, removing the smallest node
    while let Some(NodeId {cost, index}) = nodes_to_visit.pop() {

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
            nodes_to_visit.push(NodeId {
                index: neighbor_index,
                cost: new_cost,
            });
        }
    }
    
    Ok((nodes_map, None))
}


/// Construct the shortest path from the goal node to the start node
/// Returns the path as a vector of nodes from start to goal
pub fn dijkstra_path<N, C>(node_map: &FxIndexMap<N, (usize, C)>, goal_index: usize) -> Result<Vec<N>, PathPlannerError> 
where 
    N: Clone,
{

    let mut path = Vec::new();
    let mut current_index = goal_index;

    // Trace back from goal to start
    while current_index != usize::MAX {
        // Add the current node to the path
        if let Some((node, &(parent_index, _))) = node_map.get_index(current_index) {
            path.push(node.clone());
            current_index = parent_index;
        } else {
            return Err(PathPlannerError::NoPathFound);
        }
    }

    // The path is in reverse order, so reverse it
    path.reverse();

    if path.is_empty() {
        return Err(PathPlannerError::NoPathFound);
    }

    Ok(path)
}


/// Node identifier 
/// - for ordering we only need cost and a way to identify the node
/// - Nodes can contain additional data, but we only need to identify them
#[derive(Debug)]
struct NodeId<T> {
    index: usize,
    cost: T
}

impl<T: Ord> Ord for NodeId<T> {
    fn cmp(&self, other: &Self) -> Ordering {
        other.cost.cmp(&self.cost)
    }
}
impl<T: Ord> PartialOrd for NodeId<T> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl<T: PartialEq> PartialEq for NodeId<T> {
    fn eq(&self, other: &Self) -> bool {
        self.cost == other.cost
    }
}
impl<T: PartialEq> Eq for NodeId<T> {}


#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::HashMap;

    // Helper function to create a test graph
    fn create_test_graph() -> HashMap<String, Vec<(String, u32)>> {
        let mut graph = HashMap::new();
        
        // Diamond-shaped graph: A -> B -> D and A -> C -> D
        graph.insert("A".to_string(), vec![
            ("B".to_string(), 1),
            ("C".to_string(), 3),
        ]);
        
        graph.insert("B".to_string(), vec![
            ("D".to_string(), 5),
        ]);
        
        graph.insert("C".to_string(), vec![
            ("D".to_string(), 1),
        ]);
        
        graph.insert("D".to_string(), vec![]);
        
        graph
    }
    
    // Helper function to create a neighbor function from a graph
    fn create_neighbor_fn(graph: &HashMap<String, Vec<(String, u32)>>) -> impl Fn(&String) -> Vec<(String, u32)> + '_ {
        move |node: &String| {
            graph.get(node).unwrap_or(&vec![]).clone()
        }
    }

    #[test]
    fn test_build_dijkstra_graph_simple() {
        let graph = create_test_graph();
        let neighbors = create_neighbor_fn(&graph);
        
        // Run Dijkstra's algorithm from node A
        let (result, _) = build_dijkstra_graph(
            "A".to_string(),
            neighbors,
            |node| node == "D" // Goal is to reach node D
        ).unwrap();
        
        // Verify costs
        let costs: HashMap<_, _> = result.iter().map(|(node, (_, cost))| (node.clone(), *cost)).collect();
        
        assert_eq!(costs.get("A").unwrap(), &0);
        assert_eq!(costs.get("B").unwrap(), &1);
        assert_eq!(costs.get("C").unwrap(), &3);
        assert_eq!(costs.get("D").unwrap(), &4); // Should be 4 via the A->C->D path
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
        let (result, _) = build_dijkstra_graph(
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

    #[test]
    fn test_dijkstra_finds_optimal_path() {
        let graph = create_test_graph();
        let neighbors = create_neighbor_fn(&graph);
        
        // Run Dijkstra's algorithm from node A to node D
        let path = dijkstra(
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
        let result = dijkstra("A".to_string(), neighbors, |node| node == "D");
        
        // Expect a NoPathFound error
        assert!(matches!(result, Err(PathPlannerError::NoPathFound)));
    }

    #[test]
    fn test_dijkstra_nodes_partial_stops_at_cost_threshold() {
        // Create a graph where high-cost nodes won't be explored
        let mut graph = HashMap::new();
        
        // A -> B -> D (cost 2) is the shortest path to goal
        // A -> C -> E/F -> G/H are high-cost paths that shouldn't be explored
        graph.insert("A".to_string(), vec![
            ("B".to_string(), 1),
            ("C".to_string(), 10),
        ]);
        
        graph.insert("B".to_string(), vec![("D".to_string(), 1)]);
        graph.insert("C".to_string(), vec![("E".to_string(), 5), ("F".to_string(), 20)]);
        graph.insert("E".to_string(), vec![("G".to_string(), 5)]);
        graph.insert("F".to_string(), vec![("H".to_string(), 1)]);
        
        // Terminal nodes
        graph.insert("D".to_string(), vec![]);
        graph.insert("G".to_string(), vec![]);
        graph.insert("H".to_string(), vec![]);
        
        let neighbors = create_neighbor_fn(&graph);
        
        // Run Dijkstra's algorithm from node A with goal D
        let node_map = dijkstra_nodes_partial(
            "A".to_string(),
            neighbors,
            |node| node == "D"
        ).unwrap();
        
        // Verify explored vs unexplored nodes
        let explored = vec!["A", "B", "C", "D"];
        let unexplored = vec!["E", "F", "G", "H"];
        
        for node in explored {
            assert!(node_map.contains_key(node), "Node {node} should be explored");
        }
        
        for node in unexplored {
            assert!(!node_map.contains_key(node), "Node {node} should not be explored");
        }
        
        // Verify costs
        let costs: HashMap<_, _> = node_map.iter().map(|(node, (_, cost))| (node.clone(), *cost)).collect();
        assert_eq!(costs.get("D").unwrap(), &2); // A->B->D = 1+1 = 2
    }

    #[test]
    fn test_dijkstra_path_reconstruction() {
        // Create a node map manually to test path building
        let mut node_map: FxIndexMap<String, (usize, u32)> = FxIndexMap::default();
        
        // Insert nodes with their parent indices and costs
        let a_index = node_map.insert_full("A".to_string(), (usize::MAX, 0)).0;
        let b_index = node_map.insert_full("B".to_string(), (a_index, 1)).0;
        let c_index = node_map.insert_full("C".to_string(), (a_index, 3)).0;
        let d_index = node_map.insert_full("D".to_string(), (c_index, 4)).0;
        
        // Test path from A to D: A -> C -> D
        let path_to_d = dijkstra_path(&node_map, d_index).unwrap();
        assert_eq!(path_to_d, vec!["A", "C", "D"].into_iter().map(String::from).collect::<Vec<_>>());
        
        // Test path from A to B: A -> B
        let path_to_b = dijkstra_path(&node_map, b_index).unwrap();
        assert_eq!(path_to_b, vec!["A", "B"].into_iter().map(String::from).collect::<Vec<_>>());
    }

    #[test]
    fn test_dijkstra_complex_graph() {
        // Create a more complex graph with multiple paths
        let mut graph = HashMap::new();
        
        graph.insert("A".to_string(), vec![("B".to_string(), 4), ("C".to_string(), 2)]);
        graph.insert("B".to_string(), vec![("C".to_string(), 1), ("D".to_string(), 5)]);
        graph.insert("C".to_string(), vec![("D".to_string(), 8), ("E".to_string(), 10)]);
        graph.insert("D".to_string(), vec![("E".to_string(), 2), ("F".to_string(), 6)]);
        graph.insert("E".to_string(), vec![("F".to_string(), 3)]);
        graph.insert("F".to_string(), vec![]);
        
        let neighbors = create_neighbor_fn(&graph);
        
        // Run Dijkstra's algorithm from node A to node F
        let path = dijkstra("A".to_string(), neighbors, |node| node == "F").unwrap();
        
        // Calculate the total cost of the path
        let mut total_cost = 0;
        for i in 0..path.len() - 1 {
            let from = &path[i];
            let to = &path[i + 1];
            let edge_cost = graph.get(from).unwrap()
                .iter()
                .find(|(node, _)| node == to)
                .map(|(_, cost)| *cost)
                .unwrap();
            total_cost += edge_cost;
        }
        
        // The total cost should be 14 (A->B->D->E->F)
        assert_eq!(total_cost, 14);
    }
}