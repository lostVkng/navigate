
pub mod dijkstra;
pub mod a_star;
mod shortest_path;

use shortest_path::shortest_path;

use crate::collections::FxIndexMap;

/// Type alias for the node map used in path planning algorithms
/// N: Node - space on a graph
/// C: Cost of reaching the node from the start
/// The tuple contains (parent_index, cost) where:
/// - parent_index is the index of the parent node in the map
/// - cost is the total cost to reach this node from the start
pub type GraphNodeMap<N, C> = FxIndexMap<N, (usize, C)>;