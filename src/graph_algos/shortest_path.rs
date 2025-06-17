use crate::errors::PathPlannerError;
use super::GraphNodeMap;

/// Construct the shortest path from the goal node to the start node
/// Returns the ordered path as a vector of nodes from start to goal
/// node_map: GraphNodeMap<N, C> - map of nodes with their parent index and cost
/// goal_index: usize - index of the goal node in the node_map
pub(crate) fn shortest_path<N, C>(node_map: &GraphNodeMap<N, C>, goal_index: usize) -> Result<Vec<N>, PathPlannerError> 
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
