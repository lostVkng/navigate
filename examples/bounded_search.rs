//! Bound a graph search and inspect its terminal outcome.
use navigate::graph_algos::bounded::{SearchLimits, dijkstra};
/// Bound a graph search and inspect its terminal outcome.
fn main() {
    let graph = [vec![(1, 2), (2, 5)], vec![(2, 1)], vec![]];
    let limits = SearchLimits {
        max_nodes: 16,
        max_frontier: 32,
        max_expansions: 32,
        max_heap_pops: 64,
        max_neighbor_polls: 128,
    };
    let report = dijkstra(
        0,
        |node| graph[*node].clone(),
        |node| *node == 2,
        limits,
        || false,
    );
    println!("{:?}", report);
}
