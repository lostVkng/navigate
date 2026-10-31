//! Dijkstra on a tiny road network with floating-point costs.
//!
//! Run with `cargo run --example dijkstra`.

use std::collections::HashMap;

use navigate::prelude::*;

fn main() {
    let mut roads: HashMap<&str, Vec<(&str, f64)>> = HashMap::new();
    roads.insert("depot", vec![("north", 1.4), ("east", 2.2)]);
    roads.insert("north", vec![("dock", 3.1)]);
    roads.insert("east", vec![("dock", 1.1)]);
    roads.insert("dock", vec![]);

    let path = Dijkstra::new()
        .with_limits(SearchLimits::expansions(32))
        .plan(
            "depot",
            |node| roads.get(node).cloned().unwrap_or_default(),
            |node| *node == "dock",
        )
        .expect("the dock is reachable");

    println!("shortest path: {}", path.join(" -> "));
}
