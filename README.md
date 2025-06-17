# navigate
[![Current Version](https://img.shields.io/crates/v/navigate)](https://crates.io/crates/navigate)
[![Documentation](https://docs.rs/navigate/badge.svg)](https://docs.rs/navigate)
[![License: Apache-2.0/MIT](https://img.shields.io/crates/l/navigate.svg)](#license)

Navigation modules in Rust

Navigation modules and algorithims in rust! Navigate is a crate seeking to make navigation easier in rust. It is focused on robotics but can be used for any navigation problem.

Current state is a work in progress. The following algorithms are implemented but more to come! 

## Using this crate

In your `Cargo.toml`, put:

``` ini
[dependencies]
navigate = "0.1.1"
```

Example Dijkstra (graph) Algorithm usage:
The algorithm takes a start node, a neighbor function, and a goal function. The neighbor function should return a vector of (node, cost) tuples. The goal function should return true when the goal is reached.

``` rust
use navigate::prelude::dijkstra;

// Helper function to create a neighbor function from a graph
// Assumes data stored as: HashMap<String, Vec<(String, u32)>>
fn create_neighbor_fn(graph: &HashMap<String, Vec<(String, u32)>>) -> impl Fn(&String) -> Vec<(String, u32)> + '_ {
    move |node: &String| {
        graph.get(node).unwrap_or(&vec![]).clone()
    }
}

fn main() {
    // Create some kind of graph structure
    let mut graph = HashMap::new();
    graph.insert("A".to_string(), vec![("B".to_string(), 1), ("C".to_string(), 3)]);
    graph.insert("B".to_string(), vec![("D".to_string(), 5)]);
    graph.insert("C".to_string(), vec![("D".to_string(), 1)]);
    graph.insert("D".to_string(), vec![]);

    let neighbors = create_neighbor_fn(&graph);

    // Run Dijkstra's algorithm from node A to node D
    let dijkstra = Dijkstra{};
    
    // Return the path of nodes to reach the goal
    let path = dijkstra.plan(
        "A".to_string(), // start node
        neighbors,
        |node| node == "D"
    )
}

```

Example RRT* (sampling) Algorithm usage:

RRT* requires a map to be defined that it will then sample points from. The map is defined by a set of polygons that represent the obstacles in the environment. In this example we'll add a goal function that allows a 10 radius range to the goal point. Our walk step function will be 5 and we'll calculate 1,000 iterations.

``` rust
use navigate::prelude::rrt_star;

fn main() {

    // Create map
    let mut map = Map::new((Point{x: 0.0, y: 0.0}, Point{x: 200.0, y: 100.0}));
    
    // Add obstacles
    map.set_obstacles(vec![
        Polygon::new(vec![Point{x: 20.0, y: 0.0}, Point{x: 20.0, y: 80.0}, Point{x: 22.0, y: 80.0}, Point{x: 22.0, y: 0.0}]).unwrap(),
        // Triangle in the middle-left
        Polygon::new(vec![
            Point{x: 50.0, y: 30.0},                   // Bottom left
            Point{x: 70.0, y: 30.0},                   // Bottom right
            Point{x: 60.0, y: 30.0 + 17.32},           // Top (height = side * √3/2)
            Point{x: 50.0, y: 30.0},                   // Close the polygon
        ]).unwrap(),
        // Hexagon in the middle-right
        Polygon::new(vec![
            Point{x: 120.0, y: 30.0},  // Bottom left
            Point{x: 140.0, y: 30.0}, // Bottom right
            Point{x: 150.0, y: 45.0}, // Middle right
            Point{x: 140.0, y: 60.0}, // Top right
            Point{x: 120.0, y: 60.0},  // Top left
            Point{x: 110.0, y: 45.0},  // Middle left
            Point{x: 120.0, y: 30.0},  // Close the polygon
        ]).unwrap(),
    ]);

    let mut rrt_star = RRTStar::new(map, Point{x: 10.0, y: 50.0}, |p| {

        // success if within 10 radius
        let goal_point = Point{x: 170.0, y: 50.0};

        let distance = euclidean(p.x, p.y, goal_point.x, goal_point.y);
        distance < 10.0
    }, 5.0, 5.0, 1000);

    // Find the path
    let rrt_star_path = rrt_star.plan();
}

```

Example RRT* visualization:
![RRT* Visualization](https://raw.githubusercontent.com/lostVkng/navigate/main/assets/rrt_star_path.png)


## Algorithms
- Dijkstra's Algorithm
- A* Algorithm
- RRT* Algorithm

## Feature Requests
Have any feature requests? Open an issue! 

## License
MIT