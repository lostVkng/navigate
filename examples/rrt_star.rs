//! Seeded RRT* around a rectangular obstacle.
//!
//! Run with `cargo run --example rrt_star`.

use navigate::prelude::*;

fn main() {
    let mut map = Map::new((Point::new(0.0, 0.0), Point::new(80.0, 40.0)));
    map.set_obstacles(vec![
        Polygon::new(vec![
            Point::new(35.0, 5.0),
            Point::new(42.0, 5.0),
            Point::new(42.0, 30.0),
            Point::new(35.0, 30.0),
        ])
        .expect("rectangle"),
    ]);

    let mut planner = RRTStar::new(
        map,
        Point::new(4.0, 20.0),
        |point| point.distance(Point::new(72.0, 20.0)) < 4.0,
        3.0,
        6.0,
        600,
    )
    .with_seed(7);

    let path = planner.plan().expect("a path around the obstacle");
    let length: f64 = path.windows(2).map(|pair| pair[0].distance(pair[1])).sum();
    println!(
        "RRT* path has {} vertices and length {:.2} m",
        path.len(),
        length
    );
}
