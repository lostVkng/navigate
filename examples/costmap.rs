//! Distance transform and inflation on a small occupancy grid.
//!
//! Run with `cargo run --example costmap`.

use navigate::prelude::*;

fn main() {
    let mut grid = GridMap::new(40, 30, 0.25, 0.0, 0.0).expect("grid");
    grid.set_aabb_occupied(Point::new(4.0, 2.0), Point::new(4.6, 6.0));
    grid.inflate(&Inflation {
        radius: 1.5,
        inscribed_radius: 0.4,
        decay: CostDecay::Exponential { scaling: 2.0 },
    })
    .expect("inflation");

    let body = Footprint::OrientedRectangle {
        length: 2.0,
        width: 1.0,
        rear_overhang: 0.4,
    };
    let blocked = grid.in_collision(Pose2d::new(4.2, 3.0, 0.0), &body);
    let clear = grid.in_collision(Pose2d::new(1.0, 1.0, 0.0), &body);
    let distance = grid.distance_at(6.0, 3.0).expect("inside the grid");
    println!(
        "obstacle cell distance at (6, 3) is {distance:.2} m; body on the obstacle: {blocked}; body in the open: collision={clear}"
    );
}
