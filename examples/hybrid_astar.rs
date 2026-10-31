//! Hybrid-A* for an example car around a rectangular obstacle.
//!
//! Run with `cargo run --example hybrid_astar`.

use navigate::prelude::*;

fn example_car() -> BicycleModel {
    BicycleModel::new(BicycleParams {
        wheelbase: 2.0,
        max_steering_angle: 0.5,
        max_steering_rate: 1.0,
        min_speed: -1.0,
        max_speed: 2.0,
        max_acceleration: 1.0,
    })
    .expect("example car")
}

fn main() {
    let mut grid = GridMap::new(80, 40, 0.5, 0.0, 0.0).expect("grid");
    grid.set_aabb_occupied(Point::new(12.0, 0.0), Point::new(14.0, 12.0));
    grid.inflate(&Inflation {
        radius: 1.2,
        inscribed_radius: 0.4,
        decay: CostDecay::Linear,
    })
    .expect("inflation");

    let body = Footprint::OrientedRectangle {
        length: 2.4,
        width: 1.2,
        rear_overhang: 0.5,
    };
    let planner = HybridAStar::new(HybridAStarConfig {
        heading_bins: 16,
        step_length: 1.0,
        analytic_distance: 8.0,
        limits: SearchLimits::expansions(50_000),
        ..HybridAStarConfig::default()
    });
    let path = planner
        .plan(
            Pose2d::new(2.0, 6.0, 0.0),
            Pose2d::new(28.0, 6.0, 0.0),
            &grid,
            &example_car(),
            &body,
        )
        .expect("path");

    let smoothed = PathSmoother::default()
        .smooth(&path.poses, &example_car(), &grid, &body)
        .expect("smooth");
    println!(
        "hybrid-A* length {:.2} m after {} expansions; smoothed polyline has {} poses",
        path.length,
        path.expansions,
        smoothed.len()
    );
}
