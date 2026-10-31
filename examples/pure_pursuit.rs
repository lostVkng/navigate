//! Track a straight reference with regulated pure pursuit.
//!
//! The example car starts one meter beside the path. A few seconds of
//! integration pull it back onto the line.
//!
//! Run with `cargo run --example pure_pursuit`.

use navigate::prelude::*;

fn main() {
    let model = BicycleModel::new(BicycleParams {
        wheelbase: 2.0,
        max_steering_angle: 0.6,
        max_steering_rate: 1.0,
        min_speed: -1.0,
        max_speed: 2.0,
        max_acceleration: 1.5,
    })
    .expect("example car");
    let tracker = PurePursuit::new(PurePursuitConfig {
        desired_speed: 1.0,
        min_speed: 0.2,
        ..PurePursuitConfig::default()
    });
    let path: Vec<_> = (0..40)
        .map(|i| Pose2d::new(i as f64 * 0.5, 0.0, 0.0))
        .collect();
    let grid = GridMap::new(50, 20, 0.5, -2.0, -5.0).expect("grid");
    let body = Footprint::Circle { radius: 0.4 };
    let mut state = BicycleState::new(Pose2d::new(0.0, 1.0, 0.0), 0.0, 0.0);

    for _ in 0..30 {
        let command = tracker.command(state.pose, state.velocity, &path, &grid, &body, &model);
        let input = BicycleInput {
            acceleration: ((command.velocity - state.velocity) / 0.1).clamp(-1.5, 1.5),
            steering_rate: ((command.steering_angle - state.steering) / 0.1).clamp(-1.0, 1.0),
        };
        state = model.step(state, input, 0.1);
    }
    println!(
        "after 3 s the example car is at ({:.2}, {:.2}), speed {:.2} m/s",
        state.pose.x, state.pose.y, state.velocity
    );
}
