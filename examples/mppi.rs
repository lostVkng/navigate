//! A few MPPI steps of an example car toward a goal.
//!
//! Run with `cargo run --example mppi`.
//! Add `--features rayon` to score the rollouts on a thread pool.

use navigate::prelude::*;

fn main() {
    let model = BicycleModel::new(BicycleParams {
        wheelbase: 2.0,
        max_steering_angle: 0.5,
        max_steering_rate: 0.8,
        min_speed: -1.0,
        max_speed: 2.0,
        max_acceleration: 1.2,
    })
    .expect("example car");
    let mut controller = Mppi::new(
        model,
        MppiConfig {
            horizon: 15,
            dt: 0.1,
            samples: 128,
            temperature: 0.5,
            seed: 1,
            reference_speed: 1.0,
            parallel: cfg!(feature = "rayon"),
            ..MppiConfig::default()
        },
    )
    .expect("controller");
    controller.add_critic(GoalCritic {
        position_weight: 3.0,
        heading_weight: 0.5,
    });
    controller.add_critic(PathTrackingCritic {
        lateral_weight: 2.0,
        heading_weight: 0.4,
        speed_weight: 0.15,
        progress_weight: 1.0,
    });
    controller.add_critic(ObstacleCritic {
        weight: 1.5,
        collision_cost: 500.0,
    });
    controller.add_critic(ControlSmoothnessCritic {
        acceleration_weight: 0.02,
        steering_rate_weight: 0.02,
        rate_weight: 0.05,
    });

    let grid = GridMap::new(40, 20, 0.5, 0.0, 0.0).expect("grid");
    let body = Footprint::Circle { radius: 0.4 };
    let path: Vec<_> = (0..16).map(|i| Pose2d::new(i as f64, 4.0, 0.0)).collect();
    let mut state = BicycleState::new(Pose2d::new(1.0, 4.2, 0.0), 0.0, 0.0);
    for _ in 0..8 {
        state = controller
            .step(state, &grid, &body, &path, Pose2d::new(12.0, 4.0, 0.0))
            .state;
    }
    println!(
        "after 8 steps the example car is at ({:.2}, {:.2}) with speed {:.2} m/s and steering {:.2} rad",
        state.pose.x, state.pose.y, state.velocity, state.steering
    );
}
