//! Criterion benchmarks for the distance transform, hybrid-A*, pure pursuit, and MPPI.
//!
//! Run with `cargo bench --bench planners`.

use criterion::{BenchmarkId, Criterion, black_box, criterion_group, criterion_main};
use navigate::prelude::*;

fn occupied_grid(size: usize) -> Vec<bool> {
    let mut occupied = vec![false; size * size];
    for (index, cell) in occupied.iter_mut().enumerate() {
        *cell = index % 23 == 0;
    }
    occupied
}

fn distance_transform(c: &mut Criterion) {
    let mut group = c.benchmark_group("distance_transform");
    group.sample_size(10);
    for size in [128_usize, 256, 512] {
        let occupied = occupied_grid(size);
        group.bench_with_input(BenchmarkId::from_parameter(size), &size, |b, &size| {
            b.iter(|| squared_distance_transform(black_box(&occupied), size, size).unwrap());
        });
    }
    group.finish();
}

fn example_car() -> BicycleModel {
    BicycleModel::new(BicycleParams {
        wheelbase: 2.0,
        max_steering_angle: 0.5,
        max_steering_rate: 1.0,
        min_speed: -1.0,
        max_speed: 2.0,
        max_acceleration: 1.2,
    })
    .unwrap()
}

fn hybrid(c: &mut Criterion) {
    let mut grid = GridMap::new(80, 40, 0.5, 0.0, 0.0).unwrap();
    grid.set_aabb_occupied(Point::new(14.0, 0.0), Point::new(16.0, 12.0));
    let body = Footprint::Circle { radius: 0.45 };
    let model = example_car();
    let planner = HybridAStar::new(HybridAStarConfig {
        heading_bins: 16,
        step_length: 1.0,
        analytic_distance: 6.0,
        limits: SearchLimits::expansions(30_000),
        ..HybridAStarConfig::default()
    });
    let start = Pose2d::new(2.0, 5.0, 0.0);
    let goal = Pose2d::new(30.0, 8.0, 0.0);
    c.bench_function("hybrid_astar_40m_grid", |b| {
        b.iter(|| {
            planner
                .plan(black_box(start), black_box(goal), &grid, &model, &body)
                .unwrap()
        });
    });
}

fn pure_pursuit(c: &mut Criterion) {
    let tracker = PurePursuit::new(PurePursuitConfig::default());
    let model = example_car();
    let path: Vec<_> = (0..80)
        .map(|i| Pose2d::new(i as f64 * 0.25, 0.0, 0.0))
        .collect();
    let grid = GridMap::new(40, 20, 0.5, -1.0, -5.0).unwrap();
    let body = Footprint::Circle { radius: 0.4 };
    let pose = Pose2d::new(1.0, 0.4, 0.05);
    c.bench_function("pure_pursuit_command", |b| {
        b.iter(|| tracker.command(black_box(pose), 1.0, &path, &grid, &body, &model));
    });
}

fn mppi(c: &mut Criterion) {
    let mut group = c.benchmark_group("mppi_step");
    group.sample_size(10);
    let grid = GridMap::new(40, 20, 0.5, 0.0, 0.0).unwrap();
    let body = Footprint::Circle { radius: 0.4 };
    let path: Vec<_> = (0..20).map(|i| Pose2d::new(i as f64, 4.0, 0.0)).collect();
    let goal = Pose2d::new(14.0, 4.0, 0.0);
    let state = BicycleState::new(Pose2d::new(1.0, 4.2, 0.0), 0.4, 0.0);
    for samples in [256_usize, 1024] {
        group.bench_with_input(
            BenchmarkId::from_parameter(samples),
            &samples,
            |b, &samples| {
                let mut controller = Mppi::new(
                    example_car(),
                    MppiConfig {
                        horizon: 20,
                        dt: 0.1,
                        samples,
                        temperature: 0.8,
                        seed: 1,
                        reference_speed: 1.0,
                        parallel: cfg!(feature = "rayon"),
                        ..MppiConfig::default()
                    },
                )
                .unwrap();
                controller.add_critic(GoalCritic {
                    position_weight: 2.0,
                    heading_weight: 0.4,
                });
                controller.add_critic(PathTrackingCritic {
                    lateral_weight: 1.5,
                    heading_weight: 0.3,
                    speed_weight: 0.1,
                    progress_weight: 0.8,
                });
                controller.add_critic(ObstacleCritic {
                    weight: 1.0,
                    collision_cost: 200.0,
                });
                controller.add_critic(ControlSmoothnessCritic {
                    acceleration_weight: 0.02,
                    steering_rate_weight: 0.02,
                    rate_weight: 0.04,
                });
                controller.add_critic(ConstraintCritic { weight: 0.5 });
                b.iter(|| controller.step(black_box(state), &grid, &body, &path, goal));
            },
        );
    }
    group.finish();
}

criterion_group!(benches, distance_transform, hybrid, pure_pursuit, mppi);
criterion_main!(benches);
