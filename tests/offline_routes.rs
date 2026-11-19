//! Offline hybrid-A* routes on synthetic maps.
//!
//! These tests are the milestone check: open space, a narrow aisle, swept
//! clearance, a reverse parking maneuver, an unreachable goal, invalid input,
//! and a bounded search. Feasibility comes from [`check_planned_path`], which
//! integrates the curvature and sweeps the footprint. The drawing produced by
//! `cargo run --example route_demo` is not the proof.

use std::time::Duration;

use navigate::planning::synthetic::{self, EXAMPLE_WHEELBASE_M, RouteQuery, UTURN_BULB_WIDTH_M};
use navigate::prelude::*;

fn assert_route(query: &RouteQuery, expansions: usize, length: f64) -> PlannedPath {
    let started = std::time::Instant::now();
    let path = query
        .plan()
        .unwrap_or_else(|error| panic!("{} failed to plan: {error}", query.name));
    let elapsed = started.elapsed();
    query
        .check(&path)
        .unwrap_or_else(|error| panic!("{} failed the feasibility check: {error}", query.name));
    let again = query
        .plan()
        .unwrap_or_else(|error| panic!("{} second plan: {error}", query.name));
    assert_eq!(path.poses, again.poses, "{}", query.name);
    assert_eq!(path.curvature, again.curvature, "{}", query.name);
    assert_eq!(path.segment_length, again.segment_length, "{}", query.name);
    assert_eq!(path.expansions, again.expansions, "{}", query.name);
    assert!((path.cost - again.cost).abs() < 1e-9, "{}", query.name);
    assert_eq!(path.expansions, expansions, "{} expansions", query.name);
    assert!(
        (path.length - length).abs() < 1e-9,
        "{} length {:.9}, expected {length:.9}",
        query.name,
        path.length
    );
    assert!(path.length > 0.0, "{}", query.name);
    eprintln!(
        "{:<22} length {:.9} m  cost {:.9}  expansions {:5}  {:?}",
        query.name, path.length, path.cost, path.expansions, elapsed
    );
    path
}

#[test]
fn placeholder_turning_radius_comes_from_the_steering_stop() {
    let model = synthetic::example_model();
    let radius = model.min_turning_radius();
    let expected = EXAMPLE_WHEELBASE_M / synthetic::EXAMPLE_MAX_STEERING_RAD.tan();
    assert!((radius - expected).abs() < 1e-12);
    assert!((model.max_curvature() * radius - 1.0).abs() < 1e-12);
}

#[test]
fn open_field_is_short_feasible_and_repeatable() {
    let query = synthetic::open_field();
    let path = assert_route(&query, 1, 4.210767950);
    let straight = query.start.distance(query.goal);
    assert!(
        path.length < straight * 1.35,
        "open-field length {} vs straight {straight}",
        path.length
    );
}

#[test]
fn narrow_aisle_stays_inside_the_corridor() {
    let query = synthetic::narrow_aisle();
    assert!(
        !query.map.in_collision(query.start, &query.footprint),
        "start should fit in the aisle"
    );
    let path = assert_route(&query, 3, 5.5);
    for pose in &path.poses {
        assert!(
            (0.85..1.15).contains(&pose.y),
            "rear axle left the aisle at {pose:?}"
        );
    }
}

#[test]
fn aisle_tighter_than_the_sweep_margin_is_unreachable() {
    let query = synthetic::blocked_aisle();
    assert!(
        !query.map.in_collision(query.start, &query.footprint),
        "the start pose itself is free; motion is what does not fit"
    );
    let error = query.plan().unwrap_err();
    assert!(
        matches!(error, PathPlannerError::NoPathFound),
        "blocked aisle: {error}"
    );
}

#[test]
fn obstacle_bypass_clears_the_wall_with_the_swept_body() {
    let query = synthetic::obstacle_bypass();
    let path = assert_route(&query, 75, 7.050705411);
    assert!(
        path.poses.iter().any(|pose| pose.y > 3.5),
        "route did not pass the wall"
    );
}

#[test]
fn parking_bay_uses_reverse_and_forward_only_cannot() {
    let query = synthetic::parking_bay();
    assert!(!query.map.in_collision(query.goal, &query.footprint));
    let path = assert_route(&query, 191, 2.744220511);
    assert!(
        path.segment_length.iter().any(|length| *length < 0.0),
        "parking route never reversed, length {}",
        path.length
    );

    let forward = query.clone().forward_only();
    let error = forward.plan().unwrap_err();
    assert!(
        matches!(error, PathPlannerError::NoPathFound),
        "forward-only parking: {error}"
    );
}

#[test]
fn turning_radius_decides_whether_the_u_turn_fits() {
    let tight = synthetic::uturn_course(synthetic::EXAMPLE_MAX_STEERING_RAD);
    assert!(tight.model.min_turning_radius() * 2.0 < UTURN_BULB_WIDTH_M + 0.9);
    let path = assert_route(&tight, 196, 8.175929983);
    assert!(
        path.curvature.iter().any(|kappa| kappa.abs() > 0.2),
        "u-turn did not steer"
    );
    assert!(
        path.segment_length.iter().all(|length| *length >= 0.0),
        "forward-only u-turn reversed"
    );

    let wide_angle = 0.18;
    let wide = synthetic::uturn_course(wide_angle);
    assert!(wide.model.min_turning_radius() * 2.0 > UTURN_BULB_WIDTH_M);
    let error = wide.plan().unwrap_err();
    assert!(
        matches!(error, PathPlannerError::NoPathFound),
        "wide turning radius: {error}"
    );
}

#[test]
fn a_sealed_goal_is_unreachable_without_being_in_collision() {
    let query = synthetic::walled_off_goal();
    assert!(!query.map.in_collision(query.start, &query.footprint));
    assert!(!query.map.in_collision(query.goal, &query.footprint));
    let error = query.plan().unwrap_err();
    assert!(
        matches!(error, PathPlannerError::NoPathFound),
        "walled goal: {error}"
    );
}

#[test]
fn collisions_at_the_endpoints_are_reported_before_the_search() {
    let mut blocked = synthetic::open_field();
    blocked
        .map
        .set_aabb_occupied(Point::new(4.6, 1.8), Point::new(5.4, 2.8));
    let error = blocked.plan().unwrap_err();
    assert!(
        matches!(error, PathPlannerError::GoalInCollision),
        "{error}"
    );

    let mut blocked = synthetic::open_field();
    blocked
        .map
        .set_aabb_occupied(Point::new(0.3, 1.5), Point::new(1.2, 2.5));
    let error = blocked.plan().unwrap_err();
    assert!(
        matches!(error, PathPlannerError::StartInCollision),
        "{error}"
    );
}

#[test]
fn invalid_inputs_are_rejected() {
    let query = synthetic::open_field();
    let planner = HybridAStar::new(HybridAStarConfig {
        step_length: 0.0,
        ..query.config.clone()
    });
    let error = planner
        .plan(
            query.start,
            query.goal,
            &query.map,
            &query.model,
            &query.footprint,
        )
        .unwrap_err();
    assert!(matches!(error, PathPlannerError::InvalidConfiguration(_)));

    let planner = HybridAStar::new(HybridAStarConfig {
        sweep_tolerance: -0.01,
        ..query.config.clone()
    });
    let error = planner
        .plan(
            query.start,
            query.goal,
            &query.map,
            &query.model,
            &query.footprint,
        )
        .unwrap_err();
    assert!(
        matches!(error, PathPlannerError::InvalidConfiguration(ref message) if message.contains("sweep_tolerance")),
        "{error}"
    );

    let planner = HybridAStar::new(query.config.clone());
    let error = planner
        .plan(
            Pose2d::new(f64::NAN, 2.0, 0.0),
            query.goal,
            &query.map,
            &query.model,
            &query.footprint,
        )
        .unwrap_err();
    assert!(
        matches!(error, PathPlannerError::InvalidConfiguration(ref message) if message.contains("finite")),
        "{error}"
    );

    let error = planner
        .plan(
            query.start,
            query.goal,
            &query.map,
            &query.model,
            &Footprint::Circle { radius: f64::NAN },
        )
        .unwrap_err();
    assert!(
        matches!(error, PathPlannerError::InvalidConfiguration(_)),
        "{error}"
    );

    let error = planner
        .plan(
            query.start,
            query.goal,
            &query.map,
            &query.model,
            &Footprint::OrientedRectangle {
                length: 0.4,
                width: 0.2,
                rear_overhang: 0.5,
            },
        )
        .unwrap_err();
    assert!(
        matches!(error, PathPlannerError::InvalidConfiguration(_)),
        "{error}"
    );
}

#[test]
fn expansion_and_time_budgets_stop_the_search() {
    let mut query = synthetic::open_field();
    query.config.analytic_distance = 0.0;
    query.config.limits = SearchLimits::expansions(3);
    let error = query.plan().unwrap_err();
    assert!(
        matches!(
            error,
            PathPlannerError::SearchLimitExceeded {
                kind: SearchLimitKind::Expansions,
                expansions: 3
            }
        ),
        "{error}"
    );

    query.config.limits = SearchLimits::duration(Duration::ZERO);
    let error = query.plan().unwrap_err();
    assert!(
        matches!(
            error,
            PathPlannerError::SearchLimitExceeded {
                kind: SearchLimitKind::Duration,
                ..
            }
        ),
        "{error}"
    );
}
