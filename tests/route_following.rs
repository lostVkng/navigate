//! Offline route-following simulation.
//!
//! The plant is the placeholder bicycle. The tracker is the Stanley follower
//! in [`follow_route`]. A passing run is a kinematic result on these maps. It
//! is not evidence that a physical vehicle can stop or stay clear.

use navigate::planning::synthetic::{self, RouteQuery};
use navigate::prelude::*;

fn ticket(query: &RouteQuery) -> RoutePlan {
    plan_route(
        &query.map,
        &RouteRequest {
            start: query.start,
            goal: query.goal,
            model: &query.model,
            footprint: &query.footprint,
            config: &query.config,
            clearance: 0.0,
        },
    )
    .unwrap_or_else(|error| panic!("{} failed to plan: {error}", query.name))
}

fn drive(
    query: &RouteQuery,
    plan: &RoutePlan,
    config: &FollowConfig,
    model: &BicycleModel,
) -> Result<TrackingReport, TrackingError> {
    follow_route(&FollowRequest {
        plan,
        map: &query.map,
        model,
        footprint: &query.footprint,
        clearance: 0.0,
        sweep_tolerance: query.config.sweep_tolerance,
        config,
    })
}

fn drive_placeholder(
    query: &RouteQuery,
    plan: &RoutePlan,
) -> Result<TrackingReport, TrackingError> {
    drive(query, plan, &FollowConfig::placeholder(), &query.model)
}

fn planned_gears(path: &PlannedPath) -> usize {
    let mut gears = 0usize;
    let mut reverse = None;
    for length in path.segment_length.iter().copied() {
        if length.abs() < 1e-9 {
            continue;
        }
        let next = length < 0.0;
        if let Some(previous) = reverse {
            if previous != next {
                gears += 1;
            }
        }
        reverse = Some(next);
    }
    gears
}

struct Locked {
    cross: f64,
    heading: f64,
    goal_xy: f64,
    goal_yaw: f64,
    duration: f64,
    distance: f64,
    gears: usize,
}

fn assert_locked(name: &str, report: &TrackingReport, locked: Locked) {
    eprintln!(
        "{name:<16} cross {:.6} m  heading {:.6} rad  goal {:.6} m {:.6} rad  {:.2} s",
        report.max_cross_track,
        report.max_heading_error,
        report.goal_position_error,
        report.goal_heading_error,
        report.duration
    );
    for (label, got, expect) in [
        ("cross-track", report.max_cross_track, locked.cross),
        ("heading", report.max_heading_error, locked.heading),
        ("goal position", report.goal_position_error, locked.goal_xy),
        ("goal heading", report.goal_heading_error, locked.goal_yaw),
        ("duration", report.duration, locked.duration),
        ("distance", report.distance_traveled, locked.distance),
    ] {
        assert!(
            (got - expect).abs() < 1e-9,
            "{name} {label} {got:.12} != {expect:.12}"
        );
    }
    assert_eq!(report.direction_changes, locked.gears, "{name} gears");
    assert!(report.max_overshoot < 1e-6, "{name} overshoot");
    assert!(report.samples.len() > 2, "{name} samples");
    let end = report.samples.last().unwrap();
    assert!(
        (end.pose.x - report.final_state.pose.x).abs() < 1e-12,
        "{name} final sample"
    );
}

#[test]
fn open_field_tracks_and_repeats() {
    let query = synthetic::open_field();
    let plan = ticket(&query);
    let report = drive_placeholder(&query, &plan).expect("open field");
    let again = drive_placeholder(&query, &plan).expect("open field repeat");
    assert_eq!(report.samples, again.samples);
    assert_eq!(report.max_cross_track, again.max_cross_track);
    assert_eq!(report.goal_position_error, again.goal_position_error);
    assert_locked(
        "open field",
        &report,
        Locked {
            cross: 0.010141156372,
            heading: 0.073592878668,
            goal_xy: 0.039772253354,
            goal_yaw: 0.001609945090,
            duration: 27.2,
            distance: 4.171358329,
            gears: planned_gears(plan.path()),
        },
    );
}

#[test]
fn narrow_aisle_tracks_inside_the_corridor() {
    let query = synthetic::narrow_aisle();
    let plan = ticket(&query);
    let report = drive_placeholder(&query, &plan).expect("narrow aisle");
    assert_locked(
        "narrow aisle",
        &report,
        Locked {
            cross: 0.0,
            heading: 0.0,
            goal_xy: 0.039392862439,
            goal_yaw: 0.0,
            duration: 21.5,
            distance: 5.460607138,
            gears: 0,
        },
    );
    for sample in &report.samples {
        assert!(
            (0.70..1.30).contains(&sample.pose.y),
            "rear axle left the aisle at {:?}",
            sample.pose
        );
    }
}

#[test]
fn reverse_park_changes_gear() {
    let query = synthetic::parking_bay();
    let plan = ticket(&query);
    let gears = planned_gears(plan.path());
    assert!(gears >= 1, "planned parking route never changes gear");
    let report = drive_placeholder(&query, &plan).expect("reverse park");
    assert_locked(
        "reverse park",
        &report,
        Locked {
            cross: 0.093449086993,
            heading: 0.243316265985,
            goal_xy: 0.082400131239,
            goal_yaw: 0.298338087211,
            duration: 42.05,
            distance: 2.694490726,
            gears,
        },
    );
    assert!(
        report.samples.iter().any(|sample| sample.reverse),
        "simulation never selected reverse"
    );
    assert!(
        report.samples.iter().any(|sample| sample.velocity < -0.02),
        "plant never drove backward"
    );
}

#[test]
fn u_turn_stays_forward() {
    let query = synthetic::uturn_course(synthetic::EXAMPLE_MAX_STEERING_RAD);
    let plan = ticket(&query);
    assert_eq!(planned_gears(plan.path()), 0);
    let report = drive_placeholder(&query, &plan).expect("u-turn");
    let again = drive_placeholder(&query, &plan).expect("u-turn repeat");
    assert_eq!(report.samples, again.samples);
    assert_locked(
        "u-turn",
        &report,
        Locked {
            cross: 0.008604071696,
            heading: 0.058680236908,
            goal_xy: 0.039955945774,
            goal_yaw: 0.011005836105,
            duration: 48.8,
            distance: 8.129481570,
            gears: 0,
        },
    );
    assert!(report.samples.iter().all(|sample| !sample.reverse));
    assert!(
        report.final_state.pose.distance(query.goal) < 0.25,
        "final pose {:?} is far from the goal {:?}",
        report.final_state.pose,
        query.goal
    );
}

#[test]
fn a_short_delay_degrades_tracking_and_a_long_one_collides() {
    let query = synthetic::uturn_course(synthetic::EXAMPLE_MAX_STEERING_RAD);
    let plan = ticket(&query);
    let nominal = drive_placeholder(&query, &plan).expect("nominal u-turn");

    let mut delayed = FollowConfig::placeholder();
    delayed.actuator_delay = 0.40;
    let degraded = drive(&query, &plan, &delayed, &query.model).expect("0.40 s delay still tracks");
    assert!(
        degraded.max_cross_track > nominal.max_cross_track + 0.01,
        "delay did not increase cross-track: nominal {} delayed {}",
        nominal.max_cross_track,
        degraded.max_cross_track
    );

    let mut late = FollowConfig::placeholder();
    late.actuator_delay = 1.20;
    let error = drive(&query, &plan, &late, &query.model).expect_err("1.20 s delay should fail");
    assert!(
        matches!(error, TrackingError::Collision { .. }),
        "long delay: {error}"
    );
}

#[test]
fn a_slower_steering_rate_degrades_and_a_tight_stop_collides() {
    let query = synthetic::uturn_course(synthetic::EXAMPLE_MAX_STEERING_RAD);
    let plan = ticket(&query);
    let nominal = drive_placeholder(&query, &plan).expect("nominal u-turn");

    let slower = BicycleModel::new(BicycleParams {
        max_steering_rate: 0.55,
        ..query.model.params()
    })
    .expect("slower steering rate");
    let degraded = drive(&query, &plan, &FollowConfig::placeholder(), &slower)
        .expect("0.55 rad/s still tracks");
    assert!(
        degraded.max_cross_track > nominal.max_cross_track + 5e-4,
        "rate limit did not increase cross-track: nominal {} limited {}",
        nominal.max_cross_track,
        degraded.max_cross_track
    );

    let tight = BicycleModel::new(BicycleParams {
        max_steering_angle: 0.22,
        ..query.model.params()
    })
    .expect("tight steering stop");
    let mut loose = FollowConfig::placeholder();
    loose.tolerances.max_cross_track = 8.0;
    loose.tolerances.max_heading_error = 4.0;
    loose.tolerances.goal_xy = 8.0;
    loose.tolerances.goal_yaw = 4.0;
    loose.max_duration = 40.0;
    let error = drive(&query, &plan, &loose, &tight).expect_err("tight stop should fail");
    assert!(
        matches!(error, TrackingError::Collision { .. }),
        "tight steering stop: {error}"
    );
}

#[test]
fn seeded_noise_repeats_and_moves_the_path() {
    let query = synthetic::open_field();
    let plan = ticket(&query);
    let quiet = drive_placeholder(&query, &plan).expect("quiet");
    let mut noisy = FollowConfig::placeholder();
    noisy.disturbance = Disturbance {
        seed: 11,
        position_stddev: 0.01,
        heading_stddev: 0.03,
        acceleration_stddev: 0.05,
        steering_rate_stddev: 0.04,
    };
    let first = drive(&query, &plan, &noisy, &query.model).expect("noise");
    let second = drive(&query, &plan, &noisy, &query.model).expect("noise repeat");
    assert_eq!(first.samples, second.samples);
    assert_eq!(first.max_cross_track, second.max_cross_track);
    assert!(
        (first.max_cross_track - quiet.max_cross_track).abs() > 1e-6
            || (first.final_state.pose.x - quiet.final_state.pose.x).abs() > 1e-6,
        "noise did not change the run"
    );
    eprintln!(
        "open field noise cross {:.6} m (quiet {:.6} m)",
        first.max_cross_track, quiet.max_cross_track
    );
}

#[test]
fn a_stale_ticket_is_refused_until_refresh() {
    let query = synthetic::open_field();
    let plan = ticket(&query);
    let mut map = query.map.clone();
    let revision = map.revision();
    map.set_frame_id("odom").expect("rename");
    assert_ne!(map.revision(), revision);
    let stale = follow_route(&FollowRequest {
        plan: &plan,
        map: &map,
        model: &query.model,
        footprint: &query.footprint,
        clearance: 0.0,
        sweep_tolerance: query.config.sweep_tolerance,
        config: &FollowConfig::placeholder(),
    });
    assert!(
        matches!(stale, Err(TrackingError::StaleRoute)),
        "stale ticket was followed: {stale:?}"
    );
    assert!(plan.validated_path(&map, 0.0).is_none());

    let request = RouteRequest {
        start: query.start,
        goal: query.goal,
        model: &query.model,
        footprint: &query.footprint,
        config: &query.config,
        clearance: 0.0,
    };
    let RouteRefresh::StillClear(fresh) = refresh_route(&map, &plan, &request).expect("refresh")
    else {
        panic!("renaming the frame should not force a new search");
    };
    assert!(fresh.is_current(&map, 0.0));
    assert!(!plan.is_current(&map, 0.0));
    let report = follow_route(&FollowRequest {
        plan: &fresh,
        map: &map,
        model: &query.model,
        footprint: &query.footprint,
        clearance: 0.0,
        sweep_tolerance: query.config.sweep_tolerance,
        config: &FollowConfig::placeholder(),
    })
    .expect("refreshed ticket");
    assert!(report.goal_position_error <= 0.12);

    let wrong_clearance = follow_route(&FollowRequest {
        plan: &fresh,
        map: &map,
        model: &query.model,
        footprint: &query.footprint,
        clearance: 0.05,
        sweep_tolerance: query.config.sweep_tolerance,
        config: &FollowConfig::placeholder(),
    });
    assert!(matches!(wrong_clearance, Err(TrackingError::StaleRoute)));
}

#[test]
fn invalid_follow_configuration_is_rejected() {
    let query = synthetic::open_field();
    let plan = ticket(&query);
    let mut config = FollowConfig::placeholder();
    config.dt = f64::NAN;
    let error = drive(&query, &plan, &config, &query.model).unwrap_err();
    assert!(
        matches!(error, TrackingError::InvalidConfiguration(ref message) if message.contains("dt")),
        "{error}"
    );

    config = FollowConfig::placeholder();
    config.actuator_delay = -1.0;
    let error = drive(&query, &plan, &config, &query.model).unwrap_err();
    assert!(
        matches!(error, TrackingError::InvalidConfiguration(ref message) if message.contains("actuator_delay")),
        "{error}"
    );

    config = FollowConfig::placeholder();
    config.disturbance.steering_rate_stddev = -0.1;
    let error = drive(&query, &plan, &config, &query.model).unwrap_err();
    assert!(
        matches!(error, TrackingError::InvalidConfiguration(_)),
        "{error}"
    );

    config = FollowConfig::placeholder();
    config.tolerances.goal_yaw = 0.0;
    let error = drive(&query, &plan, &config, &query.model).unwrap_err();
    assert!(
        matches!(error, TrackingError::InvalidConfiguration(ref message) if message.contains("goal_yaw")),
        "{error}"
    );

    let error = follow_route(&FollowRequest {
        plan: &plan,
        map: &query.map,
        model: &query.model,
        footprint: &query.footprint,
        clearance: -0.01,
        sweep_tolerance: 0.02,
        config: &FollowConfig::placeholder(),
    })
    .unwrap_err();
    assert!(
        matches!(error, TrackingError::InvalidConfiguration(ref message) if message.contains("clearance")),
        "{error}"
    );

    let error = follow_route(&FollowRequest {
        plan: &plan,
        map: &query.map,
        model: &query.model,
        footprint: &query.footprint,
        clearance: 0.0,
        sweep_tolerance: 0.0,
        config: &FollowConfig::placeholder(),
    })
    .unwrap_err();
    assert!(
        matches!(error, TrackingError::InvalidConfiguration(ref message) if message.contains("sweep_tolerance")),
        "{error}"
    );

    let mut params = query.model.params();
    params.min_speed = 0.0;
    let forward_only = BicycleModel::new(params).unwrap();
    let parking = synthetic::parking_bay();
    let parking_plan = ticket(&parking);
    let error = drive(
        &parking,
        &parking_plan,
        &FollowConfig::placeholder(),
        &forward_only,
    )
    .unwrap_err();
    assert!(
        matches!(error, TrackingError::InvalidConfiguration(ref message) if message.contains("min_speed")),
        "{error}"
    );
}
