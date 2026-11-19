//! Occupancy snapshots and replanning on top of the swept hybrid-A* planner.
//!
//! A ticket is valid only for the frame name, revision, and clearance it was
//! checked against. Inserting or removing cells makes the old ticket stale
//! even when the geometry is still clear.

use navigate::planning::synthetic::{self, RouteQuery};
use navigate::prelude::*;

fn request<'a>(query: &'a RouteQuery, clearance: f64) -> RouteRequest<'a> {
    RouteRequest {
        start: query.start,
        goal: query.goal,
        model: &query.model,
        footprint: &query.footprint,
        config: &query.config,
        clearance,
    }
}

fn paint(map: &mut GridMap, patch: (Point, Point), occupancy: Occupancy) -> u64 {
    let before = map.revision();
    let changed = map.apply_aabb(patch.0, patch.1, occupancy).expect("patch");
    assert!(changed, "patch did not change a cell");
    assert_eq!(map.revision(), before + 1);
    map.revision()
}

#[test]
fn shifted_origin_uses_world_meters_not_cell_indexes() {
    let mut local = GridMap::new(70, 40, 0.1, 0.0, 0.0).unwrap();
    let mut shifted = GridMap::new(70, 40, 0.1, 10.0, -4.0).unwrap();
    shifted.set_frame_id("synthetic-shifted").unwrap();
    let block = (Point::new(2.8, 0.0), Point::new(3.3, 2.4));
    local
        .apply_aabb(block.0, block.1, Occupancy::Occupied)
        .unwrap();
    shifted
        .apply_aabb(
            Point::new(block.0.x + 10.0, block.0.y - 4.0),
            Point::new(block.1.x + 10.0, block.1.y - 4.0),
            Occupancy::Occupied,
        )
        .unwrap();
    let (lx, ly) = local.cell_center(0, 0);
    assert!((lx - 0.05).abs() < 1e-12 && (ly - 0.05).abs() < 1e-12);
    let (sx, sy) = shifted.cell_center(0, 0);
    assert!((sx - 10.05).abs() < 1e-12 && (sy - (-3.95)).abs() < 1e-12);
    assert_ne!(local.frame_id(), shifted.frame_id());

    let query = synthetic::example_model();
    let body = synthetic::example_footprint();
    let config = synthetic::example_config();
    let local_request = RouteRequest {
        start: Pose2d::new(1.0, 2.0, 0.0),
        goal: Pose2d::new(5.0, 2.2, 0.0),
        model: &query,
        footprint: &body,
        config: &config,
        clearance: 0.0,
    };
    let shifted_request = RouteRequest {
        start: Pose2d::new(11.0, -2.0, 0.0),
        goal: Pose2d::new(15.0, -1.8, 0.0),
        ..local_request
    };
    let local_plan = plan_route(&local, &local_request).unwrap();
    let shifted_plan = plan_route(&shifted, &shifted_request).unwrap();
    assert!((local_plan.path().length - shifted_plan.path().length).abs() < 1e-6);
    assert_eq!(local_plan.path().expansions, shifted_plan.path().expansions);
    assert!(local_plan.validated_path(&shifted, 0.0).is_none());
    assert!(shifted_plan.validated_path(&local, 0.0).is_none());
    assert!(local_plan.path().poses.iter().any(|pose| pose.y > 2.4));
}

#[test]
fn unknown_band_blocks_and_freeing_it_allows_a_new_ticket() {
    let mut query = synthetic::gate_lane();
    let open = plan_route(&query.map, &request(&query, 0.0)).unwrap();
    let band = synthetic::gate_seal();
    paint(&mut query.map, band, Occupancy::Unknown);
    assert!(query.map.unknown_cells() > 0);
    assert!(open.validated_path(&query.map, 0.0).is_none());
    let blocked = plan_route(&query.map, &request(&query, 0.0)).unwrap_err();
    assert!(
        matches!(blocked, PathPlannerError::NoPathFound),
        "{blocked}"
    );
    let cleared = query
        .map
        .apply_aabb(band.0, band.1, Occupancy::Free)
        .unwrap();
    assert!(cleared);
    assert_eq!(query.map.unknown_cells(), 0);
    let plan = plan_route(&query.map, &request(&query, 0.0)).unwrap();
    query.check(plan.path()).unwrap();
    assert!(plan.validated_path(&query.map, 0.0).is_some());
}

#[test]
fn obstacle_cycle_invalidates_replans_and_can_fail_closed() {
    let mut query = synthetic::replan_corridor();
    let initial = plan_route(&query.map, &request(&query, 0.0)).unwrap();
    query.check(initial.path()).unwrap();
    assert!(initial.validated_path(&query.map, 0.0).is_some());
    let initial_length = initial.path().length;
    assert_eq!(initial.path().expansions, 1);
    assert!((initial_length - 4.2).abs() < 1e-9);

    paint(
        &mut query.map,
        synthetic::replan_side_patch(),
        Occupancy::Occupied,
    );
    assert!(initial.validated_path(&query.map, 0.0).is_none());
    let side = refresh_route(&query.map, &initial, &request(&query, 0.0)).unwrap();
    let RouteRefresh::StillClear(side_plan) = side else {
        panic!("side patch should leave the geometry clear: {side:?}");
    };
    assert_eq!(side_plan.path().poses, initial.path().poses);
    assert_ne!(side_plan.revision(), initial.revision());
    assert!(side_plan.validated_path(&query.map, 0.0).is_some());
    assert!(initial.validated_path(&query.map, 0.0).is_none());
    assert!(side_plan.validated_path(&query.map, 0.05).is_none());

    paint(
        &mut query.map,
        synthetic::replan_plug(),
        Occupancy::Occupied,
    );
    assert!(side_plan.validated_path(&query.map, 0.0).is_none());
    let around = refresh_route(&query.map, &side_plan, &request(&query, 0.0)).unwrap();
    let RouteRefresh::Replanned(around_plan) = around else {
        panic!("plug should force a replan: {around:?}");
    };
    query.check(around_plan.path()).unwrap();
    assert!(around_plan.path().length > initial_length + 0.2);
    assert!(
        around_plan.path().poses.iter().any(|pose| pose.y > 2.0),
        "replan did not use the gap above the plug"
    );
    assert!(around_plan.validated_path(&query.map, 0.0).is_some());
    assert_eq!(around_plan.path().expansions, 157);
    assert!((around_plan.path().length - 4.964951193).abs() < 1e-9);

    let plug = synthetic::replan_plug();
    let removed = query
        .map
        .apply_aabb(plug.0, plug.1, Occupancy::Free)
        .unwrap();
    assert!(removed);
    assert!(around_plan.validated_path(&query.map, 0.0).is_none());
    let kept = refresh_route(&query.map, &around_plan, &request(&query, 0.0)).unwrap();
    let RouteRefresh::StillClear(_) = kept else {
        panic!("removing the plug leaves the detour clear: {kept:?}");
    };
    let fresh = plan_route(&query.map, &request(&query, 0.0)).unwrap();
    assert!(fresh.path().length + 0.2 < around_plan.path().length);
    assert!((fresh.path().length - initial_length).abs() < 1e-6);
}

#[test]
fn clearance_is_applied_once_and_does_not_paint_cells() {
    let mut query = synthetic::replan_corridor();
    let gap = (Point::new(2.6, 1.45), Point::new(3.2, 1.95));
    // Free opening is 0.50 m. The 0.30 m body fits; growing it by 0.12 m per
    // side does not. The cells in the opening stay free either way.
    paint(
        &mut query.map,
        (Point::new(2.6, 0.8), Point::new(3.2, 1.45)),
        Occupancy::Occupied,
    );
    paint(
        &mut query.map,
        (Point::new(2.6, 1.95), Point::new(3.2, 2.6)),
        Occupancy::Occupied,
    );
    let (ix, iy) = query.map.world_to_cell(2.9, 1.7).unwrap();
    assert_eq!(query.map.occupancy(ix, iy), Some(Occupancy::Free));
    let open = plan_route(&query.map, &request(&query, 0.0));
    assert!(open.is_ok(), "{open:?}");
    assert_eq!(query.map.occupancy(ix, iy), Some(Occupancy::Free));
    let tight = plan_route(&query.map, &request(&query, 0.12)).unwrap_err();
    assert!(matches!(tight, PathPlannerError::NoPathFound), "{tight}");
    assert_eq!(query.map.occupancy(ix, iy), Some(Occupancy::Free));
    let _ = gap;
}

#[test]
fn closing_the_gate_reports_no_safe_route() {
    let mut query = synthetic::gate_lane();
    let initial = plan_route(&query.map, &request(&query, 0.0)).unwrap();
    query.check(initial.path()).unwrap();
    paint(&mut query.map, synthetic::gate_seal(), Occupancy::Occupied);
    assert!(initial.validated_path(&query.map, 0.0).is_none());
    let sealed = refresh_route(&query.map, &initial, &request(&query, 0.0)).unwrap();
    let RouteRefresh::NoSafeRoute(error) = sealed else {
        panic!("a closed gate must not yield a route: {sealed:?}");
    };
    assert!(matches!(error, PathPlannerError::NoPathFound), "{error}");
    assert!(initial.validated_path(&query.map, 0.0).is_none());
}

#[test]
fn a_search_budget_does_not_validate_a_blocked_replan() {
    let mut query = synthetic::replan_corridor();
    let initial = plan_route(&query.map, &request(&query, 0.0)).unwrap();
    paint(
        &mut query.map,
        synthetic::replan_plug(),
        Occupancy::Occupied,
    );
    query.config.analytic_distance = 0.0;
    query.config.limits = SearchLimits::expansions(0);
    let refresh = refresh_route(&query.map, &initial, &request(&query, 0.0)).unwrap();
    let RouteRefresh::NoSafeRoute(error) = refresh else {
        panic!("budget must not produce a ticket: {refresh:?}");
    };
    assert!(
        matches!(
            error,
            PathPlannerError::SearchLimitExceeded {
                kind: SearchLimitKind::Expansions,
                ..
            }
        ),
        "{error}"
    );
    assert!(initial.validated_path(&query.map, 0.0).is_none());
}

#[test]
fn stale_and_invalid_requests_are_not_tickets() {
    let query = synthetic::replan_corridor();
    let plan = plan_route(&query.map, &request(&query, 0.0)).unwrap();
    let mut renamed = query.map.clone();
    renamed.set_frame_id("other").unwrap();
    assert!(plan.validated_path(&renamed, 0.0).is_none());
    let error = plan_route(
        &query.map,
        &RouteRequest {
            clearance: f64::NAN,
            ..request(&query, 0.0)
        },
    )
    .unwrap_err();
    assert!(matches!(error, PathPlannerError::InvalidConfiguration(_)));
    let error = refresh_route(
        &query.map,
        &plan,
        &RouteRequest {
            clearance: -0.1,
            ..request(&query, 0.0)
        },
    )
    .unwrap_err();
    assert!(matches!(error, PathPlannerError::InvalidConfiguration(_)));
    assert!(plan.is_current(&query.map, 0.0));
}
