//! Feasibility check for a route produced by [`super::HybridAStar`].
//!
//! The search already rejects primitives that sweep into an obstacle. This
//! check repeats that test from the stored curvature and arc length, so a
//! caller can audit a path without trusting the picture of it or re-running
//! the search. A path that spins in place, exceeds the turning radius, or
//! clips an obstacle between samples fails here.

use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Pose2d, angle_difference};
use crate::grid::GridMap;
use crate::motion::BicycleModel;

use super::{HybridAStarConfig, PlannedPath};

/// Confirm that `path` is a collision-free constant-curvature route to `goal`.
///
/// Integration uses [`Pose2d::advance`](crate::geometry::Pose2d::advance) and
/// must reproduce every stored pose and heading. Curvature must stay inside
/// the model's steering limit, which is the configured minimum turning radius.
/// A heading change with zero arc length (an in-place spin) fails that test.
/// The footprint is swept along each segment with `config.sweep_tolerance`.
pub fn check_planned_path(
    path: &PlannedPath,
    goal: Pose2d,
    map: &GridMap,
    model: &BicycleModel,
    footprint: &Footprint,
    config: &HybridAStarConfig,
) -> Result<(), PathPlannerError> {
    if !config.sweep_tolerance.is_finite() || config.sweep_tolerance <= 0.0 {
        return Err(PathPlannerError::InvalidConfiguration(
            "sweep_tolerance must be finite and positive".into(),
        ));
    }
    if !config.goal_xy_tolerance.is_finite() || !config.goal_yaw_tolerance.is_finite() {
        return Err(PathPlannerError::InvalidConfiguration(
            "goal tolerances must be finite".into(),
        ));
    }
    if path.poses.is_empty() {
        return Err(infeasible("path has no poses"));
    }
    if path.curvature.len() != path.poses.len() || path.segment_length.len() != path.poses.len() {
        return Err(infeasible(
            "pose, curvature, and segment-length tables differ in length",
        ));
    }
    if map.in_collision(path.poses[0], footprint) {
        return Err(PathPlannerError::StartInCollision);
    }
    let max_kappa = model.max_curvature();
    let mut integrated = path.poses[0];
    let mut length = 0.0;
    for index in 1..path.poses.len() {
        let kappa = path.curvature[index];
        let segment = path.segment_length[index];
        if !kappa.is_finite() || !segment.is_finite() {
            return Err(infeasible("curvature or segment length is non-finite"));
        }
        if kappa.abs() > max_kappa + 1e-6 {
            return Err(infeasible(
                "curvature exceeds the configured minimum turning radius",
            ));
        }
        if map.sweep_in_collision(
            integrated,
            kappa,
            segment,
            footprint,
            config.sweep_tolerance,
        ) {
            return Err(infeasible(
                "swept footprint hits an obstacle or leaves the map",
            ));
        }
        integrated = integrated.advance(kappa, segment);
        if integrated.distance(path.poses[index]) > 1e-6 {
            return Err(infeasible(
                "poses do not match constant-curvature integration",
            ));
        }
        if angle_difference(integrated.theta, path.poses[index].theta).abs() > 1e-6 {
            return Err(infeasible(
                "heading does not match constant-curvature integration",
            ));
        }
        length += segment.abs();
    }
    if (length - path.length).abs() > 1e-5 {
        return Err(infeasible("reported length does not match the segments"));
    }
    let end = path.poses[path.poses.len() - 1];
    if end.distance(goal) > config.goal_xy_tolerance + 1e-6 {
        return Err(infeasible("final position is outside the goal tolerance"));
    }
    if angle_difference(end.theta, goal.theta).abs() > config.goal_yaw_tolerance + 1e-6 {
        return Err(infeasible("final heading is outside the goal tolerance"));
    }
    Ok(())
}

fn infeasible(message: &str) -> PathPlannerError {
    PathPlannerError::PathInfeasible(message.into())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph_algos::SearchLimits;
    use crate::motion::BicycleParams;

    fn model() -> BicycleModel {
        BicycleModel::new(BicycleParams {
            wheelbase: 0.30,
            max_steering_angle: 0.50,
            max_steering_rate: 1.0,
            min_speed: -0.5,
            max_speed: 0.8,
            max_acceleration: 0.4,
        })
        .unwrap()
    }

    fn body() -> Footprint {
        Footprint::OrientedRectangle {
            length: 0.48,
            width: 0.30,
            rear_overhang: 0.12,
        }
    }

    fn config() -> HybridAStarConfig {
        HybridAStarConfig {
            sweep_tolerance: 0.02,
            goal_xy_tolerance: 0.2,
            goal_yaw_tolerance: 0.3,
            limits: SearchLimits::none(),
            ..HybridAStarConfig::default()
        }
    }

    fn straight_path() -> (GridMap, PlannedPath, Pose2d) {
        let map = GridMap::new(40, 40, 0.1, 0.0, 0.0).unwrap();
        let start = Pose2d::new(1.0, 2.0, 0.0);
        let goal = Pose2d::new(2.0, 2.0, 0.0);
        let path = PlannedPath {
            poses: vec![start, goal],
            curvature: vec![0.0, 0.0],
            segment_length: vec![0.0, 1.0],
            length: 1.0,
            cost: 1.0,
            expansions: 0,
        };
        (map, path, goal)
    }

    #[test]
    fn a_straight_segment_passes() {
        let (map, path, goal) = straight_path();
        check_planned_path(&path, goal, &map, &model(), &body(), &config()).unwrap();
    }

    #[test]
    fn an_in_place_spin_is_rejected() {
        let map = GridMap::new(40, 40, 0.1, 0.0, 0.0).unwrap();
        let start = Pose2d::new(2.0, 2.0, 0.0);
        let spun = Pose2d::new(2.0, 2.0, 1.0);
        let path = PlannedPath {
            poses: vec![start, spun],
            curvature: vec![0.0, 0.0],
            segment_length: vec![0.0, 0.0],
            length: 0.0,
            cost: 0.0,
            expansions: 0,
        };
        let error =
            check_planned_path(&path, spun, &map, &model(), &body(), &config()).unwrap_err();
        assert!(matches!(error, PathPlannerError::PathInfeasible(_)));
    }

    #[test]
    fn a_segment_that_only_clips_between_samples_is_rejected() {
        let mut map = GridMap::new(80, 80, 0.05, 0.0, 0.0).unwrap();
        // The cell around x = 1.50 is occupied. Endpoints of a 0.80 m move
        // from x = 1.0 miss it; the body crosses it in between.
        map.set_occupancy(30, 20, crate::grid::Occupancy::Occupied)
            .unwrap();
        let body = Footprint::OrientedRectangle {
            length: 0.40,
            width: 0.20,
            rear_overhang: 0.10,
        };
        let start = Pose2d::new(1.0, 1.0, 0.0);
        let end = start.advance(0.0, 0.80);
        assert!(!map.in_collision(start, &body));
        assert!(!map.in_collision(end, &body));
        let path = PlannedPath {
            poses: vec![start, end],
            curvature: vec![0.0, 0.0],
            segment_length: vec![0.0, 0.80],
            length: 0.80,
            cost: 0.80,
            expansions: 0,
        };
        let error = check_planned_path(&path, end, &map, &model(), &body, &config()).unwrap_err();
        assert!(
            matches!(error, PathPlannerError::PathInfeasible(ref message) if message.contains("swept")),
            "{error}"
        );
    }

    #[test]
    fn curvature_above_the_turning_limit_is_rejected() {
        let (map, mut path, goal) = straight_path();
        path.curvature[1] = model().max_curvature() + 0.1;
        path.poses[1] = path.poses[0].advance(path.curvature[1], path.segment_length[1]);
        let error =
            check_planned_path(&path, goal, &map, &model(), &body(), &config()).unwrap_err();
        assert!(
            matches!(error, PathPlannerError::PathInfeasible(ref message) if message.contains("turning")),
            "{error}"
        );
    }
}
