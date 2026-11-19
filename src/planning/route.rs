//! A route ticket bound to one occupancy snapshot, and a replan when that
//! snapshot changes.
//!
//! [`plan_route`] searches with [`super::HybridAStar`] and then runs
//! [`super::check_planned_path`]. The ticket stores the grid frame name and
//! [`crate::GridMap::revision`] it was checked against. After the grid
//! changes, [`RoutePlan::validated_path`] returns `None` until [`refresh_route`]
//! either re-checks the same geometry or searches again. An old ticket is
//! never reported as valid for a newer snapshot.
//!
//! Physical clearance, when requested, grows the footprint once. The sweep
//! tolerance still applies on top of that grown body as a numerical sampling
//! bound. The cost inflation layer is not a second copy of this clearance, and
//! neither number is a stopping distance.

use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Pose2d};
use crate::grid::GridMap;
use crate::motion::BicycleModel;

use super::{HybridAStar, HybridAStarConfig, PlannedPath, check_planned_path};

/// Inputs for one offline plan or refresh.
///
/// `clearance` is meters added on every side of `footprint` before the swept
/// collision test. Zero checks the body itself. It is not added again inside
/// the grid, and it is not a promise about braking distance.
#[derive(Clone, Copy, Debug)]
pub struct RouteRequest<'a> {
    /// Rear-axle start.
    pub start: Pose2d,
    /// Rear-axle goal.
    pub goal: Pose2d,
    /// Vehicle whose minimum turning radius limits the search.
    pub model: &'a BicycleModel,
    /// Body outline. Clearance is applied on top of this shape.
    pub footprint: &'a Footprint,
    /// Search settings. `sweep_tolerance` stays the numerical sweep bound.
    pub config: &'a HybridAStarConfig,
    /// Extra meters of hard clearance around the body. Must be finite and
    /// non-negative.
    pub clearance: f64,
}

/// A path that passed [`check_planned_path`] for one grid snapshot.
#[derive(Clone, Debug, PartialEq)]
pub struct RoutePlan {
    path: PlannedPath,
    revision: u64,
    frame_id: String,
    clearance: f64,
}

impl RoutePlan {
    /// Stored geometry.
    ///
    /// This remains readable after the grid changes. It is not a claim that
    /// the path is still valid. Use [`Self::validated_path`] for that.
    pub fn path(&self) -> &PlannedPath {
        &self.path
    }

    /// Grid revision the path was checked against.
    pub fn revision(&self) -> u64 {
        self.revision
    }

    /// Frame name the path was checked against.
    pub fn frame_id(&self) -> &str {
        &self.frame_id
    }

    /// Clearance, in meters, used for that check.
    pub fn clearance(&self) -> f64 {
        self.clearance
    }

    /// `true` when `map` is still the snapshot this ticket was checked for
    /// and `clearance` is the same value.
    ///
    /// Frame names compare as exact strings. Revisions compare as integers.
    /// Clearance compares with `==` on the stored value. There is no time
    /// window and no nearest-revision match.
    pub fn is_current(&self, map: &GridMap, clearance: f64) -> bool {
        self.revision == map.revision()
            && self.frame_id == map.frame_id()
            && self.clearance == clearance
    }

    /// The path when [`Self::is_current`] is true.
    pub fn validated_path(&self, map: &GridMap, clearance: f64) -> Option<&PlannedPath> {
        self.is_current(map, clearance).then_some(&self.path)
    }
}

/// What [`refresh_route`] did with a ticket after the caller may have edited
/// the grid.
#[derive(Clone, Debug, PartialEq)]
pub enum RouteRefresh {
    /// The previous geometry was checked against the current snapshot.
    ///
    /// When the snapshot was unchanged this is a clone of the previous ticket.
    /// When the snapshot changed and the geometry is still clear, this is a
    /// new ticket for the new revision. The previous ticket is not current.
    StillClear(RoutePlan),
    /// The previous geometry hits the current grid. The search found a
    /// replacement that passed the same swept check.
    Replanned(RoutePlan),
    /// The previous geometry is not clear on the current grid, and no
    /// replacement passed the check.
    ///
    /// The error is the planner or checker result: [`PathPlannerError::NoPathFound`],
    /// [`PathPlannerError::SearchLimitExceeded`], a start or goal collision, or
    /// [`PathPlannerError::PathInfeasible`]. It is not a validated route.
    NoSafeRoute(PathPlannerError),
}

/// Search and stamp a ticket for the current grid snapshot.
pub fn plan_route(
    map: &GridMap,
    request: &RouteRequest<'_>,
) -> Result<RoutePlan, PathPlannerError> {
    let body = checked_body(request)?;
    let path = HybridAStar::new(request.config.clone()).plan(
        request.start,
        request.goal,
        map,
        request.model,
        &body,
    )?;
    accept(map, request, &body, path)
}

/// Decide whether `previous` still holds on `map`, and search again if it does not.
///
/// An unchanged snapshot returns [`RouteRefresh::StillClear`] without searching.
/// A changed snapshot re-runs the swept check. A path that still misses
/// obstacles is stamped at the new revision. A path that hits is replaced by
/// a new search, or reported as [`RouteRefresh::NoSafeRoute`]. The previous
/// ticket is left untouched and is not current after a snapshot change.
pub fn refresh_route(
    map: &GridMap,
    previous: &RoutePlan,
    request: &RouteRequest<'_>,
) -> Result<RouteRefresh, PathPlannerError> {
    let body = checked_body(request)?;
    if previous.is_current(map, request.clearance) {
        return Ok(RouteRefresh::StillClear(previous.clone()));
    }
    let same_start = previous
        .path
        .poses
        .first()
        .is_some_and(|pose| pose.distance(request.start) <= 1e-6);
    if same_start
        && check_planned_path(
            &previous.path,
            request.goal,
            map,
            request.model,
            &body,
            request.config,
        )
        .is_ok()
    {
        return Ok(RouteRefresh::StillClear(stamp(
            map,
            previous.path.clone(),
            request.clearance,
        )));
    }
    match HybridAStar::new(request.config.clone()).plan(
        request.start,
        request.goal,
        map,
        request.model,
        &body,
    ) {
        Ok(path) => Ok(RouteRefresh::Replanned(accept(map, request, &body, path)?)),
        Err(error) => Ok(RouteRefresh::NoSafeRoute(error)),
    }
}

fn checked_body(request: &RouteRequest<'_>) -> Result<Footprint, PathPlannerError> {
    if !request.clearance.is_finite() || request.clearance < 0.0 {
        return Err(PathPlannerError::InvalidConfiguration(
            "clearance must be finite and non-negative".into(),
        ));
    }
    Ok(request.footprint.inflated(request.clearance))
}

fn accept(
    map: &GridMap,
    request: &RouteRequest<'_>,
    body: &Footprint,
    path: PlannedPath,
) -> Result<RoutePlan, PathPlannerError> {
    check_planned_path(
        &path,
        request.goal,
        map,
        request.model,
        body,
        request.config,
    )?;
    Ok(stamp(map, path, request.clearance))
}

fn stamp(map: &GridMap, path: PlannedPath, clearance: f64) -> RoutePlan {
    RoutePlan {
        path,
        revision: map.revision(),
        frame_id: map.frame_id().to_owned(),
        clearance,
    }
}
