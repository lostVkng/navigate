//! Synthetic maps for the offline route demo and the integration tests.
//!
//! Every length here is a round placeholder. [`EXAMPLE_WHEELBASE_M`], the
//! steering limit, and the footprint are not measurements of a vehicle, and
//! changing them is the supported way to change the minimum turning radius
//! (`wheelbase / tan(max steering)`). A route that passes on these maps is a
//! statement about this geometry only.

use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Point, Pose2d};
use crate::graph_algos::SearchLimits;
use crate::grid::{GridMap, Occupancy};
use crate::motion::{BicycleModel, BicycleParams};

use super::{HybridAStar, HybridAStarConfig, PlannedPath, check_planned_path};

/// Placeholder wheelbase, in meters.
pub const EXAMPLE_WHEELBASE_M: f64 = 0.30;
/// Placeholder steering stop, in radians.
pub const EXAMPLE_MAX_STEERING_RAD: f64 = 0.50;
/// Placeholder steering-rate limit, in radians per second.
pub const EXAMPLE_MAX_STEERING_RATE: f64 = 1.2;
/// Placeholder body length, in meters.
pub const EXAMPLE_LENGTH_M: f64 = 0.48;
/// Placeholder body width, in meters.
pub const EXAMPLE_WIDTH_M: f64 = 0.30;
/// Placeholder distance from the rear axle back to the body, in meters.
pub const EXAMPLE_REAR_OVERHANG_M: f64 = 0.12;

/// Width of the bulb in [`uturn_course`], in meters.
///
/// A vehicle whose minimum turning diameter is larger than this bulb, and
/// larger than the lanes that feed it, has no forward-only U-turn.
pub const UTURN_BULB_WIDTH_M: f64 = 1.8;

/// One offline planning problem: map, poses, and the vehicle that must drive it.
#[derive(Clone, Debug)]
pub struct RouteQuery {
    /// Short name used by the demo and the test output.
    pub name: &'static str,
    /// Synthetic occupancy grid.
    pub map: GridMap,
    /// Rear-axle start pose.
    pub start: Pose2d,
    /// Rear-axle goal pose.
    pub goal: Pose2d,
    /// Bicycle model. Its minimum turning radius is the curvature limit.
    pub model: BicycleModel,
    /// Body swept along the route.
    pub footprint: Footprint,
    /// Search settings, including the sweep tolerance and the expansion budget.
    pub config: HybridAStarConfig,
}

impl RouteQuery {
    /// Run hybrid-A* on this query.
    pub fn plan(&self) -> Result<PlannedPath, PathPlannerError> {
        HybridAStar::new(self.config.clone()).plan(
            self.start,
            self.goal,
            &self.map,
            &self.model,
            &self.footprint,
        )
    }

    /// Re-check curvature, integration, the goal tolerance, and the swept body.
    pub fn check(&self, path: &PlannedPath) -> Result<(), PathPlannerError> {
        check_planned_path(
            path,
            self.goal,
            &self.map,
            &self.model,
            &self.footprint,
            &self.config,
        )
    }

    /// Same query with reverse primitives and Reeds–Shepp shots disabled.
    pub fn forward_only(mut self) -> Self {
        self.config.allow_reverse = false;
        self.name = match self.name {
            "parking bay" => "parking bay, forward only",
            other => other,
        };
        self
    }
}

/// Placeholder compact vehicle at [`EXAMPLE_MAX_STEERING_RAD`].
pub fn example_model() -> BicycleModel {
    model_with_steering(EXAMPLE_MAX_STEERING_RAD)
}

/// Same placeholder wheelbase with a caller-chosen steering stop.
pub fn model_with_steering(max_steering_angle: f64) -> BicycleModel {
    BicycleModel::new(BicycleParams {
        wheelbase: EXAMPLE_WHEELBASE_M,
        max_steering_angle,
        max_steering_rate: EXAMPLE_MAX_STEERING_RATE,
        min_speed: -0.6,
        max_speed: 0.8,
        max_acceleration: 0.5,
    })
    .expect("placeholder vehicle parameters are finite and positive")
}

/// Placeholder rectangle centered on the rear axle in the lateral direction.
pub fn example_footprint() -> Footprint {
    Footprint::OrientedRectangle {
        length: EXAMPLE_LENGTH_M,
        width: EXAMPLE_WIDTH_M,
        rear_overhang: EXAMPLE_REAR_OVERHANG_M,
    }
}

/// Search settings sized for the placeholder vehicle and the maps in this module.
pub fn example_config() -> HybridAStarConfig {
    HybridAStarConfig {
        step_length: 0.30,
        heading_bins: 16,
        steering_samples: 5,
        allow_reverse: true,
        reverse_penalty: 0.4,
        steering_penalty: 0.05,
        steering_change_penalty: 0.1,
        direction_change_penalty: 0.3,
        costmap_weight: 0.5,
        analytic_distance: 5.0,
        goal_xy_tolerance: 0.20,
        goal_yaw_tolerance: 0.35,
        nominal_speed: 0.6,
        sample_spacing: 0.10,
        sweep_tolerance: 0.02,
        limits: SearchLimits::expansions(30_000),
    }
}

fn query(
    name: &'static str,
    map: GridMap,
    start: Pose2d,
    goal: Pose2d,
    model: BicycleModel,
) -> RouteQuery {
    RouteQuery {
        name,
        map,
        start,
        goal,
        model,
        footprint: example_footprint(),
        config: example_config(),
    }
}

fn grid(width_m: f64, height_m: f64, resolution: f64) -> GridMap {
    let width = (width_m / resolution).round() as usize;
    let height = (height_m / resolution).round() as usize;
    GridMap::new(width, height, resolution, 0.0, 0.0).expect("synthetic grid")
}

fn finish(map: &mut GridMap) {
    map.compute_distance_field()
        .expect("synthetic distance field");
}

/// Empty rectangle. The straight route is a regression for open space.
pub fn open_field() -> RouteQuery {
    let mut map = grid(6.0, 4.0, 0.10);
    finish(&mut map);
    query(
        "open field",
        map,
        Pose2d::new(0.8, 2.0, 0.0),
        Pose2d::new(5.0, 2.3, 0.0),
        example_model(),
    )
}

/// Corridor wider than the swept body.
pub fn narrow_aisle() -> RouteQuery {
    let mut map = grid(7.0, 2.0, 0.05);
    // Free band y in [0.70, 1.30]. Walls end on cell boundaries.
    map.set_aabb_occupied(Point::new(0.0, 0.0), Point::new(7.0, 0.70));
    map.set_aabb_occupied(Point::new(0.0, 1.30), Point::new(7.0, 2.0));
    finish(&mut map);
    query(
        "narrow aisle",
        map,
        Pose2d::new(0.7, 1.0, 0.0),
        Pose2d::new(6.2, 1.0, 0.0),
        example_model(),
    )
}

/// Corridor the true rectangle can sit in, but not with the sweep margin.
///
/// Free band y in [0.84, 1.16] is 0.32 m wide. The 0.30 m body has 0.01 m of
/// side clearance, which is less than the 0.02 m sweep tolerance, so every
/// motion primitive is rejected. The start pose itself is still free.
pub fn blocked_aisle() -> RouteQuery {
    let mut map = grid(7.0, 2.0, 0.02);
    map.set_aabb_occupied(Point::new(0.0, 0.0), Point::new(7.0, 0.84));
    map.set_aabb_occupied(Point::new(0.0, 1.16), Point::new(7.0, 2.0));
    finish(&mut map);
    query(
        "blocked aisle",
        map,
        Pose2d::new(0.7, 1.0, 0.0),
        Pose2d::new(6.2, 1.0, 0.0),
        example_model(),
    )
}

/// Wall that blocks the straight line. The route has to go around it.
pub fn obstacle_bypass() -> RouteQuery {
    let mut map = grid(8.0, 6.0, 0.10);
    map.set_aabb_occupied(Point::new(3.2, 0.0), Point::new(3.6, 3.4));
    finish(&mut map);
    query(
        "obstacle bypass",
        map,
        Pose2d::new(0.8, 2.0, 0.0),
        Pose2d::new(6.8, 2.0, 0.0),
        example_model(),
    )
}

/// Stall opening onto a lane. The goal heading points out of the stall, so a
/// forward-only vehicle cannot finish inside it.
pub fn parking_bay() -> RouteQuery {
    let mut map = grid(4.0, 3.2, 0.10);
    // Stall interior x in [1.5, 2.3], y in [0.6, 1.4], opening toward +y.
    map.set_aabb_occupied(Point::new(0.0, 0.0), Point::new(1.5, 1.4));
    map.set_aabb_occupied(Point::new(2.3, 0.0), Point::new(4.0, 1.4));
    map.set_aabb_occupied(Point::new(1.5, 0.0), Point::new(2.3, 0.6));
    finish(&mut map);
    query(
        "parking bay",
        map,
        Pose2d::new(0.6, 2.2, 0.0),
        Pose2d::new(1.9, 0.78, std::f64::consts::FRAC_PI_2),
        example_model(),
    )
}

/// Goal sealed inside a hollow box. The goal pose is free; no route reaches it.
///
/// The free space outside the box is only a short hallway, so exhausting the
/// search is a bounded piece of work and the result is [`PathPlannerError::NoPathFound`]
/// rather than an expansion-budget failure.
pub fn walled_off_goal() -> RouteQuery {
    let mut map = grid(4.0, 4.0, 0.10);
    fill_occupied(&mut map);
    carve_free(&mut map, Point::new(0.45, 1.95), Point::new(1.45, 2.45));
    carve_free(&mut map, Point::new(1.95, 1.70), Point::new(3.05, 2.70));
    finish(&mut map);
    query(
        "walled-off goal",
        map,
        Pose2d::new(0.9, 2.2, 0.0),
        Pose2d::new(2.5, 2.2, 0.0),
        example_model(),
    )
}

fn fill_occupied(map: &mut GridMap) {
    for iy in 0..map.height() {
        for ix in 0..map.width() {
            map.set_occupancy(ix, iy, Occupancy::Occupied).unwrap();
        }
    }
}

fn carve_free(map: &mut GridMap, min: Point, max: Point) {
    for iy in 0..map.height() {
        for ix in 0..map.width() {
            let (x, y) = map.cell_center(ix, iy);
            if x >= min.x && x <= max.x && y >= min.y && y <= max.y {
                map.set_occupancy(ix, iy, Occupancy::Free).unwrap();
            }
        }
    }
}

/// U-shaped course. `max_steering_angle` selects the minimum turning radius.
///
/// The bulb is [`UTURN_BULB_WIDTH_M`] wide. Forward motion only is the right
/// comparison: reverse would allow a multi-point turn that hides the radius
/// limit.
pub fn uturn_course(max_steering_angle: f64) -> RouteQuery {
    let mut map = grid(6.0, 3.6, 0.10);
    map.set_aabb_occupied(Point::new(0.0, 0.0), Point::new(6.0, 0.4));
    map.set_aabb_occupied(Point::new(0.0, 3.0), Point::new(6.0, 3.6));
    // Divider closes the left end. The bulb is x in [4.2, 6.0].
    map.set_aabb_occupied(Point::new(0.0, 1.3), Point::new(4.2, 2.1));
    finish(&mut map);
    let mut problem = query(
        "u-turn",
        map,
        Pose2d::new(0.9, 0.85, 0.0),
        Pose2d::new(0.9, 2.55, std::f64::consts::PI),
        model_with_steering(max_steering_angle),
    );
    problem.config.allow_reverse = false;
    problem
}
