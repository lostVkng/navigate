//! Hybrid-A\* for a bicycle-model vehicle.

use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Pose2d, angle_difference};
use crate::graph_algos::{AStar, Dijkstra, SearchLimits};
use crate::grid::{GridMap, Occupancy};
use crate::motion::{
    BicycleModel, CurvePath, dubins_length, dubins_path, reeds_shepp_length, reeds_shepp_path,
};

/// Tunables for [`HybridAStar`].
///
/// Costs are in meters. Penalties are dimensionless multipliers on length, so
/// `reverse_penalty = 1` makes a meter of reverse travel cost two meters.
#[derive(Clone, Debug)]
pub struct HybridAStarConfig {
    /// Arc length of one motion primitive, in meters.
    pub step_length: f64,
    /// Number of heading buckets around the circle. At least 4.
    pub heading_bins: usize,
    /// How many steering angles to sample in `[-δ_max, δ_max]`, including the ends.
    pub steering_samples: usize,
    /// Generate reverse primitives when `true`. Analytic shots then use
    /// Reeds–Shepp; otherwise they use Dubins paths.
    pub allow_reverse: bool,
    /// Extra cost per meter traveled in reverse.
    pub reverse_penalty: f64,
    /// Extra cost per meter, scaled by `|δ| / δ_max`.
    pub steering_penalty: f64,
    /// Extra cost per meter, scaled by `|δ - δ_previous| / δ_max`.
    pub steering_change_penalty: f64,
    /// Added once when a primitive changes between forward and reverse.
    pub direction_change_penalty: f64,
    /// Multiplier on the grid's `[0, 1]` traversal cost, in meters of extra
    /// cost per meter traveled through a cell of cost `1`.
    pub costmap_weight: f64,
    /// Try an analytic shot when the Euclidean distance to the goal is at most
    /// this many meters.
    pub analytic_distance: f64,
    /// Goal is reached when the position is within this many meters.
    pub goal_xy_tolerance: f64,
    /// Goal is reached when the heading is within this many radians.
    pub goal_yaw_tolerance: f64,
    /// Speed assumed when converting the steering-rate limit into a per-step
    /// change in steering angle. Meters per second.
    pub nominal_speed: f64,
    /// Spacing used to collision-check and to emit the dense path, in meters.
    pub sample_spacing: f64,
    /// Expansion and time budgets for the search.
    pub limits: SearchLimits,
}

impl Default for HybridAStarConfig {
    fn default() -> Self {
        Self {
            step_length: 0.8,
            heading_bins: 24,
            steering_samples: 5,
            allow_reverse: true,
            reverse_penalty: 1.0,
            steering_penalty: 0.05,
            steering_change_penalty: 0.2,
            direction_change_penalty: 0.5,
            costmap_weight: 2.0,
            analytic_distance: 10.0,
            goal_xy_tolerance: 0.5,
            goal_yaw_tolerance: 0.35,
            nominal_speed: 1.0,
            sample_spacing: 0.25,
            limits: SearchLimits::none(),
        }
    }
}

impl HybridAStarConfig {
    fn validate(&self) -> Result<(), PathPlannerError> {
        if !self.step_length.is_finite() || self.step_length <= 0.0 {
            return Err(bad("step_length must be finite and positive"));
        }
        if self.heading_bins < 4 {
            return Err(bad("heading_bins must be at least 4"));
        }
        if self.steering_samples == 0 {
            return Err(bad("steering_samples must be at least 1"));
        }
        for (name, value) in [
            ("reverse_penalty", self.reverse_penalty),
            ("steering_penalty", self.steering_penalty),
            ("steering_change_penalty", self.steering_change_penalty),
            ("direction_change_penalty", self.direction_change_penalty),
            ("costmap_weight", self.costmap_weight),
            ("analytic_distance", self.analytic_distance),
            ("goal_xy_tolerance", self.goal_xy_tolerance),
            ("goal_yaw_tolerance", self.goal_yaw_tolerance),
            ("nominal_speed", self.nominal_speed),
            ("sample_spacing", self.sample_spacing),
        ] {
            if !value.is_finite() || value < 0.0 {
                return Err(bad(&format!("{name} must be finite and non-negative")));
            }
        }
        if self.nominal_speed == 0.0 || self.sample_spacing == 0.0 {
            return Err(bad("nominal_speed and sample_spacing must be positive"));
        }
        Ok(())
    }
}

fn bad(message: &str) -> PathPlannerError {
    PathPlannerError::InvalidConfiguration(message.into())
}

/// Pose sequence produced by [`HybridAStar`].
///
/// `curvature[i]` and `segment_length[i]` describe the constant-curvature step
/// from `poses[i - 1]` to `poses[i]`. Both are zero at index 0. A negative
/// `segment_length` is reverse travel. Re-integrating with
/// [`Pose2d::advance`](crate::geometry::Pose2d::advance) reproduces `poses`.
#[derive(Clone, Debug, PartialEq)]
pub struct PlannedPath {
    /// Sampled rear-axle poses, start through goal.
    pub poses: Vec<Pose2d>,
    /// Curvature, in inverse meters, of the segment that arrives at each pose.
    pub curvature: Vec<f64>,
    /// Signed arc length, in meters, of the segment that arrives at each pose.
    pub segment_length: Vec<f64>,
    /// Sum of the absolute segment lengths, in meters.
    pub length: f64,
    /// Search cost of the goal state, in meters of penalized travel.
    pub cost: f64,
    /// Nodes whose successors were generated.
    pub expansions: usize,
}

/// Hybrid-A\* global planner.
#[derive(Clone, Debug, Default)]
pub struct HybridAStar {
    config: HybridAStarConfig,
}

impl HybridAStar {
    /// Planner with the given configuration.
    ///
    /// The configuration is checked when [`Self::plan`] runs, so an invalid
    /// value is a [`PathPlannerError`] rather than a panic.
    pub fn new(config: HybridAStarConfig) -> Self {
        Self { config }
    }

    /// Configuration this planner was built with.
    pub fn config(&self) -> &HybridAStarConfig {
        &self.config
    }

    /// Search from `start` to `goal` on `map`.
    ///
    /// `model` supplies the wheelbase and the curvature, speed, and steering
    /// limits. `footprint` is collision-checked along every primitive and
    /// along an analytic shot. The start and the goal must themselves be
    /// collision-free and inside the grid.
    pub fn plan(
        &self,
        start: Pose2d,
        goal: Pose2d,
        map: &GridMap,
        model: &BicycleModel,
        footprint: &Footprint,
    ) -> Result<PlannedPath, PathPlannerError> {
        self.config.validate()?;
        if map.in_collision(start, footprint) {
            return Err(PathPlannerError::StartInCollision);
        }
        if map.in_collision(goal, footprint) {
            return Err(PathPlannerError::GoalInCollision);
        }
        if self.is_goal(start, goal) {
            return Ok(trivial_path(start));
        }

        let radius = model.min_turning_radius();
        let holonomic = holonomic_costs(&self.config, goal, map)?;
        let search = AStar::new().with_limits(self.config.limits.clone());
        let result = search.plan_keyed(
            CarState::start(start),
            |state: &CarState| discrete_key(state.pose, map, self.config.heading_bins),
            |state: &CarState| successors(state, goal, map, model, footprint, &self.config, radius),
            |state: &CarState| heuristic(state, goal, map, &holonomic, &self.config, radius),
            |state: &CarState| self.is_goal(state.pose, goal),
        )?;
        Ok(densify(
            &result.path,
            result.cost,
            result.expansions,
            self.config.sample_spacing,
        ))
    }

    fn is_goal(&self, pose: Pose2d, goal: Pose2d) -> bool {
        pose.distance(goal) <= self.config.goal_xy_tolerance
            && angle_difference(pose.theta, goal.theta).abs() <= self.config.goal_yaw_tolerance
    }
}

#[derive(Clone, Debug)]
struct CarState {
    pose: Pose2d,
    steering: f64,
    reverse: bool,
    has_moved: bool,
    arrival: Arrival,
}

impl CarState {
    fn start(pose: Pose2d) -> Self {
        Self {
            pose,
            steering: 0.0,
            reverse: false,
            has_moved: false,
            arrival: Arrival::Start,
        }
    }
}

#[derive(Clone, Debug)]
enum Arrival {
    Start,
    Arc { curvature: f64, length: f64 },
    Shot(Box<CurvePath>),
}

fn discrete_key(pose: Pose2d, map: &GridMap, bins: usize) -> (i32, i32, u16) {
    let (x, y) = map.world_to_cell(pose.x, pose.y).unwrap_or((0, 0));
    (x as i32, y as i32, heading_bin(pose.theta, bins))
}

fn heading_bin(theta: f64, bins: usize) -> u16 {
    let scaled = theta.rem_euclid(std::f64::consts::TAU) * (bins as f64) / std::f64::consts::TAU;
    let mut bin = scaled.round() as isize;
    bin = bin.rem_euclid(bins as isize);
    bin as u16
}

fn heuristic(
    state: &CarState,
    goal: Pose2d,
    map: &GridMap,
    holonomic: &[f64],
    config: &HybridAStarConfig,
    radius: f64,
) -> f64 {
    if state.pose.distance(goal) <= config.goal_xy_tolerance
        && angle_difference(state.pose.theta, goal.theta).abs() <= config.goal_yaw_tolerance
    {
        return 0.0;
    }
    let grid = map
        .world_to_cell(state.pose.x, state.pose.y)
        .map(|(x, y)| holonomic[y * map.width() + x])
        .unwrap_or(f64::INFINITY);
    let motion = if config.allow_reverse {
        reeds_shepp_length(state.pose, goal, radius)
    } else {
        dubins_length(state.pose, goal, radius)
    };
    grid.max(motion.unwrap_or(0.0))
}

fn holonomic_costs(
    config: &HybridAStarConfig,
    goal: Pose2d,
    map: &GridMap,
) -> Result<Vec<f64>, PathPlannerError> {
    let (gx, gy) = map
        .world_to_cell(goal.x, goal.y)
        .ok_or(PathPlannerError::OutOfBounds)?;
    let weight = config.costmap_weight;
    let resolution = map.resolution();
    let nodes = Dijkstra::new().get_all_nodes((gx as i32, gy as i32), move |&(x, y)| {
        grid_neighbors(x, y, map, resolution, weight)
    })?;
    let mut field = vec![f64::INFINITY; map.width() * map.height()];
    for (&(x, y), &(_, cost)) in &nodes {
        if x < 0 || y < 0 {
            continue;
        }
        let (x, y) = (x as usize, y as usize);
        if x < map.width() && y < map.height() {
            field[y * map.width() + x] = cost;
        }
    }
    Ok(field)
}

fn grid_neighbors(
    x: i32,
    y: i32,
    map: &GridMap,
    resolution: f64,
    weight: f64,
) -> Vec<((i32, i32), f64)> {
    const OFFSETS: [(i32, i32); 8] = [
        (1, 0),
        (-1, 0),
        (0, 1),
        (0, -1),
        (1, 1),
        (1, -1),
        (-1, 1),
        (-1, -1),
    ];
    let mut out = Vec::with_capacity(8);
    for (dx, dy) in OFFSETS {
        let nx = x + dx;
        let ny = y + dy;
        if !traversable(map, nx, ny) {
            continue;
        }
        if dx != 0 && dy != 0 && (!traversable(map, x + dx, y) || !traversable(map, x, y + dy)) {
            continue;
        }
        let step = if dx != 0 && dy != 0 {
            resolution * std::f64::consts::SQRT_2
        } else {
            resolution
        };
        let cell_cost = map.cost(nx as usize, ny as usize).unwrap_or(0.0);
        let edge = step * (1.0 + weight * f64::from(cell_cost));
        out.push(((nx, ny), edge));
    }
    out
}

fn traversable(map: &GridMap, x: i32, y: i32) -> bool {
    if x < 0 || y < 0 {
        return false;
    }
    matches!(map.occupancy(x as usize, y as usize), Some(Occupancy::Free))
}

fn successors(
    state: &CarState,
    goal: Pose2d,
    map: &GridMap,
    model: &BicycleModel,
    footprint: &Footprint,
    config: &HybridAStarConfig,
    radius: f64,
) -> Vec<(CarState, f64)> {
    let mut out = Vec::with_capacity(config.steering_samples * 2 + 1);
    primitive_successors(state, map, model, footprint, config, &mut out);
    if state.pose.distance(goal) <= config.analytic_distance {
        if let Some(shot) = analytic_successor(state, goal, map, model, footprint, config, radius) {
            out.push(shot);
        }
    }
    out
}

fn primitive_successors(
    state: &CarState,
    map: &GridMap,
    model: &BicycleModel,
    footprint: &Footprint,
    config: &HybridAStarConfig,
    out: &mut Vec<(CarState, f64)>,
) {
    let max_steer = model.params().max_steering_angle;
    let angles = linspace(-max_steer, max_steer, config.steering_samples);
    let directions = if config.allow_reverse {
        [1.0, -1.0]
    } else {
        [1.0, 0.0]
    };
    let rate_window = config.step_length / config.nominal_speed * model.params().max_steering_rate;
    for steering in angles {
        if state.has_moved && (steering - state.steering).abs() > rate_window + 1e-6 {
            continue;
        }
        for &direction in &directions {
            if direction == 0.0 {
                continue;
            }
            let length = direction * config.step_length;
            let curvature = model.curvature_from_steering(steering);
            if !sweep_clear(
                state.pose,
                curvature,
                length,
                map,
                footprint,
                config.sample_spacing,
            ) {
                continue;
            }
            let travel = length.abs();
            let mut cost = travel;
            if direction < 0.0 {
                cost += config.reverse_penalty * travel;
            }
            let steer_scale = if max_steer > 0.0 {
                steering.abs() / max_steer
            } else {
                0.0
            };
            let change_scale = if max_steer > 0.0 {
                (steering - state.steering).abs() / max_steer
            } else {
                0.0
            };
            cost += config.steering_penalty * steer_scale * travel;
            cost += config.steering_change_penalty * change_scale * travel;
            if state.has_moved && (direction < 0.0) != state.reverse {
                cost += config.direction_change_penalty;
            }
            cost += traversal_cost(state.pose, curvature, length, map, config);
            let end = state.pose.advance(curvature, length);
            out.push((
                CarState {
                    pose: end,
                    steering,
                    reverse: direction < 0.0,
                    has_moved: true,
                    arrival: Arrival::Arc { curvature, length },
                },
                cost,
            ));
        }
    }
}

fn analytic_successor(
    state: &CarState,
    goal: Pose2d,
    map: &GridMap,
    model: &BicycleModel,
    footprint: &Footprint,
    config: &HybridAStarConfig,
    radius: f64,
) -> Option<(CarState, f64)> {
    let path = if config.allow_reverse {
        reeds_shepp_path(state.pose, goal, radius)?
    } else {
        dubins_path(state.pose, goal, radius)?
    };
    if path_hits(&path, state.pose, map, footprint, config.sample_spacing) {
        return None;
    }
    let mut cost = 0.0;
    let mut cursor = state.pose;
    let mut reverse = state.reverse;
    for segment in &path.segments {
        let travel = segment.length.abs();
        cost += travel;
        if segment.length < 0.0 {
            cost += config.reverse_penalty * travel;
        }
        let steering = model.steering_from_curvature(segment.curvature);
        let scale = steering.abs() / model.params().max_steering_angle;
        cost += config.steering_penalty * scale * travel;
        let segment_reverse = segment.length < 0.0;
        if state.has_moved && segment_reverse != reverse {
            cost += config.direction_change_penalty;
        }
        reverse = segment_reverse;
        cost += traversal_cost(cursor, segment.curvature, segment.length, map, config);
        cursor = cursor.advance(segment.curvature, segment.length);
    }
    Some((
        CarState {
            pose: goal,
            steering: 0.0,
            reverse,
            has_moved: true,
            arrival: Arrival::Shot(Box::new(path)),
        },
        cost,
    ))
}

fn traversal_cost(
    start: Pose2d,
    curvature: f64,
    length: f64,
    map: &GridMap,
    config: &HybridAStarConfig,
) -> f64 {
    if config.costmap_weight == 0.0 || length.abs() < 1e-12 {
        return 0.0;
    }
    let pieces = sample_count(length.abs(), config.sample_spacing);
    let piece = length / pieces as f64;
    let mut pose = start;
    let mut cost = 0.0;
    for _ in 0..pieces {
        pose = pose.advance(curvature, piece);
        let cell = map.cost_at(pose.x, pose.y).unwrap_or(0.0);
        cost += config.costmap_weight * f64::from(cell) * piece.abs();
    }
    cost
}

fn sweep_clear(
    start: Pose2d,
    curvature: f64,
    length: f64,
    map: &GridMap,
    footprint: &Footprint,
    spacing: f64,
) -> bool {
    let pieces = sample_count(length.abs(), spacing);
    let piece = length / pieces as f64;
    let mut pose = start;
    for _ in 0..pieces {
        pose = pose.advance(curvature, piece);
        if map.in_collision(pose, footprint) {
            return false;
        }
    }
    true
}

fn path_hits(
    path: &CurvePath,
    start: Pose2d,
    map: &GridMap,
    footprint: &Footprint,
    spacing: f64,
) -> bool {
    let mut pose = start;
    for segment in &path.segments {
        if !sweep_clear(
            pose,
            segment.curvature,
            segment.length,
            map,
            footprint,
            spacing,
        ) {
            return true;
        }
        pose = pose.advance(segment.curvature, segment.length);
    }
    false
}

fn sample_count(length: f64, spacing: f64) -> usize {
    (length / spacing).ceil().max(1.0) as usize
}

fn linspace(min: f64, max: f64, count: usize) -> Vec<f64> {
    if count <= 1 {
        return vec![(min + max) * 0.5];
    }
    (0..count)
        .map(|i| min + (max - min) * (i as f64) / (count as f64 - 1.0))
        .collect()
}

fn densify(chain: &[CarState], cost: f64, expansions: usize, spacing: f64) -> PlannedPath {
    let mut poses = vec![chain[0].pose];
    let mut curvature = vec![0.0];
    let mut segment_length = vec![0.0];
    for state in chain.iter().skip(1) {
        match &state.arrival {
            Arrival::Start => {}
            Arrival::Arc {
                curvature: kappa,
                length,
            } => append_arc(
                &mut poses,
                &mut curvature,
                &mut segment_length,
                *kappa,
                *length,
                spacing,
            ),
            Arrival::Shot(path) => {
                for segment in &path.segments {
                    append_arc(
                        &mut poses,
                        &mut curvature,
                        &mut segment_length,
                        segment.curvature,
                        segment.length,
                        spacing,
                    );
                }
            }
        }
    }
    let length = segment_length.iter().map(|value| value.abs()).sum();
    PlannedPath {
        poses,
        curvature,
        segment_length,
        length,
        cost,
        expansions,
    }
}

fn append_arc(
    poses: &mut Vec<Pose2d>,
    curvature: &mut Vec<f64>,
    segment_length: &mut Vec<f64>,
    kappa: f64,
    length: f64,
    spacing: f64,
) {
    if length.abs() < 1e-12 {
        return;
    }
    let pieces = sample_count(length.abs(), spacing);
    let piece = length / pieces as f64;
    let Some(mut pose) = poses.last().copied() else {
        return;
    };
    for _ in 0..pieces {
        pose = pose.advance(kappa, piece);
        poses.push(pose);
        curvature.push(kappa);
        segment_length.push(piece);
    }
}

fn trivial_path(pose: Pose2d) -> PlannedPath {
    PlannedPath {
        poses: vec![pose],
        curvature: vec![0.0],
        segment_length: vec![0.0],
        length: 0.0,
        cost: 0.0,
        expansions: 0,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use crate::motion::BicycleParams;

    fn example_model() -> BicycleModel {
        BicycleModel::new(BicycleParams {
            wheelbase: 2.0,
            max_steering_angle: 0.6,
            max_steering_rate: 1.5,
            min_speed: -1.0,
            max_speed: 2.0,
            max_acceleration: 1.0,
        })
        .unwrap()
    }

    fn footprint() -> Footprint {
        Footprint::Circle { radius: 0.4 }
    }

    fn config() -> HybridAStarConfig {
        HybridAStarConfig {
            step_length: 1.0,
            heading_bins: 16,
            steering_samples: 5,
            analytic_distance: 12.0,
            goal_xy_tolerance: 0.6,
            goal_yaw_tolerance: 0.4,
            sample_spacing: 0.4,
            limits: SearchLimits::expansions(20_000),
            ..HybridAStarConfig::default()
        }
    }

    fn assert_feasible(path: &PlannedPath, model: &BicycleModel, map: &GridMap, body: &Footprint) {
        assert!(path.poses.len() >= 2, "path was empty");
        let mut pose = path.poses[0];
        let max_kappa = model.max_curvature() + 1e-6;
        for i in 1..path.poses.len() {
            assert!(
                path.curvature[i].abs() <= max_kappa,
                "curvature {} exceeds {max_kappa}",
                path.curvature[i]
            );
            pose = pose.advance(path.curvature[i], path.segment_length[i]);
            let error = pose.distance(path.poses[i]);
            assert!(error < 1e-6, "integration drift {error}");
            assert!(
                !map.in_collision(path.poses[i], body),
                "collision at {:?}",
                path.poses[i]
            );
        }
    }

    #[test]
    fn open_field_path_is_feasible_and_deterministic() {
        let map = GridMap::new(60, 30, 0.5, 0.0, 0.0).unwrap();
        let model = example_model();
        let body = footprint();
        let planner = HybridAStar::new(config());
        let start = Pose2d::new(2.0, 7.5, 0.0);
        let goal = Pose2d::new(24.0, 7.5, 0.0);
        let first = planner.plan(start, goal, &map, &model, &body).unwrap();
        let second = planner.plan(start, goal, &map, &model, &body).unwrap();
        assert_eq!(first.poses.len(), second.poses.len());
        assert!((first.cost - second.cost).abs() < 1e-9);
        assert_feasible(&first, &model, &map, &body);
        let end = *first.poses.last().unwrap();
        assert!(end.distance(goal) <= config().goal_xy_tolerance + 1e-6);
        assert!(
            angle_difference(end.theta, goal.theta).abs() <= config().goal_yaw_tolerance + 1e-6
        );
        assert!(first.length < 30.0, "path length {}", first.length);
    }

    #[test]
    fn path_goes_around_a_blocking_obstacle() {
        let mut map = GridMap::new(50, 40, 0.5, 0.0, 0.0).unwrap();
        map.set_aabb_occupied(Point::new(10.0, 0.0), Point::new(11.5, 12.0));
        map.inflate(&crate::grid::Inflation {
            radius: 0.8,
            inscribed_radius: 0.3,
            decay: crate::grid::CostDecay::Linear,
        })
        .unwrap();
        let model = example_model();
        let body = footprint();
        let path = HybridAStar::new(config())
            .plan(
                Pose2d::new(2.0, 5.0, 0.0),
                Pose2d::new(20.0, 5.0, 0.0),
                &map,
                &model,
                &body,
            )
            .unwrap();
        assert_feasible(&path, &model, &map, &body);
        assert!(
            path.poses.iter().any(|pose| pose.y > 12.0),
            "path did not clear the wall"
        );
    }

    #[test]
    fn reversing_reaches_a_goal_behind_the_vehicle() {
        let map = GridMap::new(30, 20, 0.5, 0.0, 0.0).unwrap();
        let model = example_model();
        let body = footprint();
        let mut config = config();
        config.allow_reverse = true;
        let path = HybridAStar::new(config)
            .plan(
                Pose2d::new(8.0, 5.0, 0.0),
                Pose2d::new(4.0, 5.0, 0.0),
                &map,
                &model,
                &body,
            )
            .unwrap();
        assert_feasible(&path, &model, &map, &body);
        assert!(
            path.segment_length.iter().any(|length| *length < 0.0),
            "expected a reverse segment, length {}",
            path.length
        );
        assert!(
            path.length < 12.0,
            "reverse path was too long: {}",
            path.length
        );
    }
}
