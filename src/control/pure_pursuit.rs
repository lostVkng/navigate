//! Regulated pure pursuit.
//!
//! Pure pursuit fits a circular arc from the rear axle to a lookahead point on
//! the reference path. With the lookahead expressed in the vehicle frame, the
//! curvature is
//!
//! ```text
//! κ = 2 y_ℓ / (x_ℓ² + y_ℓ²)
//! ```
//!
//! The lookahead distance scales with speed, `ℓ = clamp(|v| ℓ_t, ℓ_min, ℓ_max)`,
//! which is the adaptive rule the tracker is built on.
//!
//! Three regulations then reduce the linear speed (arXiv:2305.20026):
//!
//! * **Curvature.** Let `r_min` be the radius below which a turn is considered
//!   sharp. The regulated speed is `v / (r_min |κ|)` when `|κ| > 1/r_min`, and
//!   `v` otherwise. Gentle bends are left alone; tighter bends slow down in
//!   proportion to their radius.
//! * **Proximity.** Let `d_O` be the distance from the footprint's inscribed
//!   disk to the nearest occupied cell and `d_prox` the distance at which
//!   slowing begins. For `d_O ≤ d_prox` the speed is multiplied by
//!   `α d_O / d_prox` with `0 < α ≤ 1`.
//! * **Collision.** The commanded arc is sampled forward for
//!   `collision_horizon` seconds. Any hit sets the speed to zero.
//!
//! The commanded speed is the minimum of the curvature, proximity, and
//! goal-approach regulations, then lifted to `min_speed` while the vehicle
//! still has path left and is not stopping for a collision. The steering angle
//! is `atan(κ L)` clamped to the bicycle model's steering limit, and the yaw
//! rate uses that saturated curvature: `ω = v κ_cmd`.

use crate::control::VehicleCommand;
use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Pose2d};
use crate::grid::GridMap;
use crate::motion::{BicycleModel, project_onto_path};

/// Parameters of [`PurePursuit`].
#[derive(Clone, Debug)]
pub struct PurePursuitConfig {
    /// Seconds of speed used to grow the lookahead. `ℓ = |v| * lookahead_time`.
    pub lookahead_time: f64,
    /// Smallest lookahead distance, in meters. Also used when the vehicle is stopped.
    pub lookahead_min: f64,
    /// Largest lookahead distance, in meters.
    pub lookahead_max: f64,
    /// Unregulated speed along the path, in meters per second.
    pub desired_speed: f64,
    /// Floor applied to a positive regulated speed so the vehicle does not crawl.
    pub min_speed: f64,
    /// Turning radius, in meters, below which curvature regulation slows the vehicle.
    pub regulate_radius: f64,
    /// Obstacle distance, in meters, inside which proximity regulation is active.
    pub proximity_distance: f64,
    /// Proximity gain `α` in `(0, 1]`. Larger values slow down sooner.
    pub proximity_gain: f64,
    /// Distance-to-goal, in meters, inside which speed scales down linearly.
    pub goal_slow_distance: f64,
    /// How many seconds of the commanded arc are collision-checked.
    pub collision_horizon: f64,
}

impl Default for PurePursuitConfig {
    fn default() -> Self {
        Self {
            lookahead_time: 1.5,
            lookahead_min: 0.6,
            lookahead_max: 3.0,
            desired_speed: 1.0,
            min_speed: 0.15,
            regulate_radius: 1.5,
            proximity_distance: 1.5,
            proximity_gain: 1.0,
            goal_slow_distance: 1.5,
            collision_horizon: 1.5,
        }
    }
}

/// Geometric path tracker with regulated speed.
#[derive(Clone, Debug)]
pub struct PurePursuit {
    config: PurePursuitConfig,
}

impl PurePursuit {
    /// Tracker using `config`. The values are checked in [`Self::command`].
    pub fn new(config: PurePursuitConfig) -> Self {
        Self { config }
    }

    /// Configuration this tracker was built with.
    pub fn config(&self) -> &PurePursuitConfig {
        &self.config
    }

    /// Command that tracks `path` from `pose` at the current `speed`.
    ///
    /// `speed` is only used to scale the lookahead. The returned velocity is
    /// the regulated desired speed, not an integration of `speed`. An empty or
    /// single-pose path, or a pose already at the end within `lookahead_min`,
    /// produces a stopped command. Invalid configuration returns that error
    /// only through a zero command's absence: this method returns a command
    /// and reports configuration failures as [`VehicleCommand::stopped`] when
    /// the numbers cannot be used, so a control loop is not forced to unwrap.
    /// Call [`Self::validate`] to surface the error explicitly.
    pub fn command(
        &self,
        pose: Pose2d,
        speed: f64,
        path: &[Pose2d],
        map: &GridMap,
        footprint: &Footprint,
        model: &BicycleModel,
    ) -> VehicleCommand {
        if self.validate().is_err() || path.len() < 2 {
            return VehicleCommand::stopped();
        }
        let Some(projection) = project_onto_path(path, pose, 0) else {
            return VehicleCommand::stopped();
        };
        if projection.distance_remaining <= self.config.lookahead_min * 0.25
            && pose.distance(*path.last().unwrap_or(&pose)) <= self.config.lookahead_min
        {
            return VehicleCommand::stopped();
        }

        let lookahead_distance = (speed.abs() * self.config.lookahead_time)
            .clamp(self.config.lookahead_min, self.config.lookahead_max);
        let Some(target) = lookahead_point(
            path,
            projection.segment,
            projection.fraction,
            lookahead_distance,
        ) else {
            return VehicleCommand::stopped();
        };

        let local = pose.inverse_transform_point(target.point);
        let denom = local.x * local.x + local.y * local.y;
        let mut curvature = if denom < 1e-8 {
            0.0
        } else {
            2.0 * local.y / denom
        };
        if target.reverse {
            curvature = -curvature;
        }

        let mut velocity = regulate_speed(
            &self.config,
            curvature,
            projection.distance_remaining,
            obstacle_clearance(map, pose, footprint),
        );
        if target.reverse {
            velocity = -velocity;
        }
        if arc_collides(
            pose,
            curvature,
            velocity,
            map,
            footprint,
            model,
            &self.config,
        ) {
            return VehicleCommand::stopped();
        }

        let steering = model.steering_from_curvature(curvature);
        let commanded = model.curvature_from_steering(steering);
        VehicleCommand {
            velocity,
            steering_angle: steering,
            curvature: commanded,
            angular_velocity: velocity * commanded,
        }
    }

    /// Reject non-finite or inconsistent parameters.
    pub fn validate(&self) -> Result<(), PathPlannerError> {
        let c = &self.config;
        if !(c.lookahead_time.is_finite() && c.lookahead_time > 0.0) {
            return Err(bad("lookahead_time must be positive"));
        }
        if !(c.lookahead_min.is_finite() && c.lookahead_max.is_finite())
            || c.lookahead_min <= 0.0
            || c.lookahead_min > c.lookahead_max
        {
            return Err(bad("lookahead distances must satisfy 0 < min ≤ max"));
        }
        if !(c.desired_speed.is_finite() && c.desired_speed > 0.0) {
            return Err(bad("desired_speed must be positive"));
        }
        if !(c.min_speed.is_finite() && (0.0..c.desired_speed).contains(&c.min_speed)) {
            return Err(bad("min_speed must lie in [0, desired_speed)"));
        }
        if !(c.regulate_radius.is_finite() && c.regulate_radius > 0.0) {
            return Err(bad("regulate_radius must be positive"));
        }
        if !(c.proximity_distance.is_finite() && c.proximity_distance >= 0.0) {
            return Err(bad("proximity_distance must be non-negative"));
        }
        if !(c.proximity_gain.is_finite() && c.proximity_gain > 0.0 && c.proximity_gain <= 1.0) {
            return Err(bad("proximity_gain must lie in (0, 1]"));
        }
        if !(c.goal_slow_distance.is_finite() && c.goal_slow_distance >= 0.0) {
            return Err(bad("goal_slow_distance must be non-negative"));
        }
        if !(c.collision_horizon.is_finite() && c.collision_horizon >= 0.0) {
            return Err(bad("collision_horizon must be non-negative"));
        }
        Ok(())
    }
}

fn bad(message: &str) -> PathPlannerError {
    PathPlannerError::InvalidConfiguration(message.into())
}

struct Lookahead {
    point: crate::geometry::Point,
    reverse: bool,
}

fn lookahead_point(
    path: &[Pose2d],
    mut segment: usize,
    mut fraction: f64,
    mut left: f64,
) -> Option<Lookahead> {
    while segment + 1 < path.len() {
        let start = path[segment];
        let end = path[segment + 1];
        let length = start.distance(end).max(1e-9);
        let available = (1.0 - fraction) * length;
        if left <= available {
            let travel = fraction + left / length;
            return Some(Lookahead {
                point: interpolate(start, end, travel.clamp(0.0, 1.0)),
                reverse: segment_is_reverse(start, end),
            });
        }
        left -= available;
        segment += 1;
        fraction = 0.0;
    }
    path.last().map(|pose| Lookahead {
        point: pose.point(),
        reverse: false,
    })
}

fn interpolate(start: Pose2d, end: Pose2d, fraction: f64) -> crate::geometry::Point {
    crate::geometry::Point::new(
        start.x + fraction * (end.x - start.x),
        start.y + fraction * (end.y - start.y),
    )
}

fn segment_is_reverse(start: Pose2d, end: Pose2d) -> bool {
    let travel = (end.y - start.y).atan2(end.x - start.x);
    crate::geometry::angle_difference(travel, start.theta).abs() > std::f64::consts::FRAC_PI_2
}

fn regulate_speed(
    config: &PurePursuitConfig,
    curvature: f64,
    remaining: f64,
    clearance: Option<f64>,
) -> f64 {
    let mut speed = config.desired_speed;
    if curvature.abs() > 1e-6 {
        let radius = 1.0 / curvature.abs();
        if radius < config.regulate_radius {
            speed = (speed * radius / config.regulate_radius).min(speed);
        }
    }
    if let Some(clearance) = clearance {
        if clearance <= config.proximity_distance && config.proximity_distance > 0.0 {
            let scaled = speed * config.proximity_gain * clearance / config.proximity_distance;
            speed = scaled.min(speed);
        }
    }
    if remaining <= config.goal_slow_distance && config.goal_slow_distance > 0.0 {
        speed = (speed * remaining / config.goal_slow_distance).min(speed);
    }
    if remaining > config.lookahead_min * 0.25 {
        speed = speed.max(config.min_speed);
    }
    speed.max(0.0)
}

fn obstacle_clearance(map: &GridMap, pose: Pose2d, footprint: &Footprint) -> Option<f64> {
    let distance = map.distance_at(pose.x, pose.y)?;
    if !distance.is_finite() {
        return None;
    }
    Some((f64::from(distance) - footprint.inscribed_radius()).max(0.0))
}

fn arc_collides(
    pose: Pose2d,
    curvature: f64,
    velocity: f64,
    map: &GridMap,
    footprint: &Footprint,
    model: &BicycleModel,
    config: &PurePursuitConfig,
) -> bool {
    if map.in_collision(pose, footprint) {
        return true;
    }
    let distance = velocity.abs() * config.collision_horizon;
    if distance < 1e-6 {
        return false;
    }
    let signed = distance.copysign(velocity);
    let kappa = model.curvature_from_steering(model.steering_from_curvature(curvature));
    let spacing = map.resolution().clamp(0.05, 0.25);
    let pieces = (distance / spacing).ceil().max(1.0) as usize;
    let piece = signed / pieces as f64;
    let mut cursor = pose;
    for _ in 0..pieces {
        cursor = cursor.advance(kappa, piece);
        if map.in_collision(cursor, footprint) {
            return true;
        }
    }
    false
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Point;
    use crate::motion::{BicycleParams, BicycleState};

    fn model() -> BicycleModel {
        BicycleModel::new(BicycleParams {
            wheelbase: 2.0,
            max_steering_angle: 0.6,
            max_steering_rate: 1.0,
            min_speed: -1.5,
            max_speed: 2.0,
            max_acceleration: 2.0,
        })
        .unwrap()
    }

    fn straight(n: usize) -> Vec<Pose2d> {
        (0..n).map(|i| Pose2d::new(i as f64, 0.0, 0.0)).collect()
    }

    #[test]
    fn offset_vehicle_converges_toward_a_straight_path() {
        let tracker = PurePursuit::new(PurePursuitConfig {
            min_speed: 0.2,
            desired_speed: 1.0,
            collision_horizon: 0.5,
            ..PurePursuitConfig::default()
        });
        let path = straight(30);
        let map = GridMap::new(80, 40, 0.5, -5.0, -10.0).unwrap();
        let body = Footprint::Circle { radius: 0.3 };
        let bicycle = model();
        let mut state = BicycleState::new(Pose2d::new(0.0, 1.2, 0.0), 0.0, 0.0);
        for _ in 0..40 {
            let command = tracker.command(state.pose, state.velocity, &path, &map, &body, &bicycle);
            // The tracker reports a speed; realize it with one acceleration-limited step.
            let accel = (command.velocity - state.velocity).clamp(-2.0, 2.0);
            let steer_rate = (command.steering_angle - state.steering).clamp(-1.0, 1.0);
            state = bicycle.step(
                state,
                crate::motion::BicycleInput {
                    acceleration: accel / 0.1,
                    steering_rate: steer_rate / 0.1,
                },
                0.1,
            );
        }
        assert!(state.pose.y.abs() < 0.4, "lateral error {}", state.pose.y);
        assert!(
            state.pose.x > 2.0,
            "did not make progress, x={}",
            state.pose.x
        );
    }

    #[test]
    fn curvature_regulation_slows_a_tight_turn() {
        let tracker = PurePursuit::new(PurePursuitConfig {
            min_speed: 0.0,
            regulate_radius: 4.0,
            desired_speed: 1.5,
            lookahead_min: 1.5,
            lookahead_max: 1.5,
            lookahead_time: 1.0,
            goal_slow_distance: 0.0,
            proximity_distance: 0.0,
            collision_horizon: 0.0,
            proximity_gain: 1.0,
        });
        let map = GridMap::new(40, 40, 0.5, -5.0, -5.0).unwrap();
        let body = Footprint::Circle { radius: 0.2 };
        let bicycle = model();
        let straight_cmd = tracker.command(
            Pose2d::new(0.0, 0.0, 0.0),
            1.0,
            &straight(20),
            &map,
            &body,
            &bicycle,
        );
        let corner: Vec<_> = (0..10)
            .map(|i| Pose2d::new(0.0, i as f64 * 0.4, std::f64::consts::FRAC_PI_2))
            .collect();
        let corner_cmd = tracker.command(
            Pose2d::new(0.0, 0.0, 0.0),
            1.0,
            &corner,
            &map,
            &body,
            &bicycle,
        );
        assert!(
            straight_cmd.velocity > corner_cmd.velocity + 0.2,
            "straight {} corner {}",
            straight_cmd.velocity,
            corner_cmd.velocity
        );
        assert!(corner_cmd.steering_angle.abs() > 0.05);
    }

    #[test]
    fn proximity_and_collision_reduce_speed() {
        let mut map = GridMap::new(40, 40, 0.5, 0.0, 0.0).unwrap();
        map.set_aabb_occupied(Point::new(8.0, 4.0), Point::new(9.0, 6.0));
        map.compute_distance_field().unwrap();
        let body = Footprint::Circle { radius: 0.4 };
        let bicycle = model();
        let path = straight(20);
        let tracker = PurePursuit::new(PurePursuitConfig {
            min_speed: 0.0,
            proximity_distance: 3.0,
            proximity_gain: 1.0,
            goal_slow_distance: 0.0,
            collision_horizon: 0.0,
            regulate_radius: 0.1,
            ..PurePursuitConfig::default()
        });
        let far = tracker.command(
            Pose2d::new(1.0, 5.0, 0.0),
            1.0,
            &path,
            &map,
            &body,
            &bicycle,
        );
        let near = tracker.command(
            Pose2d::new(6.5, 5.0, 0.0),
            1.0,
            &path,
            &map,
            &body,
            &bicycle,
        );
        assert!(
            near.velocity < far.velocity,
            "near {} far {}",
            near.velocity,
            far.velocity
        );

        let blocking = PurePursuit::new(PurePursuitConfig {
            min_speed: 0.0,
            collision_horizon: 2.0,
            desired_speed: 1.0,
            proximity_distance: 0.0,
            goal_slow_distance: 0.0,
            ..PurePursuitConfig::default()
        });
        map.set_aabb_occupied(Point::new(3.0, 4.5), Point::new(4.5, 5.5));
        let stopped = blocking.command(
            Pose2d::new(1.0, 5.0, 0.0),
            1.0,
            &path,
            &map,
            &body,
            &bicycle,
        );
        assert!(
            stopped.velocity.abs() < 1e-9,
            "velocity {}",
            stopped.velocity
        );
    }
}
