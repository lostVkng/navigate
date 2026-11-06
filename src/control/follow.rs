//! Offline simulation of a bicycle following a validated route.
//!
//! The tracker is a rear-axle Stanley controller with curvature feedforward
//! (Hoffmann, Tomlin, Montemerlo, and Thrun, Stanford University, 2007, the
//! steering law used on Stanley in the DARPA Grand Challenge). Pure pursuit is
//! already in this crate, and it is the wrong law for this check. Its lookahead
//! point can fall past a cusp, so a reverse park gets one blended command
//! instead of a stop and a gear change. Stanley steers from the cross-track and
//! heading errors of the active direction segment only, which are the same
//! errors the report records.
//!
//! The steering command, with the rear axle as the reference point, is
//!
//! ```text
//! δ = atan(κ L) + σ k_θ θ_e − atan2(k_e e, k_s + |v|)
//! ```
//!
//! `e` is the rear-axle offset, positive when the axle is to the left of the
//! path heading. `θ_e` is the path heading minus the vehicle heading.
//! `σ` is `+1` going forward and `−1` in reverse: yaw rate changes sign with
//! speed, so the heading term has to change sign with the gear. The cross-track
//! term does not, because the lateral velocity changes sign as well. `atan(κ L)`
//! is the steering angle of the planned arc and does not flip either; a
//! positive curvature is still a left steer while reversing.
//!
//! Direction segments are runs of the same sign of
//! [`PlannedPath::segment_length`](crate::planning::PlannedPath::segment_length).
//! The speed setpoint ramps to zero at the end of a segment. The gear flips
//! only after the plant speed has fallen below [`FollowConfig::switch_speed`].
//! The bicycle cannot yaw while stopped, so the last segment may crawl until
//! the heading is inside the goal band.
//!
//! Setpoints sit in a delay line for [`FollowConfig::actuator_delay`] seconds.
//! The plant then chases the delayed setpoint with [`BicycleModel::step`](crate::motion::BicycleModel::step),
//! which enforces the steering angle, steering rate, speed, and acceleration
//! limits. Pose noise is applied only to the measurement the tracker sees.
//! Actuation noise is added to the delayed acceleration and steering rate.
//! Both are zero-mean Gaussians from a seeded generator. The recorded errors
//! and the swept collision test use the plant pose.
//!
//! Every gain, speed, and delay below is a placeholder. A run that stays inside
//! tolerance shows that this kinematic model could follow this route. It is not
//! a measurement of a steering map, a braking distance, or a physical stop.

use std::collections::VecDeque;

use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::geometry::{Footprint, Pose2d, angle_difference};
use crate::grid::GridMap;
use crate::motion::{BicycleInput, BicycleModel, BicycleState, PathProjection, project_onto_path};
use crate::planning::{PlannedPath, RoutePlan};

/// Which tracking bound a simulation exceeded.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TrackingLimitKind {
    /// Distance from the rear axle to the active direction segment, in meters.
    ///
    /// This is the lateral error while the axle is beside the segment, and the
    /// straight-line miss once the axle has passed the segment's end.
    CrossTrack,
    /// Absolute heading error against the path, in radians.
    Heading,
}

/// Why [`follow_route`] refused to report a successful run.
#[derive(Clone, Debug, PartialEq)]
pub enum TrackingError {
    /// A caller-supplied number cannot be simulated.
    InvalidConfiguration(String),
    /// [`RoutePlan::is_current`](crate::planning::RoutePlan::is_current) is false.
    ///
    /// The stored path is not followed. Refresh the ticket against the grid
    /// before calling again.
    StaleRoute,
    /// Cross-track or heading left the configured band.
    TrackingExceeded {
        /// Which bound failed.
        kind: TrackingLimitKind,
        /// Absolute error at the failing sample.
        value: f64,
        /// Configured bound.
        limit: f64,
        /// Simulation time, in seconds.
        time: f64,
    },
    /// The footprint, grown by the route clearance, hit the grid while moving.
    ///
    /// The sweep uses the same half-chord test as the planner. It covers the
    /// arc the plant just drove, not the planned path.
    Collision {
        /// Simulation time at the end of the colliding step, in seconds.
        time: f64,
    },
    /// The plant stopped at the end of the route outside the goal box.
    GoalMiss {
        /// Distance from the rear axle to the route's final pose, in meters.
        position: f64,
        /// Absolute heading error against that pose, in radians.
        heading: f64,
    },
    /// The duration budget ended before the goal.
    DidNotFinish {
        /// Simulation time, in seconds.
        time: f64,
    },
    /// The integrated plant state was no longer a finite pose.
    Diverged {
        /// Simulation time, in seconds.
        time: f64,
    },
}

impl std::fmt::Display for TrackingError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            TrackingError::InvalidConfiguration(message) => {
                write!(f, "invalid configuration: {message}")
            }
            TrackingError::StaleRoute => {
                write!(f, "route ticket does not match the grid snapshot")
            }
            TrackingError::TrackingExceeded {
                kind,
                value,
                limit,
                time,
            } => {
                let name = match kind {
                    TrackingLimitKind::CrossTrack => "cross-track error",
                    TrackingLimitKind::Heading => "heading error",
                };
                write!(f, "{name} {value:.4} exceeded {limit:.4} at t={time:.3} s")
            }
            TrackingError::Collision { time } => {
                write!(f, "swept footprint collided at t={time:.3} s")
            }
            TrackingError::GoalMiss { position, heading } => {
                write!(f, "goal miss by {position:.4} m and {heading:.4} rad")
            }
            TrackingError::DidNotFinish { time } => {
                write!(f, "simulation ended at t={time:.3} s before the goal")
            }
            TrackingError::Diverged { time } => {
                write!(f, "plant state became non-finite at t={time:.3} s")
            }
        }
    }
}

impl std::error::Error for TrackingError {}

/// Bounds that make a simulated run a failure.
///
/// Distances are meters. Headings are radians. Every value must be finite and
/// positive. These are acceptance gates for the kinematic simulation, not a
/// calibrated safety envelope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingTolerances {
    /// Largest allowed distance from the rear axle to the active segment.
    pub max_cross_track: f64,
    /// Largest allowed absolute heading error along the route.
    pub max_heading_error: f64,
    /// Largest allowed distance from the final rear axle to the route's last pose.
    pub goal_xy: f64,
    /// Largest allowed absolute heading error at that pose.
    pub goal_yaw: f64,
}

/// Zero-mean Gaussian measurement and actuation noise.
///
/// A disturbance with every standard deviation at zero does not draw random
/// numbers. Any positive deviation uses [`Self::seed`] and is deterministic
/// across runs. Deviations are meters, radians, meters per second squared, and
/// radians per second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Disturbance {
    /// Seed for the Box–Muller generator.
    pub seed: u64,
    /// Standard deviation of the measured rear-axle `x` and `y`, in meters.
    pub position_stddev: f64,
    /// Standard deviation of the measured heading, in radians.
    pub heading_stddev: f64,
    /// Standard deviation added to the plant acceleration command.
    pub acceleration_stddev: f64,
    /// Standard deviation added to the plant steering-rate command.
    pub steering_rate_stddev: f64,
}

impl Disturbance {
    /// No measurement noise and no actuation noise.
    pub fn none() -> Self {
        Self {
            seed: 0,
            position_stddev: 0.0,
            heading_stddev: 0.0,
            acceleration_stddev: 0.0,
            steering_rate_stddev: 0.0,
        }
    }

    fn active(self) -> bool {
        self.position_stddev > 0.0
            || self.heading_stddev > 0.0
            || self.acceleration_stddev > 0.0
            || self.steering_rate_stddev > 0.0
    }
}

/// Placeholder Stanley gains, speed profile, delay, and acceptance gates.
///
/// [`Self::placeholder`] is the tuning used by the route-following tests and
/// the demo. It was chosen so the synthetic vehicle in
/// [`crate::planning::synthetic`] can track the routes that planner already
/// accepts. It was not identified from a vehicle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FollowConfig {
    /// Integration step, in seconds.
    pub dt: f64,
    /// Longest simulated time, in seconds. Hitting it is [`TrackingError::DidNotFinish`].
    pub max_duration: f64,
    /// Speed magnitude away from curves and stops, in meters per second.
    pub cruise_speed: f64,
    /// Distance over which the speed setpoint falls from cruise to zero, in meters.
    pub stop_distance: f64,
    /// Remaining distance, in meters, that counts as the end of a direction segment.
    pub arrive_distance: f64,
    /// Plant speed, in meters per second, below which a gear may change.
    pub switch_speed: f64,
    /// Path radius, in meters, below which the cruise speed scales down linearly.
    pub curvature_slow_radius: f64,
    /// Stanley cross-track gain `k_e`, in inverse seconds.
    pub cross_track_gain: f64,
    /// Softening speed `k_s`, in meters per second. Keeps the cross-track term finite when stopped.
    pub softening_speed: f64,
    /// Heading gain `k_θ`, dimensionless.
    pub heading_gain: f64,
    /// How far ahead, in meters, a steer change may be commanded early.
    ///
    /// The tracker reads the next path curvature whose steering angle differs
    /// by more than 0.20 rad. It switches the feedforward to that curvature
    /// only inside the distance the plant can slew at its steering-rate limit,
    /// and never farther than this preview.
    pub curvature_preview: f64,
    /// Distance, in meters, over which a steer change is meant to finish.
    ///
    /// The speed setpoint is capped at `rate * steer_slew_distance / gap`, and
    /// it is not lowered below [`Self::switch_speed`]. A steering rate that
    /// cannot finish `gap` radians in that distance at `switch_speed` will
    /// leave the path. This is a placeholder scheduling rule, not a measured
    /// steering response.
    pub steer_slew_distance: f64,
    /// Steering error, in radians, that must be closed before a segment starts moving.
    ///
    /// The plant can steer while stopped. Each direction segment waits at the
    /// cusp until the steering angle is this close to the Stanley setpoint, then
    /// releases the brake. A path that already sits on the steering stop has no
    /// extra angle left for feedback, so the axle has to enter that arc on the
    /// path rather than chase it after the gear is in.
    pub steer_align: f64,
    /// Time from a setpoint to the plant input, in seconds.
    ///
    /// Rounded to the nearest whole number of steps. Zero applies the setpoint
    /// on the same step. This is not a measured transport lag.
    pub actuator_delay: f64,
    /// When to fail the run.
    pub tolerances: TrackingTolerances,
    /// Seeded measurement and actuation noise.
    pub disturbance: Disturbance,
}

impl Default for FollowConfig {
    fn default() -> Self {
        Self::placeholder()
    }
}

impl FollowConfig {
    /// Gains and gates for the placeholder synthetic vehicle.
    ///
    /// Wheelbase 0.30 m, steering stop 0.50 rad, and the footprint in
    /// [`crate::planning::synthetic`] are assumed by the speed and the preview
    /// distance. A different vehicle needs a different [`FollowConfig`].
    pub fn placeholder() -> Self {
        Self {
            dt: 0.05,
            max_duration: 90.0,
            cruise_speed: 0.30,
            stop_distance: 0.55,
            arrive_distance: 0.04,
            switch_speed: 0.05,
            curvature_slow_radius: 1.35,
            cross_track_gain: 2.4,
            softening_speed: 0.25,
            heading_gain: 1.15,
            curvature_preview: 0.40,
            steer_slew_distance: 0.05,
            steer_align: 0.08,
            actuator_delay: 0.0,
            tolerances: TrackingTolerances {
                max_cross_track: 0.12,
                max_heading_error: 0.50,
                goal_xy: 0.12,
                goal_yaw: 0.30,
            },
            disturbance: Disturbance::none(),
        }
    }

    /// Reject non-finite or inconsistent simulation settings.
    pub fn validate(&self) -> Result<(), TrackingError> {
        if !self.dt.is_finite() || self.dt <= 0.0 || self.dt > 1.0 {
            return Err(invalid("dt must lie in (0, 1]"));
        }
        if !self.max_duration.is_finite() || self.max_duration < self.dt {
            return Err(invalid("max_duration must be finite and at least dt"));
        }
        let steps = self.max_duration / self.dt;
        if !steps.is_finite() || steps > 200_000.0 {
            return Err(invalid("max_duration / dt exceeds 200000 steps"));
        }
        if !self.cruise_speed.is_finite() || self.cruise_speed <= 0.0 {
            return Err(invalid("cruise_speed must be finite and positive"));
        }
        if !self.stop_distance.is_finite() || self.stop_distance <= 0.0 {
            return Err(invalid("stop_distance must be finite and positive"));
        }
        if !self.arrive_distance.is_finite()
            || self.arrive_distance <= 0.0
            || self.arrive_distance > self.stop_distance
        {
            return Err(invalid(
                "arrive_distance must be finite and lie in (0, stop_distance]",
            ));
        }
        if !self.switch_speed.is_finite()
            || self.switch_speed < 0.0
            || self.switch_speed >= self.cruise_speed
        {
            return Err(invalid("switch_speed must lie in [0, cruise_speed)"));
        }
        if !self.curvature_slow_radius.is_finite() || self.curvature_slow_radius <= 0.0 {
            return Err(invalid("curvature_slow_radius must be finite and positive"));
        }
        if !self.cross_track_gain.is_finite() || self.cross_track_gain <= 0.0 {
            return Err(invalid("cross_track_gain must be finite and positive"));
        }
        if !self.softening_speed.is_finite() || self.softening_speed < 0.0 {
            return Err(invalid("softening_speed must be finite and non-negative"));
        }
        if !self.heading_gain.is_finite() || self.heading_gain <= 0.0 {
            return Err(invalid("heading_gain must be finite and positive"));
        }
        if !self.curvature_preview.is_finite() || self.curvature_preview < 0.0 {
            return Err(invalid("curvature_preview must be finite and non-negative"));
        }
        if !self.steer_slew_distance.is_finite()
            || self.steer_slew_distance <= 0.0
            || self.steer_slew_distance > 2.0
        {
            return Err(invalid("steer_slew_distance must lie in (0, 2]"));
        }
        if !self.steer_align.is_finite() || self.steer_align <= 0.0 || self.steer_align > 1.0 {
            return Err(invalid("steer_align must lie in (0, 1]"));
        }
        if !self.actuator_delay.is_finite()
            || self.actuator_delay < 0.0
            || self.actuator_delay > self.max_duration
        {
            return Err(invalid(
                "actuator_delay must be finite and lie in [0, max_duration]",
            ));
        }
        validate_tolerances(&self.tolerances)?;
        validate_disturbance(&self.disturbance)?;
        Ok(())
    }
}

fn validate_tolerances(tolerances: &TrackingTolerances) -> Result<(), TrackingError> {
    for (name, value) in [
        ("max_cross_track", tolerances.max_cross_track),
        ("max_heading_error", tolerances.max_heading_error),
        ("goal_xy", tolerances.goal_xy),
        ("goal_yaw", tolerances.goal_yaw),
    ] {
        if !value.is_finite() || value <= 0.0 {
            return Err(invalid(&format!("{name} must be finite and positive")));
        }
    }
    Ok(())
}

fn validate_disturbance(disturbance: &Disturbance) -> Result<(), TrackingError> {
    for (name, value) in [
        ("position_stddev", disturbance.position_stddev),
        ("heading_stddev", disturbance.heading_stddev),
        ("acceleration_stddev", disturbance.acceleration_stddev),
        ("steering_rate_stddev", disturbance.steering_rate_stddev),
    ] {
        if !value.is_finite() || value < 0.0 {
            return Err(invalid(&format!("{name} must be finite and non-negative")));
        }
    }
    Ok(())
}

fn invalid(message: &str) -> TrackingError {
    TrackingError::InvalidConfiguration(message.into())
}

/// One recorded plant sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingSample {
    /// Simulation time, in seconds.
    pub time: f64,
    /// Rear-axle pose of the plant.
    pub pose: Pose2d,
    /// Plant speed, in meters per second. Negative is reverse.
    pub velocity: f64,
    /// Plant steering angle, in radians.
    pub steering: f64,
    /// Signed lateral error, in meters. Positive is to the left of the path heading.
    pub cross_track: f64,
    /// Path heading minus plant heading, in radians.
    pub heading_error: f64,
    /// `true` while the active route segment is reverse.
    pub reverse: bool,
}

/// Errors and the trajectory from one successful simulation.
///
/// [`Self::max_cross_track`] is the largest absolute lateral error.
/// [`Self::max_overshoot`] is how far the axle went past the end of the active
/// segment. The acceptance test fails if either one exceeds
/// [`TrackingTolerances::max_cross_track`].
#[derive(Clone, Debug, PartialEq)]
pub struct TrackingReport {
    /// Plant samples, including the start and the stopping pose.
    pub samples: Vec<TrackingSample>,
    /// Largest absolute lateral error, in meters.
    pub max_cross_track: f64,
    /// Largest distance past the end of the active segment, in meters.
    pub max_overshoot: f64,
    /// Largest absolute heading error, in radians.
    pub max_heading_error: f64,
    /// Root-mean-square lateral error, in meters.
    pub rms_cross_track: f64,
    /// Distance from the final rear axle to the route's last pose, in meters.
    pub goal_position_error: f64,
    /// Absolute heading error against that pose, in radians.
    pub goal_heading_error: f64,
    /// Gear changes the plant completed.
    pub direction_changes: usize,
    /// Time of the final sample, in seconds.
    pub duration: f64,
    /// Arc length the plant drove, in meters.
    pub distance_traveled: f64,
    /// Plant state at the final sample.
    pub final_state: BicycleState,
}

/// What [`follow_route`] needs besides the ticket.
///
/// `model` is the plant. Its steering angle, steering rate, speed, and
/// acceleration limits are the actuators. They can be tighter than the model
/// that planned the route; the route is not regenerated.
///
/// `footprint` is the body before clearance. The sweep grows it by `clearance`,
/// which must equal the clearance stamped on `plan`. `sweep_tolerance` is the
/// numerical half-chord bound, the same quantity the planner used, not an
/// extra safety margin.
#[derive(Clone, Copy, Debug)]
pub struct FollowRequest<'a> {
    /// Ticket to follow. A stale ticket is [`TrackingError::StaleRoute`].
    pub plan: &'a RoutePlan,
    /// Occupancy grid the ticket was checked against.
    pub map: &'a GridMap,
    /// Plant bicycle, including actuator limits.
    pub model: &'a BicycleModel,
    /// Body outline before clearance.
    pub footprint: &'a Footprint,
    /// Clearance, in meters, that has to match the ticket.
    pub clearance: f64,
    /// Half-chord sweep bound, in meters. Must be finite and positive.
    pub sweep_tolerance: f64,
    /// Tracker, speed profile, delay, noise, and gates.
    pub config: &'a FollowConfig,
}

/// Simulate `request` and return the tracking report.
///
/// The plant starts at the route's first pose, stopped, with zero steering.
/// A stale ticket returns [`TrackingError::StaleRoute`] before any integration.
/// Crossing a tolerance or sweeping the footprint into the grid returns the
/// matching error and does not report success.
pub fn follow_route(request: &FollowRequest<'_>) -> Result<TrackingReport, TrackingError> {
    request.config.validate()?;
    if !request.clearance.is_finite() || request.clearance < 0.0 {
        return Err(invalid("clearance must be finite and non-negative"));
    }
    if !request.sweep_tolerance.is_finite() || request.sweep_tolerance <= 0.0 {
        return Err(invalid("sweep_tolerance must be finite and positive"));
    }
    if !request.plan.is_current(request.map, request.clearance) {
        return Err(TrackingError::StaleRoute);
    }
    let path = request.plan.path();
    if path.poses.len() < 2 {
        return Err(invalid("route has fewer than two poses"));
    }
    let spans = direction_spans(path);
    ensure_plant_can_drive(request.model, &spans)?;
    let body = request.footprint.inflated(request.clearance);
    let mut noise = Noise::new(request.config.disturbance);
    let delay_steps = (request.config.actuator_delay / request.config.dt).round() as usize;
    let mut delayed = VecDeque::with_capacity(delay_steps + 1);
    for _ in 0..delay_steps {
        delayed.push_back(Setpoint {
            velocity: 0.0,
            steering: 0.0,
        });
    }

    let goal = path.poses[path.poses.len() - 1];
    let mut state = BicycleState::new(path.poses[0], 0.0, 0.0);
    let mut span_index = 0usize;
    let mut direction_changes = 0usize;
    let mut just_entered = false;
    let mut aligning = true;
    let mut samples = Vec::new();
    let mut max_cross = 0.0_f64;
    let mut max_overshoot = 0.0_f64;
    let mut max_heading = 0.0_f64;
    let mut traveled = 0.0_f64;
    let max_steps = (request.config.max_duration / request.config.dt).ceil() as usize;
    let mut step = 0usize;

    loop {
        let time = step as f64 * request.config.dt;
        if !finite_pose(state.pose) || !state.velocity.is_finite() || !state.steering.is_finite() {
            return Err(TrackingError::Diverged { time });
        }
        let Some(truth) = reference_on_span(
            path,
            &spans[span_index],
            state.pose,
            request.model,
            state.velocity,
            request.config,
        ) else {
            return Err(invalid("direction segment has no geometry"));
        };
        max_cross = max_cross.max(truth.cross_track.abs());
        max_overshoot = max_overshoot.max(truth.overshoot);
        max_heading = max_heading.max(truth.heading_error.abs());

        let tolerances = &request.config.tolerances;
        if truth.cross_track.abs() > tolerances.max_cross_track
            || truth.overshoot > tolerances.max_cross_track
        {
            return Err(TrackingError::TrackingExceeded {
                kind: TrackingLimitKind::CrossTrack,
                value: truth.cross_track.hypot(truth.overshoot),
                limit: tolerances.max_cross_track,
                time,
            });
        }
        if truth.heading_error.abs() > tolerances.max_heading_error {
            return Err(TrackingError::TrackingExceeded {
                kind: TrackingLimitKind::Heading,
                value: truth.heading_error.abs(),
                limit: tolerances.max_heading_error,
                time,
            });
        }

        let slow = state.velocity.abs() <= request.config.switch_speed;
        let arrived = truth.remaining <= request.config.arrive_distance && slow;
        let last = span_index + 1 == spans.len();
        if last && arrived {
            let position = state.pose.distance(goal);
            let heading = angle_difference(state.pose.theta, goal.theta).abs();
            let in_goal = position <= tolerances.goal_xy && heading <= tolerances.goal_yaw;
            let settling = heading > tolerances.goal_yaw && truth.overshoot <= tolerances.goal_xy;
            if in_goal || !settling {
                samples.push(sample(time, state, &truth, spans[span_index].reverse));
                return if in_goal {
                    Ok(assemble(
                        samples,
                        Peaks {
                            cross: max_cross,
                            overshoot: max_overshoot,
                            heading: max_heading,
                            direction_changes,
                            traveled,
                        },
                        state,
                        goal,
                    ))
                } else {
                    Err(TrackingError::GoalMiss { position, heading })
                };
            }
        } else if !last && arrived && !just_entered {
            span_index += 1;
            direction_changes += 1;
            just_entered = true;
            aligning = true;
            continue;
        }

        samples.push(sample(time, state, &truth, spans[span_index].reverse));
        if step >= max_steps {
            return Err(TrackingError::DidNotFinish { time });
        }

        let measured = noise.measure(state.pose);
        let mut setpoint = setpoint_from(
            path,
            &spans[span_index],
            measured,
            state.velocity,
            last,
            request.model,
            request.config,
        );
        if aligning {
            setpoint.velocity = 0.0;
            if (state.steering - setpoint.steering).abs() <= request.config.steer_align
                && state.velocity.abs() <= request.config.switch_speed
            {
                aligning = false;
                setpoint = setpoint_from(
                    path,
                    &spans[span_index],
                    measured,
                    state.velocity,
                    last,
                    request.model,
                    request.config,
                );
            }
        }
        // The bicycle cannot yaw while stopped. On the last segment, keep a
        // crawl until the rear axle is inside the goal box. The stop ramp
        // alone can halt on the final arc with the heading still short.
        let goal_position = state.pose.distance(goal);
        let goal_heading = angle_difference(state.pose.theta, goal.theta).abs();
        let short_of_goal = last
            && !aligning
            && goal_position <= request.config.tolerances.goal_xy * 2.0
            && truth.overshoot <= request.config.tolerances.goal_xy
            && (goal_position > request.config.tolerances.goal_xy
                || goal_heading > request.config.tolerances.goal_yaw);
        if short_of_goal && setpoint.velocity.abs() < request.config.switch_speed {
            let sign = if spans[span_index].reverse { -1.0 } else { 1.0 };
            setpoint.velocity = sign * request.config.switch_speed;
        }
        delayed.push_back(setpoint);
        let applied = delayed.pop_front().unwrap_or(setpoint);
        let input = chase(state, applied, request.model, request.config.dt, &mut noise);
        let next = request.model.step(state, input, request.config.dt);
        let distance = 0.5 * (state.velocity + next.velocity) * request.config.dt;
        let curvature = request.model.curvature_from_steering(next.steering);
        if request.map.sweep_in_collision(
            state.pose,
            curvature,
            distance,
            &body,
            request.sweep_tolerance,
        ) {
            return Err(TrackingError::Collision {
                time: time + request.config.dt,
            });
        }
        traveled += distance.abs();
        state = next;
        just_entered = false;
        step += 1;
    }
}

fn finite_pose(pose: Pose2d) -> bool {
    pose.x.is_finite() && pose.y.is_finite() && pose.theta.is_finite()
}

fn sample(time: f64, state: BicycleState, truth: &Reference, reverse: bool) -> TrackingSample {
    TrackingSample {
        time,
        pose: state.pose,
        velocity: state.velocity,
        steering: state.steering,
        cross_track: truth.cross_track,
        heading_error: truth.heading_error,
        reverse,
    }
}

struct Peaks {
    cross: f64,
    overshoot: f64,
    heading: f64,
    direction_changes: usize,
    traveled: f64,
}

fn assemble(
    samples: Vec<TrackingSample>,
    peaks: Peaks,
    state: BicycleState,
    goal: Pose2d,
) -> TrackingReport {
    let mean_square = if samples.is_empty() {
        0.0
    } else {
        samples
            .iter()
            .map(|sample| sample.cross_track * sample.cross_track)
            .sum::<f64>()
            / samples.len() as f64
    };
    TrackingReport {
        duration: samples.last().map(|sample| sample.time).unwrap_or(0.0),
        rms_cross_track: mean_square.sqrt(),
        goal_position_error: state.pose.distance(goal),
        goal_heading_error: angle_difference(state.pose.theta, goal.theta).abs(),
        samples,
        max_cross_track: peaks.cross,
        max_overshoot: peaks.overshoot,
        max_heading_error: peaks.heading,
        direction_changes: peaks.direction_changes,
        distance_traveled: peaks.traveled,
        final_state: state,
    }
}

fn ensure_plant_can_drive(model: &BicycleModel, spans: &[Span]) -> Result<(), TrackingError> {
    let params = model.params();
    if spans.iter().any(|span| !span.reverse) && params.max_speed <= 0.0 {
        return Err(invalid(
            "plant max_speed must be positive to follow a forward segment",
        ));
    }
    if spans.iter().any(|span| span.reverse) && params.min_speed >= 0.0 {
        return Err(invalid(
            "plant min_speed must be negative to follow a reverse segment",
        ));
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
struct Span {
    first: usize,
    last: usize,
    reverse: bool,
}

fn direction_spans(path: &PlannedPath) -> Vec<Span> {
    let mut spans = Vec::new();
    let mut start = 0usize;
    let mut reverse = false;
    let mut have_direction = false;
    for index in 1..path.poses.len() {
        let length = path.segment_length[index];
        if length.abs() < 1e-9 {
            continue;
        }
        let next_reverse = length < 0.0;
        if !have_direction {
            reverse = next_reverse;
            have_direction = true;
            continue;
        }
        if next_reverse != reverse {
            spans.push(Span {
                first: start,
                last: index - 1,
                reverse,
            });
            start = index - 1;
            reverse = next_reverse;
        }
    }
    spans.push(Span {
        first: start,
        last: path.poses.len() - 1,
        reverse,
    });
    spans
}

struct Reference {
    cross_track: f64,
    heading_error: f64,
    remaining: f64,
    overshoot: f64,
    curvature: f64,
    steer_gap: f64,
    steer_distance: f64,
}

fn reference_on_span(
    path: &PlannedPath,
    span: &Span,
    pose: Pose2d,
    model: &BicycleModel,
    speed: f64,
    config: &FollowConfig,
) -> Option<Reference> {
    if span.last <= span.first {
        return None;
    }
    let poses = &path.poses[span.first..=span.last];
    let projection = project_onto_path(poses, pose, 0)?;
    let path_heading = projection.point.theta;
    let dx = pose.x - projection.point.x;
    let dy = pose.y - projection.point.y;
    let cross_track = -dx * path_heading.sin() + dy * path_heading.cos();
    let change = steer_change(path, span, &projection, model);
    let curvature = if change.distance <= steer_window(change.gap, speed, model, config) {
        change.ahead
    } else {
        change.here
    };
    Some(Reference {
        cross_track,
        heading_error: angle_difference(pose.theta, path_heading),
        remaining: projection.distance_remaining,
        overshoot: overshoot(&path.poses, span, pose),
        curvature,
        steer_gap: change.gap,
        steer_distance: change.distance,
    })
}

fn steer_window(gap: f64, speed: f64, model: &BicycleModel, config: &FollowConfig) -> f64 {
    if gap <= 0.20 {
        return 0.0;
    }
    let rate = model.params().max_steering_rate.max(1e-4);
    let raw = speed.abs().max(config.switch_speed) * gap / rate;
    let farthest = config.curvature_preview.max(config.steer_slew_distance);
    raw.clamp(config.steer_slew_distance, farthest)
}

fn overshoot(poses: &[Pose2d], span: &Span, pose: Pose2d) -> f64 {
    if span.last <= span.first {
        return 0.0;
    }
    let end = poses[span.last];
    let prev = poses[span.last - 1];
    let tx = end.x - prev.x;
    let ty = end.y - prev.y;
    let norm = tx.hypot(ty);
    if norm < 1e-9 {
        return 0.0;
    }
    let past = ((pose.x - end.x) * tx + (pose.y - end.y) * ty) / norm;
    past.max(0.0)
}

struct SteerChange {
    here: f64,
    ahead: f64,
    gap: f64,
    distance: f64,
}

fn steer_change(
    path: &PlannedPath,
    span: &Span,
    projection: &PathProjection,
    model: &BicycleModel,
) -> SteerChange {
    let poses = &path.poses[span.first..=span.last];
    let here = segment_curvature(path, span, projection.segment);
    let here_steer = model.steering_from_curvature(here);
    let mut walked = 0.0;
    if projection.segment + 1 < poses.len() {
        let length = poses[projection.segment]
            .distance(poses[projection.segment + 1])
            .max(1e-9);
        walked += (1.0 - projection.fraction) * length;
    }
    let mut next = projection.segment + 1;
    while next + 1 < poses.len() && walked < 1.5 {
        let ahead = segment_curvature(path, span, next);
        let gap = (model.steering_from_curvature(ahead) - here_steer).abs();
        if gap > 0.20 {
            return SteerChange {
                here,
                ahead,
                gap,
                distance: walked,
            };
        }
        let length = poses[next].distance(poses[next + 1]).max(1e-9);
        walked += length;
        next += 1;
    }
    SteerChange {
        here,
        ahead: here,
        gap: 0.0,
        distance: f64::INFINITY,
    }
}

fn segment_curvature(path: &PlannedPath, span: &Span, local_segment: usize) -> f64 {
    let index = span.first + local_segment + 1;
    path.curvature.get(index).copied().unwrap_or(0.0)
}

#[derive(Clone, Copy, Debug)]
struct Setpoint {
    velocity: f64,
    steering: f64,
}

fn setpoint_from(
    path: &PlannedPath,
    span: &Span,
    measured: Pose2d,
    speed: f64,
    last: bool,
    model: &BicycleModel,
    config: &FollowConfig,
) -> Setpoint {
    let Some(reference) = reference_on_span(path, span, measured, model, speed, config) else {
        return Setpoint {
            velocity: 0.0,
            steering: 0.0,
        };
    };
    let limits = model.params();
    let feedforward = model.steering_from_curvature(reference.curvature);
    let heading = config.heading_gain * reference.heading_error;
    let heading = if span.reverse { -heading } else { heading };
    let cross = (config.cross_track_gain * reference.cross_track)
        .atan2(config.softening_speed + speed.abs());
    let steering = (feedforward + heading - cross)
        .clamp(-limits.max_steering_angle, limits.max_steering_angle);
    let mut velocity = speed_setpoint(
        config,
        &reference,
        span.reverse,
        last,
        limits.max_steering_rate,
    );
    velocity = velocity.clamp(limits.min_speed, limits.max_speed);
    Setpoint { velocity, steering }
}

fn speed_setpoint(
    config: &FollowConfig,
    reference: &Reference,
    reverse: bool,
    last: bool,
    max_steering_rate: f64,
) -> f64 {
    let mut speed = config.cruise_speed;
    if reference.curvature.abs() > 1e-4 {
        let radius = 1.0 / reference.curvature.abs();
        if radius < config.curvature_slow_radius {
            speed *= radius / config.curvature_slow_radius;
        }
    }
    if reference.steer_gap > 0.20 && reference.steer_distance < 1.25 {
        let slew = if max_steering_rate < 1e-4 {
            config.switch_speed
        } else {
            (max_steering_rate * config.steer_slew_distance / reference.steer_gap)
                .max(config.switch_speed)
        };
        speed = speed.min(slew);
    }
    let remaining_scale = (reference.remaining / config.stop_distance).clamp(0.0, 1.0);
    speed *= remaining_scale;
    if reference.remaining <= config.arrive_distance {
        let fix_heading = last
            && reference.heading_error.abs() > config.tolerances.goal_yaw
            && reference.overshoot <= config.tolerances.goal_xy;
        speed = if fix_heading {
            (config.cruise_speed * 0.25).clamp(config.switch_speed, 0.12)
        } else {
            0.0
        };
    }
    if reverse { -speed } else { speed }
}

fn chase(
    state: BicycleState,
    applied: Setpoint,
    model: &BicycleModel,
    dt: f64,
    noise: &mut Noise,
) -> BicycleInput {
    let params = model.params();
    let acceleration = ((applied.velocity - state.velocity) / dt)
        .clamp(-params.max_acceleration, params.max_acceleration);
    let steering_rate = ((applied.steering - state.steering) / dt)
        .clamp(-params.max_steering_rate, params.max_steering_rate);
    let (acceleration_noise, steering_noise) = noise.actuation();
    BicycleInput {
        acceleration: acceleration + acceleration_noise,
        steering_rate: steering_rate + steering_noise,
    }
}

struct Noise {
    rng: Option<StdRng>,
    spare: Option<f64>,
    disturbance: Disturbance,
}

impl Noise {
    fn new(disturbance: Disturbance) -> Self {
        Self {
            rng: disturbance
                .active()
                .then(|| StdRng::seed_from_u64(disturbance.seed)),
            spare: None,
            disturbance,
        }
    }

    fn measure(&mut self, pose: Pose2d) -> Pose2d {
        if self.rng.is_none() {
            return pose;
        }
        Pose2d::new(
            pose.x + self.gaussian() * self.disturbance.position_stddev,
            pose.y + self.gaussian() * self.disturbance.position_stddev,
            pose.theta + self.gaussian() * self.disturbance.heading_stddev,
        )
    }

    fn actuation(&mut self) -> (f64, f64) {
        if self.rng.is_none() {
            return (0.0, 0.0);
        }
        (
            self.gaussian() * self.disturbance.acceleration_stddev,
            self.gaussian() * self.disturbance.steering_rate_stddev,
        )
    }

    fn gaussian(&mut self) -> f64 {
        if let Some(spare) = self.spare.take() {
            return spare;
        }
        let Some(rng) = self.rng.as_mut() else {
            return 0.0;
        };
        let mut u1 = rng.random::<f64>();
        if u1 < 1e-12 {
            u1 = 1e-12;
        }
        let u2 = rng.random::<f64>();
        let magnitude = (-2.0 * u1.ln()).sqrt();
        let angle = std::f64::consts::TAU * u2;
        self.spare = Some(magnitude * angle.sin());
        magnitude * angle.cos()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::planning::PlannedPath;

    #[test]
    fn spans_split_where_the_gear_changes() {
        let path = PlannedPath {
            poses: vec![
                Pose2d::new(0.0, 0.0, 0.0),
                Pose2d::new(1.0, 0.0, 0.0),
                Pose2d::new(1.4, 0.0, 0.0),
                Pose2d::new(0.9, 0.0, 0.0),
            ],
            curvature: vec![0.0, 0.0, 0.0, 0.0],
            segment_length: vec![0.0, 1.0, 0.4, -0.5],
            length: 1.9,
            cost: 1.9,
            expansions: 0,
        };
        let spans = direction_spans(&path);
        assert_eq!(spans.len(), 2);
        assert!(!spans[0].reverse);
        assert!(spans[1].reverse);
        assert_eq!(spans[0].last, spans[1].first);
        assert_eq!(spans[1].last, 3);
    }

    #[test]
    fn placeholder_config_is_accepted_and_bad_numbers_are_not() {
        FollowConfig::placeholder().validate().unwrap();
        let mut config = FollowConfig::placeholder();
        config.dt = 0.0;
        assert!(matches!(
            config.validate(),
            Err(TrackingError::InvalidConfiguration(_))
        ));
        config = FollowConfig::placeholder();
        config.actuator_delay = -0.1;
        assert!(matches!(
            config.validate(),
            Err(TrackingError::InvalidConfiguration(ref message)) if message.contains("actuator_delay")
        ));
        config = FollowConfig::placeholder();
        config.disturbance.position_stddev = f64::NAN;
        assert!(matches!(
            config.validate(),
            Err(TrackingError::InvalidConfiguration(_))
        ));
        config = FollowConfig::placeholder();
        config.tolerances.max_cross_track = 0.0;
        assert!(matches!(
            config.validate(),
            Err(TrackingError::InvalidConfiguration(ref message)) if message.contains("max_cross_track")
        ));
    }
}
