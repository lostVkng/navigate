//! Cost terms for [`crate::control::Mppi`].
//!
//! A critic scores one rollout. Lower is better. The controller sums every
//! attached critic. Built-in critics cover path tracking, the terminal goal,
//! obstacle proximity, control smoothness, and constraint saturation. A custom
//! [`Critic`] can be added beside them.

use crate::geometry::{Footprint, Pose2d, angle_difference};
use crate::grid::GridMap;
use crate::motion::{BicycleInput, BicycleState, project_onto_path};

/// One noisy trajectory and the world it was scored against.
pub struct Rollout<'a> {
    /// Rear-axle states, length `horizon + 1`, including the measured state.
    pub states: &'a [BicycleState],
    /// Controls applied between the states. Length `horizon`.
    pub inputs: &'a [BicycleInput],
    /// How far each raw sample was pulled back onto the control bounds, L1.
    pub saturations: &'a [f64],
    /// Integration step in seconds.
    pub dt: f64,
    /// Occupancy, distance, and cost grid.
    pub map: &'a GridMap,
    /// Footprint used by the obstacle critic.
    pub footprint: &'a Footprint,
    /// Reference polyline. May be empty.
    pub path: &'a [Pose2d],
    /// Terminal pose.
    pub goal: Pose2d,
    /// Speed the path-tracking critic tries to hold, in meters per second.
    pub reference_speed: f64,
}

/// A term in the rollout cost.
pub trait Critic: Send + Sync {
    /// Non-negative contribution, apart from an optional progress reward.
    fn evaluate(&self, rollout: &Rollout<'_>) -> f64;
}

/// Lateral error, heading error, speed error, and a reward for progress.
#[derive(Clone, Debug)]
pub struct PathTrackingCritic {
    /// Weight on squared lateral error, per state.
    pub lateral_weight: f64,
    /// Weight on squared heading error, per state.
    pub heading_weight: f64,
    /// Weight on squared speed error against [`Rollout::reference_speed`].
    pub speed_weight: f64,
    /// Subtracted once, times the final distance traveled along the path.
    pub progress_weight: f64,
}

impl Critic for PathTrackingCritic {
    fn evaluate(&self, rollout: &Rollout<'_>) -> f64 {
        if rollout.path.len() < 2 {
            return 0.0;
        }
        let mut hint = 0usize;
        let mut cost = 0.0;
        let mut progress = 0.0;
        for state in rollout.states.iter().skip(1) {
            if let Some(projection) = project_onto_path(rollout.path, state.pose, hint) {
                hint = projection.segment;
                cost += self.lateral_weight * projection.lateral * projection.lateral;
                let heading = angle_difference(projection.point.theta, state.pose.theta);
                cost += self.heading_weight * heading * heading;
                progress = projection.distance_along;
            }
            let speed_error = state.velocity - rollout.reference_speed;
            cost += self.speed_weight * speed_error * speed_error;
        }
        cost - self.progress_weight * progress
    }
}

/// Squared position and heading error of the last state.
#[derive(Clone, Debug)]
pub struct GoalCritic {
    /// Weight on squared Euclidean distance to the goal.
    pub position_weight: f64,
    /// Weight on squared heading error.
    pub heading_weight: f64,
}

impl Critic for GoalCritic {
    fn evaluate(&self, rollout: &Rollout<'_>) -> f64 {
        let Some(last) = rollout.states.last() else {
            return 0.0;
        };
        let distance = last.pose.distance(rollout.goal);
        let heading = angle_difference(last.pose.theta, rollout.goal.theta);
        self.position_weight * distance * distance + self.heading_weight * heading * heading
    }
}

/// Grid cost along the rollout, plus a fixed penalty on footprint collision.
#[derive(Clone, Debug)]
pub struct ObstacleCritic {
    /// Multiplier on the cell cost integrated over time.
    pub weight: f64,
    /// Added once for every state whose footprint is in collision.
    pub collision_cost: f64,
}

impl Critic for ObstacleCritic {
    fn evaluate(&self, rollout: &Rollout<'_>) -> f64 {
        let mut cost = 0.0;
        for state in rollout.states.iter().skip(1) {
            if rollout.map.in_collision(state.pose, rollout.footprint) {
                // Charged per colliding state so an earlier or longer collision
                // costs more than a glance at the final step.
                cost += self.collision_cost;
            } else if let Some(cell) = rollout.map.cost_at(state.pose.x, state.pose.y) {
                cost += self.weight * f64::from(cell) * rollout.dt;
            }
        }
        cost
    }
}

/// Penalize large controls and large control changes.
#[derive(Clone, Debug)]
pub struct ControlSmoothnessCritic {
    /// Weight on squared acceleration.
    pub acceleration_weight: f64,
    /// Weight on squared steering rate.
    pub steering_rate_weight: f64,
    /// Weight on squared differences between consecutive inputs.
    pub rate_weight: f64,
}

impl Critic for ControlSmoothnessCritic {
    fn evaluate(&self, rollout: &Rollout<'_>) -> f64 {
        let mut cost = 0.0;
        for (index, input) in rollout.inputs.iter().enumerate() {
            cost += self.acceleration_weight * input.acceleration * input.acceleration;
            cost += self.steering_rate_weight * input.steering_rate * input.steering_rate;
            if index > 0 {
                let previous = rollout.inputs[index - 1];
                let da = input.acceleration - previous.acceleration;
                let dw = input.steering_rate - previous.steering_rate;
                cost += self.rate_weight * (da * da + dw * dw);
            }
        }
        cost
    }
}

/// Penalize the amount by which samples were clamped to the control limits.
#[derive(Clone, Debug)]
pub struct ConstraintCritic {
    /// Weight on the summed saturation reported by the rollout.
    pub weight: f64,
}

impl Critic for ConstraintCritic {
    fn evaluate(&self, rollout: &Rollout<'_>) -> f64 {
        self.weight * rollout.saturations.iter().sum::<f64>()
    }
}
