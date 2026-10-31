//! Kinematic bicycle (Ackermann) model.
//!
//! The reference point is the center of the rear axle. With wheelbase `L`,
//! speed `v`, and front-wheel steering angle `δ` the continuous model is
//!
//! ```text
//! ẋ = v cos θ
//! ẏ = v sin θ
//! θ̇ = v tan(δ) / L
//! v̇ = a
//! δ̇ = ω
//! ```
//!
//! [`BicycleModel::step`] integrates one control interval by applying the
//! acceleration and steering rate first (clamped to the configured limits),
//! then moving the pose on the constant-curvature arc of the updated steering
//! angle at the average of the old and new speeds. That arc is exact for a
//! steering angle held over the interval; the speed average is the trapezoid
//! of a constant acceleration.

use crate::errors::PathPlannerError;
use crate::geometry::Pose2d;

/// Limits of a car-like vehicle.
///
/// Angles are radians. Speeds are meters per second. Acceleration is meters
/// per second squared. Steering rate is radians per second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BicycleParams {
    /// Distance from the rear axle to the front axle. Must be positive.
    pub wheelbase: f64,
    /// Largest |steering angle|. Must lie in `(0, π/2)`.
    pub max_steering_angle: f64,
    /// Largest |steering rate|. Must be non-negative.
    pub max_steering_rate: f64,
    /// Most negative allowed speed. Negative values allow reversing.
    pub min_speed: f64,
    /// Largest forward speed. Must be at least `min_speed`.
    pub max_speed: f64,
    /// Largest |longitudinal acceleration|.
    pub max_acceleration: f64,
}

impl BicycleParams {
    /// Reject non-finite or geometrically impossible parameters.
    pub fn validate(self) -> Result<(), PathPlannerError> {
        if !self.wheelbase.is_finite() || self.wheelbase <= 0.0 {
            return Err(invalid("wheelbase must be finite and positive"));
        }
        if !self.max_steering_angle.is_finite()
            || self.max_steering_angle <= 0.0
            || self.max_steering_angle >= std::f64::consts::FRAC_PI_2
        {
            return Err(invalid(
                "max steering angle must lie strictly between 0 and π/2",
            ));
        }
        if !self.max_steering_rate.is_finite() || self.max_steering_rate < 0.0 {
            return Err(invalid("max steering rate must be finite and non-negative"));
        }
        if !self.min_speed.is_finite()
            || !self.max_speed.is_finite()
            || self.min_speed > self.max_speed
        {
            return Err(invalid(
                "speed limits must be finite and min_speed ≤ max_speed",
            ));
        }
        if !self.max_acceleration.is_finite() || self.max_acceleration < 0.0 {
            return Err(invalid("max acceleration must be finite and non-negative"));
        }
        Ok(())
    }
}

fn invalid(message: &str) -> PathPlannerError {
    PathPlannerError::InvalidConfiguration(message.into())
}

/// Rear-axle state of the bicycle.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BicycleState {
    /// Pose of the rear axle.
    pub pose: Pose2d,
    /// Longitudinal speed in meters per second. Negative is reverse.
    pub velocity: f64,
    /// Front steering angle in radians. Positive steers left.
    pub steering: f64,
}

impl BicycleState {
    /// State at `pose` with the given speed and steering angle.
    pub fn new(pose: Pose2d, velocity: f64, steering: f64) -> Self {
        Self {
            pose,
            velocity,
            steering,
        }
    }
}

/// Command applied over one control interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BicycleInput {
    /// Longitudinal acceleration in meters per second squared.
    pub acceleration: f64,
    /// Steering-angle rate in radians per second.
    pub steering_rate: f64,
}

impl BicycleInput {
    /// Zero acceleration and zero steering rate.
    pub fn zero() -> Self {
        Self {
            acceleration: 0.0,
            steering_rate: 0.0,
        }
    }
}

/// Validated bicycle model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BicycleModel {
    params: BicycleParams,
}

impl BicycleModel {
    /// Validate `params` and keep them.
    pub fn new(params: BicycleParams) -> Result<Self, PathPlannerError> {
        params.validate()?;
        Ok(Self { params })
    }

    /// Parameters the model was built with.
    pub fn params(&self) -> BicycleParams {
        self.params
    }

    /// `tan(δ_max) / L`, the tightest curvature the steering limit allows.
    pub fn max_curvature(&self) -> f64 {
        self.params.max_steering_angle.tan() / self.params.wheelbase
    }

    /// `L / tan(δ_max)`, the minimum turning radius.
    pub fn min_turning_radius(&self) -> f64 {
        self.params.wheelbase / self.params.max_steering_angle.tan()
    }

    /// Curvature `tan(δ) / L` for a steering angle. Non-finite angles yield `0`.
    pub fn curvature_from_steering(&self, steering: f64) -> f64 {
        if !steering.is_finite() {
            return 0.0;
        }
        steering.tan() / self.params.wheelbase
    }

    /// Steering angle `atan(κ L)` for a curvature, clamped to the steering limit.
    pub fn steering_from_curvature(&self, curvature: f64) -> f64 {
        if !curvature.is_finite() {
            return 0.0;
        }
        let angle = (curvature * self.params.wheelbase).atan();
        angle.clamp(
            -self.params.max_steering_angle,
            self.params.max_steering_angle,
        )
    }

    /// Integrate `input` over `dt` seconds.
    ///
    /// Non-positive or non-finite `dt` returns `state` unchanged. Non-finite
    /// commands are treated as zero. The acceleration, steering rate, resulting
    /// speed, and resulting steering angle are all clamped to the model limits,
    /// so a caller cannot drive the state outside them.
    pub fn step(&self, state: BicycleState, input: BicycleInput, dt: f64) -> BicycleState {
        if !dt.is_finite() || dt <= 0.0 {
            return state;
        }
        let accel = finite_or_zero(input.acceleration)
            .clamp(-self.params.max_acceleration, self.params.max_acceleration);
        let steer_rate = finite_or_zero(input.steering_rate).clamp(
            -self.params.max_steering_rate,
            self.params.max_steering_rate,
        );
        let velocity =
            (state.velocity + accel * dt).clamp(self.params.min_speed, self.params.max_speed);
        let steering = (state.steering + steer_rate * dt).clamp(
            -self.params.max_steering_angle,
            self.params.max_steering_angle,
        );
        let average_speed = 0.5 * (state.velocity + velocity);
        let curvature = self.curvature_from_steering(steering);
        BicycleState {
            pose: state.pose.advance(curvature, average_speed * dt),
            velocity,
            steering,
        }
    }

    /// Drive a signed distance at a constant steering angle.
    ///
    /// The steering angle is clamped. This is the geometric primitive used by
    /// the global planner; it does not consult the acceleration limit.
    pub fn propagate(&self, pose: Pose2d, steering: f64, distance: f64) -> Pose2d {
        let steering = finite_or_zero(steering).clamp(
            -self.params.max_steering_angle,
            self.params.max_steering_angle,
        );
        pose.advance(self.curvature_from_steering(steering), distance)
    }
}

fn finite_or_zero(value: f64) -> f64 {
    if value.is_finite() { value } else { 0.0 }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn model() -> BicycleModel {
        BicycleModel::new(BicycleParams {
            wheelbase: 2.0,
            max_steering_angle: 0.5,
            max_steering_rate: 1.0,
            min_speed: -1.0,
            max_speed: 2.0,
            max_acceleration: 1.5,
        })
        .unwrap()
    }

    #[test]
    fn straight_line_integrates_speed() {
        let model = model();
        let start = BicycleState::new(Pose2d::new(0.0, 0.0, 0.0), 1.0, 0.0);
        let end = model.step(start, BicycleInput::zero(), 0.5);
        assert!((end.pose.x - 0.5).abs() < 1e-9);
        assert!(end.pose.y.abs() < 1e-9);
        assert!((end.velocity - 1.0).abs() < 1e-12);
    }

    #[test]
    fn constant_steer_matches_the_curvature_definition() {
        let model = model();
        let steering = 0.3;
        let speed = 1.0;
        let dt = 0.4;
        let start = BicycleState::new(Pose2d::new(0.0, 0.0, 0.0), speed, steering);
        let end = model.step(start, BicycleInput::zero(), dt);
        let kappa = steering.tan() / model.params().wheelbase;
        let expected_yaw = kappa * speed * dt;
        assert!(
            (end.pose.theta - expected_yaw).abs() < 1e-9,
            "{}",
            end.pose.theta
        );
        assert!((end.steering - steering).abs() < 1e-12);
    }

    #[test]
    fn limits_are_enforced() {
        let model = model();
        let start = BicycleState::new(Pose2d::new(0.0, 0.0, 0.0), 2.0, 0.5);
        let end = model.step(
            start,
            BicycleInput {
                acceleration: 100.0,
                steering_rate: 100.0,
            },
            1.0,
        );
        assert!(end.velocity <= model.params().max_speed + 1e-12);
        assert!(end.steering.abs() <= model.params().max_steering_angle + 1e-12);
    }

    #[test]
    fn invalid_wheelbase_is_rejected() {
        let result = BicycleModel::new(BicycleParams {
            wheelbase: 0.0,
            max_steering_angle: 0.4,
            max_steering_rate: 1.0,
            min_speed: 0.0,
            max_speed: 1.0,
            max_acceleration: 1.0,
        });
        assert!(matches!(
            result,
            Err(PathPlannerError::InvalidConfiguration(_))
        ));
    }

    #[test]
    fn turning_radius_matches_the_steering_limit() {
        let model = model();
        let radius = model.min_turning_radius();
        assert!((model.max_curvature() * radius - 1.0).abs() < 1e-12);
        assert!(radius > model.params().wheelbase);
    }
}
