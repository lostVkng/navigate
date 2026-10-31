//! Actuator command shared by the path trackers.

/// What a tracker asks the vehicle to do over the next control interval.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VehicleCommand {
    /// Longitudinal speed in meters per second. Negative is reverse.
    pub velocity: f64,
    /// Front steering angle in radians. Positive steers left.
    pub steering_angle: f64,
    /// Curvature `tan(steering_angle) / wheelbase`, in inverse meters.
    pub curvature: f64,
    /// Yaw rate `velocity * curvature`, in radians per second.
    pub angular_velocity: f64,
}

impl VehicleCommand {
    /// A command that leaves the vehicle stopped and the steering at zero.
    pub fn stopped() -> Self {
        Self {
            velocity: 0.0,
            steering_angle: 0.0,
            curvature: 0.0,
            angular_velocity: 0.0,
        }
    }
}
