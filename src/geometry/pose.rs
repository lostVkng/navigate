//! Poses in the plane and the constant-curvature motion that connects them.

use super::Point;

/// Fold `angle` into `(-\pi, \pi]`.
///
/// Multiples of `2\pi` that land on `-\pi` are mapped to `+\pi`, so the result
/// has a single representative for a half-turn.
pub fn wrap_to_pi(angle: f64) -> f64 {
    if !angle.is_finite() {
        return angle;
    }
    let tau = std::f64::consts::TAU;
    let mut wrapped = angle.rem_euclid(tau);
    if wrapped > std::f64::consts::PI {
        wrapped -= tau;
    }
    wrapped
}

/// Signed smallest difference `to - from`, folded into `(-\pi, \pi]`.
pub fn angle_difference(from: f64, to: f64) -> f64 {
    wrap_to_pi(to - from)
}

/// A pose in the plane: a point and a heading.
///
/// `theta` is radians, zero along `+x`, and increases toward `+y` (a left turn).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Pose2d {
    /// World `x`.
    pub x: f64,
    /// World `y`.
    pub y: f64,
    /// Heading in radians.
    pub theta: f64,
}

impl Pose2d {
    /// Pose at `(x, y)` with heading `theta` radians.
    pub fn new(x: f64, y: f64, theta: f64) -> Self {
        Self { x, y, theta }
    }

    /// The same pose with [`wrap_to_pi`] applied to the heading.
    pub fn wrapped(self) -> Self {
        Self {
            theta: wrap_to_pi(self.theta),
            ..self
        }
    }

    /// Translation component as a [`Point`].
    pub fn point(self) -> Point {
        Point::new(self.x, self.y)
    }

    /// Euclidean distance between the translation components.
    pub fn distance(self, other: Self) -> f64 {
        (self.x - other.x).hypot(self.y - other.y)
    }

    /// Express `point` in the frame attached to this pose.
    ///
    /// The local `x` axis points along `theta`. The local `y` axis points to
    /// the left.
    pub fn inverse_transform_point(self, point: Point) -> Point {
        let dx = point.x - self.x;
        let dy = point.y - self.y;
        let (c, s) = (self.theta.cos(), self.theta.sin());
        Point::new(c * dx + s * dy, -s * dx + c * dy)
    }

    /// Move along a constant-curvature arc.
    ///
    /// `curvature` is in inverse meters, positive for a left turn.
    /// `distance` is signed arc length in meters: positive moves along the
    /// heading, negative reverses. The integration is exact for a constant
    /// curvature:
    ///
    /// ```text
    /// Δθ = κ Δs
    /// Δx = (sin(θ + Δθ) - sin θ) / κ
    /// Δy = (cos θ - cos(θ + Δθ)) / κ
    /// ```
    ///
    /// A curvature smaller than `1e-12` is treated as a straight line. The
    /// returned heading is folded into `(-\pi, \pi]`. Non-finite inputs return
    /// `self`.
    pub fn advance(self, curvature: f64, distance: f64) -> Self {
        if !curvature.is_finite() || !distance.is_finite() {
            return self;
        }
        if curvature.abs() < 1e-12 {
            return Self {
                x: self.x + distance * self.theta.cos(),
                y: self.y + distance * self.theta.sin(),
                theta: wrap_to_pi(self.theta),
            };
        }
        let dtheta = distance * curvature;
        Self {
            x: self.x + ((self.theta + dtheta).sin() - self.theta.sin()) / curvature,
            y: self.y + (self.theta.cos() - (self.theta + dtheta).cos()) / curvature,
            theta: wrap_to_pi(self.theta + dtheta),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn wrap_is_principal_value() {
        let pi = std::f64::consts::PI;
        assert!((wrap_to_pi(0.0)).abs() < 1e-12);
        assert!((wrap_to_pi(3.0 * pi) - pi).abs() < 1e-12);
        assert!(
            (wrap_to_pi(-3.0 * pi) - pi).abs() < 1e-9 || (wrap_to_pi(-3.0 * pi) + pi).abs() < 1e-9
        );
        assert!((angle_difference(0.1, 0.1)).abs() < 1e-12);
        assert!(angle_difference(0.0, 2.0 * pi).abs() < 1e-9);
    }

    #[test]
    fn straight_advance_matches_heading() {
        let start = Pose2d::new(1.0, 2.0, 0.0);
        let end = start.advance(0.0, 3.0);
        assert!((end.x - 4.0).abs() < 1e-12);
        assert!((end.y - 2.0).abs() < 1e-12);
    }

    #[test]
    fn left_turn_follows_the_circle() {
        // κ = 1, quarter turn forward: from heading 0 the pose should land at (1, 1) with heading π/2
        // relative to a circle centered at (0, 1) if we start at the origin...
        // start (0, 0, 0), κ = 1, distance = π/2.
        // center of rotation is to the left at (0, 1). After a quarter turn we are at (1, 1), heading π/2.
        let end = Pose2d::new(0.0, 0.0, 0.0).advance(1.0, std::f64::consts::FRAC_PI_2);
        assert!((end.x - 1.0).abs() < 1e-9, "x={}", end.x);
        assert!((end.y - 1.0).abs() < 1e-9, "y={}", end.y);
        assert!((end.theta - std::f64::consts::FRAC_PI_2).abs() < 1e-9);
    }
}
