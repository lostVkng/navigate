//! Robot footprints for collision checks against a grid.
//!
//! A footprint is expressed in the vehicle frame. The pose origin is the
//! reference point the motion model integrates, typically the center of the
//! rear axle for a bicycle model. [`Footprint::OrientedRectangle`] therefore
//! takes a rear overhang so the rectangle can extend behind that point.

use super::{Point, Pose2d};

/// Planar shape attached to a [`Pose2d`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Footprint {
    /// Disk centered on the pose origin.
    Circle {
        /// Radius in meters.
        radius: f64,
    },
    /// Rectangle aligned with the pose heading.
    ///
    /// `length` runs along the heading and `width` is lateral. The pose origin
    /// sits `rear_overhang` meters behind the rear edge, so the front edge is
    /// `length - rear_overhang` meters ahead of the origin. The origin is
    /// expected to lie inside the rectangle (`0 ≤ rear_overhang ≤ length`).
    OrientedRectangle {
        /// Total length along the heading, in meters.
        length: f64,
        /// Total width, in meters.
        width: f64,
        /// Distance from the pose origin back to the rear edge, in meters.
        rear_overhang: f64,
    },
}

impl Footprint {
    /// Largest distance from the pose origin to a point of the footprint.
    pub fn circumradius(self) -> f64 {
        match self {
            Footprint::Circle { radius } => radius.max(0.0),
            Footprint::OrientedRectangle {
                length,
                width,
                rear_overhang,
            } => {
                let front = (length - rear_overhang).max(0.0);
                let rear = rear_overhang.max(0.0);
                let half_w = width.abs() * 0.5;
                front.hypot(half_w).max(rear.hypot(half_w))
            }
        }
    }

    /// Radius of the largest disk centered on the pose origin and contained in
    /// the footprint. Zero when the origin is outside the shape.
    pub fn inscribed_radius(self) -> f64 {
        match self {
            Footprint::Circle { radius } => radius.max(0.0),
            Footprint::OrientedRectangle {
                length,
                width,
                rear_overhang,
            } => {
                let front = length - rear_overhang;
                if front < 0.0 || rear_overhang < 0.0 {
                    return 0.0;
                }
                front.min(rear_overhang).min(width.abs() * 0.5).max(0.0)
            }
        }
    }

    /// World-frame corners of a rectangular footprint, in boundary order.
    ///
    /// A circle has no corners and returns an empty array.
    pub fn rectangle_corners(self, pose: Pose2d) -> Option<[Point; 4]> {
        let Footprint::OrientedRectangle {
            length,
            width,
            rear_overhang,
        } = self
        else {
            return None;
        };
        let front = length - rear_overhang;
        let half_w = width * 0.5;
        let local = [
            (-rear_overhang, -half_w),
            (front, -half_w),
            (front, half_w),
            (-rear_overhang, half_w),
        ];
        let (c, s) = (pose.theta.cos(), pose.theta.sin());
        let mut corners = [Point::new(0.0, 0.0); 4];
        for (i, (lx, ly)) in local.into_iter().enumerate() {
            corners[i] = Point::new(pose.x + c * lx - s * ly, pose.y + s * lx + c * ly);
        }
        Some(corners)
    }

    /// Axis-aligned bounds of the footprint in the world frame.
    ///
    /// Returns `None` when the pose or the shape is non-finite. A circle uses
    /// its radius. A rectangle uses its four corners, which is exact because
    /// the shape is their convex hull.
    pub fn world_aabb(self, pose: Pose2d) -> Option<(f64, f64, f64, f64)> {
        if !pose.x.is_finite() || !pose.y.is_finite() || !pose.theta.is_finite() {
            return None;
        }
        match self {
            Footprint::Circle { radius } => {
                if !radius.is_finite() {
                    return None;
                }
                let radius = radius.max(0.0);
                Some((
                    pose.x - radius,
                    pose.y - radius,
                    pose.x + radius,
                    pose.y + radius,
                ))
            }
            Footprint::OrientedRectangle {
                length,
                width,
                rear_overhang,
            } => {
                if !length.is_finite() || !width.is_finite() || !rear_overhang.is_finite() {
                    return None;
                }
                let corners = self.rectangle_corners(pose)?;
                let mut min_x = f64::INFINITY;
                let mut max_x = f64::NEG_INFINITY;
                let mut min_y = f64::INFINITY;
                let mut max_y = f64::NEG_INFINITY;
                for corner in corners {
                    min_x = min_x.min(corner.x);
                    max_x = max_x.max(corner.x);
                    min_y = min_y.min(corner.y);
                    max_y = max_y.max(corner.y);
                }
                Some((min_x, min_y, max_x, max_y))
            }
        }
    }

    /// Footprint enlarged by `margin` meters on every side.
    ///
    /// The rear axle stays at the same body point. The enlarged rectangle
    /// contains the Minkowski sum of the original rectangle with a disk of
    /// radius `margin`. Its square corners extend past that disk by
    /// `margin * (√2 - 1)`, so a collision test against the enlarged rectangle
    /// is conservative. A non-finite or negative margin leaves the shape
    /// unchanged.
    pub fn inflated(self, margin: f64) -> Self {
        if !margin.is_finite() || margin <= 0.0 {
            return self;
        }
        match self {
            Footprint::Circle { radius } => Footprint::Circle {
                radius: radius.max(0.0) + margin,
            },
            Footprint::OrientedRectangle {
                length,
                width,
                rear_overhang,
            } => Footprint::OrientedRectangle {
                length: length + 2.0 * margin,
                width: width + 2.0 * margin,
                rear_overhang: rear_overhang + margin,
            },
        }
    }

    /// Greatest distance from the instantaneous center of rotation to a point
    /// of the footprint.
    ///
    /// `curvature` uses the same sign as [`Pose2d::advance`](super::Pose2d::advance):
    /// positive curves left, so the center lies on the local `+y` axis at
    /// `1 / curvature`. A near-zero curvature is a translation and returns
    /// [`f64::INFINITY`], because the center is at infinity and every body
    /// point travels the same distance.
    pub fn radius_about_icr(self, curvature: f64) -> f64 {
        if !curvature.is_finite() || curvature.abs() < 1e-12 {
            return f64::INFINITY;
        }
        let icr_y = 1.0 / curvature;
        match self {
            Footprint::Circle { radius } => icr_y.abs() + radius.max(0.0),
            Footprint::OrientedRectangle {
                length,
                width,
                rear_overhang,
            } => {
                let front = length - rear_overhang;
                let half_w = width.abs() * 0.5;
                let corners = [
                    (-rear_overhang, -half_w),
                    (front, -half_w),
                    (front, half_w),
                    (-rear_overhang, half_w),
                ];
                corners
                    .into_iter()
                    .map(|(x, y)| x.hypot(y - icr_y))
                    .fold(0.0_f64, f64::max)
            }
        }
    }
}

/// How far a footprint point can be from the nearer endpoint of a short arc.
///
/// A body point at distance `r` from the instantaneous center, moving through
/// an angle `|κ Δs|`, is at most `2 r sin(|Δθ| / 4)` from the nearer of the
/// two endpoint positions. The midpoint of the arc is the worst sample. On a
/// straight segment every point translates by `|Δs|`, so the bound is
/// `|Δs| / 2`.
///
/// The bound is the input to a swept collision test: if both endpoint
/// footprints, enlarged by this distance, are clear, every intermediate
/// placement of the rigid body is clear as well.
pub fn sweep_half_chord(footprint: Footprint, curvature: f64, step: f64) -> f64 {
    let step = step.abs();
    if !step.is_finite() || step == 0.0 {
        return 0.0;
    }
    if !curvature.is_finite() || curvature.abs() < 1e-12 {
        return step * 0.5;
    }
    let radius = footprint.radius_about_icr(curvature);
    if !radius.is_finite() {
        return step * 0.5;
    }
    let turn = (curvature.abs() * step).min(std::f64::consts::TAU);
    2.0 * radius * (turn * 0.25).sin()
}

/// Longest rear-axle step whose [`sweep_half_chord`] is at most `tolerance`.
///
/// The step is also capped at a quarter turn so a loose tolerance cannot skip
/// an entire loop. Invalid tolerances return `0`.
pub fn sweep_step_length(footprint: Footprint, curvature: f64, tolerance: f64) -> f64 {
    if !tolerance.is_finite() || tolerance <= 0.0 {
        return 0.0;
    }
    if !curvature.is_finite() || curvature.abs() < 1e-12 {
        return (tolerance * 2.0).max(1e-6);
    }
    let radius = footprint.radius_about_icr(curvature).max(1e-9);
    let ratio = (tolerance / (2.0 * radius)).clamp(0.0, 1.0);
    let turn = (ratio.asin() * 4.0).clamp(1e-6, std::f64::consts::FRAC_PI_2);
    turn / curvature.abs()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::Pose2d;

    fn body() -> Footprint {
        Footprint::OrientedRectangle {
            length: 1.0,
            width: 0.4,
            rear_overhang: 0.2,
        }
    }

    #[test]
    fn straight_half_chord_is_half_the_step() {
        let gap = sweep_half_chord(Footprint::Circle { radius: 0.4 }, 0.0, -1.2);
        assert!((gap - 0.6).abs() < 1e-12);
    }

    #[test]
    fn half_chord_covers_intermediate_corners() {
        let body = body();
        let curvature = 0.8;
        let step = 0.5;
        let bound = sweep_half_chord(body, curvature, step);
        let start = Pose2d::new(0.0, 0.0, 0.3);
        let end = start.advance(curvature, step);
        let start_corners = body.rectangle_corners(start).unwrap();
        let end_corners = body.rectangle_corners(end).unwrap();
        for fraction in [0.25, 0.5, 0.75] {
            let mid = start.advance(curvature, step * fraction);
            let corners = body.rectangle_corners(mid).unwrap();
            for i in 0..4 {
                let nearest = corners[i]
                    .distance(start_corners[i])
                    .min(corners[i].distance(end_corners[i]));
                assert!(
                    nearest <= bound + 1e-9,
                    "corner {i} at {fraction} is {nearest} from both ends, bound {bound}"
                );
            }
        }
    }

    #[test]
    fn step_length_respects_the_tolerance() {
        let body = body();
        for curvature in [0.0, 0.4, 1.5, -2.0] {
            let step = sweep_step_length(body, curvature, 0.02);
            let gap = sweep_half_chord(body, curvature, step);
            assert!(
                gap <= 0.02 + 1e-9,
                "curvature {curvature}: gap {gap} at step {step}"
            );
        }
    }

    #[test]
    fn inflation_keeps_the_rear_axle_and_grows_every_side() {
        let grown = body().inflated(0.1);
        let Footprint::OrientedRectangle {
            length,
            width,
            rear_overhang,
        } = grown
        else {
            panic!("shape changed");
        };
        assert!((length - 1.2).abs() < 1e-12);
        assert!((width - 0.6).abs() < 1e-12);
        assert!((rear_overhang - 0.3).abs() < 1e-12);
        let front = length - rear_overhang;
        assert!((front - 0.9).abs() < 1e-12);
    }
}
