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
}
