//! Shared representation of Dubins and Reeds–Shepp paths.

use crate::geometry::Pose2d;

/// One circular arc or straight segment of a bounded-curvature path.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CurveSegment {
    /// Curvature in inverse meters. Positive is a left turn, negative a right
    /// turn, and zero a straight segment.
    pub curvature: f64,
    /// Signed arc length in meters. Negative lengths are driven in reverse.
    pub length: f64,
}

/// A concatenation of [`CurveSegment`]s.
///
/// The path is feasible for any vehicle whose maximum curvature is at least
/// the largest absolute curvature of its segments and, when every length is
/// non-negative, that can only drive forward.
#[derive(Clone, Debug, PartialEq)]
pub struct CurvePath {
    /// Segments in the order they are driven.
    pub segments: Vec<CurveSegment>,
    /// Sum of the absolute segment lengths, in meters.
    pub length: f64,
    /// Turning radius the segments were generated for, in meters.
    pub radius: f64,
}

impl CurvePath {
    pub(crate) fn from_segments(segments: Vec<CurveSegment>, radius: f64) -> Self {
        let length = segments.iter().map(|segment| segment.length.abs()).sum();
        Self {
            segments,
            length,
            radius,
        }
    }

    /// Pose reached by driving every segment from `start`.
    pub fn endpoint(&self, start: Pose2d) -> Pose2d {
        self.segments.iter().fold(start, |pose, segment| {
            pose.advance(segment.curvature, segment.length)
        })
    }

    /// Sample the path, including `start` and the endpoint of every piece.
    ///
    /// `spacing` is the nominal arc-length step in meters. Non-positive or
    /// non-finite spacing falls back to `0.25` m. Each segment is split into
    /// equal pieces no longer than `spacing`.
    pub fn sample(&self, start: Pose2d, spacing: f64) -> Vec<Pose2d> {
        let step = if spacing.is_finite() && spacing > 0.0 {
            spacing
        } else {
            0.25
        };
        let mut poses = vec![start];
        let mut pose = start;
        for segment in &self.segments {
            if segment.length.abs() < 1e-12 {
                continue;
            }
            let pieces = (segment.length.abs() / step).ceil().max(1.0) as usize;
            let piece = segment.length / pieces as f64;
            for _ in 0..pieces {
                pose = pose.advance(segment.curvature, piece);
                poses.push(pose);
            }
        }
        poses
    }
}

pub(crate) fn empty_path(radius: f64) -> CurvePath {
    CurvePath {
        segments: Vec::new(),
        length: 0.0,
        radius,
    }
}

/// Build segments from unit-radius steer/length pairs.
///
/// `steer` is `+1` left, `-1` right, `0` straight. `unit_length` is in units
/// of the turning radius and may be negative.
pub(crate) fn segments_from_units(steer: &[i8], unit_length: &[f64], radius: f64) -> CurvePath {
    let mut segments = Vec::with_capacity(steer.len());
    for (&steer, &unit) in steer.iter().zip(unit_length) {
        if !unit.is_finite() || unit.abs() < 1e-12 {
            continue;
        }
        let curvature = if steer == 0 {
            0.0
        } else {
            f64::from(steer) / radius
        };
        segments.push(CurveSegment {
            curvature,
            length: unit * radius,
        });
    }
    CurvePath::from_segments(segments, radius)
}
