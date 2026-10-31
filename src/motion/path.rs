//! Queries against a polyline of poses.
//!
//! Controllers treat a planned path as an ordered polyline. Projection is a
//! linear scan, which is the right cost for the few hundred samples a local
//! planner produces. A `hint` index lets a rollout resume near the previous
//! sample instead of starting from the first vertex.

use crate::geometry::{Pose2d, angle_difference};

/// Where a query point sits relative to a polyline.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PathProjection {
    /// Index of the segment's first vertex. The segment runs to `segment + 1`.
    pub segment: usize,
    /// Interpolation parameter along that segment, in `[0, 1]`.
    pub fraction: f64,
    /// Closest point on the polyline.
    pub point: Pose2d,
    /// Arc length from the start of the polyline to `point`.
    pub distance_along: f64,
    /// Arc length from `point` to the end of the polyline.
    pub distance_remaining: f64,
    /// Signed lateral offset of the query from the path tangent, in meters.
    /// Positive means the query is to the left of the direction of travel.
    pub lateral: f64,
    /// `true` when this segment's displacement points opposite its stored heading.
    pub reverse: bool,
}

/// Project `pose` onto `path`.
///
/// `hint` is a segment index to search first; the scan still covers the whole
/// path, but it starts at `hint` so a forward-moving query usually stops
/// nearby. An empty path or a path of one pose returns `None`.
pub fn project_onto_path(path: &[Pose2d], pose: Pose2d, hint: usize) -> Option<PathProjection> {
    if path.len() < 2 {
        return None;
    }
    let lengths = segment_lengths(path);
    let total: f64 = lengths.iter().sum();
    let mut best: Option<(f64, PathProjection)> = None;
    let start = hint.min(path.len() - 2);
    for offset in 0..path.len() - 1 {
        let segment = (start + offset) % (path.len() - 1);
        let projection = project_segment(path, &lengths, segment, pose, total);
        let score = (pose.x - projection.point.x).hypot(pose.y - projection.point.y);
        if best
            .as_ref()
            .map(|(best_score, _)| score < *best_score)
            .unwrap_or(true)
        {
            best = Some((score, projection));
        }
    }
    best.map(|(_, projection)| projection)
}

fn segment_lengths(path: &[Pose2d]) -> Vec<f64> {
    path.windows(2)
        .map(|pair| pair[0].distance(pair[1]))
        .collect()
}

fn project_segment(
    path: &[Pose2d],
    lengths: &[f64],
    segment: usize,
    pose: Pose2d,
    total: f64,
) -> PathProjection {
    let a = path[segment];
    let b = path[segment + 1];
    let abx = b.x - a.x;
    let aby = b.y - a.y;
    let length = lengths[segment];
    let fraction = if length < 1e-12 {
        0.0
    } else {
        (((pose.x - a.x) * abx + (pose.y - a.y) * aby) / (length * length)).clamp(0.0, 1.0)
    };
    let point = Pose2d::new(
        a.x + fraction * abx,
        a.y + fraction * aby,
        a.theta + fraction * angle_difference(a.theta, b.theta),
    );
    let along_before: f64 = lengths[..segment].iter().sum();
    let distance_along = along_before + fraction * length;
    let tangent = if length < 1e-12 {
        a.theta
    } else {
        aby.atan2(abx)
    };
    let dx = pose.x - point.x;
    let dy = pose.y - point.y;
    let lateral = -dx * tangent.sin() + dy * tangent.cos();
    let reverse =
        length >= 1e-12 && angle_difference(tangent, a.theta).abs() > std::f64::consts::FRAC_PI_2;
    PathProjection {
        segment,
        fraction,
        point,
        distance_along,
        distance_remaining: (total - distance_along).max(0.0),
        lateral,
        reverse,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projection_lands_on_a_straight_segment() {
        let path = [
            Pose2d::new(0.0, 0.0, 0.0),
            Pose2d::new(2.0, 0.0, 0.0),
            Pose2d::new(4.0, 0.0, 0.0),
        ];
        let projection = project_onto_path(&path, Pose2d::new(1.0, 0.5, 0.0), 0).unwrap();
        assert_eq!(projection.segment, 0);
        assert!((projection.point.x - 1.0).abs() < 1e-9);
        assert!(projection.point.y.abs() < 1e-9);
        assert!((projection.lateral - 0.5).abs() < 1e-9);
        assert!(!projection.reverse);
        assert!((projection.distance_remaining - 3.0).abs() < 1e-9);
    }
}
