//! Dubins paths: shortest forward curves of bounded curvature.
//!
//! L. E. Dubins, "On curves of minimal length with a constraint on average
//! curvature, and with prescribed initial and terminal positions and tangents"
//! (*American Journal of Mathematics*, 1957) showed that a shortest forward
//! path is one of six words: LSL, RSR, LSR, RSL, RLR, LRL. `L` and `R` are
//! arcs of the minimum turning radius and `S` is a straight segment. All six
//! are evaluated and the shortest one that reaches the goal is returned.
//!
//! Lengths below are solved in units of the turning radius, in the normalized
//! frame used by the standard construction: `d` is the distance between the
//! poses divided by the radius, `α` is the start heading relative to the line
//! that joins them, and `β` is the goal heading relative to that same line.

use crate::geometry::{Pose2d, angle_difference};

use super::curves::{CurvePath, empty_path, segments_from_units};

/// Shortest forward Dubins path from `start` to `goal` at turning radius `radius`.
///
/// Returns `None` when `radius` is not finite and positive or when no candidate
/// reaches the goal (the six families cover every pose pair with a positive
/// radius; `None` here means a numerical rejection). A coincident pose pair
/// returns an empty path of length zero.
pub fn dubins_path(start: Pose2d, goal: Pose2d, radius: f64) -> Option<CurvePath> {
    if !radius.is_finite() || radius <= 0.0 {
        return None;
    }
    if same_pose(start, goal, radius) {
        return Some(empty_path(radius));
    }
    let (distance, alpha, beta) = normalized(start, goal, radius);
    type Family = (fn(f64, f64, f64) -> Option<[f64; 3]>, [i8; 3]);
    let families: [Family; 6] = [
        (lsl, [1, 0, 1]),
        (rsr, [-1, 0, -1]),
        (lsr, [1, 0, -1]),
        (rsl, [-1, 0, 1]),
        (rlr, [-1, 1, -1]),
        (lrl, [1, -1, 1]),
    ];
    let mut best: Option<CurvePath> = None;
    for (formula, steer) in families {
        let Some(lengths) = formula(alpha, beta, distance) else {
            continue;
        };
        let path = segments_from_units(&steer, &lengths, radius);
        if !reaches(start, goal, &path, radius) {
            continue;
        }
        if best
            .as_ref()
            .map(|current| path.length < current.length)
            .unwrap_or(true)
        {
            best = Some(path);
        }
    }
    best
}

/// Length of [`dubins_path`], if one exists.
pub fn dubins_length(start: Pose2d, goal: Pose2d, radius: f64) -> Option<f64> {
    dubins_path(start, goal, radius).map(|path| path.length)
}

fn same_pose(start: Pose2d, goal: Pose2d, radius: f64) -> bool {
    start.distance(goal) <= 1e-8 * radius.max(1.0)
        && angle_difference(start.theta, goal.theta).abs() <= 1e-8
}

fn reaches(start: Pose2d, goal: Pose2d, path: &CurvePath, radius: f64) -> bool {
    let end = path.endpoint(start);
    let scale = radius.max(1.0);
    end.distance(goal) <= 1e-5 * scale && angle_difference(end.theta, goal.theta).abs() <= 1e-5
}

fn normalized(start: Pose2d, goal: Pose2d, radius: f64) -> (f64, f64, f64) {
    let dx = goal.x - start.x;
    let dy = goal.y - start.y;
    let distance = dx.hypot(dy) / radius;
    let theta = dy.atan2(dx);
    let alpha = (start.theta - theta).rem_euclid(std::f64::consts::TAU);
    let beta = (goal.theta - theta).rem_euclid(std::f64::consts::TAU);
    (distance, alpha, beta)
}

fn lsl(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    let c_ab = (alpha - beta).cos();
    let tmp0 = d + sa - sb;
    let p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sa - sb);
    if p_sq < 0.0 {
        return None;
    }
    let tmp1 = (cb - ca).atan2(tmp0);
    Some([
        (-alpha + tmp1).rem_euclid(std::f64::consts::TAU),
        p_sq.sqrt(),
        (beta - tmp1).rem_euclid(std::f64::consts::TAU),
    ])
}

fn rsr(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    let c_ab = (alpha - beta).cos();
    let tmp0 = d - sa + sb;
    let p_sq = 2.0 + d * d - 2.0 * c_ab + 2.0 * d * (sb - sa);
    if p_sq < 0.0 {
        return None;
    }
    let tmp1 = (ca - cb).atan2(tmp0);
    Some([
        (alpha - tmp1).rem_euclid(std::f64::consts::TAU),
        p_sq.sqrt(),
        (-beta + tmp1).rem_euclid(std::f64::consts::TAU),
    ])
}

fn lsr(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    let c_ab = (alpha - beta).cos();
    let p_sq = -2.0 + d * d + 2.0 * c_ab + 2.0 * d * (sa + sb);
    if p_sq < 0.0 {
        return None;
    }
    let p = p_sq.sqrt();
    let tmp2 = (-ca - cb).atan2(d + sa + sb) - (-2.0_f64).atan2(p);
    Some([
        (-alpha + tmp2).rem_euclid(std::f64::consts::TAU),
        p,
        (-beta + tmp2).rem_euclid(std::f64::consts::TAU),
    ])
}

fn rsl(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    let c_ab = (alpha - beta).cos();
    let p_sq = d * d - 2.0 + 2.0 * c_ab - 2.0 * d * (sa + sb);
    if p_sq < 0.0 {
        return None;
    }
    let p = p_sq.sqrt();
    let tmp2 = (ca + cb).atan2(d - sa - sb) - (-2.0_f64).atan2(p);
    Some([
        (alpha - tmp2).rem_euclid(std::f64::consts::TAU),
        p,
        (beta - tmp2).rem_euclid(std::f64::consts::TAU),
    ])
}

fn rlr(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    let c_ab = (alpha - beta).cos();
    let tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sa - sb)) / 8.0;
    if !(-1.0..=1.0).contains(&tmp) {
        return None;
    }
    let p = (std::f64::consts::TAU - tmp.acos()).rem_euclid(std::f64::consts::TAU);
    let t = (alpha - (ca - cb).atan2(d - sa + sb) + p / 2.0).rem_euclid(std::f64::consts::TAU);
    let q = (alpha - beta - t + p).rem_euclid(std::f64::consts::TAU);
    Some([t, p, q])
}

fn lrl(alpha: f64, beta: f64, d: f64) -> Option<[f64; 3]> {
    let (sa, ca) = alpha.sin_cos();
    let (sb, cb) = beta.sin_cos();
    let c_ab = (alpha - beta).cos();
    let tmp = (6.0 - d * d + 2.0 * c_ab + 2.0 * d * (sb - sa)) / 8.0;
    if !(-1.0..=1.0).contains(&tmp) {
        return None;
    }
    let p = (std::f64::consts::TAU - tmp.acos()).rem_euclid(std::f64::consts::TAU);
    let t = (-alpha - (ca - cb).atan2(d + sa - sb) + p / 2.0).rem_euclid(std::f64::consts::TAU);
    let q = (beta - alpha - t + p).rem_euclid(std::f64::consts::TAU);
    Some([t, p, q])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_ahead_is_a_line() {
        let path =
            dubins_path(Pose2d::new(0.0, 0.0, 0.0), Pose2d::new(5.0, 0.0, 0.0), 1.0).unwrap();
        assert!((path.length - 5.0).abs() < 1e-6, "{}", path.length);
        let end = path.endpoint(Pose2d::new(0.0, 0.0, 0.0));
        assert!(end.distance(Pose2d::new(5.0, 0.0, 0.0)) < 1e-6);
    }

    #[test]
    fn random_poses_are_reached_and_respect_curvature() {
        use rand::rngs::StdRng;
        use rand::{Rng, SeedableRng};
        let mut rng = StdRng::seed_from_u64(11);
        for _ in 0..80 {
            let start = Pose2d::new(
                rng.random_range(-5.0..5.0),
                rng.random_range(-5.0..5.0),
                rng.random_range(-3.0..3.0),
            );
            let goal = Pose2d::new(
                rng.random_range(-5.0..5.0),
                rng.random_range(-5.0..5.0),
                rng.random_range(-3.0..3.0),
            );
            let radius = 1.25;
            let path = dubins_path(start, goal, radius).expect("dubins path");
            let end = path.endpoint(start);
            assert!(
                end.distance(goal) < 1e-4,
                "{} -> {:?}",
                end.distance(goal),
                end
            );
            assert!(angle_difference(end.theta, goal.theta).abs() < 1e-4);
            for segment in &path.segments {
                assert!(segment.length >= -1e-9);
                let kappa = segment.curvature.abs();
                assert!(kappa <= 1.0 / radius + 1e-9, "{kappa}");
            }
        }
    }
}
