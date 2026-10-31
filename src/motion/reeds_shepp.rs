//! Reeds–Shepp paths: shortest curves of bounded curvature that may reverse.
//!
//! J. A. Reeds and L. A. Shepp, "Optimal paths for a car that goes both
//! forwards and backwards" (*Pacific Journal of Mathematics* 145, 1990) proved
//! that a shortest path uses at most five pieces drawn from `{L, S, R}`, with
//! cusps allowed between pieces. The families implemented here are CSC, CCC,
//! CCCC, CCSC, CSCC, and CCSCC, each closed under time reversal (swap forward
//! and reverse) and reflection (swap left and right). That is the standard set
//! of 48 words. Segment lengths are solved at unit turning radius in the
//! start-pose frame and then scaled back to meters.
//!
//! A negative segment length is driven in reverse. The returned path is the
//! shortest candidate whose integrated endpoint matches the goal.

use crate::geometry::{Pose2d, angle_difference};

use super::curves::{CurvePath, empty_path, segments_from_units};

const PI: f64 = std::f64::consts::PI;
const TAU: f64 = std::f64::consts::TAU;

/// Shortest Reeds–Shepp path from `start` to `goal` at turning radius `radius`.
///
/// Returns `None` when `radius` is not finite and positive or when every
/// candidate misses the goal numerically. A coincident pose pair returns an
/// empty path.
pub fn reeds_shepp_path(start: Pose2d, goal: Pose2d, radius: f64) -> Option<CurvePath> {
    if !radius.is_finite() || radius <= 0.0 {
        return None;
    }
    if start.distance(goal) <= 1e-8 * radius.max(1.0)
        && angle_difference(start.theta, goal.theta).abs() <= 1e-8
    {
        return Some(empty_path(radius));
    }

    let (x, y, phi) = goal_in_start_frame(start, goal, radius);
    let mut best: Option<CurvePath> = None;
    for formula in FORMULAS {
        for (fx, fy, fphi, flip, reflect) in symmetries(x, y, phi) {
            let Some(mut word) = formula(fx, fy, fphi) else {
                continue;
            };
            if flip {
                negate_lengths(&mut word);
            }
            if reflect {
                negate_steers(&mut word);
            }
            let path = segments_from_units(
                &word.steer[..word.n as usize],
                &word.length[..word.n as usize],
                radius,
            );
            if !reaches(start, goal, &path, radius) {
                continue;
            }
            if best
                .as_ref()
                .map(|current| path.length + 1e-9 < current.length)
                .unwrap_or(true)
            {
                best = Some(path);
            }
        }
    }
    best
}

/// Length in meters of [`reeds_shepp_path`], if one exists.
pub fn reeds_shepp_length(start: Pose2d, goal: Pose2d, radius: f64) -> Option<f64> {
    reeds_shepp_path(start, goal, radius).map(|path| path.length)
}

#[derive(Clone, Copy)]
struct Word {
    n: u8,
    steer: [i8; 5],
    length: [f64; 5],
}

fn word(steer: &[i8], length: &[f64]) -> Word {
    let mut out = Word {
        n: steer.len() as u8,
        steer: [0; 5],
        length: [0.0; 5],
    };
    for (i, (&steer, &length)) in steer.iter().zip(length).enumerate() {
        out.steer[i] = steer;
        out.length[i] = length;
    }
    out
}

fn negate_lengths(word: &mut Word) {
    for length in &mut word.length {
        *length = -*length;
    }
}

fn negate_steers(word: &mut Word) {
    for steer in &mut word.steer {
        *steer = -*steer;
    }
}

fn goal_in_start_frame(start: Pose2d, goal: Pose2d, radius: f64) -> (f64, f64, f64) {
    let dx = (goal.x - start.x) / radius;
    let dy = (goal.y - start.y) / radius;
    let (s, c) = start.theta.sin_cos();
    let x = c * dx + s * dy;
    let y = -s * dx + c * dy;
    let phi = angle_difference(0.0, goal.theta - start.theta);
    (x, y, phi)
}

fn reaches(start: Pose2d, goal: Pose2d, path: &CurvePath, radius: f64) -> bool {
    let end = path.endpoint(start);
    let scale = radius.max(1.0);
    end.distance(goal) <= 1e-4 * scale && angle_difference(end.theta, goal.theta).abs() <= 1e-4
}

/// `(x, y, phi, timeflip, reflect)`.
fn symmetries(x: f64, y: f64, phi: f64) -> [(f64, f64, f64, bool, bool); 4] {
    [
        (x, y, phi, false, false),
        (-x, y, -phi, true, false),
        (x, -y, -phi, false, true),
        (-x, -y, phi, true, true),
    ]
}

fn polar(x: f64, y: f64) -> (f64, f64) {
    (x.hypot(y), y.atan2(x))
}

fn mod2pi(angle: f64) -> f64 {
    let mut value = angle % TAU;
    if value < -PI {
        value += TAU;
    } else if value > PI {
        value -= TAU;
    }
    value
}

type Formula = fn(f64, f64, f64) -> Option<Word>;

const FORMULAS: [Formula; 12] = [
    left_straight_left,
    left_straight_right,
    left_right_left,
    left_right_left_reversed_tail,
    left_right_left_reversed_end,
    left_right_left_right,
    left_right_left_right_alt,
    left_right_straight_left,
    left_straight_right_left,
    left_right_straight_right,
    left_straight_left_right,
    left_right_straight_left_right,
];

fn left_straight_left(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u, t) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if (0.0..=PI).contains(&t) {
        let v = mod2pi(phi - t);
        if (0.0..=PI).contains(&v) {
            return Some(word(&[1, 0, 1], &[t, u, v]));
        }
    }
    None
}

fn left_straight_right(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, t1) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    let u1_sq = u1 * u1;
    if u1_sq >= 4.0 {
        let u = (u1_sq - 4.0).sqrt();
        let theta = 2.0_f64.atan2(u);
        let t = mod2pi(t1 + theta);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some(word(&[1, 0, -1], &[t, u, v]));
        }
    }
    None
}

fn left_right_left(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 <= 4.0 {
        let turn = (0.25 * u1).acos();
        let t = mod2pi(turn + theta + PI / 2.0);
        let u = mod2pi(PI - 2.0 * turn);
        let v = mod2pi(phi - t - u);
        return Some(word(&[1, -1, 1], &[t, -u, v]));
    }
    None
}

fn left_right_left_reversed_tail(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 <= 4.0 {
        let turn = (0.25 * u1).acos();
        let t = mod2pi(turn + theta + PI / 2.0);
        let u = mod2pi(PI - 2.0 * turn);
        let v = mod2pi(-phi + t + u);
        return Some(word(&[1, -1, 1], &[t, -u, -v]));
    }
    None
}

fn left_right_left_reversed_end(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 <= 4.0 && u1 > 1e-9 {
        let argument = 1.0 - u1 * u1 * 0.125;
        if !(-1.0..=1.0).contains(&argument) {
            return None;
        }
        let u = argument.acos();
        let asin_argument = 2.0 * u.sin() / u1;
        if !(-1.0..=1.0).contains(&asin_argument) {
            return None;
        }
        let turn = asin_argument.asin();
        let t = mod2pi(-turn + theta + PI / 2.0);
        let v = mod2pi(t - u - phi);
        return Some(word(&[1, -1, 1], &[t, u, -v]));
    }
    None
}

fn left_right_left_right(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 <= 2.0 {
        let argument = (u1 + 2.0) * 0.25;
        if !(0.0..=1.0).contains(&argument) {
            return None;
        }
        let turn = argument.acos();
        let t = mod2pi(theta + turn + PI / 2.0);
        let u = mod2pi(turn);
        let v = mod2pi(phi - t + 2.0 * u);
        if t >= 0.0 && u >= 0.0 && v >= 0.0 {
            return Some(word(&[1, -1, 1, -1], &[t, u, -u, -v]));
        }
    }
    None
}

fn left_right_left_right_alt(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 <= 1e-9 {
        return None;
    }
    let u2 = (20.0 - u1 * u1) / 16.0;
    if (0.0..=1.0).contains(&u2) {
        let u = u2.acos();
        let asin_argument = 2.0 * u.sin() / u1;
        if !(-1.0..=1.0).contains(&asin_argument) {
            return None;
        }
        let turn = asin_argument.asin();
        let t = mod2pi(theta + turn + PI / 2.0);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some(word(&[1, -1, 1, -1], &[t, -u, -u, v]));
        }
    }
    None
}

fn left_right_straight_left(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 >= 2.0 {
        let root = (u1 * u1 - 4.0).sqrt();
        let u = root - 2.0;
        let turn = 2.0_f64.atan2(root);
        let t = mod2pi(theta + turn + PI / 2.0);
        let v = mod2pi(t - phi + PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return Some(word(&[1, -1, 0, 1], &[t, -PI / 2.0, -u, -v]));
        }
    }
    None
}

fn left_straight_right_left(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x - phi.sin(), y - 1.0 + phi.cos());
    if u1 >= 2.0 {
        let root = (u1 * u1 - 4.0).sqrt();
        let u = root - 2.0;
        let turn = root.atan2(2.0);
        let t = mod2pi(theta - turn + PI / 2.0);
        let v = mod2pi(t - phi - PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return Some(word(&[1, 0, -1, 1], &[t, u, PI / 2.0, -v]));
        }
    }
    None
}

fn left_right_straight_right(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 >= 2.0 {
        let t = mod2pi(theta + PI / 2.0);
        let u = u1 - 2.0;
        let v = mod2pi(phi - t - PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return Some(word(&[1, -1, 0, -1], &[t, -PI / 2.0, -u, -v]));
        }
    }
    None
}

fn left_straight_left_right(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 >= 2.0 {
        let t = mod2pi(theta);
        let u = u1 - 2.0;
        let v = mod2pi(phi - t - PI / 2.0);
        if t >= 0.0 && v >= 0.0 {
            return Some(word(&[1, 0, 1, -1], &[t, u, PI / 2.0, -v]));
        }
    }
    None
}

fn left_right_straight_left_right(x: f64, y: f64, phi: f64) -> Option<Word> {
    let (u1, theta) = polar(x + phi.sin(), y - 1.0 - phi.cos());
    if u1 >= 4.0 {
        let root = (u1 * u1 - 4.0).sqrt();
        let u = root - 4.0;
        let turn = 2.0_f64.atan2(root);
        let t = mod2pi(theta + turn + PI / 2.0);
        let v = mod2pi(t - phi);
        if t >= 0.0 && v >= 0.0 {
            return Some(word(&[1, -1, 0, 1, -1], &[t, -PI / 2.0, -u, -PI / 2.0, v]));
        }
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::dubins_length;

    #[test]
    fn forward_straight_matches_the_distance() {
        let start = Pose2d::new(0.0, 0.0, 0.0);
        let goal = Pose2d::new(5.0, 0.0, 0.0);
        let path = reeds_shepp_path(start, goal, 1.0).unwrap();
        assert!((path.length - 5.0).abs() < 1e-5, "{}", path.length);
        assert!(path.endpoint(start).distance(goal) < 1e-5);
    }

    #[test]
    fn reverse_straight_is_shorter_than_a_forward_loop() {
        let start = Pose2d::new(0.0, 0.0, 0.0);
        let goal = Pose2d::new(-4.0, 0.0, 0.0);
        let reeds = reeds_shepp_path(start, goal, 1.0).unwrap();
        let dubins = dubins_length(start, goal, 1.0).unwrap();
        assert!(
            (reeds.length - 4.0).abs() < 1e-4,
            "reeds length {}",
            reeds.length
        );
        assert!(
            reeds.length + 0.5 < dubins,
            "reeds {} dubins {dubins}",
            reeds.length
        );
        assert!(reeds.segments.iter().any(|segment| segment.length < 0.0));
    }

    #[test]
    fn random_poses_reach_the_goal_within_curvature() {
        use rand::rngs::StdRng;
        use rand::{Rng, SeedableRng};
        let mut rng = StdRng::seed_from_u64(19);
        for i in 0..100 {
            let start = Pose2d::new(
                rng.random_range(-8.0..8.0),
                rng.random_range(-8.0..8.0),
                rng.random_range(-4.0..4.0),
            );
            let goal = Pose2d::new(
                rng.random_range(-8.0..8.0),
                rng.random_range(-8.0..8.0),
                rng.random_range(-4.0..4.0),
            );
            let radius = 1.5;
            let path = reeds_shepp_path(start, goal, radius).unwrap_or_else(|| {
                panic!("no reeds-shepp path for case {i}: {start:?} -> {goal:?}")
            });
            let end = path.endpoint(start);
            assert!(
                end.distance(goal) < 1e-3,
                "case {i} position error {}",
                end.distance(goal)
            );
            assert!(angle_difference(end.theta, goal.theta).abs() < 1e-3);
            for segment in &path.segments {
                assert!(segment.curvature.abs() <= 1.0 / radius + 1e-8);
            }
            if let Some(forward) = dubins_length(start, goal, radius) {
                assert!(
                    path.length <= forward + 1e-4,
                    "case {i} reeds {} exceeds dubins {forward}",
                    path.length
                );
            }
        }
    }
}
