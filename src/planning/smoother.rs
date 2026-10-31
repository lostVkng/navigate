//! Shortcut smoothing for a pose path.
//!
//! Hybrid-A\* paths are sequences of short constant-curvature primitives. This
//! smoother repeatedly replaces a subpath by a single Dubins or Reeds–Shepp
//! curve when that curve is shorter and collision-free. Each accepted piece
//! respects the bicycle model's minimum turning radius, so the result stays
//! kinematically feasible for the same vehicle. Endpoints and their headings
//! are unchanged.

use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Pose2d};
use crate::grid::GridMap;
use crate::motion::{BicycleModel, CurvePath, dubins_path, reeds_shepp_path};

/// Greedy shortcut smoother.
#[derive(Clone, Debug)]
pub struct PathSmoother {
    /// Sample spacing used for collision checks and for the emitted polyline.
    pub spacing: f64,
    /// Use Reeds–Shepp shortcuts when `true`, Dubins shortcuts otherwise.
    pub allow_reverse: bool,
    /// Maximum shortcut passes. Each pass scans the current polyline once.
    pub passes: usize,
}

impl Default for PathSmoother {
    fn default() -> Self {
        Self {
            spacing: 0.25,
            allow_reverse: true,
            passes: 3,
        }
    }
}

impl PathSmoother {
    /// Shortcut `poses` in place's sense: returns a new polyline.
    ///
    /// The first and last poses are preserved. A path of fewer than three
    /// poses is returned unchanged. Invalid spacing is rejected.
    pub fn smooth(
        &self,
        poses: &[Pose2d],
        model: &BicycleModel,
        map: &GridMap,
        footprint: &Footprint,
    ) -> Result<Vec<Pose2d>, PathPlannerError> {
        if !self.spacing.is_finite() || self.spacing <= 0.0 {
            return Err(PathPlannerError::InvalidConfiguration(
                "smoother spacing must be finite and positive".into(),
            ));
        }
        if poses.len() < 3 {
            return Ok(poses.to_vec());
        }
        let radius = model.min_turning_radius();
        let mut current = poses.to_vec();
        for _ in 0..self.passes {
            let next = self.one_pass(&current, radius, map, footprint);
            if next.len() == current.len()
                && polyline_length(&next) + 1e-6 >= polyline_length(&current)
            {
                break;
            }
            current = next;
        }
        Ok(current)
    }

    fn one_pass(
        &self,
        poses: &[Pose2d],
        radius: f64,
        map: &GridMap,
        footprint: &Footprint,
    ) -> Vec<Pose2d> {
        let mut output = vec![poses[0]];
        let mut index = 0usize;
        while index < poses.len() - 1 {
            let mut landed = index + 1;
            let mut chosen: Option<CurvePath> = None;
            for candidate in (index + 2..poses.len()).rev() {
                let Some(path) = self.connect(poses[index], poses[candidate], radius) else {
                    continue;
                };
                if path.length + 1e-3 >= subpath_length(poses, index, candidate) {
                    continue;
                }
                if hits(&path, poses[index], map, footprint, self.spacing) {
                    continue;
                }
                landed = candidate;
                chosen = Some(path);
                break;
            }
            if let Some(path) = chosen {
                let samples = path.sample(poses[index], self.spacing);
                let interior = samples.len().saturating_sub(1);
                output.extend(samples.into_iter().skip(1).take(interior.saturating_sub(1)));
                output.push(poses[landed]);
            } else {
                output.push(poses[landed]);
            }
            index = landed;
        }
        output
    }

    fn connect(&self, start: Pose2d, goal: Pose2d, radius: f64) -> Option<CurvePath> {
        if self.allow_reverse {
            reeds_shepp_path(start, goal, radius)
        } else {
            dubins_path(start, goal, radius)
        }
    }
}

fn hits(
    path: &CurvePath,
    start: Pose2d,
    map: &GridMap,
    footprint: &Footprint,
    spacing: f64,
) -> bool {
    let mut pose = start;
    for segment in &path.segments {
        let pieces = (segment.length.abs() / spacing).ceil().max(1.0) as usize;
        if segment.length.abs() < 1e-12 {
            continue;
        }
        let piece = segment.length / pieces as f64;
        for _ in 0..pieces {
            pose = pose.advance(segment.curvature, piece);
            if map.in_collision(pose, footprint) {
                return true;
            }
        }
    }
    false
}

fn subpath_length(poses: &[Pose2d], from: usize, to: usize) -> f64 {
    poses[from..=to]
        .windows(2)
        .map(|pair| pair[0].distance(pair[1]))
        .sum()
}

fn polyline_length(poses: &[Pose2d]) -> f64 {
    subpath_length(poses, 0, poses.len().saturating_sub(1))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::motion::BicycleParams;

    #[test]
    fn shortcut_removes_a_detour_and_stays_clear() {
        let map = GridMap::new(40, 40, 0.5, 0.0, 0.0).unwrap();
        let model = BicycleModel::new(BicycleParams {
            wheelbase: 2.0,
            max_steering_angle: 0.5,
            max_steering_rate: 1.0,
            min_speed: -1.0,
            max_speed: 2.0,
            max_acceleration: 1.0,
        })
        .unwrap();
        let body = Footprint::Circle { radius: 0.3 };
        // A long way around an empty room. Headings follow the polyline.
        let via = [
            Pose2d::new(2.0, 2.0, 0.0),
            Pose2d::new(6.0, 2.0, 0.0),
            Pose2d::new(6.0, 14.0, std::f64::consts::FRAC_PI_2),
            Pose2d::new(16.0, 14.0, 0.0),
            Pose2d::new(16.0, 2.0, -std::f64::consts::FRAC_PI_2),
            Pose2d::new(18.0, 2.0, 0.0),
        ];
        let smoothed = PathSmoother::default()
            .smooth(&via, &model, &map, &body)
            .unwrap();
        assert_eq!(smoothed[0], via[0]);
        assert_eq!(*smoothed.last().unwrap(), *via.last().unwrap());
        assert!(polyline_length(&smoothed) < polyline_length(&via) - 1.0);
        for pose in &smoothed {
            assert!(!map.in_collision(*pose, &body), "{pose:?}");
        }
    }
}
