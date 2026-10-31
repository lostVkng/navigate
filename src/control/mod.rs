//! Path tracking and model-predictive control for a bicycle-model vehicle.
//!
//! [`PurePursuit`] is a geometric tracker with the curvature, obstacle-proximity,
//! and predictive-collision speed regulations from "Regulated Pure Pursuit for
//! Robot Path Tracking" (arXiv:2305.20026). [`Mppi`] is a sampling-based
//! model-predictive controller (Williams, Aldrich, Theodorou, *Journal of
//! Guidance, Control, and Dynamics*, 2017) with optional action-derivative
//! noise (arXiv:2112.09988) and an optional `rayon` feature for parallel
//! rollouts.

mod command;
mod critics;
mod mppi;
mod pure_pursuit;

pub use command::VehicleCommand;
pub use critics::{
    ConstraintCritic, ControlSmoothnessCritic, Critic, GoalCritic, ObstacleCritic,
    PathTrackingCritic, Rollout,
};
pub use mppi::{Mppi, MppiConfig, MppiOutput, NoiseModel};
pub use pure_pursuit::{PurePursuit, PurePursuitConfig};
