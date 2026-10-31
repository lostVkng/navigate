//! Kinematic models and bounded-curvature paths for a car-like vehicle.
//!
//! The bicycle model integrates a rear-axle reference point. Dubins paths are
//! the shortest forward-only curves with a minimum turning radius (Dubins,
//! 1957). Reeds–Shepp paths are the shortest curves when the vehicle may also
//! reverse (Reeds and Shepp, *Pacific Journal of Mathematics*, 1990).

mod bicycle;
mod curves;
mod dubins;
mod path;
mod reeds_shepp;

pub use bicycle::{BicycleInput, BicycleModel, BicycleParams, BicycleState};
pub use curves::{CurvePath, CurveSegment};
pub use dubins::{dubins_length, dubins_path};
pub use path::{PathProjection, project_onto_path};
pub use reeds_shepp::{reeds_shepp_length, reeds_shepp_path};
