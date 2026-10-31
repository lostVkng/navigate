//! Occupancy grids, Euclidean distance fields, and distance-based inflation.
//!
//! A [`GridMap`] is a row-major array of cells. The world frame is continuous:
//! cell `(0, 0)` covers the square whose minimum corner is the grid origin, and
//! each cell is `resolution` meters on a side. Planners query three related
//! layers:
//!
//! * **occupancy** — free, occupied, or unknown;
//! * **distance** — exact Euclidean distance in meters from each cell center to
//!   the nearest occupied cell center ([`squared_distance_transform`]);
//! * **cost** — a configurable decay of that distance, in `[0, 1]`, used as a
//!   traversal penalty. Occupied cells are blocked by the occupancy test rather
//!   than by the cost.

mod distance_transform;
mod map;

pub use distance_transform::squared_distance_transform;
pub use map::{CostDecay, GridMap, Inflation, Occupancy};
