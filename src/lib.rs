//! Navigation algorithms for graphs, grids, and car-like vehicles.
//!
//! The crate is organized by the kind of problem being solved:
//!
//! * [`graph_algos`] — Dijkstra and A* on an arbitrary graph, including
//!   floating-point costs, deterministic tie-breaking, and expansion or time
//!   budgets.
//! * [`sampling_algos`] — RRT\* for a point robot among polygonal obstacles.
//! * [`grid`] — an occupancy grid, an exact Euclidean distance transform, and
//!   distance-based cost inflation.
//! * [`motion`] — a kinematic bicycle model, Dubins paths, and Reeds–Shepp paths.
//! * [`planning`] — hybrid-A\* for a car-like robot, plus a shortcut smoother.
//! * [`control`] — regulated pure pursuit and model-predictive path-integral
//!   control.
//!
//! [`prelude`] re-exports the types used by a typical program.

#![warn(missing_docs)]

pub mod collections;
pub mod control;
pub mod errors;
pub mod geometry;
pub mod graph_algos;
pub mod grid;
pub mod motion;
pub mod planning;
pub mod prelude;
pub mod sampling_algos;
