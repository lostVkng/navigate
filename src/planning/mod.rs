//! Global planning for a car-like vehicle on a grid.
//!
//! [`HybridAStar`] searches a discretization of position and heading. Motion
//! primitives come from the bicycle model, and a Dubins or Reeds–Shepp curve
//! is tried as an analytic connection to the goal. The search itself is
//! [`crate::AStar::plan_keyed`]: the key is the discrete pose and the record
//! keeps the continuous pose that produced it.
//!
//! The obstacle heuristic is a cost-aware 8-connected Dijkstra run backward
//! from the goal (reusing [`crate::Dijkstra`]). The non-holonomic heuristic is
//! the length of a Dubins or Reeds–Shepp path with the vehicle's minimum
//! turning radius. The search uses the larger of the two, which is the
//! cost-aware hybrid heuristic described by Macenski, Booker, Wallace, and
//! Fischer (arXiv:2401.13078) on top of the hybrid-A\* search of Dolgov,
//! Thrun, Montemerlo, and Diebel (*International Journal of Robotics Research*,
//! 2010).

mod hybrid_astar;
mod smoother;

pub use hybrid_astar::{HybridAStar, HybridAStarConfig, PlannedPath};
pub use smoother::PathSmoother;
