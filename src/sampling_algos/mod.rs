//! Sampling-based planners.
//!
//! [`rrt_star`] grows a tree of collision-free straight segments for a point
//! robot. It is the right default when the robot can be treated as a point (or
//! the obstacles have already been inflated by the footprint) and a
//! kinematically feasible curve is not required. Car-like robots that must
//! respect a turning radius should use [`crate::planning::HybridAStar`] instead.

pub mod rrt_star;
