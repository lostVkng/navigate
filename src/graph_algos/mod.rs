//! Discrete graph search: Dijkstra and A*.
//!
//! Both algorithms expand the cheapest open state first. Costs implement
//! [`SearchCost`], so integer weights and floating-point weights share one
//! implementation. Ties break deterministically: equal `f` prefers the larger
//! path cost `g` (the state the heuristic says is closer to the goal), then the
//! earlier insertion. [`SearchLimits`] optionally stops the search after a
//! number of expansions or a wall-clock duration.
//!
//! A* here is the Hart–Nilsson–Raphael algorithm (1968). With a zero heuristic
//! it is Dijkstra's algorithm (1959). An admissible heuristic (one that never
//! overestimates the remaining cost) keeps A* optimal for non-negative edge
//! weights. A consistent heuristic also keeps it from reopening states; the
//! implementation still accepts a cheaper rediscovery if one shows up.

mod cost;
mod limits;
mod search;

pub mod a_star;
pub mod dijkstra;

pub use a_star::AStar;
pub use cost::SearchCost;
pub use dijkstra::Dijkstra;
pub use limits::SearchLimits;
pub use search::KeyedSearchResult;

use crate::collections::FxIndexMap;

/// Closed set for an identity search.
///
/// The key is the state. The value is `(parent_index, cost)` where
/// `parent_index` indexes this same map and is [`usize::MAX`] for the start
/// state. `cost` is the cheapest path cost discovered from the start.
pub type GraphNodeMap<N, C> = FxIndexMap<N, (usize, C)>;
