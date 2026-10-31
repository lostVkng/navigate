//! Recoverable failures from planning, geometry, and control helpers.
//!
//! Library paths do not panic on invalid geometric input. Callers inspect these
//! errors and decide whether to relax a tolerance, raise a limit, or report the
//! failure.

use std::fmt;

/// Failure while building or querying planar geometry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum GeometryError {
    /// A polygon needs at least three vertices.
    InvalidPolygon,
}

impl fmt::Display for GeometryError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            GeometryError::InvalidPolygon => {
                write!(f, "a polygon requires at least three vertices")
            }
        }
    }
}

impl std::error::Error for GeometryError {}

/// Which budget a bounded graph search exhausted.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchLimitKind {
    /// More nodes would have been expanded than [`crate::SearchLimits::max_expansions`].
    Expansions,
    /// The elapsed time exceeded [`crate::SearchLimits::max_duration`].
    Duration,
}

/// Failure while searching for or tracking a path.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum PathPlannerError {
    /// The search exhausted every reachable state without meeting the goal test.
    NoPathFound,
    /// The goal test rejected the candidate and no alternative remains.
    GoalFailure,
    /// A sampler could not produce a usable point (empty bounds, for example).
    NoValidPointFound,
    /// The spatial index used by a sampling planner reported a failure.
    ///
    /// Retained so existing matches keep compiling. Current planners recompute
    /// Euclidean distances themselves and do not surface index-internal metrics.
    KdTreeError(String),
    /// A cost was NaN, infinite in a way that cannot be added, or otherwise not a
    /// usable search cost.
    InvalidCost,
    /// An edge cost was negative. Dijkstra and A* require non-negative costs.
    NegativeCost,
    /// The search stopped because an expansion or time budget was exhausted.
    SearchLimitExceeded {
        /// Which budget fired.
        kind: SearchLimitKind,
        /// Nodes whose successors were generated before the budget fired.
        expansions: usize,
    },
    /// A parameter set is inconsistent (non-positive wheelbase, empty grid, ...).
    InvalidConfiguration(String),
    /// A query fell outside the grid or the configured world bounds.
    OutOfBounds,
    /// The start pose is already in collision with the grid or its footprint.
    StartInCollision,
    /// The goal pose is in collision, so no feasible path can end there.
    GoalInCollision,
    /// A failure that does not fit the variants above.
    Other(String),
}

impl fmt::Display for PathPlannerError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            PathPlannerError::NoPathFound => write!(f, "no path to the goal was found"),
            PathPlannerError::GoalFailure => write!(f, "the goal test could not be satisfied"),
            PathPlannerError::NoValidPointFound => write!(f, "no valid sample point was found"),
            PathPlannerError::KdTreeError(msg) => write!(f, "spatial index error: {msg}"),
            PathPlannerError::InvalidCost => {
                write!(f, "a search cost was NaN or could not be added")
            }
            PathPlannerError::NegativeCost => {
                write!(f, "edge costs must be non-negative")
            }
            PathPlannerError::SearchLimitExceeded { kind, expansions } => {
                let which = match kind {
                    SearchLimitKind::Expansions => "expansion",
                    SearchLimitKind::Duration => "time",
                };
                write!(
                    f,
                    "search stopped after {expansions} expansions because the {which} limit was reached"
                )
            }
            PathPlannerError::InvalidConfiguration(msg) => {
                write!(f, "invalid configuration: {msg}")
            }
            PathPlannerError::OutOfBounds => write!(f, "query is outside the map bounds"),
            PathPlannerError::StartInCollision => write!(f, "the start pose is in collision"),
            PathPlannerError::GoalInCollision => write!(f, "the goal pose is in collision"),
            PathPlannerError::Other(msg) => write!(f, "{msg}"),
        }
    }
}

impl std::error::Error for PathPlannerError {}

impl From<kdtree::ErrorKind> for PathPlannerError {
    fn from(error: kdtree::ErrorKind) -> Self {
        PathPlannerError::KdTreeError(error.to_string())
    }
}
