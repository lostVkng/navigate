//! Optional budgets for graph search.

use std::time::Duration;

use crate::errors::{PathPlannerError, SearchLimitKind};

/// Upper bounds on how long a search may run.
///
/// Both fields are optional. [`SearchLimits::none`] (the default) runs until
/// the goal is found or the reachable graph is exhausted.
///
/// Expansions count nodes whose successors are generated. A node that is popped
/// and immediately recognized as the goal does not consume an expansion.
/// Popped entries that are already worse than a known path (stale heap
/// entries) do not consume an expansion either. The clock is read before each
/// expansion.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SearchLimits {
    /// Maximum number of nodes whose successors may be generated.
    pub max_expansions: Option<usize>,
    /// Maximum wall-clock time from the start of the search.
    pub max_duration: Option<Duration>,
}

impl Default for SearchLimits {
    fn default() -> Self {
        Self::none()
    }
}

impl SearchLimits {
    /// No expansion cap and no time cap.
    pub fn none() -> Self {
        Self {
            max_expansions: None,
            max_duration: None,
        }
    }

    /// Stop after `max_expansions` nodes have had their successors generated.
    pub fn expansions(max_expansions: usize) -> Self {
        Self {
            max_expansions: Some(max_expansions),
            max_duration: None,
        }
    }

    /// Stop after `max_duration` has elapsed.
    pub fn duration(max_duration: Duration) -> Self {
        Self {
            max_expansions: None,
            max_duration: Some(max_duration),
        }
    }

    pub(crate) fn exceeded(
        &self,
        expansions: usize,
        started: std::time::Instant,
    ) -> Option<PathPlannerError> {
        if let Some(max) = self.max_expansions {
            if expansions >= max {
                return Some(PathPlannerError::SearchLimitExceeded {
                    kind: SearchLimitKind::Expansions,
                    expansions,
                });
            }
        }
        if let Some(limit) = self.max_duration {
            if started.elapsed() >= limit {
                return Some(PathPlannerError::SearchLimitExceeded {
                    kind: SearchLimitKind::Duration,
                    expansions,
                });
            }
        }
        None
    }
}
