#![doc = include_str!("../../docs/bounded-search.md")]
//! Cooperative, deterministic A* and Dijkstra with explicit work/storage limits.
//!
//! Costs are nonnegative `u64`; all cost additions are checked. A* requires a
//! fixed admissible heuristic, zero at every goal. Inconsistent admissible
//! heuristics are supported by reopening improved nodes. Neighbors must be
//! stable and ordered for reproducible ties. Callbacks and iterator `next()`
//! must return promptly: this synchronous API cannot preempt caller code.
use crate::collections::FxIndexMap;
use std::{cmp::Ordering, collections::BinaryHeap, hash::Hash};

/// Independent limits on retained graph/queue entries and performed work.
/// These count entries, not bytes owned by caller-defined nodes/iterators.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SearchLimits {
    /// Maximum distinct nodes retained, including the start.
    pub max_nodes: usize,
    /// Maximum heap entries retained, including stale entries.
    pub max_frontier: usize,
    /// Maximum calls to the successor generator.
    pub max_expansions: usize,
    /// Maximum heap removals, including stale entries.
    pub max_heap_pops: usize,
    /// Includes polls returning `None`, so an infinite iterator is bounded too.
    pub max_neighbor_polls: usize,
}

/// The exhausted bound; no partial route is represented as a successful path.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SearchBudget {
    /// A new node would exceed the retained-node limit.
    Nodes,
    /// Pushing another entry would exceed the heap size limit.
    Frontier,
    /// Another successor list would exceed the expansion limit.
    Expansions,
    /// Removing another entry would exceed the heap-pop limit.
    HeapPops,
    /// Polling the current iterator would exceed the neighbor-poll limit.
    NeighborPolls,
}

/// Work performed before the search returned.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct SearchStats {
    /// Number of distinct nodes retained.
    pub discovered: usize,
    /// Number of successor lists requested.
    pub expanded: usize,
    /// Number of heap entries removed.
    pub heap_pops: usize,
    /// Iterator polls including those that returned no successor.
    pub neighbor_polls: usize,
    /// Maximum simultaneous heap entries.
    pub peak_frontier: usize,
}

/// Terminal result of one search.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SearchOutcome<N> {
    /// A goal was reached.
    Found {
        /// Ordered nodes from the start through the goal, inclusive.
        path: Vec<N>,
        /// Sum of edge costs along the path.
        cost: u64,
    },
    /// The reachable graph was exhausted without reaching a goal.
    NoPath,
    /// The indicated limit prevented the next operation.
    BudgetExhausted(SearchBudget),
    /// The caller requested cancellation.
    Cancelled,
    /// A path cost or priority exceeded the range of u64.
    CostOverflow,
    /// An observed goal had nonzero heuristic. Admissibility elsewhere is caller-owned.
    InvalidHeuristic,
    /// Container growth failed. Allocations performed inside caller code are not controlled.
    AllocationFailed,
}

/// The terminal outcome together with its measured work.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SearchReport<N> {
    /// How the search terminated.
    pub outcome: SearchOutcome<N>,
    /// Work counters at termination.
    pub stats: SearchStats,
}

/// Queued node-map index and the cost used to detect superseded entries.
#[derive(Debug, Clone, PartialEq, Eq)]
struct Entry {
    /// Index of the state represented by this queue entry.
    index: usize,
    /// Accumulated cost from the start when this entry was created.
    cost: u64,
    /// Estimated total cost used to order the frontier.
    priority: u64,
    /// Insertion order for deterministic queue ties.
    serial: usize,
}
impl Ord for Entry {
    fn cmp(&self, other: &Self) -> Ordering {
        other
            .priority
            .cmp(&self.priority)
            .then_with(|| other.serial.cmp(&self.serial))
            .then_with(|| other.index.cmp(&self.index))
            .then_with(|| other.cost.cmp(&self.cost))
    }
}
impl PartialOrd for Entry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

/// Search with stable FIFO ordering among equal priorities. Improved nodes are
/// reopened; stale heap entries count toward the pop budget but are not expanded.
/// Cancellation is checked before callbacks, iterator polls and path reconstruction.
///
/// Every callback, node clone/hash/equality operation and iterator destructor must
/// itself be bounded. For a deadline, have `cancelled` inspect an external clock.
/// No background thread, allocation byte limit or hard real-time guarantee is implied.
pub fn a_star<N, IT, NN, H, G, F>(
    start: N,
    mut neighbors: NN,
    mut heuristic: H,
    mut goal: G,
    limits: SearchLimits,
    mut cancelled: F,
) -> SearchReport<N>
where
    N: Eq + Hash + Clone,
    IT: IntoIterator<Item = (N, u64)>,
    NN: FnMut(&N) -> IT,
    H: FnMut(&N) -> u64,
    G: FnMut(&N) -> bool,
    F: FnMut() -> bool,
{
    let mut stats = SearchStats::default();
    let outcome = search(
        start,
        &mut neighbors,
        &mut heuristic,
        &mut goal,
        limits,
        &mut cancelled,
        &mut stats,
    );
    SearchReport { outcome, stats }
}

/// Dijkstra is the same bounded search with a zero heuristic.
pub fn dijkstra<N, IT, NN, G, F>(
    start: N,
    neighbors: NN,
    goal: G,
    limits: SearchLimits,
    cancelled: F,
) -> SearchReport<N>
where
    N: Eq + Hash + Clone,
    IT: IntoIterator<Item = (N, u64)>,
    NN: FnMut(&N) -> IT,
    G: FnMut(&N) -> bool,
    F: FnMut() -> bool,
{
    a_star(start, neighbors, |_| 0, goal, limits, cancelled)
}

/// Run the bounded search, recording work for every terminal outcome.
fn search<N, IT, NN, H, G, F>(
    start: N,
    neighbors: &mut NN,
    heuristic: &mut H,
    goal: &mut G,
    limits: SearchLimits,
    cancelled: &mut F,
    stats: &mut SearchStats,
) -> SearchOutcome<N>
where
    N: Eq + Hash + Clone,
    IT: IntoIterator<Item = (N, u64)>,
    NN: FnMut(&N) -> IT,
    H: FnMut(&N) -> u64,
    G: FnMut(&N) -> bool,
    F: FnMut() -> bool,
{
    use SearchOutcome::*;
    if cancelled() {
        return Cancelled;
    }
    if limits.max_nodes == 0 {
        return BudgetExhausted(SearchBudget::Nodes);
    }
    if limits.max_frontier == 0 {
        return BudgetExhausted(SearchBudget::Frontier);
    }
    let mut nodes: FxIndexMap<N, (usize, u64)> = FxIndexMap::default();
    let mut frontier = BinaryHeap::new();
    if nodes.try_reserve(1).is_err() || frontier.try_reserve(1).is_err() {
        return AllocationFailed;
    }
    let priority = heuristic(&start);
    nodes.insert(start, (usize::MAX, 0));
    frontier.push(Entry {
        index: 0,
        cost: 0,
        priority,
        serial: 0,
    });
    stats.discovered = 1;
    stats.peak_frontier = 1;
    loop {
        if cancelled() {
            return Cancelled;
        }
        if frontier.is_empty() {
            return NoPath;
        }
        if stats.heap_pops == limits.max_heap_pops {
            return BudgetExhausted(SearchBudget::HeapPops);
        }
        let Some(entry) = frontier.pop() else {
            return NoPath;
        };
        stats.heap_pops += 1;
        let (node, &(_, best)) = nodes
            .get_index(entry.index)
            .expect("queued index belongs to retained node map");
        if entry.cost != best {
            continue;
        }
        if goal(node) {
            if entry.priority != entry.cost {
                return InvalidHeuristic;
            }
            let mut path = Vec::new();
            let mut index = entry.index;
            while index != usize::MAX {
                if cancelled() {
                    return Cancelled;
                }
                if path.try_reserve(1).is_err() {
                    return AllocationFailed;
                }
                let (node, &(parent, _)) = nodes
                    .get_index(index)
                    .expect("parent belongs to retained node map");
                path.push(node.clone());
                index = parent;
            }
            path.reverse();
            return Found { path, cost: best };
        }
        if cancelled() {
            return Cancelled;
        }
        if stats.expanded == limits.max_expansions {
            return BudgetExhausted(SearchBudget::Expansions);
        }
        stats.expanded += 1;
        let mut adjacent = neighbors(node).into_iter();
        loop {
            if cancelled() {
                return Cancelled;
            }
            if stats.neighbor_polls == limits.max_neighbor_polls {
                return BudgetExhausted(SearchBudget::NeighborPolls);
            }
            stats.neighbor_polls += 1;
            let Some((neighbor, edge)) = adjacent.next() else {
                break;
            };
            let Some(cost) = best.checked_add(edge) else {
                return CostOverflow;
            };
            let existing = nodes
                .get_full(&neighbor)
                .map(|(i, _, &(_, cost))| (i, cost));
            if existing.is_some_and(|(_, previous)| previous <= cost) {
                continue;
            }
            if existing.is_none() && nodes.len() == limits.max_nodes {
                return BudgetExhausted(SearchBudget::Nodes);
            }
            if frontier.len() == limits.max_frontier {
                return BudgetExhausted(SearchBudget::Frontier);
            }
            if cancelled() {
                return Cancelled;
            }
            let Some(priority) = cost.checked_add(heuristic(&neighbor)) else {
                return CostOverflow;
            };
            if frontier.try_reserve(1).is_err() {
                return AllocationFailed;
            }
            let index = match existing {
                Some((index, _)) => {
                    nodes[index] = (entry.index, cost);
                    index
                }
                None => {
                    if nodes.try_reserve(1).is_err() {
                        return AllocationFailed;
                    }
                    let (index, _) = nodes.insert_full(neighbor, (entry.index, cost));
                    stats.discovered += 1;
                    index
                }
            };
            frontier.push(Entry {
                index,
                cost,
                priority,
                serial: stats.neighbor_polls,
            });
            stats.peak_frontier = stats.peak_frontier.max(frontier.len());
        }
    }
}
