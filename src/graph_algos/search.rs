//! Shared open-set machinery for Dijkstra and A*.
//!
//! Two storages sit on top of the same ordering rules:
//!
//! * **Identity search** keys the closed set by the state itself. Finding a
//!   cheaper path overwrites that state in place. This is the right model when
//!   the state *is* the identity (a graph vertex, a grid cell).
//! * **Keyed search** keeps an arena of immutable records and a side table from
//!   key to the best record. A cheaper path appends a new record instead of
//!   mutating the old one, so parent pointers remain geometrically consistent.
//!   Hybrid-A* uses this: the key is the discretized `(cell, heading)` while
//!   the record stores the continuous pose that produced it.
//!
//! # Ordering
//!
//! The open set is a binary heap. The element popped next is the one with
//!
//! 1. smaller `f` (path cost plus heuristic; for Dijkstra `f = g`),
//! 2. then larger `g` (A* tie break toward the node closer to the goal, which
//!    expands fewer nodes when several states share an f-value),
//! 3. then smaller insertion sequence.
//!
//! The sequence number makes the order a total deterministic order. Equal-cost
//! paths therefore do not depend on allocator or hash iteration order: the
//! first path that reached a state wins, because a later path replaces a state
//! only when its cost is strictly smaller.

use std::cmp::Ordering;
use std::hash::Hash;
use std::time::Instant;

use indexmap::map::Entry::{Occupied, Vacant};

use super::GraphNodeMap;
use super::cost::{add_edge, add_heuristic};
use super::limits::SearchLimits;
use crate::collections::FxIndexMap;
use crate::errors::PathPlannerError;
use crate::graph_algos::cost::SearchCost;

/// Outcome of a successful keyed search.
#[derive(Clone, Debug)]
pub struct KeyedSearchResult<N, C> {
    /// States from the start through the goal. Each state is the immutable
    /// record written when that path was discovered, so consecutive states are
    /// exactly the parent and child the successor function produced.
    pub path: Vec<N>,
    /// Cost of `path.last()`, in the same units as the edge weights.
    pub cost: C,
    /// Nodes whose successors were generated.
    pub expansions: usize,
}

#[derive(Clone, Copy, Debug)]
struct HeapItem<C> {
    f: C,
    g: C,
    seq: u64,
    index: usize,
}

impl<C: SearchCost> PartialEq for HeapItem<C> {
    fn eq(&self, other: &Self) -> bool {
        self.cmp(other) == Ordering::Equal
    }
}

impl<C: SearchCost> Eq for HeapItem<C> {}

impl<C: SearchCost> PartialOrd for HeapItem<C> {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl<C: SearchCost> Ord for HeapItem<C> {
    fn cmp(&self, other: &Self) -> Ordering {
        // `BinaryHeap` pops the greatest element. Reverse `f` and the sequence
        // so smaller f and earlier insertions come out first; leave `g` in
        // natural order so a larger path cost wins an f-tie.
        other
            .f
            .cmp_cost(&self.f)
            .then_with(|| self.g.cmp_cost(&other.g))
            .then_with(|| other.seq.cmp(&self.seq))
    }
}

struct Budget {
    limits: SearchLimits,
    started: Instant,
    expansions: usize,
}

impl Budget {
    fn new(limits: SearchLimits) -> Self {
        Self {
            limits,
            started: Instant::now(),
            expansions: 0,
        }
    }

    fn allow_expansion(&mut self) -> Result<(), PathPlannerError> {
        if let Some(err) = self.limits.exceeded(self.expansions, self.started) {
            return Err(err);
        }
        self.expansions += 1;
        Ok(())
    }
}

fn reconstruct<N: Clone>(
    parent: impl Fn(usize) -> usize,
    state: impl Fn(usize) -> N,
    goal: usize,
    node_count: usize,
) -> Result<Vec<N>, PathPlannerError> {
    let mut path = Vec::new();
    let mut current = goal;
    for _ in 0..=node_count {
        path.push(state(current));
        let next = parent(current);
        if next == usize::MAX {
            path.reverse();
            return Ok(path);
        }
        current = next;
    }
    Err(PathPlannerError::NoPathFound)
}

/// Closed set produced by an identity search.
pub(crate) struct IdentityOutcome<N, C> {
    /// States reached, with parent index and path cost.
    pub nodes: GraphNodeMap<N, C>,
    /// Index of the goal state when the goal test succeeded.
    pub goal: Option<usize>,
}

/// Dijkstra / A* over states that are their own keys.
pub(crate) fn search_identity<N, C, IT, NN, H, G>(
    limits: SearchLimits,
    start: N,
    neighbors: NN,
    mut heuristic: H,
    mut goal_fn: G,
) -> Result<IdentityOutcome<N, C>, PathPlannerError>
where
    N: Eq + Hash + Clone,
    C: SearchCost,
    NN: Fn(&N) -> IT,
    IT: IntoIterator<Item = (N, C)>,
    H: FnMut(&N) -> C,
    G: FnMut(&N) -> bool,
{
    let mut open = std::collections::BinaryHeap::new();
    let mut nodes: GraphNodeMap<N, C> = GraphNodeMap::default();
    let mut budget = Budget::new(limits);
    let mut seq = 0_u64;

    let start_index = nodes.insert_full(start.clone(), (usize::MAX, C::zero())).0;
    let h0 = heuristic(&start);
    let f0 = add_heuristic(C::zero(), h0)?;
    open.push(HeapItem {
        f: f0,
        g: C::zero(),
        seq,
        index: start_index,
    });
    seq += 1;

    while let Some(item) = open.pop() {
        let Some((node_ref, &(_, best_g))) = nodes.get_index(item.index) else {
            continue;
        };
        if item.g.cmp_cost(&best_g) == Ordering::Greater {
            continue;
        }
        // Clone so the neighbor closure can run without holding a borrow on
        // the map we are about to update.
        let node = node_ref.clone();
        if goal_fn(&node) {
            return Ok(IdentityOutcome {
                nodes,
                goal: Some(item.index),
            });
        }
        budget.allow_expansion()?;
        let generated: Vec<(N, C)> = neighbors(&node).into_iter().collect();
        let g = nodes
            .get_index(item.index)
            .map(|(_, rec)| rec.1)
            .unwrap_or(item.g);

        for (neighbor, edge) in generated {
            let new_g = add_edge(g, edge)?;
            if let Some((_, old_g)) = nodes.get(&neighbor) {
                if new_g.cmp_cost(old_g) != Ordering::Less {
                    continue;
                }
            }
            let h = heuristic(&neighbor);
            let f = add_heuristic(new_g, h)?;
            let neighbor_index = match nodes.entry(neighbor) {
                Vacant(entry) => {
                    let index = entry.index();
                    entry.insert((item.index, new_g));
                    index
                }
                Occupied(mut entry) => {
                    let index = entry.index();
                    entry.insert((item.index, new_g));
                    index
                }
            };
            open.push(HeapItem {
                f,
                g: new_g,
                seq,
                index: neighbor_index,
            });
            seq += 1;
        }
    }

    Ok(IdentityOutcome { nodes, goal: None })
}

struct ArenaRecord<N, C> {
    state: N,
    parent: usize,
    g: C,
}

/// A* with an explicit discretization key.
///
/// `key_of` maps a continuous (or otherwise rich) state onto the bucket that
/// should be expanded at most once for a given cost. A strictly cheaper path
/// to the same key appends a fresh record; existing records are never rewritten.
pub(crate) fn search_keyed<N, K, C, IT, NN, H, G, KF>(
    limits: SearchLimits,
    start: N,
    key_of: KF,
    neighbors: NN,
    mut heuristic: H,
    mut goal_fn: G,
) -> Result<KeyedSearchResult<N, C>, PathPlannerError>
where
    N: Clone,
    K: Eq + Hash,
    C: SearchCost,
    NN: Fn(&N) -> IT,
    IT: IntoIterator<Item = (N, C)>,
    H: FnMut(&N) -> C,
    G: FnMut(&N) -> bool,
    KF: Fn(&N) -> K,
{
    let mut open = std::collections::BinaryHeap::new();
    let mut arena: Vec<ArenaRecord<N, C>> = Vec::new();
    let mut best: FxIndexMap<K, usize> = FxIndexMap::default();
    let mut budget = Budget::new(limits);
    let mut seq = 0_u64;

    let start_key = key_of(&start);
    arena.push(ArenaRecord {
        state: start.clone(),
        parent: usize::MAX,
        g: C::zero(),
    });
    best.insert(start_key, 0);

    let h0 = heuristic(&start);
    let f0 = add_heuristic(C::zero(), h0)?;
    open.push(HeapItem {
        f: f0,
        g: C::zero(),
        seq,
        index: 0,
    });
    seq += 1;

    while let Some(item) = open.pop() {
        let key = key_of(&arena[item.index].state);
        match best.get(&key) {
            Some(&best_index) if best_index != item.index => continue,
            _ => {}
        }
        if goal_fn(&arena[item.index].state) {
            let path = reconstruct(
                |i| arena[i].parent,
                |i| arena[i].state.clone(),
                item.index,
                arena.len(),
            )?;
            return Ok(KeyedSearchResult {
                path,
                cost: arena[item.index].g,
                expansions: budget.expansions,
            });
        }
        budget.allow_expansion()?;
        let generated: Vec<(N, C)> = neighbors(&arena[item.index].state).into_iter().collect();
        let parent_index = item.index;
        let g = arena[parent_index].g;

        for (neighbor, edge) in generated {
            let new_g = add_edge(g, edge)?;
            let key = key_of(&neighbor);
            if let Some(&prev_index) = best.get(&key) {
                if new_g.cmp_cost(&arena[prev_index].g) != Ordering::Less {
                    continue;
                }
            }
            let h = heuristic(&neighbor);
            let f = add_heuristic(new_g, h)?;
            let index = arena.len();
            arena.push(ArenaRecord {
                state: neighbor,
                parent: parent_index,
                g: new_g,
            });
            best.insert(key, index);
            open.push(HeapItem {
                f,
                g: new_g,
                seq,
                index,
            });
            seq += 1;
        }
    }

    Err(PathPlannerError::NoPathFound)
}

/// Walk parent indices stored in an identity-search map.
pub(crate) fn path_from_map<N, C>(
    nodes: &GraphNodeMap<N, C>,
    goal: usize,
) -> Result<Vec<N>, PathPlannerError>
where
    N: Clone,
{
    let mut path = Vec::new();
    let mut current = goal;
    for _ in 0..=nodes.len() {
        let Some((node, &(parent, _))) = nodes.get_index(current) else {
            return Err(PathPlannerError::NoPathFound);
        };
        path.push(node.clone());
        if parent == usize::MAX {
            path.reverse();
            return Ok(path);
        }
        current = parent;
    }
    Err(PathPlannerError::NoPathFound)
}
