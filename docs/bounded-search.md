# Bounded graph search

`graph_algos::bounded::{a_star, dijkstra}` return a `SearchReport` with an explicit
outcome and work counters. They accept nonnegative `u64` edge and heuristic costs.
Every path-cost and priority addition is checked. `CostOverflow` stops the search;
it never wraps, saturates, drops the offending edge, or claims a partial optimum.

```rust
use navigate::graph_algos::bounded::{dijkstra, SearchLimits, SearchOutcome};

let graph = [vec![(1, 2), (2, 5)], vec![(2, 1)], vec![]];
let limits = SearchLimits {
    max_nodes: 16,
    max_frontier: 32,
    max_expansions: 32,
    max_heap_pops: 64,
    max_neighbor_polls: 128,
};
let report = dijkstra(0, |node| graph[*node].clone(), |node| *node == 2,
                      limits, || false);
assert_eq!(report.outcome, SearchOutcome::Found { path: vec![0, 1, 2], cost: 3 });
```

## Optimality and determinism

The graph, edge costs, goal predicate and heuristic must remain fixed for one
search. A* requires an admissible heuristic: it must never overestimate the least
remaining cost to **any** goal, and must be zero at every goal. An observed goal
with nonzero heuristic returns `InvalidHeuristic`; general admissibility cannot
be checked by the search. An inconsistent but admissible heuristic is supported:
strictly improved nodes are reopened. Dijkstra uses a zero heuristic. Zero-cost
edges and cycles are supported; equal-cost alternatives do not replace parents.

Nodes must satisfy Rust's `Eq`/`Hash` contract. Neighbors must have a stable order
for repeatable paths. The bounded queue breaks equal priorities by insertion
order, including reopened nodes. Hash-table iteration never determines queue
order. Reversing the caller's neighbor order can legitimately choose a different
equal-cost route. This API does not canonicalize arbitrary unordered input.

## Limits and cancellation

- `max_nodes` limits distinct retained graph entries, including the start.
- `max_frontier` limits retained heap entries, including stale entries.
- `max_expansions` limits non-goal, non-stale nodes whose neighbors are requested.
- `max_heap_pops` counts all popped entries, including stale entries.
- `max_neighbor_polls` counts every `Iterator::next()` call, including a final
  `None`. It stops an infinite iterator even when every yielded edge is ignored.

Bounds are checked before exceeding them. A goal may be returned with zero
expansions/neighbor polls, but still needs a retained start and one heap pop.
Exhausting an iterator may require one extra poll beyond the number of edges;
without that poll the result is `BudgetExhausted`, not an unproven `NoPath`.
The returned counters never exceed their corresponding limits.

Cancellation is cooperative. The predicate is checked before external callbacks,
iterator polls and path reconstruction. It can consult an atomic flag or an
external deadline clock. If cancelled while reconstructing a found route, the
result is `Cancelled`, not a stale successful path.

**The caller must bound callback work.** This synchronous function cannot preempt
a blocking neighbor callback, an individual `next()`, a heuristic/goal predicate,
node cloning/hashing/equality, or an iterator/node destructor. Returning a huge
prebuilt `Vec` can allocate before the search sees its first neighbor. Prefer a
lazy iterator whose individual operations have known bounds. A bounded number of
such operations is not a hard real-time guarantee.

Storage limits count entries, not bytes: generic nodes and iterators may own
arbitrary memory. The internal graph, heap and returned route retain O(nodes +
frontier) entries, with allocator/container capacity overhead. Container growth
uses fallible reservation and reports `AllocationFailed` when possible; allocation
inside caller code remains caller-owned. No partial graph is returned on failure.
