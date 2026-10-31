# Changelog

## 0.2.0

Planning and control for car-like vehicles, plus the search and sampling fixes
those planners sit on. This release changes the 0.1 graph-search API.

### Breaking changes

- Dijkstra and A* take costs that implement `SearchCost`
  instead of `Ord + Zero`. The integers, `f32`, and `f64` implement it.
  A custom cost type needs a `SearchCost` implementation. `NaN` and integer
  overflow return `PathPlannerError::InvalidCost`. A negative edge returns
  `PathPlannerError::NegativeCost` instead of being queued.
- `AStar` and `Dijkstra` are no longer empty structs. Construct them with
  `AStar::new()` or `Dijkstra::new()` (both implement `Default`) and attach a
  budget with `with_limits`. The struct literals `AStar {}` and `Dijkstra {}`
  no longer compile.
- `dijkstra::Node`, the public binary-heap entry, is removed. It was an
  implementation detail of the open set.
- RRT* no longer writes progress to stdout.
- RRT* path cost no longer counts the final connection twice. Vertex cost now
  matches the polyline length of the path that reaches it.
- The RRT* rewiring neighborhood is `neighbor_radius`. The previous extra
  filter that discarded neighbors farther than `max_step_size` is gone.
  `max_step_size` only limits steering. The nearest vertex is always eligible
  as a parent, so a radius smaller than the step cannot stall the tree.
- `RRTStar::get_edges` returns real parent/child segments and omits the root
  self-pair the old implementation emitted.
- `Point` is `Copy`.
- `PathPlannerError` and `GeometryError` implement `Display` and `Error`, and
  `PathPlannerError` has variants for invalid costs, budgets, configuration,
  bounds, and start or goal collisions.

### Added

- Optional [`SearchLimits`] on Dijkstra and A*: a maximum number of expansions,
  a wall-clock budget, or both. Equal `f` values expand the larger `g` first
  (closer to the goal under the heuristic), then the earlier insertion, so
  repeated runs on the same graph return the same path.
- [`AStar::plan_keyed`] for searches whose closed-set identity is a function of
  a richer state. Records are immutable, so a reconstructed path is the exact
  sequence of states the successor function produced.
- [`GridMap`]: occupancy, an exact Euclidean distance transform (Felzenszwalb
  and Huttenlocher, 2012), and linear or exponential distance-based inflation.
  Footprints are a circle or an oriented rectangle. Collision tests treat
  contact and out-of-bounds queries as collisions.
- Kinematic bicycle model with wheelbase, steering angle, steering rate, speed,
  and acceleration limits.
- Dubins paths (forward only) and Reeds–Shepp paths (forward and reverse).
- Hybrid-A* for a bicycle-model vehicle: heading-discretized states, motion
  primitives, an analytic Dubins or Reeds–Shepp shot, cost-aware search on the
  grid, optional reverse penalties, and a heuristic that takes the worse of a
  holonomic cost-aware Dijkstra and the non-holonomic path length. The search
  reuses `AStar::plan_keyed` and the obstacle heuristic reuses `Dijkstra`.
- A shortcut smoother that replaces hybrid-A* subpaths with shorter
  collision-free Dubins or Reeds–Shepp curves.
- Regulated pure pursuit: speed-scaled lookahead, curvature regulation,
  obstacle-proximity regulation, goal slowdown, and a forward collision check
  (arXiv:2305.20026).
- Model-predictive path integral control with pluggable critics (path, goal,
  obstacles, control smoothness, constraint saturation), independent or
  action-derivative noise (arXiv:2112.09988), a seedable generator, and an
  optional `rayon` feature for parallel rollouts.
- `criterion` benchmarks in `benches/planners.rs` and runnable examples under
  `examples/`.

## 0.1.1

- Dijkstra, A*, RRT*, and the polygon helpers.
