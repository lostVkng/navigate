# Changelog

## 0.2.3

Offline simulation of a bicycle following a validated route.

- [`follow_route`] integrates [`BicycleModel`] along a [`RoutePlan`] with a
  rear-axle Stanley tracker. Forward and reverse segments are separate gears:
  the speed setpoint stops at a cusp, the steering slews, then the sign flips.
  Pure pursuit stays the geometric tracker it already was; its lookahead can
  step past a cusp, so it is not the follower used here.
- The plant enforces the bicycle's steering angle, steering rate, speed, and
  acceleration limits. Setpoints can be delayed by a whole number of steps.
  Measurement and actuation noise are zero-mean Gaussians from a seeded
  generator. The same seed repeats.
- A run reports cross-track error, heading error, goal pose error, and whether
  the footprint swept into the grid during the simulated motion. Exceeding a
  tolerance or colliding returns [`TrackingError`] instead of a success report.
  A ticket whose frame, revision, or clearance does not match the grid returns
  [`TrackingError::StaleRoute`] and is not followed.
- Gains, speeds, the delay, and the acceptance gates in
  [`FollowConfig::placeholder`] match the synthetic vehicle. They are not
  measurements, and a passing run is not a stopping test.
- `cargo test --test route_following` and
  `cargo run --release --example route_follow_demo`.

## 0.2.2

Offline occupancy snapshots and replanning on the existing swept hybrid-A* planner.

- [`GridMap`] carries a `frame_id` (default `"map"`) and a `revision`. The
  revision starts at 0 and advances by one when a cell actually changes or the
  frame name changes. Repeating the same occupancy, and recomputing the
  distance or cost layer, do not advance it. There is no timestamp.
- [`GridMap::apply_aabb`] paints free, occupied, or unknown cells. Unknown
  cells and anything outside the grid stay blocked. Out-of-map space cannot be
  marked free.
- [`plan_route`] / [`refresh_route`] stamp a [`RoutePlan`] for one
  `(frame_id, revision, clearance)` triple. After the grid changes,
  [`RoutePlan::validated_path`] is `None` until a refresh re-checks or
  searches again. [`RouteRefresh::NoSafeRoute`] is the explicit blocked
  result and is not a ticket.
- `clearance` grows the footprint once. `sweep_tolerance` remains the
  numerical sampling bound on that grown body. The cost-inflation layer is
  not a second hard clearance.
- `cargo test --test obstacle_replans` and `cargo run --release --example replan_demo`.

## 0.2.1

Swept-footprint collision checks for the existing hybrid-A* planner, plus
synthetic route tests and a drawing of those routes.

- Hybrid-A* and the shortcut smoother reject a motion unless the whole
  footprint, not only the sampled poses, stays inside the grid and off
  occupied cells. The split size is [`HybridAStarConfig::sweep_tolerance`]
  (default 0.02 m). That value is a numerical tolerance, not a safety margin.
- [`GridMap::in_collision`] treats a footprint that overhangs the border as a
  collision even when the rear axle is still inside.
- [`check_planned_path`] re-integrates a stored route and repeats the sweep.
  An in-place spin, a curvature past the steering limit, or a hit between
  samples returns [`PathPlannerError::PathInfeasible`].
- `HybridAStarConfig` and `PathSmoother` each gain a `sweep_tolerance` field.
  Struct literals that name every field need to set it; `..Default::default()`
  does not.
- [`planning::synthetic`] holds the placeholder vehicle and the maps used by
  `cargo test --test offline_routes` and `cargo run --release --example route_demo`.
  Those lengths are not measurements.
- Non-finite start and goal poses and a footprint that does not contain the
  rear axle return [`PathPlannerError::InvalidConfiguration`].

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
