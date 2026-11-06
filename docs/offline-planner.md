# Offline steering-limited routes

Hybrid-A* is the car-like planner already in this crate. It searches a
discretized rear-axle pose with [`AStar::plan_keyed`], expands
[`BicycleModel`] arcs, and finishes with a Dubins curve (forward only) or a
Reeds–Shepp curve (forward and reverse). The heuristic is the worse of a
cost-aware grid Dijkstra and the length of that bounded-curvature curve.

A grid point path is the wrong tool here: RRT* in this crate drives a point
robot along straight segments and does not enforce a turning radius. Plain
grid A* can turn in place. Hybrid-A* is the search that already speaks the
bicycle model, so the offline milestone extends that search instead of adding
a second planner.

The minimum turning radius is not a separate constant. It is

```text
wheelbase / tan(max steering angle)
```

on the [`BicycleModel`] passed to [`HybridAStar::plan`]. A caller who wants a
tighter or wider vehicle changes the wheelbase or the steering stop. Reverse
is optional (`HybridAStarConfig::allow_reverse`). Turning it off switches the
analytic shot from Reeds–Shepp to Dubins and drops the reverse primitives.

## What the search guarantees

Every primitive and every analytic shot is rejected unless
[`GridMap::sweep_in_collision`] says the footprint stays on the grid and off
occupied or unknown cells for the whole arc. Samples are spaced so the
half-chord of the farthest body point is at most `sweep_tolerance` (default
0.02 m). Both endpoints are then tested with the footprint enlarged by that
half-chord. A rectangle is enlarged on every side, which contains the disk
Minkowski sum and is slightly larger at the corners.

`sweep_tolerance` is a numerical tolerance. A body that clears an obstacle by
less than that distance can be rejected even though a perfect geometric test
would allow it. It is not a calibrated stopping distance or a safety margin.

[`check_planned_path`] repeats the integration and the sweep from the stored
curvature and arc length. Use it on any route you keep. A picture of the route
is not the check. The route demo runs the check and exits with an error if it
fails.

## Run

```text
cargo test --test offline_routes
cargo test --lib planning
cargo run --release --example route_demo
cargo run --example hybrid_astar
cargo bench --bench planners -- hybrid_astar
```

`route_demo` plans four synthetic maps, prints length, cost, expansions,
reverse distance, and wall time, and writes:

- `assets/route_demo.svg`
- `assets/route_demo.png`

One release run on an x86-64 guest (`rustc` 1.99, `cargo run --release --example route_demo`) planned the four pictures in:

| Route | Length | Expansions | Reverse | Time |
| --- | ---: | ---: | ---: | ---: |
| Open field | 4.211 m | 1 | 0 | 0.5 ms |
| Narrow aisle | 5.500 m | 3 | 0 | 0.4 ms |
| Obstacle bypass | 7.051 m | 75 | 0 | 2.7 ms |
| Reverse park | 2.744 m | 191 | 1.030 m | 7.7 ms |

These are single shots, not a criterion estimate. The locked regression is the expansion count and the length, which do not depend on the clock. The whole `offline_routes` test file finished in about 0.2 s in debug on the same guest.

The integration test locks the same routes: open field, narrow aisle, aisle
tighter than the sweep margin, obstacle bypass, reverse parking (and the same
bay with reverse disabled), a U-turn that fits only when the turning radius
is small enough, a sealed goal, endpoint collisions, invalid poses and
footprints, and expansion and time budgets. Two runs of each successful query
must return the same poses.

The placeholder vehicle used by those maps is in
[`planning::synthetic`]:

| Quantity | Value | Status |
| --- | --- | --- |
| Wheelbase | 0.30 m | Placeholder |
| Steering stop | 0.50 rad | Placeholder |
| Implied minimum radius | `0.30 / tan(0.50)` ≈ 0.549 m | Derived, not measured |
| Body | 0.48 m × 0.30 m, rear overhang 0.12 m | Placeholder |
| Sweep tolerance | 0.02 m | Numerical tolerance |

Nothing in that table was measured on hardware. Replacing a placeholder with
a measured value is a caller change: construct a `BicycleModel` and a
`Footprint` and pass them to `plan`. Do not treat the demo picture as a
template for a real robot.

## Obstacle snapshots and replanning

[`GridMap`] is the occupancy interface. It stores resolution, the minimum
corner of cell `(0, 0)`, and an opaque `frame_id`. Cell `(i, j)` covers

```text
[origin_x + i * resolution, origin_x + (i + 1) * resolution)
[origin_y + j * resolution, origin_y + (j + 1) * resolution)
```

in that frame. The grid does not convert between frames. [`Occupancy`] is
`Free`, `Occupied`, or `Unknown`. Unknown cells are blocked. A query outside
the grid is blocked. [`GridMap::apply_aabb`] cannot mark those outside cells
free.

`revision` starts at 0. It advances by one when a mutating call changes at
least one cell, or when `set_frame_id` changes the name. Writing the same
occupancy again does not advance it. Rebuilding the distance field or the
soft cost layer does not advance it. There is no timestamp, so validity does
not depend on wall-clock time.

[`plan_route`] runs hybrid-A* and [`check_planned_path`], then stamps a
[`RoutePlan`] with the frame, the revision, and the clearance it used.
[`RoutePlan::path`] stays readable after the grid changes, and
[`RoutePlan::validated_path`] returns `None` until the triple matches again.
[`refresh_route`] either re-stamps the same geometry ([`RouteRefresh::StillClear`]),
replaces it ([`RouteRefresh::Replanned`]), or returns
[`RouteRefresh::NoSafeRoute`]. The previous ticket is not updated in place.

`RouteRequest::clearance` grows the footprint once before that check. It is
not also painted into the grid. `sweep_tolerance` is still only the numerical
half-chord bound. [`GridMap::inflate`] writes a soft traversal cost and does
not change occupancy. Using all three as if they were the same safety margin
would count the body more than once. None of them is a stopping distance.

```text
cargo test --test obstacle_replans
cargo run --release --example replan_demo
```

The demo writes `assets/replan_demo.svg` and `assets/replan_demo.png`. One
release run on the same x86-64 guest:

| Snapshot | Length | Expansions | Revision | Time |
| --- | ---: | ---: | ---: | ---: |
| Initial corridor | 4.200 m | 1 | 4 | 0.3 ms |
| Replan around a plug | 4.344 m | 30 | 5 | 1.0 ms |
| Gate closed | no safe route | — | 3 | 47 ms |

Debug and release can pick different feasible detours because the sweep test
uses transcendental functions. The regression lock is the debug test:
initial length 4.200 m in 1 expansion, replan length 4.965 m in 157
expansions, and `NoPathFound` when the gate is closed. `obstacle_replans`
finished in about 0.4 s in debug.

## Later integration

A program that owns a robot can adapt a sensor-neutral occupancy snapshot
into `GridMap` later. That adapter is not in this crate. It would supply
resolution, origin, and `frame_id`; copy free, occupied, and unknown without
turning unknown into free; and pass the body plus one clearance into
`plan_route`. It would drop a `RoutePlan` when the frame, revision, or
clearance changes. It would not rasterize the footprint into occupied cells.

This crate does not read depth, odometry, or camera frames, and it does not
treat a synthetic patch as a camera. Wheelbase, steering map, body outline,
and any physical clearance still have to be measured before a route is used
on hardware. A sequence of static grids does not predict moving obstacles and
does not prove the vehicle can stop.

## What this does not establish

A feasible simulated route is not evidence that a physical vehicle can follow
it, and it is not a safety proof. Tracking that route with the Stanley
simulation is a separate check, described in [route-following.md](route-following.md).
That check can reject a stale ticket and a plant that leaves the path. It
still does not establish a stopping distance.

Still uncalibrated, and intentionally not invented here:

- wheelbase, track, and body outline of any particular robot
- the map from a steering command to a front-wheel angle
- steering rate, speed, and acceleration the actuators can actually hold
- control delay and stopping distance
- where the body is relative to the rear axle
- how far the sensors see, and whether the space behind the robot is observed

Reverse in this crate is a geometric option. It does not look behind the
vehicle. A layout that is only solvable by reversing should stay out of any
runtime that cannot see that space.

This crate does not command motors and does not read sensors. The occupancy
interface is `GridMap` plus `plan_route`. Filling that grid from depth or
odometry belongs in the program that owns the robot.

Search limits (`SearchLimits`) bound expansions or wall time. Hitting a limit
is [`PathPlannerError::SearchLimitExceeded`], which is different from
[`PathPlannerError::NoPathFound`]. A budget that expires before the reachable
set is exhausted does not prove the goal is unreachable.

[`AStar::plan_keyed`]: ../src/graph_algos/a_star.rs
[`BicycleModel`]: ../src/motion/bicycle.rs
[`HybridAStar::plan`]: ../src/planning/hybrid_astar.rs
[`GridMap::sweep_in_collision`]: ../src/grid/map.rs
[`check_planned_path`]: ../src/planning/check.rs
[`planning::synthetic`]: ../src/planning/synthetic.rs
[`PathPlannerError::SearchLimitExceeded`]: ../src/errors.rs
[`PathPlannerError::NoPathFound`]: ../src/errors.rs
[`GridMap`]: ../src/grid/map.rs
[`Occupancy`]: ../src/grid/map.rs
[`GridMap::apply_aabb`]: ../src/grid/map.rs
[`plan_route`]: ../src/planning/route.rs
[`RoutePlan`]: ../src/planning/route.rs
[`RoutePlan::path`]: ../src/planning/route.rs
[`RoutePlan::validated_path`]: ../src/planning/route.rs
[`refresh_route`]: ../src/planning/route.rs
[`RouteRefresh::StillClear`]: ../src/planning/route.rs
[`RouteRefresh::Replanned`]: ../src/planning/route.rs
[`RouteRefresh::NoSafeRoute`]: ../src/planning/route.rs
[`GridMap::inflate`]: ../src/grid/map.rs
