# Offline route following

[`follow_route`] plays a validated [`RoutePlan`] back on the kinematic
[`BicycleModel`]. The plant is the same rear-axle bicycle the planner already
integrates. The body is the same [`Footprint`], grown by the clearance stamped
on the ticket, and swept with [`GridMap::sweep_in_collision`] at the same
numerical `sweep_tolerance`. Nothing here is a new vehicle model.

The tracker is Stanley, not the regulated pure pursuit already in this crate.
Pure pursuit steers toward a lookahead point. On a route with a cusp that
point can land in the next gear, so one command blends forward and reverse.
Stanley steers from the cross-track and heading error of the active direction
segment only:

```text
δ = atan(κ L) + σ k_θ θ_e − atan2(k_e e, k_s + |v|)
```

`e` is positive when the rear axle is to the left of the path heading. `θ_e`
is the path heading minus the vehicle heading. `σ` is `+1` forward and `−1`
in reverse, because yaw rate changes sign with speed and the heading term has
to change with it. The curvature feedforward does not flip: a positive `κ` is
still a left steer while reversing. `κ` is read from the planned segment, and
from the next segment when a steer change is close enough that the steering
rate can finish it inside [`FollowConfig::curvature_preview`].

A direction change is a run of the same sign in `segment_length`. The speed
setpoint ramps to zero at the end of the run. The gear flips only after the
plant speed is under [`FollowConfig::switch_speed`], and the next run waits
until the steering angle is within [`FollowConfig::steer_align`] of the new
setpoint. The bicycle cannot yaw while stopped, so the last run may crawl
until the goal heading is inside the band.

The speed cap in front of a large steer change is

```text
v ≤ max(switch_speed, rate * steer_slew_distance / gap)
```

`rate` is the plant's steering-rate limit and `gap` is the change in steering
angle. The cap is not lowered below `switch_speed`, so a rate that cannot
finish the change at that crawl leaves the path. That is a scheduling rule
for this simulation, not a measured steering response.

Setpoints sit in a delay line for [`FollowConfig::actuator_delay`] seconds,
rounded to a step. The plant then chases the delayed speed and steering with
[`BicycleModel::step`], which clamps acceleration, speed, steering rate, and
steering angle. Pose noise is added only to the measurement the tracker sees.
Actuation noise is added to the delayed acceleration and steering rate. Both
are zero-mean Gaussians from `StdRng`. A disturbance with every standard
deviation at zero does not draw numbers. Any positive deviation is
deterministic for [`Disturbance::seed`].

Errors and the collision test use the plant pose, not the noisy measurement.
Cross-track is the lateral offset. Overshoot is how far the axle has gone
past the end of the active segment; it counts against the same meter gate.
Heading error is the path heading minus the plant heading. The goal errors
are against the route's final pose. A miss, a gate, or a swept collision
returns [`TrackingError`]. The function does not return a success report for
those runs.

[`RoutePlan::is_current`] is the gate in front of the integrator. A different
frame, revision, or clearance returns [`TrackingError::StaleRoute`] and does
not move the plant. [`refresh_route`] can mint a new ticket; the old one stays
stale.

## Run

```text
cargo test --lib control::follow
cargo test --test route_following
cargo run --release --example route_follow_demo
```

The demo writes:

- `assets/route_follow_demo.svg`
- `assets/route_follow_demo.png`

Both show the planned route, the simulated rear-axle trajectory, and the
signed cross-track against time. The locked numbers are the debug test.
Debug and a release build can differ in the last bits because the sweep and
the tracker use transcendental functions. On this x86-64 guest the release
demo printed the same four rows as the debug table below.

`route_following` covers an open field, a narrow aisle, a reverse park with
gear changes, a forward U-turn, a 0.40 s delay that still tracks with a larger
cross-track error, a 1.20 s delay that sweeps the body into the grid, a
steering rate of 0.55 rad/s that increases cross-track without failing, a
steering stop of 0.22 rad that collides on the U-turn even when the error
gates are widened, seeded noise, a renamed grid frame, a clearance mismatch,
and rejected configuration. Two runs with the same seed return the same
samples.

On an x86-64 guest, debug, `rustc` 1.99, the placeholder configuration:

| Route | Max cross-track | Max heading | Goal position | Goal heading | Gears |
| --- | ---: | ---: | ---: | ---: | ---: |
| Open field | 0.0101 m | 0.074 rad | 0.0398 m | 0.0016 rad | 0 |
| Narrow aisle | 0 | 0 | 0.0394 m | 0 | 0 |
| Reverse park | 0.0934 m | 0.243 rad | 0.0824 m | 0.298 rad | 1 |
| U-turn | 0.0086 m | 0.059 rad | 0.0400 m | 0.011 rad | 0 |

The whole `route_following` file finished in about 0.2 s in debug. The
integration test locks these figures to `1e-9`.

## Placeholder numbers

[`FollowConfig::placeholder`] is the tuning used above. It assumes the
synthetic vehicle in [`planning::synthetic`]:

| Quantity | Value | Status |
| --- | --- | --- |
| Wheelbase | 0.30 m | Placeholder, from the planner |
| Steering stop | 0.50 rad | Placeholder, from the planner |
| Steering rate | 1.2 rad/s | Placeholder, from the planner |
| Cruise speed | 0.30 m/s | Placeholder |
| Cross-track gain `k_e` | 2.4 /s | Placeholder |
| Softening speed `k_s` | 0.25 m/s | Placeholder |
| Heading gain `k_θ` | 1.15 | Placeholder |
| Curvature preview | 0.40 m | Placeholder |
| Slew distance | 0.05 m | Placeholder |
| Steer-align band | 0.08 rad | Placeholder |
| Actuator delay | 0 s | Placeholder |
| Cross-track gate | 0.12 m | Acceptance gate, not a clearance |
| Heading gate | 0.50 rad | Acceptance gate |
| Goal box | 0.12 m, 0.30 rad | Acceptance gate |

Replacing a gain is a caller change: build a [`FollowConfig`] and pass it in
[`FollowRequest`]. Do not treat the demo plot as a controller to copy onto a
robot.

## What this does not establish

A run that stays inside the gates shows that this kinematic model, with these
placeholder gains, could follow this route on this grid. It does not show
that a vehicle can stop, that the steering command matches a wheel angle, or
that the delay matches a real actuator.

Still uncalibrated, and intentionally not invented here:

- wheelbase, track, and body outline of any particular robot
- the map from a steering command to a front-wheel angle
- the steering rate, speed, and acceleration the actuators can hold
- transport delay and stopping distance
- the noise that a real pose estimate actually has

The collision check is the swept footprint of the simulated motion. It is the
same numerical half-chord test the planner uses. A body that clears an
obstacle by less than `sweep_tolerance` can be reported as a hit. That
tolerance is not a safety margin.

This crate does not command motors and does not read sensors.

[`follow_route`]: ../src/control/follow.rs
[`RoutePlan`]: ../src/planning/route.rs
[`BicycleModel`]: ../src/motion/bicycle.rs
[`Footprint`]: ../src/geometry/footprint.rs
[`GridMap::sweep_in_collision`]: ../src/grid/map.rs
[`FollowConfig::curvature_preview`]: ../src/control/follow.rs
[`FollowConfig::switch_speed`]: ../src/control/follow.rs
[`FollowConfig::steer_align`]: ../src/control/follow.rs
[`FollowConfig::actuator_delay`]: ../src/control/follow.rs
[`BicycleModel::step`]: ../src/motion/bicycle.rs
[`Disturbance::seed`]: ../src/control/follow.rs
[`TrackingError`]: ../src/control/follow.rs
[`RoutePlan::is_current`]: ../src/planning/route.rs
[`TrackingError::StaleRoute`]: ../src/control/follow.rs
[`refresh_route`]: ../src/planning/route.rs
[`FollowConfig::placeholder`]: ../src/control/follow.rs
[`planning::synthetic`]: ../src/planning/synthetic.rs
[`FollowConfig`]: ../src/control/follow.rs
[`FollowRequest`]: ../src/control/follow.rs
