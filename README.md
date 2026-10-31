# navigate

[![Current Version](https://img.shields.io/crates/v/navigate)](https://crates.io/crates/navigate)
[![Documentation](https://img.shields.io/docsrs/navigate)](https://docs.rs/navigate)
[![License: MIT](https://img.shields.io/crates/l/navigate.svg)](#license)

Search, sampling, grids, and car-like planning and control in Rust.

`navigate` is a general-purpose library. Graph search works on any state you can hash. The grid, bicycle model, and controllers are parameterized by resolution, wheelbase, and limits, so the same code covers a small example car and a larger one. Nothing here depends on a particular robot, middleware, or downstream project.

## Install

```toml
[dependencies]
navigate = "0.2"
```

Parallel rollout scoring for the model-predictive controller is optional:

```toml
navigate = { version = "0.2", features = ["rayon"] }
```

The 0.2 release changes the 0.1 graph-search constructors and the cost-type bound. See [CHANGELOG.md](CHANGELOG.md).

## Choosing an algorithm

| You have | Use |
| --- | --- |
| A graph with non-negative edge weights and no estimate of remaining cost | [`Dijkstra`](src/graph_algos/dijkstra.rs) |
| The same graph, plus a heuristic that does not overestimate | [`AStar`](src/graph_algos/a_star.rs) |
| A point robot and polygonal obstacles, and any heading is drivable | [`RRTStar`](src/sampling_algos/rrt_star.rs) |
| A car-like robot that must respect a turning radius on a grid | [`HybridAStar`](src/planning/hybrid_astar.rs), then optionally [`PathSmoother`](src/planning/smoother.rs) |
| A path to follow and a simple, explainable speed law | [`PurePursuit`](src/control/pure_pursuit.rs) |
| A path to follow while trading off obstacles, smoothness, and actuator limits | [`Mppi`](src/control/mppi.rs) |

Dijkstra and A* accept integer or floating-point costs. Equal `f` values expand the state with the larger path cost first, then the one inserted earlier, so the same graph yields the same path. [`SearchLimits`](src/graph_algos/limits.rs) stops a search after a number of expansions or a wall-clock duration.

RRT* grows a tree of straight segments. Give it a seed with `with_seed` when a test must repeat. It does not print.

Hybrid-A* discretizes position and heading, expands bicycle-model motion primitives, and tries a Dubins (forward only) or Reeds–Shepp (forward and reverse) curve into the goal. The heuristic is the worse of a cost-aware 8-connected Dijkstra on the grid and the length of that non-holonomic curve. Both pieces reuse the graph searches above.

Regulated pure pursuit fits a circular arc to a lookahead point and then slows down for tight curvature, for nearby obstacles, and for the goal. It also samples the commanded arc and stops if the footprint would hit something.

MPPI keeps a nominal sequence of acceleration and steering-rate commands, rolls out noisy copies with the bicycle model, and replaces the nominal with the cost-weighted average. Critics for path tracking, the goal, obstacles, control smoothness, and constraint saturation are included; add your own by implementing `Critic`. `NoiseModel::ActionDerivative` integrates noise on the control derivative so the samples are smooth (arXiv:2112.09988). The `rayon` feature parallelizes the rollouts without changing the result for a given seed.

## A floating-point path

```rust
use navigate::prelude::Dijkstra;

let path = Dijkstra::new()
    .plan(
        "a",
        |node: &&str| match *node {
            "a" => vec![("b", 1.5_f64), ("c", 0.4)],
            "b" => vec![("d", 1.0)],
            "c" => vec![("d", 1.0)],
            _ => vec![],
        },
        |node| *node == "d",
    )
    .expect("reachable");
assert_eq!(path, vec!["a", "c", "d"]);
```

## An example car

The runnable programs under [`examples/`](examples) use the same made-up vehicle: wheelbase 2 m, steering limit about half a radian, and a grid of a few tens of meters. They print a one-line result and do not draw a window.

```text
cargo run --example dijkstra
cargo run --example rrt_star
cargo run --example costmap
cargo run --example reeds_shepp
cargo run --example hybrid_astar
cargo run --example pure_pursuit
cargo run --example mppi
cargo run --example mppi --features rayon
cargo bench --bench planners
cargo bench --bench planners --features rayon
```

## Benchmarks

Measured with `cargo bench --bench planners` on a 4-thread x86-64 guest. Times are the criterion point estimate. This is not an embedded ARM core; the controller is sized so a few thousand samples stay in the tens of hertz on a single core of a machine like this one, and the `rayon` feature splits the rollouts across the cores that are free.

| Benchmark | Time |
| --- | --- |
| Euclidean distance transform, 128×128 | 195 µs |
| Euclidean distance transform, 256×256 | 802 µs |
| Euclidean distance transform, 512×512 | 4.89 ms |
| Hybrid-A*, 40 m × 20 m grid at 0.5 m, one obstacle | 6.92 ms |
| Regulated pure pursuit, one command | 2.48 µs |
| MPPI, horizon 20, 256 samples | 2.91 ms (0.89 ms with `rayon`) |
| MPPI, horizon 20, 1024 samples | 11.4 ms (3.51 ms with `rayon`) |

At 1024 samples the sequential update is about 88 Hz on this guest. Scaling the same measurement, a couple of thousand samples is still a few tens of hertz on one core. A slower core will land lower; the rollout itself does not allocate.

## What is not here

Hybrid-A* is complete for its motion primitives and its grid, not a proof of the globally shortest drivable curve. The obstacle heuristic is a grid distance, so it can be slightly pessimistic or slightly optimistic relative to a continuous holonomic path; the non-holonomic length is what keeps the search honest about turning. There is no incremental replanner, no distance-table cache for Reeds–Shepp, and the smoother only shortcuts with feasible curves. It does not solve a general optimal-control smoothing problem.

## License

MIT. See [LICENSE](LICENSE).
