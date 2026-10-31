//! Compare a forward Dubins path with a reversing Reeds–Shepp path.
//!
//! The goal sits directly behind the example car. Reversing is a straight
//! line; driving only forward has to loop.
//!
//! Run with `cargo run --example reeds_shepp`.

use navigate::prelude::*;

fn main() {
    let start = Pose2d::new(0.0, 0.0, 0.0);
    let goal = Pose2d::new(-6.0, 0.5, 0.2);
    let radius = 2.5;
    let forward = dubins_path(start, goal, radius).expect("dubins");
    let either = reeds_shepp_path(start, goal, radius).expect("reeds-shepp");
    println!(
        "example car, minimum radius {radius:.1} m: Dubins {:.2} m, Reeds–Shepp {:.2} m ({} segments)",
        forward.length,
        either.length,
        either.segments.len()
    );
}
