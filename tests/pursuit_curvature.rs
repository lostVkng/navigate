use navigate::{control::pursuit_curvature, geometry::Point};

#[test]
fn lookahead_curvature_matches_signed_circles_and_handles_degenerate_points() {
    for (x, y, expected) in [
        (1.0, 1.0, 1.0),
        (1.0, -1.0, -1.0),
        (2.0, 0.0, 0.0),
        (0.0, 0.0, 0.0),
        (0.0, 0.00001, 0.0),
    ] {
        assert!((pursuit_curvature(Point::new(x, y)).unwrap() - expected).abs() < 1e-12);
    }
    assert_eq!(pursuit_curvature(Point::new(f64::NAN, 1.0)), None);
}

#[test]
fn cutoff_extreme_coordinates_and_invalid_inputs() {
    assert_eq!(pursuit_curvature(Point::new(0., 1e-4)), Some(20_000.));
    let k = pursuit_curvature(Point::new(f64::MAX, f64::MAX)).unwrap();
    assert!((k * f64::MAX - 1.).abs() < 1e-12);
    assert_eq!(pursuit_curvature(Point::new(1.0, f64::INFINITY)), None);
}
