use num_traits::{Num, Signed, Float};
use crate::errors::GeometryError;


/// Manhattan distance
pub fn manhattan_distance<T>(x1: T, y1: T, x2: T, y2: T) -> T
where 
    T: Num + Copy + Signed,
    {
    (x1 - x2).abs() + (y1 - y2).abs()
}

/// Euclidean distance
pub fn euclidean<T>(x1: T, y1: T, x2: T, y2: T) -> T
where 
    T: Float,
    {
    ((x1 - x2).powi(2) + (y1 - y2).powi(2)).sqrt()
}

/// Squared Euclidean distance
pub fn squared_euclidean<T>(x1: T, y1: T, x2: T, y2: T) -> T
where 
    T: Float,
    {
    (x1 - x2).powi(2) + (y1 - y2).powi(2)
}


/// 2D Point
#[derive(Clone, Debug, PartialEq)]
pub struct Point {
    pub x: f64,
    pub y: f64,
}


/// 2D Polygon
#[derive(Clone, Debug, PartialEq)]
pub struct Polygon {
    pub points: Vec<Point>, // Clockwise or counter-clockwise
}

impl Polygon {

    /// Create a new polygon from a list of points
    pub fn new(points: Vec<Point>) -> Result<Self, GeometryError> {
        if points.len() < 3 {
            return Err(GeometryError::InvalidPolygon);
        }
        Ok(Self { points })
    }
    
    /// Check if a point is inside the polygon
    /// Uses Ray-casting algorithm
    /// https://en.wikipedia.org/wiki/Point_in_polygon
    pub fn contains(&self, point: &Point) -> bool {
        let mut inside = false;
        let n = self.points.len();

        // Iterate over each edge of the polygon
        for i in 0..n {
            let j = (i + 1) % n; // Next vertex (wraps around to 0)
            let vi = &self.points[i];
            let vj = &self.points[j];

            // Check if the point is on the same y-level as the edge and if the x-coordinate
            // lies within the edge's x-range, and the ray intersects the edge
            if (vi.y > point.y) != (vj.y > point.y) &&
               point.x < (vj.x - vi.x) * (point.y - vi.y) / (vj.y - vi.y) + vi.x {
                inside = !inside;
            }
        }

        inside
    }

    /// Check if a line segment intersects with the polygon
    /// Fails if the line passes through the polygon or stops inside the polygon
    pub fn line_intersects(&self, start: &Point, end: &Point) -> bool {

        let n = self.points.len();

        for i in 0..n {
            let j = (i + 1) % n; // Next vertex (wraps around to 0)
            let vi = &self.points[i];
            let vj = &self.points[j];

            if segments_intersect(&start, &end, vi, vj) {
                return true;
            }
        }

        false
    }
}


/// Point Orientation
/// Returns:
/// 0 if collinear, >0 if clockwise, <0 if counterclockwise.
fn orientation(p: &Point, q: &Point, r: &Point) -> f64 {
    (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y)
}

/// Helper function to check if point q lies on segment pr.
fn on_segment(p: &Point, q: &Point, r: &Point) -> bool {
    q.x <= f64::max(p.x, r.x) &&
    q.x >= f64::min(p.x, r.x) &&
    q.y <= f64::max(p.y, r.y) &&
    q.y >= f64::min(p.y, r.y)
}

/// Helper function to check if segments p1q1 and p2q2 intersect.
fn segments_intersect(p1: &Point, q1: &Point, p2: &Point, q2: &Point) -> bool {
    let o1 = orientation(p1, q1, p2);
    let o2 = orientation(p1, q1, q2);
    let o3 = orientation(p2, q2, p1);
    let o4 = orientation(p2, q2, q1);

    // General case: segments intersect if orientations differ
    if o1 * o2 < 0.0 && o3 * o4 < 0.0 {
        return true;
    }

    // Special cases: collinear and point lies on segment
    if o1 == 0.0 && on_segment(p1, p2, q1) { return true; }
    if o2 == 0.0 && on_segment(p1, q2, q1) { return true; }
    if o3 == 0.0 && on_segment(p2, p1, q2) { return true; }
    if o4 == 0.0 && on_segment(p2, q1, q2) { return true; }

    false
}

