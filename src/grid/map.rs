//! Grid storage, inflation, and footprint collision checks.

use crate::errors::PathPlannerError;
use crate::geometry::{Footprint, Point, Pose2d};

use super::distance_transform::squared_distance_transform;

/// What a single grid cell represents.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Occupancy {
    /// Nothing is known to block this cell.
    Free,
    /// The cell is blocked.
    Occupied,
    /// The cell has not been observed. Treated as blocked by collision checks
    /// and as non-traversable by the grid search heuristic. It does not
    /// generate a distance field of its own.
    Unknown,
}

/// How inflated cost falls off with distance from the nearest obstacle.
///
/// Costs are dimensionless and lie in `[0, 1]`. `1` is the cost inside the
/// inscribed radius (the region a disk of that radius cannot enter without
/// touching an obstacle center). `0` is the cost at or beyond `radius`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum CostDecay {
    /// `cost = 1 - (d - inscribed) / (radius - inscribed)` between the two radii.
    Linear,
    /// `cost = exp(-scaling * (d - inscribed))` between the two radii.
    ///
    /// `scaling` is in inverse meters. Larger values drop the cost faster.
    /// The value is still forced to `0` at `radius`, so the field has a hard
    /// cutoff.
    Exponential {
        /// Inverse-meter decay rate. Must be finite and non-negative.
        scaling: f64,
    },
}

/// Parameters for [`GridMap::inflate`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Inflation {
    /// Distance, in meters, at which the added cost becomes zero.
    pub radius: f64,
    /// Distances at or below this, in meters, receive cost `1`.
    pub inscribed_radius: f64,
    /// Shape of the decay between the inscribed radius and `radius`.
    pub decay: CostDecay,
}

impl Inflation {
    fn cost_at(&self, distance_m: f64) -> f32 {
        if !distance_m.is_finite() {
            return 0.0;
        }
        if distance_m <= self.inscribed_radius {
            return 1.0;
        }
        if distance_m >= self.radius {
            return 0.0;
        }
        let span = (self.radius - self.inscribed_radius).max(1e-9);
        let raw = match self.decay {
            CostDecay::Linear => 1.0 - (distance_m - self.inscribed_radius) / span,
            CostDecay::Exponential { scaling } => {
                (-scaling * (distance_m - self.inscribed_radius)).exp()
            }
        };
        raw.clamp(0.0, 1.0) as f32
    }
}

/// A metric occupancy grid with an optional distance field and cost layer.
#[derive(Clone, Debug)]
pub struct GridMap {
    width: usize,
    height: usize,
    resolution: f64,
    origin_x: f64,
    origin_y: f64,
    cells: Vec<Occupancy>,
    /// Distance in meters from each cell center to the nearest occupied cell
    /// center. Empty until [`Self::compute_distance_field`] or [`Self::inflate`].
    distance_m: Vec<f32>,
    /// Traversal cost in `[0, 1]` produced by [`Self::inflate`].
    cost: Vec<f32>,
    distances_ready: bool,
    unknown_count: usize,
}

impl GridMap {
    /// Empty free grid.
    ///
    /// `origin` is the minimum corner of cell `(0, 0)`, in meters. `resolution`
    /// is the cell size in meters and must be finite and positive.
    pub fn new(
        width: usize,
        height: usize,
        resolution: f64,
        origin_x: f64,
        origin_y: f64,
    ) -> Result<Self, PathPlannerError> {
        if width == 0 || height == 0 {
            return Err(PathPlannerError::InvalidConfiguration(
                "grid width and height must be non-zero".into(),
            ));
        }
        if !resolution.is_finite() || resolution <= 0.0 {
            return Err(PathPlannerError::InvalidConfiguration(
                "grid resolution must be finite and positive".into(),
            ));
        }
        if !origin_x.is_finite() || !origin_y.is_finite() {
            return Err(PathPlannerError::InvalidConfiguration(
                "grid origin must be finite".into(),
            ));
        }
        let Some(cells) = width.checked_mul(height) else {
            return Err(PathPlannerError::InvalidConfiguration(
                "grid dimensions overflow".into(),
            ));
        };
        Ok(Self {
            width,
            height,
            resolution,
            origin_x,
            origin_y,
            cells: vec![Occupancy::Free; cells],
            distance_m: Vec::new(),
            cost: vec![0.0; cells],
            distances_ready: false,
            unknown_count: 0,
        })
    }

    /// Number of columns.
    pub fn width(&self) -> usize {
        self.width
    }

    /// Number of rows.
    pub fn height(&self) -> usize {
        self.height
    }

    /// Cell size in meters.
    pub fn resolution(&self) -> f64 {
        self.resolution
    }

    /// Minimum corner of cell `(0, 0)`, in meters.
    pub fn origin(&self) -> (f64, f64) {
        (self.origin_x, self.origin_y)
    }

    /// `true` after a distance field has been computed and occupancy has not
    /// changed since.
    pub fn distances_ready(&self) -> bool {
        self.distances_ready
    }

    /// Occupancy of cell `(ix, iy)`, or `None` when the index is outside the grid.
    pub fn occupancy(&self, ix: usize, iy: usize) -> Option<Occupancy> {
        if ix >= self.width || iy >= self.height {
            return None;
        }
        Some(self.cells[iy * self.width + ix])
    }

    /// Inflated traversal cost of a cell, in `[0, 1]`.
    ///
    /// Before [`Self::inflate`] every cost is `0`.
    pub fn cost(&self, ix: usize, iy: usize) -> Option<f32> {
        if ix >= self.width || iy >= self.height {
            return None;
        }
        Some(self.cost[iy * self.width + ix])
    }

    /// Distance in meters from the center of cell `(ix, iy)` to the nearest
    /// occupied cell center.
    ///
    /// `None` when the index is outside the grid or the distance field is stale.
    /// `+∞` when the grid contains no occupied cell.
    pub fn distance(&self, ix: usize, iy: usize) -> Option<f32> {
        if !self.distances_ready || ix >= self.width || iy >= self.height {
            return None;
        }
        Some(self.distance_m[iy * self.width + ix])
    }

    /// Replace the occupancy of one cell.
    ///
    /// Invalidates the distance field. Out-of-range indexes return
    /// [`PathPlannerError::OutOfBounds`].
    pub fn set_occupancy(
        &mut self,
        ix: usize,
        iy: usize,
        occupancy: Occupancy,
    ) -> Result<(), PathPlannerError> {
        if ix >= self.width || iy >= self.height {
            return Err(PathPlannerError::OutOfBounds);
        }
        let index = iy * self.width + ix;
        self.note_unknown_change(self.cells[index], occupancy);
        self.cells[index] = occupancy;
        self.distances_ready = false;
        Ok(())
    }

    /// Mark every cell whose center lies inside the axis-aligned rectangle as occupied.
    ///
    /// Coordinates are in meters. Cells outside the grid are ignored.
    pub fn set_aabb_occupied(&mut self, min: Point, max: Point) {
        let (x0, y0) = self.clamp_cell_range(min.x, min.y, max.x, max.y);
        for iy in y0.0..y0.1 {
            for ix in x0.0..x0.1 {
                let center = self.cell_center(ix, iy);
                if center.0 >= min.x && center.0 <= max.x && center.1 >= min.y && center.1 <= max.y
                {
                    let index = iy * self.width + ix;
                    self.note_unknown_change(self.cells[index], Occupancy::Occupied);
                    self.cells[index] = Occupancy::Occupied;
                }
            }
        }
        self.distances_ready = false;
    }

    /// Cell that contains the world point, if it lies inside the grid.
    pub fn world_to_cell(&self, x: f64, y: f64) -> Option<(usize, usize)> {
        if !x.is_finite() || !y.is_finite() {
            return None;
        }
        let gx = ((x - self.origin_x) / self.resolution).floor();
        let gy = ((y - self.origin_y) / self.resolution).floor();
        if gx < 0.0 || gy < 0.0 || gx >= self.width as f64 || gy >= self.height as f64 {
            return None;
        }
        Some((gx as usize, gy as usize))
    }

    /// World coordinates of the center of cell `(ix, iy)`.
    pub fn cell_center(&self, ix: usize, iy: usize) -> (f64, f64) {
        (
            self.origin_x + (ix as f64 + 0.5) * self.resolution,
            self.origin_y + (iy as f64 + 0.5) * self.resolution,
        )
    }

    /// Inflated cost at a world point, using the cell that contains it.
    pub fn cost_at(&self, x: f64, y: f64) -> Option<f32> {
        let (ix, iy) = self.world_to_cell(x, y)?;
        self.cost(ix, iy)
    }

    /// Distance in meters from a world point's cell center to the nearest
    /// occupied cell center.
    pub fn distance_at(&self, x: f64, y: f64) -> Option<f32> {
        let (ix, iy) = self.world_to_cell(x, y)?;
        self.distance(ix, iy)
    }

    /// Recompute the Euclidean distance field from occupied cells.
    ///
    /// Distances are center-to-center and exact on the integer lattice (see
    /// [`squared_distance_transform`]). The conversion to meters is
    /// `sqrt(squared_cells) * resolution`.
    pub fn compute_distance_field(&mut self) -> Result<(), PathPlannerError> {
        let occupied: Vec<bool> = self
            .cells
            .iter()
            .map(|cell| *cell == Occupancy::Occupied)
            .collect();
        let squared = squared_distance_transform(&occupied, self.width, self.height)?;
        self.distance_m = squared
            .into_iter()
            .map(|value| {
                if value.is_infinite() {
                    f32::INFINITY
                } else {
                    (value.sqrt() * self.resolution) as f32
                }
            })
            .collect();
        self.distances_ready = true;
        Ok(())
    }

    /// Compute the distance field and write a `[0, 1]` cost layer.
    ///
    /// Occupied cells receive cost `1` and are still reported as occupied.
    /// Unknown cells keep cost `0` and remain non-traversable. Free cells
    /// receive the configured decay of their distance to the nearest obstacle.
    pub fn inflate(&mut self, params: &Inflation) -> Result<(), PathPlannerError> {
        if !params.radius.is_finite() || params.radius < 0.0 {
            return Err(PathPlannerError::InvalidConfiguration(
                "inflation radius must be finite and non-negative".into(),
            ));
        }
        if !params.inscribed_radius.is_finite() || params.inscribed_radius < 0.0 {
            return Err(PathPlannerError::InvalidConfiguration(
                "inscribed radius must be finite and non-negative".into(),
            ));
        }
        if params.inscribed_radius > params.radius {
            return Err(PathPlannerError::InvalidConfiguration(
                "inscribed radius cannot exceed the inflation radius".into(),
            ));
        }
        if let CostDecay::Exponential { scaling } = params.decay {
            if !scaling.is_finite() || scaling < 0.0 {
                return Err(PathPlannerError::InvalidConfiguration(
                    "exponential cost scaling must be finite and non-negative".into(),
                ));
            }
        }
        self.compute_distance_field()?;
        for (index, cell) in self.cells.iter().enumerate() {
            self.cost[index] = match cell {
                Occupancy::Occupied => 1.0,
                Occupancy::Unknown => 0.0,
                Occupancy::Free => params.cost_at(f64::from(self.distance_m[index])),
            };
        }
        Ok(())
    }

    /// `true` when the footprint touches a non-free cell or leaves the grid.
    ///
    /// Contact counts as collision. Out-of-bounds queries are collisions, so a
    /// planner cannot drive off the map. The test is exact against cell squares
    /// (circle–AABB or separating-axis for an oriented rectangle), with a
    /// distance-field rejection when the query is clearly in open space.
    pub fn in_collision(&self, pose: Pose2d, footprint: &Footprint) -> bool {
        if !pose.x.is_finite() || !pose.y.is_finite() || !pose.theta.is_finite() {
            return true;
        }
        if self.world_to_cell(pose.x, pose.y).is_none() {
            return true;
        }
        if self.clear_by_distance(pose, footprint) {
            return false;
        }
        match *footprint {
            Footprint::Circle { radius } => self.circle_hits(pose.x, pose.y, radius.max(0.0)),
            Footprint::OrientedRectangle { .. } => self.rectangle_hits(pose, footprint),
        }
    }

    fn note_unknown_change(&mut self, previous: Occupancy, next: Occupancy) {
        if previous == Occupancy::Unknown {
            self.unknown_count = self.unknown_count.saturating_sub(1);
        }
        if next == Occupancy::Unknown {
            self.unknown_count += 1;
        }
    }

    fn clear_by_distance(&self, pose: Pose2d, footprint: &Footprint) -> bool {
        if !self.distances_ready || self.unknown_count != 0 {
            return false;
        }
        let Some((ix, iy)) = self.world_to_cell(pose.x, pose.y) else {
            return false;
        };
        let edt = f64::from(self.distance_m[iy * self.width + ix]);
        // The stored distance is center-to-center. The query can sit half a
        // diagonal from its cell center, and the obstacle occupies a square
        // that extends half a diagonal beyond its center. Both margins are
        // `resolution * sqrt(2) / 2`, so the conservative gap is one full
        // diagonal.
        edt > footprint.circumradius() + self.resolution * std::f64::consts::SQRT_2
    }

    fn circle_hits(&self, x: f64, y: f64, radius: f64) -> bool {
        let min = Point::new(x - radius, y - radius);
        let max = Point::new(x + radius, y + radius);
        let (xs, ys) = self.cell_window(min, max);
        for iy in ys.0..ys.1 {
            for ix in xs.0..xs.1 {
                if self.cells[iy * self.width + ix] == Occupancy::Free {
                    continue;
                }
                let (min_x, min_y, max_x, max_y) = self.cell_bounds(ix, iy);
                if circle_intersects_aabb(x, y, radius, min_x, min_y, max_x, max_y) {
                    return true;
                }
            }
        }
        false
    }

    fn rectangle_hits(&self, pose: Pose2d, footprint: &Footprint) -> bool {
        let Some(corners) = footprint.rectangle_corners(pose) else {
            return true;
        };
        let (min_x, max_x, min_y, max_y) = corners.iter().fold(
            (
                f64::INFINITY,
                f64::NEG_INFINITY,
                f64::INFINITY,
                f64::NEG_INFINITY,
            ),
            |(min_x, max_x, min_y, max_y), corner| {
                (
                    min_x.min(corner.x),
                    max_x.max(corner.x),
                    min_y.min(corner.y),
                    max_y.max(corner.y),
                )
            },
        );
        let (xs, ys) = self.cell_window(Point::new(min_x, min_y), Point::new(max_x, max_y));
        for iy in ys.0..ys.1 {
            for ix in xs.0..xs.1 {
                if self.cells[iy * self.width + ix] == Occupancy::Free {
                    continue;
                }
                let (cmin_x, cmin_y, cmax_x, cmax_y) = self.cell_bounds(ix, iy);
                if obb_intersects_aabb(&corners, cmin_x, cmin_y, cmax_x, cmax_y) {
                    return true;
                }
            }
        }
        false
    }

    fn cell_bounds(&self, ix: usize, iy: usize) -> (f64, f64, f64, f64) {
        let min_x = self.origin_x + ix as f64 * self.resolution;
        let min_y = self.origin_y + iy as f64 * self.resolution;
        (
            min_x,
            min_y,
            min_x + self.resolution,
            min_y + self.resolution,
        )
    }

    fn cell_window(&self, min: Point, max: Point) -> ((usize, usize), (usize, usize)) {
        let x0 = ((min.x - self.origin_x) / self.resolution).floor();
        let y0 = ((min.y - self.origin_y) / self.resolution).floor();
        let x1 = ((max.x - self.origin_x) / self.resolution).ceil();
        let y1 = ((max.y - self.origin_y) / self.resolution).ceil();
        let clamp = |value: f64, limit: usize| {
            if value < 0.0 {
                0
            } else if value > limit as f64 {
                limit
            } else {
                value as usize
            }
        };
        (
            (clamp(x0, self.width), clamp(x1, self.width)),
            (clamp(y0, self.height), clamp(y1, self.height)),
        )
    }

    fn clamp_cell_range(
        &self,
        min_x: f64,
        min_y: f64,
        max_x: f64,
        max_y: f64,
    ) -> ((usize, usize), (usize, usize)) {
        self.cell_window(Point::new(min_x, min_y), Point::new(max_x, max_y))
    }
}

fn circle_intersects_aabb(
    cx: f64,
    cy: f64,
    radius: f64,
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
) -> bool {
    let nx = cx.clamp(min_x, max_x);
    let ny = cy.clamp(min_y, max_y);
    let dx = cx - nx;
    let dy = cy - ny;
    dx * dx + dy * dy <= radius * radius
}

fn obb_intersects_aabb(
    corners: &[Point; 4],
    min_x: f64,
    min_y: f64,
    max_x: f64,
    max_y: f64,
) -> bool {
    let aabb = [
        Point::new(min_x, min_y),
        Point::new(max_x, min_y),
        Point::new(max_x, max_y),
        Point::new(min_x, max_y),
    ];
    let axes = [
        (1.0, 0.0),
        (0.0, 1.0),
        (corners[1].x - corners[0].x, corners[1].y - corners[0].y),
        (corners[3].x - corners[0].x, corners[3].y - corners[0].y),
    ];
    for (ax, ay) in axes {
        let (a0, a1) = project(corners, ax, ay);
        let (b0, b1) = project(&aabb, ax, ay);
        if a1 < b0 || b1 < a0 {
            return false;
        }
    }
    true
}

fn project(points: &[Point], ax: f64, ay: f64) -> (f64, f64) {
    let mut min = f64::INFINITY;
    let mut max = f64::NEG_INFINITY;
    for point in points {
        let value = point.x * ax + point.y * ay;
        min = min.min(value);
        max = max.max(value);
    }
    (min, max)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_map() -> GridMap {
        GridMap::new(20, 20, 0.5, 0.0, 0.0).unwrap()
    }

    #[test]
    fn distance_and_linear_inflation_are_monotone() {
        let mut map = open_map();
        map.set_occupancy(10, 10, Occupancy::Occupied).unwrap();
        map.inflate(&Inflation {
            radius: 2.0,
            inscribed_radius: 0.5,
            decay: CostDecay::Linear,
        })
        .unwrap();
        assert_eq!(map.distance(10, 10), Some(0.0));
        let near = map.distance(11, 10).unwrap();
        let far = map.distance(14, 10).unwrap();
        assert!((f64::from(near) - 0.5).abs() < 1e-5);
        assert!(far > near);
        let cost_near = map.cost(11, 10).unwrap();
        let cost_far = map.cost(14, 10).unwrap();
        assert!(cost_near >= cost_far);
        assert_eq!(map.cost(10, 10), Some(1.0));
    }

    #[test]
    fn exponential_decay_is_bounded() {
        let mut map = open_map();
        map.set_occupancy(2, 2, Occupancy::Occupied).unwrap();
        map.inflate(&Inflation {
            radius: 1.5,
            inscribed_radius: 0.0,
            decay: CostDecay::Exponential { scaling: 2.0 },
        })
        .unwrap();
        for iy in 0..map.height() {
            for ix in 0..map.width() {
                let cost = map.cost(ix, iy).unwrap();
                assert!((0.0..=1.0).contains(&cost), "{cost}");
            }
        }
    }

    #[test]
    fn circle_and_rectangle_collision_match_a_cell_scan() {
        let mut map = open_map();
        map.set_occupancy(8, 8, Occupancy::Occupied).unwrap();
        map.compute_distance_field().unwrap();
        let circle = Footprint::Circle { radius: 0.4 };
        let pose_hit = Pose2d::new(4.25, 4.25, 0.0);
        let pose_miss = Pose2d::new(1.25, 1.25, 0.0);
        assert!(map.in_collision(pose_hit, &circle));
        assert!(!map.in_collision(pose_miss, &circle));

        let body = Footprint::OrientedRectangle {
            length: 2.0,
            width: 1.0,
            rear_overhang: 0.5,
        };
        // Rear axle at (3.2, 4.25), heading 0: the rectangle covers x in [2.7, 4.7],
        // and the occupied cell square is [4.0, 4.5] x [4.0, 4.5].
        assert!(map.in_collision(Pose2d::new(3.2, 4.25, 0.0), &body));
        assert!(!map.in_collision(Pose2d::new(1.0, 1.0, 0.3), &body));
        // A rotated body whose corner reaches the obstacle.
        assert!(map.in_collision(Pose2d::new(3.4, 3.4, std::f64::consts::FRAC_PI_4), &body));
    }

    #[test]
    fn outside_the_grid_is_a_collision() {
        let map = open_map();
        let circle = Footprint::Circle { radius: 0.2 };
        assert!(map.in_collision(Pose2d::new(-1.0, 1.0, 0.0), &circle));
    }
}
