//! RRT\* (Karaman and Frazzoli, *International Journal of Robotics Research*, 2011).
//!
//! The tree grows by sampling a point, steering from the nearest vertex by at
//! most `max_step_size`, and connecting the new vertex to the cheapest
//! collision-free neighbor inside `neighbor_radius`. Existing neighbors are
//! then rewired when the new vertex offers them a cheaper path.
//!
//! Rewiring updates a vertex and then only the vertices that descend from it.
//! Each vertex stores its children, so that walk is proportional to the
//! subtree rather than to the whole tree. Reported distances from the spatial
//! index are squared Euclidean; every length used for a cost is recomputed
//! with [`euclidean`].
//!
//! The search writes nothing to stdout. Pass [`RRTStar::with_seed`] for a
//! reproducible sample sequence.

use std::collections::VecDeque;

use kdtree::KdTree;
use kdtree::distance::squared_euclidean as kt_squared_euclidean;
use rand::rngs::StdRng;
use rand::{Rng, SeedableRng};

use crate::errors::PathPlannerError;
use crate::geometry::{Point, Polygon, euclidean};

/// Axis-aligned world with polygonal obstacles, used by [`RRTStar`].
#[derive(Clone, Debug)]
pub struct Map {
    /// Obstacle polygons. A sample is rejected when it lies inside one or when
    /// the segment that reaches it crosses one.
    pub obstacles: Vec<Polygon>,
    /// Inclusive minimum and maximum corners of the sample domain.
    pub bounds: (Point, Point),
}

impl Map {
    /// An empty map covering `bounds`.
    pub fn new(bounds: (Point, Point)) -> Self {
        Self {
            obstacles: Vec::new(),
            bounds,
        }
    }

    /// Replace the obstacle list.
    pub fn set_obstacles(&mut self, obstacles: Vec<Polygon>) {
        self.obstacles = obstacles;
    }

    /// `true` when `point` lies inside the bounds.
    pub fn in_bounds(&self, point: Point) -> bool {
        point.x >= self.bounds.0.x
            && point.x <= self.bounds.1.x
            && point.y >= self.bounds.0.y
            && point.y <= self.bounds.1.y
    }

    /// `true` when both endpoints are inside the bounds and outside every
    /// obstacle, and the segment does not cross an obstacle edge.
    pub fn segment_free(&self, start: Point, end: Point) -> bool {
        if !self.in_bounds(start) || !self.in_bounds(end) {
            return false;
        }
        for obstacle in &self.obstacles {
            if obstacle.contains(&start)
                || obstacle.contains(&end)
                || obstacle.line_intersects(&start, &end)
            {
                return false;
            }
        }
        true
    }
}

#[derive(Clone, Copy, Debug)]
struct Node {
    point: Point,
    cost: f64,
    parent: Option<usize>,
}

enum Sampler {
    Thread,
    Seeded(Box<StdRng>),
}

/// Asymptotically optimal sampling-based planner for a point robot.
pub struct RRTStar<'a> {
    nodes: Vec<Node>,
    children: Vec<Vec<usize>>,
    tree: KdTree<f64, usize, [f64; 2]>,
    map: Map,
    start: Point,
    goal_fn: Box<dyn Fn(&Point) -> bool + 'a>,
    max_step_size: f64,
    neighbor_radius: f64,
    max_iterations: usize,
    goals: Vec<usize>,
    sampler: Sampler,
}

impl<'a> RRTStar<'a> {
    /// Build a tree rooted at `start`.
    ///
    /// `goal_fn` reports whether a vertex is close enough to count as a goal.
    /// `max_step_size` limits the steer. `neighbor_radius` is the rewiring
    /// radius in meters. `max_iterations` is the number of samples drawn.
    ///
    /// The sample stream uses the thread-local generator until
    /// [`Self::with_seed`] replaces it.
    pub fn new<G>(
        map: Map,
        start: Point,
        goal_fn: G,
        max_step_size: f64,
        neighbor_radius: f64,
        max_iterations: usize,
    ) -> Self
    where
        G: Fn(&Point) -> bool + 'a,
    {
        let nodes = vec![Node {
            point: start,
            cost: 0.0,
            parent: None,
        }];
        let mut tree = KdTree::new(2);
        // A two-dimensional tree cannot reject a two-dimensional point.
        let _ = tree.add([start.x, start.y], 0);
        Self {
            nodes,
            children: vec![Vec::new()],
            tree,
            map,
            start,
            goal_fn: Box::new(goal_fn),
            max_step_size: max_step_size.max(0.0),
            neighbor_radius: neighbor_radius.max(0.0),
            max_iterations,
            goals: Vec::new(),
            sampler: Sampler::Thread,
        }
    }

    /// Draw later samples from a deterministic generator.
    pub fn with_seed(mut self, seed: u64) -> Self {
        self.sampler = Sampler::Seeded(Box::new(StdRng::seed_from_u64(seed)));
        self
    }

    /// Every vertex currently in the tree, in insertion order.
    pub fn get_visited_points(&self) -> Vec<Point> {
        self.nodes.iter().map(|node| node.point).collect()
    }

    /// Tree edges as `(child, parent)` pairs. The root is omitted.
    pub fn get_edges(&self) -> Vec<(Point, Point)> {
        self.nodes
            .iter()
            .filter_map(|node| {
                node.parent
                    .map(|parent| (node.point, self.nodes[parent].point))
            })
            .collect()
    }

    /// Draw one sample inside the bounds.
    ///
    /// Up to 32 draws are rejected when they land inside an obstacle. If every
    /// draw is occupied the last one is returned; [`Self::plan`] still refuses
    /// to connect it.
    pub fn random_point(&mut self) -> Point {
        let width = self.map.bounds.1.x - self.map.bounds.0.x;
        let height = self.map.bounds.1.y - self.map.bounds.0.y;
        let mut point = self.start;
        for _ in 0..32 {
            let x = self.sample_unit() * width + self.map.bounds.0.x;
            let y = self.sample_unit() * height + self.map.bounds.0.y;
            point = Point::new(x, y);
            if self
                .map
                .obstacles
                .iter()
                .all(|obstacle| !obstacle.contains(&point))
            {
                break;
            }
        }
        point
    }

    /// Grow the tree and return the cheapest vertex inside the goal region.
    ///
    /// [`PathPlannerError::NoPathFound`] means no sample landed in the goal
    /// region. The path is the chain of tree vertices from the start.
    pub fn plan(&mut self) -> Result<Vec<Point>, PathPlannerError> {
        if (self.goal_fn)(&self.start) {
            return Ok(vec![self.start]);
        }
        if !self.max_step_size.is_finite() || !self.neighbor_radius.is_finite() {
            return Err(PathPlannerError::InvalidConfiguration(
                "RRT* step and radius must be finite".into(),
            ));
        }

        for _ in 0..self.max_iterations {
            let sample = self.random_point();
            let Some((nearest_index, new_point)) = self.steer_from_nearest(sample)? else {
                continue;
            };
            if !self
                .map
                .segment_free(self.nodes[nearest_index].point, new_point)
            {
                continue;
            }

            let neighbors = self.neighbors_within_radius(new_point)?;
            let parent = self.choose_parent(nearest_index, new_point, &neighbors);
            let Some((parent_index, cost)) = parent else {
                continue;
            };

            let index = self.nodes.len();
            self.nodes.push(Node {
                point: new_point,
                cost,
                parent: Some(parent_index),
            });
            self.children.push(Vec::new());
            self.children[parent_index].push(index);
            self.tree.add([new_point.x, new_point.y], index)?;

            self.rewire(index, &neighbors);
            if (self.goal_fn)(&new_point) {
                self.goals.push(index);
            }
        }

        let best = self.goals.iter().copied().min_by(|a, b| {
            self.nodes[*a]
                .cost
                .partial_cmp(&self.nodes[*b].cost)
                .unwrap_or(std::cmp::Ordering::Equal)
                .then(a.cmp(b))
        });
        match best {
            Some(index) => self.path_to(index),
            None => Err(PathPlannerError::NoPathFound),
        }
    }

    fn sample_unit(&mut self) -> f64 {
        match &mut self.sampler {
            Sampler::Thread => rand::rng().random(),
            Sampler::Seeded(rng) => rng.random(),
        }
    }

    fn steer_from_nearest(
        &self,
        sample: Point,
    ) -> Result<Option<(usize, Point)>, PathPlannerError> {
        let nearest = self
            .tree
            .nearest(&[sample.x, sample.y], 1, &kt_squared_euclidean)?;
        let Some(&(_, &index)) = nearest.first() else {
            return Ok(None);
        };
        let from = self.nodes[index].point;
        Ok(Some((index, steer(from, sample, self.max_step_size))))
    }

    /// Neighbors inside `neighbor_radius`, with true Euclidean distances.
    ///
    /// The index is queried with squared Euclidean distance because that is the
    /// metric it was given. The distance it reports is therefore a squared
    /// length and is not used.
    fn neighbors_within_radius(&self, point: Point) -> Result<Vec<(usize, f64)>, PathPlannerError> {
        let radius_sq = self.neighbor_radius * self.neighbor_radius;
        let hits = self
            .tree
            .within(&[point.x, point.y], radius_sq, &kt_squared_euclidean)?;
        let mut neighbors = Vec::new();
        for (_, &index) in hits {
            let other = self.nodes[index].point;
            let distance = euclidean(point.x, point.y, other.x, other.y);
            if distance <= self.neighbor_radius + 1e-9 && self.map.segment_free(other, point) {
                neighbors.push((index, distance));
            }
        }
        Ok(neighbors)
    }

    fn choose_parent(
        &self,
        nearest: usize,
        point: Point,
        neighbors: &[(usize, f64)],
    ) -> Option<(usize, f64)> {
        let mut best_index = nearest;
        let mut best_cost = self.nodes[nearest].cost
            + euclidean(
                point.x,
                point.y,
                self.nodes[nearest].point.x,
                self.nodes[nearest].point.y,
            );
        for &(index, distance) in neighbors {
            let cost = self.nodes[index].cost + distance;
            if cost < best_cost {
                best_cost = cost;
                best_index = index;
            }
        }
        Some((best_index, best_cost))
    }

    fn rewire(&mut self, new_index: usize, neighbors: &[(usize, f64)]) {
        let new_cost = self.nodes[new_index].cost;
        let mut adopted = Vec::new();
        for &(index, distance) in neighbors {
            if index == new_index {
                continue;
            }
            let candidate = new_cost + distance;
            if candidate + 1e-12 < self.nodes[index].cost {
                adopted.push((index, candidate));
            }
        }
        for (index, cost) in adopted {
            self.reparent(index, new_index, cost);
        }
    }

    fn reparent(&mut self, index: usize, new_parent: usize, cost: f64) {
        if let Some(old_parent) = self.nodes[index].parent {
            self.children[old_parent].retain(|child| *child != index);
        }
        self.nodes[index].parent = Some(new_parent);
        self.nodes[index].cost = cost;
        self.children[new_parent].push(index);
        self.propagate_costs(index);
    }

    /// Recompute descendant costs from the parent edge lengths.
    ///
    /// Only the children lists are walked, so a rewiring costs time
    /// proportional to the affected subtree. A generation limit stops the walk
    /// if a cycle were ever introduced.
    fn propagate_costs(&mut self, root: usize) {
        let mut queue = VecDeque::new();
        queue.push_back(root);
        let mut visits = 0usize;
        let limit = self.nodes.len().saturating_mul(2).max(1);
        while let Some(index) = queue.pop_front() {
            visits += 1;
            if visits > limit {
                break;
            }
            let child_count = self.children[index].len();
            for child_i in 0..child_count {
                let child = self.children[index][child_i];
                let parent_cost = self.nodes[index].cost;
                let parent_point = self.nodes[index].point;
                let child_point = self.nodes[child].point;
                self.nodes[child].cost = parent_cost
                    + euclidean(parent_point.x, parent_point.y, child_point.x, child_point.y);
                queue.push_back(child);
            }
        }
    }

    fn path_to(&self, mut index: usize) -> Result<Vec<Point>, PathPlannerError> {
        let mut path = Vec::new();
        for _ in 0..=self.nodes.len() {
            path.push(self.nodes[index].point);
            match self.nodes[index].parent {
                Some(parent) => index = parent,
                None => {
                    path.reverse();
                    return Ok(path);
                }
            }
        }
        Err(PathPlannerError::NoPathFound)
    }
}

fn steer(from: Point, to: Point, max_step: f64) -> Point {
    let distance = euclidean(from.x, from.y, to.x, to.y);
    if distance <= max_step || distance < 1e-12 {
        return to;
    }
    let scale = max_step / distance;
    Point::new(
        from.x + (to.x - from.x) * scale,
        from.y + (to.y - from.y) * scale,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn open_map() -> Map {
        Map::new((Point::new(0.0, 0.0), Point::new(40.0, 40.0)))
    }

    fn wall_map() -> Map {
        let mut map = open_map();
        map.set_obstacles(vec![
            Polygon::new(vec![
                Point::new(18.0, 0.0),
                Point::new(20.0, 0.0),
                Point::new(20.0, 30.0),
                Point::new(18.0, 30.0),
            ])
            .unwrap(),
        ]);
        map
    }

    fn plan(map: Map, seed: u64, iterations: usize) -> Vec<Point> {
        let mut planner = RRTStar::new(
            map,
            Point::new(2.0, 20.0),
            |point| point.distance(Point::new(36.0, 20.0)) < 3.0,
            2.0,
            4.0,
            iterations,
        )
        .with_seed(seed);
        planner.plan().unwrap()
    }

    #[test]
    fn seeded_runs_match_and_reach_the_goal() {
        let first = plan(open_map(), 5, 400);
        let second = plan(open_map(), 5, 400);
        assert_eq!(first, second);
        assert!(first.len() >= 2);
        assert!(first.last().unwrap().distance(Point::new(36.0, 20.0)) < 3.0);
        assert_eq!(first[0], Point::new(2.0, 20.0));
    }

    #[test]
    fn path_around_a_wall_does_not_cross_it() {
        let map = wall_map();
        let path = plan(map.clone(), 9, 800);
        for pair in path.windows(2) {
            assert!(
                map.segment_free(pair[0], pair[1]),
                "{:?} -> {:?} crosses the wall",
                pair[0],
                pair[1]
            );
        }
    }

    #[test]
    fn more_iterations_do_not_increase_the_best_cost() {
        let short = polyline_length(&plan(open_map(), 3, 150));
        let long = polyline_length(&plan(open_map(), 3, 600));
        assert!(
            long <= short + 1e-6,
            "longer run cost {long} exceeded short run {short}"
        );
    }

    #[test]
    fn node_cost_matches_the_path_length() {
        let mut planner = RRTStar::new(
            open_map(),
            Point::new(1.0, 1.0),
            |point| point.distance(Point::new(20.0, 20.0)) < 2.0,
            3.0,
            6.0,
            250,
        )
        .with_seed(2);
        let path = planner.plan().unwrap();
        let geometric = polyline_length(&path);
        let goal_cost = planner
            .goals
            .iter()
            .map(|index| planner.nodes[*index].cost)
            .fold(f64::INFINITY, f64::min);
        assert!(
            (goal_cost - geometric).abs() < 1e-6,
            "{goal_cost} vs {geometric}"
        );
    }

    #[test]
    fn radius_query_matches_a_brute_force_scan() {
        let mut planner =
            RRTStar::new(open_map(), Point::new(5.0, 5.0), |_| false, 2.0, 5.0, 0).with_seed(1);
        for _ in 0..40 {
            let sample = planner.random_point();
            let steered = steer(planner.nodes[0].point, sample, 50.0);
            let index = planner.nodes.len();
            planner.nodes.push(Node {
                point: steered,
                cost: 0.0,
                parent: Some(0),
            });
            planner.children.push(Vec::new());
            planner.tree.add([steered.x, steered.y], index).unwrap();
        }
        let query = Point::new(12.0, 12.0);
        let indexed = planner.neighbors_within_radius(query).unwrap();
        let mut brute = Vec::new();
        for (index, node) in planner.nodes.iter().enumerate() {
            let distance = query.distance(node.point);
            if distance <= planner.neighbor_radius + 1e-9 {
                brute.push(index);
            }
        }
        let mut from_index: Vec<_> = indexed.iter().map(|(index, _)| *index).collect();
        from_index.sort_unstable();
        brute.sort_unstable();
        assert_eq!(from_index, brute);
    }

    fn polyline_length(path: &[Point]) -> f64 {
        path.windows(2).map(|pair| pair[0].distance(pair[1])).sum()
    }
}
