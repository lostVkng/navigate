use crate::geometry::{Point, Polygon, euclidean};
use crate::errors::PathPlannerError;
use std::collections::{HashSet, VecDeque};

use kdtree::KdTree; // TODO: move off kdtree - issue with within queries distances
use kdtree::distance::squared_euclidean as kt_squared_euclidean;


/// Temporary map struct - may move to separate module
/// Used to store the space for RRT*
pub struct Map {
    pub obstacles: Vec<Polygon>,
    pub bounds: (Point, Point),
}

impl Map {

    pub fn new(bounds: (Point, Point)) -> Self {
        Self {
            obstacles: vec![],
            bounds: bounds
        }
    }

    /// add obstacles
    pub fn set_obstacles(&mut self, obstacles: Vec<Polygon>) {
        self.obstacles = obstacles;
    }
}


/// Helper node for storing Point data on the tree
#[derive(Clone, Debug, PartialEq)]
struct Node {
    pub point: Point,
    pub cost: f64, // total end to end cost of the path (euclidian distance) - used for rewiring
    pub parent_idx: Option<usize>, // Index of the parent node
}


/// RRT* Algorithm
/// http://roboticsproceedings.org/rss06/p34.pdf
pub struct RRTStar<'a> {
    nodes: Vec<Node>, // visited nodes
    tree: KdTree<f64, usize, [f64; 2]>, // stores point -> index in nodes
    map: Map,
    start_point: Point,
    goal_fn: Box<dyn Fn(&Point) -> bool + 'a>,
    max_step_size: f64,
    neighbor_radius: f64,
    max_iterations: usize,
    goal_node_idxs: HashSet<usize>,
}

impl<'a> RRTStar<'a> {
    
    /// Create a new RRT* instance
    pub fn new<G>(map: Map, start: Point, goal_fn: G, max_step_size: f64, neighbor_radius: f64, max_iterations: usize) -> Self
    where G: Fn(&Point) -> bool + 'a,
    {

        // Add start node to nodes_map
        let mut nodes = Vec::new();
        nodes.push(Node {
            point: start.clone(),
            cost: 0.0,
            parent_idx: None,
        });

        // Tree is used to store points and find nearest neighbors
        let mut tree = KdTree::new(2);
        tree.add([start.x, start.y], 0).unwrap(); // should not fail

        Self {
            nodes,
            tree,
            map,
            start_point: start.clone(),
            goal_fn: Box::new(goal_fn),
            max_step_size,
            neighbor_radius,
            max_iterations,
            goal_node_idxs: HashSet::new(),
        }
    }

    /// Get all visited points
    pub fn get_visited_points(&self) -> Vec<Point> {
        self.nodes.iter().map(|node| node.point.clone()).collect()
    }

    /// Get all edges
    pub fn get_edges(&self) -> Vec<(Point, Point)> {
        self.nodes.iter().map(|node| {
            match node.parent_idx {
                Some(parent_idx) => {
                    (node.point.clone(), self.nodes[parent_idx].point.clone())
                },
                None => (self.start_point.clone(), node.point.clone())
            }
        }).collect()
    }

    /// Get Neighbors nodes based on a point
    /// Filters out any nodes that are obstructed by an obstacle
    /// Returns a vec of (index, euclidean distance, node)
    fn get_neighbors(&self, point: &Point) -> Result<Vec<(usize, f64, Node)>, PathPlannerError> {

        // tree uses squared euclidean distances, so we need to square the radius
        match self.tree.within(&[point.x, point.y], self.neighbor_radius.powi(2), &kt_squared_euclidean) {
            Ok(res) => {
                let mut neighbors: Vec<(usize, f64, Node)> = Vec::new();
                for (_dist, node_idx) in res {

                    let idx = node_idx.clone();
                    let node = self.nodes[idx].clone();

                    // NOTE: kdtree seems to be reporting distances incorrectly, may need to migrate
                    // Calculate the actual Euclidean distance
                    let actual_distance = euclidean(
                        point.x, point.y, 
                        node.point.x, node.point.y
                    );
                    // filter points with distance > max step size
                    if actual_distance > self.max_step_size {
                        continue;
                    }

                    // check that no obstacles obstruct path
                    if !self.is_obstacle_free(&node.point, point) {
                        continue;
                    }

                    // Use the directly calculated Euclidean distance
                    neighbors.push((idx, actual_distance, node));
                }
                Ok(neighbors)
            },
            Err(e) => return Err(PathPlannerError::KdTreeError(e.to_string())),
        }
    }

    /// Get the closest and next point based on max step size and steering
    fn new_point_step(&self, random_point: &Point) -> Result<(Point, Point), PathPlannerError> {
        
        let closest_point: Vec<(f64, &usize)> = self.tree.nearest(&[random_point.x, random_point.y], 1, &kt_squared_euclidean)?;
        if closest_point.len() != 1 {
            return Err(PathPlannerError::NoValidPointFound);
        }
        let closest_node_idx: usize = closest_point[0].1.clone();
        let closest_node: Node = self.nodes[closest_node_idx].clone();

        // steer towards the closest point
        // this is to ensure the point is within step distance of the 'edge' of the tree
        let new_point: Point = self.steer(&closest_node.point, random_point);

        Ok((closest_node.point, new_point))
    }

    /// Get all dependent nodes based on a node
    fn get_dependent_nodes(&self, node_idx: usize) -> Vec<usize> {
        let mut dependent_nodes: Vec<usize> = Vec::new();
        for (idx, node) in self.nodes.iter().enumerate() {
            if node.parent_idx == Some(node_idx) {
                dependent_nodes.push(idx);
            }
        }
        dependent_nodes
    }

    /// Update dependent nodes costs if a dependent node's cost decreases
    fn update_dependent_nodes_costs(&mut self, node_idx: usize) -> Result<(), PathPlannerError> {

        // start with dependent nodes of this node
        let mut nodes_to_update: VecDeque<usize> = VecDeque::from([node_idx]);
        
        while let Some(current_idx) = nodes_to_update.pop_front() {

            // get current node
            let mut current_node = self.nodes[current_idx].clone();

            // Get the parent of the current node
            if let Some(parent_idx) = current_node.parent_idx {
                let parent_node = &self.nodes[parent_idx];
                let new_cost = parent_node.cost + euclidean(
                    current_node.point.x, 
                    current_node.point.y, 
                    parent_node.point.x, 
                    parent_node.point.y
                );
                current_node.cost = new_cost;
            }

            self.nodes[current_idx] = current_node;

            // Get all dependent nodes of the current node
            let dependent_nodes = self.get_dependent_nodes(current_idx);

            for dep_idx in dependent_nodes {

                // add to nodes to update
                nodes_to_update.push_back(dep_idx);
            }           
        }

        Ok(())
    }

    /// Find the path from end node to start node
    fn path_start_to_node(&self, end_node_idx: usize) -> Result<Vec<Point>, PathPlannerError> {
        println!("path_start_to_node");
        println!("end_node_idx: {}", end_node_idx);

        let mut path: Vec<Point> = Vec::new();
        let mut current_node_idx = Some(end_node_idx);
        while current_node_idx.is_some() {
            println!("current_node_idx: {}", current_node_idx.unwrap());
            let current_node = &self.nodes[current_node_idx.unwrap()];
            path.push(current_node.point.clone());
            current_node_idx = current_node.parent_idx;
        }
        path.reverse();
        Ok(path)
    }
    

    /// Plan a path using RRT*
    pub fn plan(&mut self) -> Result<Vec<Point>, PathPlannerError> {

        let mut current_iteration = 0;

        while current_iteration < self.max_iterations {
        
            // sample random point
            let random_point: Point = self.random_point();

            // get closest existing point and new point based on max step size
            let (closest_point, new_point): (Point, Point) = self.new_point_step(&random_point)?;

            // increment iteration
            current_iteration += 1;

            // if the point is in an obstacle, continue
            if !self.is_obstacle_free(&closest_point, &new_point) {
                continue;
            }


            // find all points in the neighborhood based on radius
            let neighbor_nodes: Vec<(usize, f64, Node)> = self.get_neighbors(&new_point)?;

            // loop through neighbors, select node with lowest cost (euclidean distance)
            let mut lowest_cost = f64::INFINITY;
            let mut lowest_cost_node_idx: Option<usize> = None;
            let mut lowest_cost_neighbor_node: Option<Node> = None;
            for (node_idx,distance, node) in &neighbor_nodes {

                let new_cost = distance + node.cost;
                if new_cost < lowest_cost {
                    lowest_cost_node_idx = Some(*node_idx);
                    lowest_cost = new_cost;
                    lowest_cost_neighbor_node = Some(node.clone());
                }
            }

            // if no node, continue
            let Some(lowest_cost_neighbor_node) = lowest_cost_neighbor_node else {
                continue;
            };

            // Create a new node from new point and add it to nodes
            let added_node = Node {
                point: new_point.clone(),
                cost: lowest_cost + euclidean(
                    new_point.x.clone(), 
                    new_point.y.clone(), 
                    lowest_cost_neighbor_node.point.x.clone(), 
                    lowest_cost_neighbor_node.point.y.clone()
                ),
                parent_idx: lowest_cost_node_idx,
            };
            self.nodes.push(added_node.clone());
            self.tree.add([new_point.x.clone(), new_point.y.clone()], self.nodes.len() - 1)?;

            // rewire - neighbor nodes can have new parent if it is a lower cost than current state
            for (node_idx, distance, node) in neighbor_nodes {
                
                let new_cost = added_node.cost + distance;
                
                // Update the node
                if new_cost < node.cost {
                    let mut new_node = node.clone();
                    new_node.cost = new_cost;
                    new_node.parent_idx = Some(self.nodes.len() - 1);

                    // update this node
                    self.nodes[node_idx] = new_node;

                    // update dependent nodes
                    self.update_dependent_nodes_costs(node_idx)?;
                }
            }

            // check if this point is in the goal region - add to goal nodes
            if (self.goal_fn)(&added_node.point) {
                self.goal_node_idxs.insert(self.nodes.len() - 1);
            }
            // print iteration
            println!("{}", current_iteration);
        }

        // Return the shortest path
        if self.goal_node_idxs.is_empty() {
            return Err(PathPlannerError::NoPathFound);
        }

        println!("Goal node idxs: {}", self.goal_node_idxs.len());
        
        // find the goal node with the lowest cost
        let mut lowest_goal = f64::INFINITY;
        let mut lowest_goal_idx: Option<usize> = None;
        for node_idx in &self.goal_node_idxs {
            let node = &self.nodes[*node_idx];
            if node.cost < lowest_goal {
                lowest_goal = node.cost;
                lowest_goal_idx = Some(*node_idx);
            }
        }

        // find the path from the lowest cost node to the start node
        let path = self.path_start_to_node(lowest_goal_idx.unwrap())?;

        Ok(path)
    }

    /// Steer is a core concept of RRT*, it is used to extend the tree towards a target point
    /// while respecting the max step size and obstacle constraints. 
    /// If the distance is less than the max step size, return the end point.
    /// Otherwise, return a point on the line segment between the start and end points
    /// that is at a distance of max_step_size from the start point.
    fn steer(&self, start: &Point, end: &Point) -> Point {

        let distance = euclidean(start.x, start.y, end.x, end.y);

        if distance < self.max_step_size {
            return end.clone();
        } else {
            let direction = Point {
                x: (end.x - start.x) / distance,
                y: (end.y - start.y) / distance,
            };
            let new_point = Point {
                x: start.x + direction.x * self.max_step_size,
                y: start.y + direction.y * self.max_step_size,
            };
            return new_point;
        }
    }

    /// Check if the line to a point is obstacle free
    fn is_obstacle_free(&self, start: &Point, end: &Point) -> bool {

        for obstacle in &self.map.obstacles {
            if obstacle.line_intersects(start, end) {
                return false;
            }
        }

        true
    }

    /// plot random point in the space
    /// returns a random point that is not in an obstacle
    pub fn random_point(&self) -> Point {

        let width = self.map.bounds.1.x - self.map.bounds.0.x;
        let height = self.map.bounds.1.y - self.map.bounds.0.y;

        Point {
            x: rand::random::<f64>() * width + self.map.bounds.0.x,
            y: rand::random::<f64>() * height + self.map.bounds.0.y,
        }
    }
}