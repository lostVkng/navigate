
#[derive(Debug)]
pub enum PathPlannerError {
    NoPathFound, // Unable to find a path to the goal
    GoalFailure, // Goal is unreachable
    NoValidPointFound, // Unable to find a valid point in the space
    KdTreeError(String),
    Other(String),
}

#[derive(Debug)]
pub enum GeometryError {
    InvalidPolygon // Polygon is invalid
}


impl From<kdtree::ErrorKind> for PathPlannerError {
    fn from(error: kdtree::ErrorKind) -> Self {
        PathPlannerError::KdTreeError(error.to_string())
    }
}