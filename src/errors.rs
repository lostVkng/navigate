
#[derive(Debug)]
pub enum PathPlannerError {
    NoPathFound, // Unable to find a path to the goal
    GoalFailure // Goal is unreachable
}
