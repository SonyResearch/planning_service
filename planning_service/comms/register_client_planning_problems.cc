#include "register_client_planning_problems.h"

namespace planning_service_client {
namespace planner {

CLIENT_REGISTER_PLANNING_PROBLEM(
    StartToGoalProblem, proto::MotionProblemDefinition::kStartToGoalProblem,
    start_to_goal_problem);

CLIENT_REGISTER_PLANNING_PROBLEM(
    UpdateTrajTowardWaypointsProblem,
    proto::MotionProblemDefinition::kUpdateTrajTowardWaypointsProblem,
    update_traj_toward_waypoints_problem);

CLIENT_REGISTER_PLANNING_PROBLEM(
    MultimodalPlanningProblem,
    proto::MotionProblemDefinition::kMultimodalPlanningProblem,
    multimodal_planning_problem);

CLIENT_REGISTER_PLANNING_PROBLEM(
    OutOfViolation, proto::MotionProblemDefinition::kOutOfViolationProblem,
    out_of_violation_problem);

CLIENT_REGISTER_PLANNING_PROBLEM(
    CartesianLinearMoveProblem,
    proto::MotionProblemDefinition::kCartesianLinearMoveProblem,
    cartesian_linear_move_problem);

CLIENT_REGISTER_PLANNING_PROBLEM(
    GlobalIKProblem, proto::MotionProblemDefinition::kGlobalIkProblem,
    global_ik_problem);

CLIENT_REGISTER_PLANNING_PROBLEM(
    GeneralizedMultimodalPlanningProblem,
    proto::MotionProblemDefinition::kGeneralizedMultimodalPlanningProblem,
    generalized_multimodal_planning_problem);

CLIENT_REGISTER_PLANNING_PROBLEM(
    FixedFramesMotion,
    proto::MotionProblemDefinition::kFixedFramesMotionProblem,
    fixed_frames_motion_problem);

void RegisterAllPlanningProblems() {
  // Touch the registration symbol to force this TU to be linked
  (void)_registered_StartToGoalProblem;
}

}  // namespace planner
}  // namespace planning_service_client
