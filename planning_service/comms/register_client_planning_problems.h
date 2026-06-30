#include "planning_service_client/planner/cartesian_linear_move_problem.h"
#include "planning_service_client/planner/fixed_frames_motion.h"
#include "planning_service_client/planner/general_multimodal_plan.h"
#include "planning_service_client/planner/global_ik_problem.h"
#include "planning_service_client/planner/multimodal_plan.h"
#include "planning_service_client/planner/out_of_violation.h"
#include "planning_service_client/planner/planning_problem_registry.h"
#include "planning_service_client/planner/start_to_goal_plan.h"
#include "planning_service_client/planner/update_traj_toward_waypoints.h"

namespace planning_service_client {
namespace planner {

void RegisterAllPlanningProblems();

}  // namespace planner
}  // namespace planning_service_client
