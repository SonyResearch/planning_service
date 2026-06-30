#include "draco_planner.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

namespace {
// Find waypoints in the nullspace of the IK solution, starting from q. First,
// all joints are moved in the specified direction (1 or -1) by a small step.
// Then it finds the direction in the nullspace by moving in the specified
// direction (1 or -1) and then keeps moving in that direction until no more IK
// solutions can be found. The waypoints are spaced by max_distance, and the
// process stops when the jump between two consecutive waypoints is less than
// max_distance * stop_ratio.
std::vector<Eigen::VectorXd> calc_nullspace_waypoints(
    const motion::planning::IkPlanner& ik_planner, const Eigen::VectorXd& q,
    const drake::multibody::Frame<double>& frame_A,
    const drake::multibody::Frame<double>& frame_B,
    const drake::math::RigidTransform<double>& X_AB, int direction = 1,
    double max_distance = 0.02, double stop_ratio = 0.5) {
  Eigen::VectorXd initial_q_direction =
      direction * Eigen::VectorXd::Ones(q.size());
  DRAKE_THROW_UNLESS(max_distance > 0);
  DRAKE_THROW_UNLESS((stop_ratio > 0) && (stop_ratio < 1.0));
  std::vector<Eigen::VectorXd> waypoints;
  motion::planning::IkPlannerOptions ik_options;
  ik_options.fix_idle_joints = true;
  ik_options.resolve_with_collision_avoidance = false;
  ik_options.ignore_multi_arm_collision = false;
  ik_options.fix_idle_joints = true;
  std::set<drake::multibody::ModelInstanceIndex> active_models;
  active_models.insert(frame_A.model_instance());
  active_models.insert(frame_B.model_instance());
  ik_planner.robot_constraints().robot_model().SetIdleModelsConfigToRef(
      &initial_q_direction, Eigen::VectorXd::Zero(q.size()), active_models);
  // Find the right direction
  auto ik_result =
      ik_planner.SolveIk(frame_A, frame_B, X_AB,
                         q + initial_q_direction.normalized(), 0, ik_options);
  if (!ik_result.is_valid()) {
    throw std::runtime_error(
        "Failed to find initial IK solution to find direction in nullspace.");
  }
  Eigen::VectorXd q_direction = (ik_result.value() - q).normalized();
  const double tol = max_distance * stop_ratio;
  Eigen::VectorXd q_next {q};
  waypoints.push_back(q);
  while (true) {
    const auto q_now = q_next;
    q_next = q_now + max_distance * q_direction;
    auto ik_result =
        ik_planner.SolveIk(frame_A, frame_B, X_AB, q_next, 0, ik_options);
    if (!ik_result.is_valid()) {
      break;
    }
    const auto q_diff = ik_result.value() - q_now;
    if (q_diff.norm() < tol) {
      logging::log()->info("Stopping, small jump: {}", q_diff.norm());
      break;
    }
    q_direction = q_diff.normalized();
    q_next = ik_result.value();
    waypoints.push_back(q_next);
  }
  return waypoints;
}
}  // namespace

SolvePlanResult DracoPlanner::SolveFixedFramesMotionPlan(
    const planning_service_client::planner::FixedFramesMotion& def,
    const planning_service_client::SystemConf& start_sysconf) const {
  const auto q =
      conversions::ToGeneralizedPosition(robot_model(), start_sysconf);
  // Get the frames
  const auto& frame_A = robot_model().GetScopedFrameByName(def.frame_A());
  const auto& frame_B = robot_model().GetScopedFrameByName(def.frame_B());
  // Get the current pose of frame B relative to frame A
  auto X_AB_current = robot_model().CalcRelativeTransform(q, frame_A, frame_B);
  double step = 0.05;  // .05 radians. We can later let user specify this.
  auto waypoints_forward = calc_nullspace_waypoints(
      ik_planner(), q, frame_A, frame_B, X_AB_current, 1, step);
  logging::log()->info("Number of waypoints forward = {}",
                       waypoints_forward.size());
  // Do the same in the opposite direction
  auto waypoints_backward = calc_nullspace_waypoints(
      ik_planner(), q, frame_A, frame_B, X_AB_current, -1, step);
  logging::log()->info("Number of waypoints backward = {}",
                       waypoints_backward.size());
  // Combine the waypoints: first go forward, then reverse, then the current
  // position, then backward, then reverse
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.insert(waypoints.end(), waypoints_forward.begin(),
                   waypoints_forward.end());
  std::reverse(waypoints_forward.begin(), waypoints_forward.end());
  waypoints.insert(waypoints.end(), waypoints_forward.begin(),
                   waypoints_forward.end());
  waypoints.push_back(q);
  waypoints.insert(waypoints.end(), waypoints_backward.begin(),
                   waypoints_backward.end());
  std::reverse(waypoints_backward.begin(), waypoints_backward.end());
  waypoints.insert(waypoints.end(), waypoints_backward.begin(),
                   waypoints_backward.end());
  waypoints.push_back(q);
  // Print the jump between two consecutive waypoints
  for (size_t i = 1; i < waypoints.size(); ++i) {
    const auto jump = (waypoints[i] - waypoints[i - 1]).norm();
    if (std::abs(jump - step) > step * 0.5 && jump > 1e-4) {
      logging::log()->warn(
          "Jump between waypoint {}/{} and {}/{}: {}. Expected around {}",
          i - 1, waypoints.size(), i, waypoints.size(), jump, step);
    }
  }
  // Let's spline the waypoints
  auto spline_time_opt = SplineAndTimePath(waypoints);
  if (!spline_time_opt.has_value()) {
    auto msg = fmt::format(
        "DracoPlanner:SolveFixedFramesMotionPlan: Could not spline the "
        "waypoints: {}",
        spline_time_opt.error());
    logging::log()->error("DracoPlanner:SolveFixedFramesMotionPlan: {}", msg);
    throw std::runtime_error(msg);
  }
  auto& [spline, timing] = spline_time_opt.value();
  // Slow it down by a factor of 5. We can later let user specify this.
  timing.ScaleTime(5.0);
  return std::make_pair(spline, timing);
}

}  // namespace planner
}  // namespace draco
