#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

using motion::system_conf_t;

TEST(TestDracoPlanner, SolvePlanCartesianLinear) {
  const auto planner = DracoPlanner(test::DualPandas());
  planning_service_client::SystemConf system_conf;
  system_conf_t sysconf;
  Eigen::VectorXd q_initial_right(7);
  q_initial_right << -0.4, 0.5, -0.2, -1.8, -1.6, 2.0, 0.0;
  Eigen::VectorXd q_initial_left(7);
  q_initial_left << 2.1, 0.7, 1.1, -2.5, 1.8, 1.6, 0.2;
  sysconf["franka_right"] = q_initial_right;
  sysconf["franka_left"] = q_initial_left;
  for (const auto& [key, value] : sysconf) {
    system_conf[key] = value;
  }
  planning_service_client::SystemConf fixed_partial_sysconf;
  fixed_partial_sysconf["franka_left"] = q_initial_left;
  const std::string right_ik_frame = "franka_right::franka_tool_location";
  // Calculate the pose of the seed for both the right and left arm
  auto seed_pose_right = planner.CalcRelativePose(sysconf, right_ik_frame);
  const Eigen::Vector3d seed_translation_right = seed_pose_right.translation();
  const Eigen::Quaterniond seed_quaternion_right =
      seed_pose_right.rotation().ToQuaternion();

  // Provide a start frp for the left arm
  planning_service_client::FrameRelativePose start_frp_right {
      "world", right_ik_frame, seed_translation_right, seed_quaternion_right};
  // Define the waypoint frps
  const double cartesian_step_length = 0.02;  // 2 cm
  planning_service_client::FrameRelativePose waypoint_final {
      "world", right_ik_frame,
      seed_translation_right
          + Eigen::Vector3d(cartesian_step_length, cartesian_step_length,
                            2 * cartesian_step_length),
      seed_quaternion_right};
  // The start anchor is defined with full sysconf
  planning_service_client::planner::Anchor start_anchor {system_conf, {}};
  // Resolve the start anchor
  auto start_conf_opt = planner.ResolveAnchorConf(start_anchor);
  // Expect the resolved start anchor to be valid
  EXPECT_TRUE(start_conf_opt.has_value())
      << "Failed to resolve the goal anchor";
  // Define the cartesian linear move plan
  planning_service_client::planner::CartesianLinearMoveProblem def {
      start_anchor, {waypoint_final}};
  auto result = planner.SolvePlan(def, "", std::nullopt, system_conf);
  // Expect the result to be valid
  EXPECT_TRUE(result.is_success())
      << "The cartesian linear move plan should be successful";
  // Get the start of the returned trajectory
  auto new_trajectory = result.system_timed_trajectory();
  EXPECT_TRUE(new_trajectory.has_key("franka_right"));
  EXPECT_FALSE(new_trajectory.has_key("franka_left"));
  auto time_scaling = new_trajectory.at("franka_right").time_scaling();
  EXPECT_NEAR(time_scaling.start_time(), 0.0, 1e-4);
  // Make sure the start of the trajectory is the same as the start conf
  auto franka_right_path = new_trajectory.at("franka_right").path();
  auto franka_right_start = franka_right_path.Value(0);
  logging::log()->critical("Franka right start: {}, initial: {}",
                           franka_right_start.transpose(),
                           q_initial_right.transpose());
  EXPECT_TRUE(franka_right_start.isApprox(q_initial_right));
  system_conf_t traj_begin_sysconf;
  traj_begin_sysconf["franka_right"] = franka_right_start;
  // The left arm should be at its initial configuration
  traj_begin_sysconf["franka_left"] = q_initial_left;
  const auto traj_begin_pose =
      planner.CalcRelativePose(traj_begin_sysconf, right_ik_frame);
  drake::math::RigidTransformd traj_begin_transform {
      start_frp_right.X_AB_quaternion(), start_frp_right.X_AB_translation()};
  auto X_to_start = traj_begin_transform.inverse() * traj_begin_pose;
  // The error must be small
  EXPECT_LT(X_to_start.translation().norm(), 1e-3);
  EXPECT_LT(X_to_start.rotation().ToAngleAxis().angle(), 1e-3);
  // Check that the end of the trajectory is the same as the end frp
  system_conf_t traj_end_sysconf;
  traj_end_sysconf["franka_right"] =
      franka_right_path.Value(franka_right_path.end_time());
  traj_end_sysconf["franka_left"] = q_initial_left;
  const auto traj_end_pose =
      planner.CalcRelativePose(traj_end_sysconf, right_ik_frame);
  drake::math::RigidTransformd right_goal_frp_pose {
      waypoint_final.X_AB_quaternion(), waypoint_final.X_AB_translation()};
  auto X_to_goal = right_goal_frp_pose.inverse() * traj_end_pose;
  // The error must be small
  EXPECT_LT(X_to_goal.translation().norm(), 1e-3);
  EXPECT_LT(X_to_goal.rotation().ToAngleAxis().angle(), 1e-3);
}

}  // namespace planner
}  // namespace draco
