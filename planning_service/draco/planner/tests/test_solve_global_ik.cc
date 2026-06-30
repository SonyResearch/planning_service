#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

//  Alias
namespace psc = planning_service_client;
using motion::system_conf_t;

TEST(TestDracoPlanner, SolveGlobalIK) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Define a joint configuration for both arms to use as seed
  psc::SystemConf ik_seed_system_conf;
  system_conf_t ik_seed_sysconf;
  Eigen::VectorXd q_initial_right(7);
  q_initial_right << -0.44111, 0.500755, -0.22664, -1.82248, -1.63484, 2.00563,
      -0.027943;
  Eigen::VectorXd q_initial_left(7);
  q_initial_left << 2.07683, 0.664395, 1.0827, -2.46029, 1.83885, 1.57929,
      0.1637;
  ik_seed_sysconf["franka_right"] = q_initial_right;
  ik_seed_sysconf["franka_left"] = q_initial_left;
  for (const auto& [key, value] : ik_seed_sysconf) {
    ik_seed_system_conf[key] = value;
  }
  // define the left arm as fixed
  psc::SystemConf fixed_partial_sysconf;
  fixed_partial_sysconf["franka_left"] = q_initial_left;
  // Calculate the pose of the right eef for this joint configuration seed
  const std::string right_ik_frame = "franka_right::franka_tool_location";
  auto seed_pose_right =
      planner.CalcRelativePose(ik_seed_sysconf, right_ik_frame);
  // Define the target pose for the right arm in the IK problem
  const Eigen::Vector3d seed_translation_right = seed_pose_right.translation();
  const Eigen::Quaterniond seed_quaternion_right =
      seed_pose_right.rotation().ToQuaternion();
  planning_service_client::FrameRelativePose target_right_pose {
      "world", right_ik_frame, seed_translation_right, seed_quaternion_right};
  // Define the problem
  psc::planner::GlobalIKProblem def {
      {target_right_pose}, ik_seed_system_conf, fixed_partial_sysconf};
  // Solve the problem
  auto motion_plan_result = planner.SolvePlan(def);

  EXPECT_TRUE(motion_plan_result.is_success());

  //  Get the start of the returned trajectory
  auto new_trajectory = motion_plan_result.system_timed_trajectory();
  EXPECT_TRUE(new_trajectory.has_key("franka_right"));
  EXPECT_TRUE(new_trajectory.has_key("franka_left"));

  auto new_time_scaling = new_trajectory.at("franka_right").time_scaling();
  EXPECT_NEAR(new_time_scaling.start_time(), 0.0, 1e-4);

  //  Make sure the start of the trajectory is the same as the start conf
  auto franka_right_path = new_trajectory.at("franka_right").path();
  auto franka_right_start = franka_right_path.Value(0);
  EXPECT_TRUE(franka_right_start.isApprox(q_initial_right));
  auto franka_left_path = new_trajectory.at("franka_left").path();
  auto franka_left_start = franka_left_path.Value(0);
  EXPECT_TRUE(franka_left_start.isApprox(q_initial_left));

  system_conf_t traj_begin_sysconf;
  traj_begin_sysconf["franka_right"] = franka_right_start;
  traj_begin_sysconf["franka_left"] = franka_left_start;
  const auto traj_begin_pose =
      planner.CalcRelativePose(traj_begin_sysconf, right_ik_frame);
  drake::math::RigidTransformd traj_begin_transform {
      target_right_pose.X_AB_quaternion(),
      target_right_pose.X_AB_translation()};
  auto X_to_start = traj_begin_transform.inverse() * traj_begin_pose;
  // The error must be small
  EXPECT_LT(X_to_start.translation().norm(), 1e-3);
  EXPECT_LT(X_to_start.rotation().ToAngleAxis().angle(), 1e-3);

  // Check that the end of the trajectory is the same as the end frp
  system_conf_t traj_end_sysconf;
  traj_end_sysconf["franka_right"] =
      franka_right_path.Value(franka_right_path.end_time());
  traj_end_sysconf["franka_left"] =
      franka_left_path.Value(franka_left_path.end_time());
  const auto traj_end_pose =
      planner.CalcRelativePose(traj_end_sysconf, right_ik_frame);
  drake::math::RigidTransformd right_goal_frp_pose {
      target_right_pose.X_AB_quaternion(),
      target_right_pose.X_AB_translation()};
  auto X_to_goal = right_goal_frp_pose.inverse() * traj_end_pose;
  // The error must be small
  EXPECT_LT(X_to_goal.translation().norm(), 1e-3);
  EXPECT_LT(X_to_goal.rotation().ToAngleAxis().angle(), 1e-3);
}

}  // namespace planner
}  // namespace draco
