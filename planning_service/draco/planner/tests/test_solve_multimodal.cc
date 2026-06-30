#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

using motion::system_conf_t;

TEST(TestDracoPlanner, TestMultimodalPlan) {
  const auto planner = DracoPlanner(test::DualPandas());
  planning_service_client::SystemConf system_conf_start;
  system_conf_t sysconf_start;
  Eigen::VectorXd q_right(7);
  q_right << -0.44, 0.501, -0.227, -1.82, -1.635, 2.005, -0.028;
  Eigen::VectorXd q_left_start(7);
  q_left_start << 2.077, 0.66, 1.08, -2.46, 1.84, 1.58, 0.16;
  Eigen::VectorXd q_left_goal(7);
  q_left_goal << -1.0, 0.66, 1.08, -2.46, 1.84, 1.58, 0.164;
  sysconf_start["franka_right"] = q_right;
  sysconf_start["franka_left"] = q_left_start;
  for (const auto& [key, value] : sysconf_start) {
    system_conf_start[key] = value;
  }
  planning_service_client::SystemConf system_conf_goal;
  system_conf_t sysconf_goal;
  sysconf_goal["franka_right"] = q_right;
  sysconf_goal["franka_left"] = q_left_goal;
  for (const auto& [key, value] : sysconf_goal) {
    system_conf_goal[key] = value;
  }

  const auto& world_frame = planner.robot_model().GetScopedFrameByName("world");
  const std::string left_ik_frame = "franka_left::franka_tool_location";

  // Calculate the pose of the seed for left arm
  auto seed_pose_left_start = planner.robot_model().CalcRelativeTransform(
      planner.robot_model().ToGeneralizedPosition(sysconf_start), world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  const Eigen::Vector3d seed_translation_left_start =
      seed_pose_left_start.translation();
  const Eigen::Quaterniond seed_quaternion_left_start =
      seed_pose_left_start.rotation().ToQuaternion();
  auto seed_pose_left_goal = planner.robot_model().CalcRelativeTransform(
      planner.robot_model().ToGeneralizedPosition(sysconf_goal), world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  const Eigen::Vector3d seed_translation_left_goal =
      seed_pose_left_goal.translation();
  const Eigen::Quaterniond seed_quaternion_left_goal =
      seed_pose_left_goal.rotation().ToQuaternion();

  // Provide a start frp for the left arm
  planning_service_client::FrameRelativePose start_frp_left {
      "world", left_ik_frame, seed_translation_left_start,
      seed_quaternion_left_start};

  const double z_movement = 0.20;  // in meters
  const Eigen::VectorXd relative_translation_left {
      Eigen::Vector3d(0.0, 0.0, z_movement)};
  const Eigen::Quaterniond relative_quaternion_left {
      Eigen::Quaterniond(1, 0, 0, 0)};
  // Define the start anchor
  planning_service_client::planner::Anchor start_anchor {system_conf_start, {}};
  // Define the goal anchor
  planning_service_client::FrameRelativePose goal_frp_left {
      "world", left_ik_frame,
      seed_translation_left_goal + relative_translation_left,
      seed_quaternion_left_goal * relative_quaternion_left};
  planning_service_client::planner::Anchor goal_anchor {
      planning_service_client::SystemConf(), {goal_frp_left}};
  // Define the start transform poses
  std::vector<planning_service_client::FrameRelativePose> start_transform_poses;
  planning_service_client::FrameRelativePose start_transform_pose {
      "world", left_ik_frame, relative_translation_left,
      relative_quaternion_left};
  start_transform_poses.push_back(start_transform_pose);
  // Define the goal transform poses
  std::vector<planning_service_client::FrameRelativePose> goal_transform_poses;
  planning_service_client::FrameRelativePose goal_transform_pose {
      "world", left_ik_frame, relative_translation_left,
      relative_quaternion_left};
  goal_transform_poses.push_back(goal_transform_pose);
  // Define the multimodal plan
  planning_service_client::planner::MultimodalPlanningProblem multimodal_plan {
      start_anchor, goal_anchor, start_transform_poses, goal_transform_poses};
  // Solve the multimodal plan
  auto multimodal_plan_result =
      planner.SolvePlan(multimodal_plan, "", std::nullopt, system_conf_start);
  EXPECT_TRUE(multimodal_plan_result.is_success());
  // Let's check the trajectory
  auto sys_traj = multimodal_plan_result.system_timed_trajectory();
  // Let's inspect the trajectory
  EXPECT_FALSE(sys_traj.has_key("franka_right"));
  EXPECT_TRUE(sys_traj.has_key("franka_left"));
  // Make sure the start of the trajectory is the same as the start conf
  auto traj_left = sys_traj.at("franka_left");
  auto franka_left_start = traj_left.Value(0);
  EXPECT_TRUE(franka_left_start.isApprox(q_left_start));
  // Make sure the end of the trajectory has the same pose as the goal conf
  auto franka_left_end = traj_left.Value(traj_left.end_time());
  system_conf_t franka_left_end_sysconf;
  franka_left_end_sysconf["franka_left"] = franka_left_end;
  // Franka right should not have moved, so we set it to the start conf
  franka_left_end_sysconf["franka_right"] = q_right;
  // Make sure the end of the trajectory has the same pose as the goal conf
  auto evaluated_goal_pose_left = planner.robot_model().CalcRelativeTransform(
      planner.robot_model().ToGeneralizedPosition(franka_left_end_sysconf),
      world_frame, planner.robot_model().GetScopedFrameByName(left_ik_frame));
  drake::math::RigidTransformd left_goal_frp_pose {
      goal_frp_left.X_AB_quaternion(), goal_frp_left.X_AB_translation()};
  auto X_to_goal_ik_end =
      left_goal_frp_pose.inverse() * evaluated_goal_pose_left;
  //   The error must be small
  EXPECT_LT(X_to_goal_ik_end.translation().norm(), 1e-3);
  EXPECT_LT(X_to_goal_ik_end.rotation().ToAngleAxis().angle(), 1e-3);
  //   Define the multimodal plan
  planning_service_client::planner::MultimodalPlanningProblem
      multimodal_plan_start_only {
          start_anchor, goal_anchor, start_transform_poses, {}};
  // Solve the multimodal plan
  auto multimodal_plan_start_only_result = planner.SolvePlan(
      multimodal_plan_start_only, "", std::nullopt, system_conf_start);
  EXPECT_TRUE(multimodal_plan_start_only_result.is_success());
  // Let's check the trajectory
  auto new_trajectory_start_only =
      multimodal_plan_start_only_result.system_timed_trajectory();
  // Let's inspect the trajectory
  EXPECT_FALSE(new_trajectory_start_only.has_key("franka_right"));
  EXPECT_TRUE(new_trajectory_start_only.has_key("franka_left"));
  auto new_time_scaling_start_only =
      new_trajectory_start_only.at("franka_left").time_scaling();
  EXPECT_NEAR(new_time_scaling_start_only.start_time(), 0.0, 1e-4);
  // The value at start must be the same as the start conf
  EXPECT_TRUE(new_trajectory_start_only.at("franka_left")
                  .Value(0)
                  .isApprox(q_left_start));
  // Define the multimodal plan
  planning_service_client::planner::MultimodalPlanningProblem
      multimodal_plan_goal_only {
          start_anchor, goal_anchor, start_transform_poses, {}};
  // Solve the multimodal plan
  auto multimodal_plan_goal_only_result = planner.SolvePlan(
      multimodal_plan_goal_only, "", std::nullopt, system_conf_start);
  EXPECT_TRUE(multimodal_plan_goal_only_result.is_success());
  // Let's check the trajectory
  auto new_trajectory_goal_only =
      multimodal_plan_goal_only_result.system_timed_trajectory();
  // Let's inspect the trajectory
  EXPECT_FALSE(new_trajectory_goal_only.has_key("franka_right"));
  EXPECT_TRUE(new_trajectory_goal_only.has_key("franka_left"));
  auto new_time_scaling_goal_only =
      new_trajectory_goal_only.at("franka_left").time_scaling();
  EXPECT_NEAR(new_time_scaling_goal_only.start_time(), 0.0, 1e-4);
  // The value at start must be the same as the start conf
  EXPECT_TRUE(new_trajectory_goal_only.at("franka_left")
                  .Value(0)
                  .isApprox(q_left_start));
}

TEST(TestDracoPlanner, TestMultimodalPlan_InvalidCartesian) {
  const auto planner = DracoPlanner(test::DualPandas());
  planning_service_client::SystemConf system_conf_start;
  system_conf_t sysconf_start;
  Eigen::VectorXd q_right(7);
  q_right << -0.4, 0.50, -0.2, -1.8, -1.6, 2.0, 0.0;
  Eigen::VectorXd q_left_start(7);
  q_left_start << 2.0, 0.6, 1.1, -2.5, 1.8, 1.6, 0.2;
  Eigen::VectorXd q_left_goal(7);
  q_left_goal << -1.0, 0.6, 1.1, -2.5, 1.8, 1.6, 0.2;
  sysconf_start["franka_right"] = q_right;
  sysconf_start["franka_left"] = q_left_start;
  for (const auto& [key, value] : sysconf_start) {
    system_conf_start[key] = value;
  }
  planning_service_client::SystemConf system_conf_goal;
  system_conf_t sysconf_goal;
  sysconf_goal["franka_right"] = q_right;
  sysconf_goal["franka_left"] = q_left_goal;
  for (const auto& [key, value] : sysconf_goal) {
    system_conf_goal[key] = value;
  }

  const auto& world_frame = planner.robot_model().GetScopedFrameByName("world");
  const std::string left_ik_frame = "franka_left::franka_tool_location";

  // Calculate the pose of the seed for left arm
  auto seed_pose_left_start = planner.robot_model().CalcRelativeTransform(
      planner.robot_model().ToGeneralizedPosition(sysconf_start), world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  const Eigen::Vector3d seed_translation_left_start =
      seed_pose_left_start.translation();
  const Eigen::Quaterniond seed_quaternion_left_start =
      seed_pose_left_start.rotation().ToQuaternion();
  auto seed_pose_left_goal = planner.robot_model().CalcRelativeTransform(
      planner.robot_model().ToGeneralizedPosition(sysconf_goal), world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  const Eigen::Vector3d seed_translation_left_goal =
      seed_pose_left_goal.translation();
  const Eigen::Quaterniond seed_quaternion_left_goal =
      seed_pose_left_goal.rotation().ToQuaternion();

  // Provide a start frp for the left arm
  planning_service_client::FrameRelativePose start_frp_left {
      "world", left_ik_frame, seed_translation_left_start,
      seed_quaternion_left_start};

  const double z_movement = 0.20;      // in meters
  const double z_movement_bad = 0.85;  // in meters, should fail
  const Eigen::VectorXd relative_translation_left {
      Eigen::Vector3d(0.0, 0.0, z_movement)};
  const Eigen::Quaterniond relative_quaternion_left {
      Eigen::Quaterniond(1, 0, 0, 0)};
  const Eigen::VectorXd relative_translation_left_bad {
      Eigen::Vector3d(0.0, 0.0, z_movement_bad)};
  // Define the start anchor
  planning_service_client::planner::Anchor start_anchor {
      planning_service_client::SystemConf(), {start_frp_left}};
  // Define the goal anchor
  planning_service_client::FrameRelativePose goal_frp_left {
      "world", left_ik_frame,
      seed_translation_left_goal + relative_translation_left,
      seed_quaternion_left_goal * relative_quaternion_left};
  planning_service_client::planner::Anchor goal_anchor {
      planning_service_client::SystemConf(), {goal_frp_left}};
  // Define the start transform poses
  planning_service_client::FrameRelativePose start_transform_pose {
      "world", left_ik_frame, relative_translation_left,
      relative_quaternion_left};
  planning_service_client::FrameRelativePose start_transform_pose_bad {
      "world", left_ik_frame, relative_translation_left_bad,
      relative_quaternion_left};
  // Define the goal transform poses
  planning_service_client::FrameRelativePose goal_transform_pose {
      "world", left_ik_frame, relative_translation_left,
      relative_quaternion_left};
  planning_service_client::FrameRelativePose goal_transform_pose_bad {
      "world", left_ik_frame, relative_translation_left_bad,
      relative_quaternion_left};
  // Define the multimodal plan
  planning_service_client::planner::MultimodalPlanningProblem
      multimodal_plan_bad_start {start_anchor,
                                 goal_anchor,
                                 {start_transform_pose_bad},
                                 {goal_transform_pose}};
  // Solve the multimodal plan
  EXPECT_THROW(
      {
        planner.SolvePlan(multimodal_plan_bad_start, "", std::nullopt,
                          system_conf_start);
      },
      std::exception);
  // Define the multimodal plan
  planning_service_client::planner::MultimodalPlanningProblem
      multimodal_plan_bad_goal {start_anchor,
                                goal_anchor,
                                {start_transform_pose},
                                {goal_transform_pose_bad}};
  EXPECT_THROW(
      {
        planner.SolvePlan(multimodal_plan_bad_goal, "", std::nullopt,
                          system_conf_start);
      },
      std::exception);
}

}  // namespace planner
}  // namespace draco
