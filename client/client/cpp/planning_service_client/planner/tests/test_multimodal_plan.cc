#include <gtest/gtest.h>

#include "planning_service_client/planner/multimodal_plan.h"

namespace planning_service_client {
namespace planner {
TEST(MultimodalPlanningProblem, ToProtoFromProto) {
  SystemConf system_conf_start;
  system_conf_start["robot_1"] = Conf(Eigen::VectorXd::Ones(5));
  system_conf_start["robot_2"] = Conf(Eigen::VectorXd::Ones(5) * 2.0);
  std::vector<FrameRelativePose> start_transform_poses;
  start_transform_poses.push_back(
      FrameRelativePose("frame_A", "frame_B", Eigen::Vector3d(1, 2, 3),
                        Eigen::Quaterniond(1, 0, 0, 0)));
  start_transform_poses.push_back(
      FrameRelativePose("frame_B", "frame_C", Eigen::Vector3d(4, 5, 6),
                        Eigen::Quaterniond(1, 0, 0, 0)));
  SystemConf goal_system_conf;
  goal_system_conf["robot_1"] = Conf(Eigen::VectorXd::Ones(5) * 3.0);
  goal_system_conf["robot_2"] = Conf(Eigen::VectorXd::Ones(5) * 4.0);
  Anchor start(system_conf_start, start_transform_poses);
  Anchor goal(goal_system_conf, std::vector<FrameRelativePose>());
  std::vector<FrameRelativePose> goal_transform_poses;
  goal_transform_poses.push_back(
      FrameRelativePose("frame_A", "frame_B", Eigen::Vector3d(7, 8, 9),
                        Eigen::Quaterniond(1, 0, 0, 0)));
  goal_transform_poses.push_back(
      FrameRelativePose("frame_B", "frame_C", Eigen::Vector3d(10, 11, 12),
                        Eigen::Quaterniond(1, 0, 0, 0)));
  bool allow_async_partial_solutions = true;
  MultimodalPlanningProblem plan(start, goal, start_transform_poses,
                                 goal_transform_poses,
                                 allow_async_partial_solutions);
  auto msg = ToProto(plan);
  auto plan_back = FromProto<MultimodalPlanningProblem>(msg);
  // Check that the start and goal anchors are the same
  EXPECT_TRUE(plan_back.start().system_conf().at("robot_1").q().isApprox(
      system_conf_start["robot_1"].q()));
  EXPECT_TRUE(plan_back.start().system_conf().at("robot_2").q().isApprox(
      system_conf_start["robot_2"].q()));
  EXPECT_TRUE(plan_back.goal().system_conf().at("robot_1").q().isApprox(
      goal_system_conf["robot_1"].q()));
  EXPECT_TRUE(plan_back.goal().system_conf().at("robot_2").q().isApprox(
      goal_system_conf["robot_2"].q()));
  // Check that the start and goal transform poses are the same
  EXPECT_EQ(plan_back.start_transform_poses().size(),
            start_transform_poses.size());
  for (size_t i = 0; i < start_transform_poses.size(); ++i) {
    EXPECT_EQ(plan_back.start_transform_poses()[i].frame_A(),
              start_transform_poses[i].frame_A());
    EXPECT_EQ(plan_back.start_transform_poses()[i].frame_B(),
              start_transform_poses[i].frame_B());
    EXPECT_TRUE(
        plan_back.start_transform_poses()[i].X_AB_translation().isApprox(
            start_transform_poses[i].X_AB_translation()));
    EXPECT_TRUE(
        plan_back.start_transform_poses()[i]
            .X_AB_quaternion()
            .coeffs()
            .isApprox(start_transform_poses[i].X_AB_quaternion().coeffs()));
  }
  EXPECT_EQ(plan_back.goal_transform_poses().size(),
            goal_transform_poses.size());
  EXPECT_EQ(plan_back.allow_async_partial_solutions(),
            allow_async_partial_solutions);
  for (size_t i = 0; i < goal_transform_poses.size(); ++i) {
    EXPECT_EQ(plan_back.goal_transform_poses()[i].frame_A(),
              goal_transform_poses[i].frame_A());
    EXPECT_EQ(plan_back.goal_transform_poses()[i].frame_B(),
              goal_transform_poses[i].frame_B());
    EXPECT_TRUE(plan_back.goal_transform_poses()[i].X_AB_translation().isApprox(
        goal_transform_poses[i].X_AB_translation()));
    EXPECT_TRUE(
        plan_back.goal_transform_poses()[i].X_AB_quaternion().coeffs().isApprox(
            goal_transform_poses[i].X_AB_quaternion().coeffs()));
  }
}
}  // namespace planner
}  // namespace planning_service_client
