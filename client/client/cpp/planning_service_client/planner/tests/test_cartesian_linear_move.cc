#include <gtest/gtest.h>

#include "planning_service_client/planner/cartesian_linear_move_problem.h"

namespace planning_service_client {
namespace planner {

TEST(CartesianLinearMoveProblem, ToProtoFromProto) {
  planning_service_client::FrameRelativePose waypoint_1_pose_1(
      "frame_A", "frame_B", Eigen::Vector3d(1.0, 2.0, 3.0),
      Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0));
  planning_service_client::FrameRelativePose waypoint_1_pose_2(
      "frame_C", "frame_D", Eigen::Vector3d(4.0, 5.0, 6.0),
      Eigen::Quaterniond(0.0, 0.0, 1.0, 0.0));

  planning_service_client::FrameRelativePose waypoint_2_pose_1(
      "frame_A", "frame_B", Eigen::Vector3d(7.0, 8.0, 9.0),
      Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0));
  planning_service_client::FrameRelativePose waypoint_2_pose_2(
      "frame_C", "frame_D", Eigen::Vector3d(10.0, 11.0, 12.0),
      Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));

  Anchor waypoint_anchor(SystemConf(), {waypoint_1_pose_1, waypoint_1_pose_2});

  bool allow_async_partial_solutions = true;
  planning_service_client::planner::CartesianLinearMoveProblem plan(
      waypoint_anchor, {waypoint_2_pose_1, waypoint_2_pose_2},
      allow_async_partial_solutions);

  auto msg = ToProto(plan);

  auto plan_back =
      FromProto<planning_service_client::planner::CartesianLinearMoveProblem>(
          msg);

  EXPECT_TRUE(plan_back.start().poses().size() == 2);
  EXPECT_TRUE(plan_back.start().poses()[0].frame_A()
              == waypoint_1_pose_1.frame_A());
  EXPECT_TRUE(plan_back.start().poses()[0].frame_B()
              == waypoint_1_pose_1.frame_B());
  EXPECT_TRUE(plan_back.start().poses()[0].X_AB_translation().isApprox(
      waypoint_1_pose_1.X_AB_translation()));
  EXPECT_TRUE(plan_back.start().poses()[0].X_AB_quaternion().coeffs().isApprox(
      waypoint_1_pose_1.X_AB_quaternion().coeffs()));
  EXPECT_TRUE(plan_back.start().poses()[1].frame_A()
              == waypoint_1_pose_2.frame_A());
  EXPECT_TRUE(plan_back.start().poses()[1].frame_B()
              == waypoint_1_pose_2.frame_B());
  EXPECT_TRUE(plan_back.start().poses()[1].X_AB_translation().isApprox(
      waypoint_1_pose_2.X_AB_translation()));
  EXPECT_TRUE(plan_back.start().poses()[1].X_AB_quaternion().coeffs().isApprox(
      waypoint_1_pose_2.X_AB_quaternion().coeffs()));
  EXPECT_EQ(plan_back.allow_async_partial_solutions(),
            allow_async_partial_solutions);
}
}  // namespace planner
}  // namespace planning_service_client
