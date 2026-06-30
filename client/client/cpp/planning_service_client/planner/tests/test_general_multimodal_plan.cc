#include <gtest/gtest.h>

#include "planning_service_client/planner/general_multimodal_plan.h"

namespace planning_service_client {
namespace planner {

TEST(GeneralizedMultimodalPlanningProblem, ToProtoFromProto) {
  std::vector<Anchor> start_anchors;
  {
    std::vector<FrameRelativePose> start_transform_poses;
    start_transform_poses.push_back(
        FrameRelativePose("frame_A", "frame_B", Eigen::Vector3d(1, 2, 3),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    start_transform_poses.push_back(
        FrameRelativePose("frame_B", "frame_C", Eigen::Vector3d(4, 5, 6),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    start_anchors.emplace_back(SystemConf(), start_transform_poses);
  }
  {
    std::vector<FrameRelativePose> start_transform_poses;
    start_transform_poses.push_back(
        FrameRelativePose("frame_D", "frame_E", Eigen::Vector3d(7, 8, 9),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    start_transform_poses.push_back(
        FrameRelativePose("frame_E", "frame_F", Eigen::Vector3d(10, 11, 12),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    start_anchors.emplace_back(SystemConf(), start_transform_poses);
  }
  std::vector<Anchor> goal_anchors;
  {
    std::vector<FrameRelativePose> goal_transform_poses;
    goal_transform_poses.push_back(
        FrameRelativePose("frame_A", "frame_B", Eigen::Vector3d(13, 14, 15),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    goal_transform_poses.push_back(
        FrameRelativePose("frame_B", "frame_C", Eigen::Vector3d(16, 17, 18),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    goal_anchors.emplace_back(SystemConf(), goal_transform_poses);
  }
  {
    std::vector<FrameRelativePose> goal_transform_poses;
    goal_transform_poses.push_back(
        FrameRelativePose("frame_D", "frame_E", Eigen::Vector3d(19, 20, 21),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    goal_transform_poses.push_back(
        FrameRelativePose("frame_E", "frame_F", Eigen::Vector3d(22, 23, 24),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    goal_anchors.emplace_back(SystemConf(), goal_transform_poses);
  }
  GeneralizedMultimodalPlanningProblem plan(start_anchors, goal_anchors, false);
  auto msg = ToProto(plan);
  auto plan_back = FromProto<GeneralizedMultimodalPlanningProblem>(msg);
  // Check that the start and goal anchors are the same
  EXPECT_EQ(plan_back.start_anchors().size(), start_anchors.size());
  for (size_t i = 0; i < start_anchors.size(); ++i) {
    const auto& anchor_orig = start_anchors[i];
    const auto& anchor_back = plan_back.start_anchors()[i];
    EXPECT_TRUE(anchor_back.system_conf().size()
                == anchor_orig.system_conf().size());
    for (const auto& [key, value] : anchor_orig.system_conf()) {
      EXPECT_TRUE(anchor_back.system_conf().at(key).q().isApprox(value.q()));
    }
    EXPECT_EQ(anchor_back.poses().size(), anchor_orig.poses().size());
    for (size_t j = 0; j < anchor_orig.poses().size(); ++j) {
      EXPECT_EQ(anchor_back.poses()[j].frame_A(),
                anchor_orig.poses()[j].frame_A());
      EXPECT_EQ(anchor_back.poses()[j].frame_B(),
                anchor_orig.poses()[j].frame_B());
      EXPECT_TRUE(anchor_back.poses()[j].X_AB_translation().isApprox(
          anchor_orig.poses()[j].X_AB_translation()));
      EXPECT_TRUE(anchor_back.poses()[j].X_AB_quaternion().coeffs().isApprox(
          anchor_orig.poses()[j].X_AB_quaternion().coeffs()));
    }
  }
  EXPECT_EQ(plan_back.goal_anchors().size(), goal_anchors.size());
  for (size_t i = 0; i < goal_anchors.size(); ++i) {
    const auto& anchor_orig = goal_anchors[i];
    const auto& anchor_back = plan_back.goal_anchors()[i];
    EXPECT_TRUE(anchor_back.system_conf().size()
                == anchor_orig.system_conf().size());
    for (const auto& [key, value] : anchor_orig.system_conf()) {
      EXPECT_TRUE(anchor_back.system_conf().at(key).q().isApprox(value.q()));
    }
    EXPECT_EQ(anchor_back.poses().size(), anchor_orig.poses().size());
    for (size_t j = 0; j < anchor_orig.poses().size(); ++j) {
      EXPECT_EQ(anchor_back.poses()[j].frame_A(),
                anchor_orig.poses()[j].frame_A());
      EXPECT_EQ(anchor_back.poses()[j].frame_B(),
                anchor_orig.poses()[j].frame_B());
      EXPECT_TRUE(anchor_back.poses()[j].X_AB_translation().isApprox(
          anchor_orig.poses()[j].X_AB_translation()));
      EXPECT_TRUE(anchor_back.poses()[j].X_AB_quaternion().coeffs().isApprox(
          anchor_orig.poses()[j].X_AB_quaternion().coeffs()));
    }
  }
  // Check that the fast_estimate_solution option is the same
  EXPECT_EQ(plan_back.fast_estimate_solution(), false);
  // Check that the partial anchor solution flags are preserved
  EXPECT_EQ(plan_back.allow_partial_start_anchor_solutions(), false);
  EXPECT_EQ(plan_back.allow_partial_goal_anchor_solutions(), false);
  EXPECT_EQ(plan_back.allow_async_partial_solutions(), false);
  EXPECT_EQ(plan_back.allow_update_active_arms(), false);
}

TEST(GeneralizedMultimodalPlanningProblem, ToProtoFromProtoWithPartialFlags) {
  std::vector<Anchor> start_anchors;
  {
    std::vector<FrameRelativePose> start_transform_poses;
    start_transform_poses.push_back(
        FrameRelativePose("frame_A", "frame_B", Eigen::Vector3d(1, 2, 3),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    start_anchors.emplace_back(SystemConf(), start_transform_poses);
  }
  std::vector<Anchor> goal_anchors;
  {
    std::vector<FrameRelativePose> goal_transform_poses;
    goal_transform_poses.push_back(
        FrameRelativePose("frame_A", "frame_B", Eigen::Vector3d(13, 14, 15),
                          Eigen::Quaterniond(1, 0, 0, 0)));
    goal_anchors.emplace_back(SystemConf(), goal_transform_poses);
  }
  // Create problem with partial solution flags enabled
  GeneralizedMultimodalPlanningProblem plan(
      start_anchors, goal_anchors, false,
      true,   // allow_partial_start_anchor_solutions
      true,   // allow_partial_goal_anchor_solutions
      true,   // allow_async_partial_solutions
      true);  // allow_update_active_arms

  auto msg = ToProto(plan);
  auto plan_back = FromProto<GeneralizedMultimodalPlanningProblem>(msg);

  // Verify that partial solution flags are correctly preserved
  EXPECT_EQ(plan_back.allow_partial_start_anchor_solutions(), true);
  EXPECT_EQ(plan_back.allow_partial_goal_anchor_solutions(), true);
  EXPECT_EQ(plan_back.allow_async_partial_solutions(), true);
  EXPECT_EQ(plan_back.allow_update_active_arms(), true);
  EXPECT_EQ(plan_back.fast_estimate_solution(), false);

  // Also verify anchors are preserved
  EXPECT_EQ(plan_back.start_anchors().size(), start_anchors.size());
  EXPECT_EQ(plan_back.goal_anchors().size(), goal_anchors.size());
}

}  // namespace planner
}  // namespace planning_service_client
