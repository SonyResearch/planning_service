/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/multibody/parsing/scoped_names.h>
#include <gtest/gtest.h>

#include "planning_service/motion/planning/ik_cache.h"

namespace motion {
namespace planning {

TEST(IkCache, CalcClosestSeedFromGlobalConfigs) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  // Generate random global configurations from joint limits
  const auto& plant = robot_model->plant();
  const auto& lb = plant.GetPositionLowerLimits();
  const auto& ub = plant.GetPositionUpperLimits();
  auto rectangle = drake::geometry::optimization::Hyperrectangle(lb, ub);
  drake::RandomGenerator gen(0);
  std::vector<Eigen::VectorXd> global_configs;
  int num_samples = 100;
  for (int i = 0; i < num_samples; ++i) {
    auto sample = rectangle.UniformSample(&gen);
    global_configs.push_back(sample);
  }
  auto ik_cache = IkCache(*robot_model, global_configs);
  // Add frames to the cache
  const auto& frame_world = robot_model->plant().GetFrameByName("world");
  const auto& frame_left_tool =
      robot_model->GetScopedFrameByName("franka_left::franka_tool_location");
  const auto& frame_right_tool =
      robot_model->GetScopedFrameByName("franka_right::franka_tool_location");
  ik_cache.AddFrames(frame_world, frame_left_tool);
  // ik_cache.AddFrames(frame_world, frame_right_tool);
  // Now let's make a pose request with both frames present
  int best_seed_idx = 64;
  auto conf = global_configs[best_seed_idx] + Eigen::VectorXd::Ones(14) * 0.001;
  auto X_world_left_tool =
      robot_model->CalcRelativeTransform(conf, frame_world, frame_left_tool);
  auto X_world_right_tool =
      robot_model->CalcRelativeTransform(conf, frame_world, frame_right_tool);
  // Now let's get them into FrameRelativePoses
  FrameRelativePoses frame_relative_poses;
  frame_relative_poses.emplace_back(&frame_world, &frame_left_tool,
                                    X_world_left_tool);
  frame_relative_poses.emplace_back(&frame_world, &frame_right_tool,
                                    X_world_right_tool);
  // Now let's get the closest seed
  auto q_zero = Eigen::VectorXd::Zero(plant.num_positions());
  auto closest_seeds =
      ik_cache.CalcClosestSeed(frame_relative_poses, q_zero, 1.0, 0.0, 10);
  EXPECT_EQ(closest_seeds.size(), 10);
  // Check that the first seed is the best seed
  EXPECT_TRUE(closest_seeds.front().isApprox(global_configs[best_seed_idx]));
  // Let's make conf the reference conf. The best seed is itself.
  closest_seeds =
      ik_cache.CalcClosestSeed(frame_relative_poses, conf, 1.0, 0.0, 10);
  EXPECT_EQ(closest_seeds.size(), 10);
  // Check that the first seed is the best seed
  EXPECT_TRUE(closest_seeds.front().isApprox(conf));
  // The second one is the best_seed_idx.
  EXPECT_TRUE(closest_seeds[1].isApprox(global_configs[best_seed_idx]));
}

TEST(IkCache, CalcClosestSeedFromGlobalConfigs2Steps) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  // Generate random global configurations from joint limits
  const auto& plant = robot_model->plant();
  const auto& lb = plant.GetPositionLowerLimits();
  const auto& ub = plant.GetPositionUpperLimits();
  auto rectangle = drake::geometry::optimization::Hyperrectangle(lb, ub);
  drake::RandomGenerator gen(0);
  std::vector<Eigen::VectorXd> global_configs;
  int num_samples = 100;
  for (int i = 0; i < num_samples; ++i) {
    auto sample = rectangle.UniformSample(&gen);
    global_configs.push_back(sample);
  }
  auto ik_cache = IkCache(*robot_model, global_configs);
  // Add frames to the cache
  const auto& frame_world = robot_model->plant().GetFrameByName("world");
  const auto& frame_left_tool =
      robot_model->GetScopedFrameByName("franka_left::franka_tool_location");
  const auto& frame_right_tool =
      robot_model->GetScopedFrameByName("franka_right::franka_tool_location");
  ik_cache.AddFrames(frame_world, frame_left_tool);
  // ik_cache.AddFrames(frame_world, frame_right_tool);
  // Now let's make a pose request with both frames present
  int best_seed_idx = 64;
  auto conf = global_configs[best_seed_idx] + Eigen::VectorXd::Ones(14) * 0.001;
  auto X_world_left_tool =
      robot_model->CalcRelativeTransform(conf, frame_world, frame_left_tool);
  auto X_world_right_tool =
      robot_model->CalcRelativeTransform(conf, frame_world, frame_right_tool);
  // Now let's get them into FrameRelativePoses
  FrameRelativePoses frame_relative_poses;
  frame_relative_poses.emplace_back(&frame_world, &frame_left_tool,
                                    X_world_left_tool);
  frame_relative_poses.emplace_back(&frame_world, &frame_right_tool,
                                    X_world_right_tool);
  // Let's make conf the reference conf. The best seed is itself.
  auto closest_seeds =
      ik_cache.CalcClosestSeed(frame_relative_poses, conf, 1.0, 0.0, 10, true);
  EXPECT_EQ(closest_seeds.size(), 10);
  // Check that the first seed is the best seed
  EXPECT_TRUE(closest_seeds.front().isApprox(conf));
  // The second one is the best_seed_idx.
  EXPECT_TRUE(closest_seeds[1].isApprox(global_configs[best_seed_idx]));
}

}  // namespace planning
}  // namespace motion
