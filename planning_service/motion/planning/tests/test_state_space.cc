/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/motion/planning/state_space.h"

namespace motion {
namespace planning {
namespace ompl {

TEST(TestRobotStateSpace, Constructor) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  ConstraintsAdapter dut;
  dut.plan_name = "test";
  dut.collision_checker = CollisionCheckerAdapter {};
  const auto dut_string {drake::yaml::SaveYamlString(dut)};
  // CI only has 1 core. But the test will pass anyways
  const int n_threads {1};
  auto robot_constraints {RobotConstraints(*robot_model, dut, n_threads)};
  RobotStateSpace space {RobotStateSpace(robot_constraints.robot_model())};
  // make sure we have the expected space dimension
  EXPECT_EQ(space.getDimension(), 10);
}

TEST(TestRobotStateSpace, ArmWithGripper) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  ConstraintsAdapter dut;
  dut.plan_name = "test";
  dut.collision_checker = CollisionCheckerAdapter {};
  const auto dut_string {drake::yaml::SaveYamlString(dut)};
  // CI only has 1 core. But the test will pass anyways
  const int n_threads {1};
  auto robot_constraints {RobotConstraints(*robot_model, dut, n_threads)};
  RobotStateSpace space {RobotStateSpace(robot_constraints.robot_model())};
  // make sure we have the expected space dimension
  EXPECT_EQ(space.getDimension(), 8);
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
