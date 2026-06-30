/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/motion/planning/validity_checker.h"

namespace motion {
namespace planning {
namespace ompl {

TEST(TestMVC, isValid) {
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
  logging::log()->info("dut_string: \n{}", dut_string);
  const auto n_threads {std::thread::hardware_concurrency()};
  auto robot_constraints = RobotConstraints(*robot_model, dut, n_threads);
  // create multibody_robot_state_space from robot_constraints
  const auto& mrs {
      std::make_shared<RobotStateSpace>(robot_constraints.robot_model())};
  // setup
  mrs->setup();
  // create SpaceInformation from mrs
  const auto& si {std::make_shared<ob::SpaceInformation>(mrs)};
  // create multibody_validity_checker from robot_constraints
  const auto& mvc {std::make_shared<ValidityChecker>(si, robot_constraints)};
  si->setStateValidityChecker(mvc);
  si->setup();

  // get space information from mvc and print out dimension
  ASSERT_EQ(mrs->getDimension(), 10);

  Eigen::VectorXd q {Eigen::VectorXd::Zero(10)};
  std::vector<Eigen::VectorXd> q_vec {};
  const double step {0.001};
  for (int i = 0; i < 100; ++i) {
    q(0) = 2.5 + step * i;
    q(2) = 1.0 + step * i;
    q(3) = -1.0 + step * i;
    q(5) = 1.5 + step * i;
    q(7) = 0.0;
    q(8) = 0.0;
    q(9) = -3.5;
    q_vec.push_back(q);
  }
  for (const auto& q : q_vec) {
    EXPECT_TRUE(mvc->isValid(q));
  }
  // create a config that is invalid
  q(0) = 2.8;
  q(2) = 2.55;
  q(3) = -3.3;
  q(5) = 3.2;
  q(7) = 2.12;
  q(8) = 2.5;
  q(9) = -3.5;
  EXPECT_FALSE(mvc->isValid(q));
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
