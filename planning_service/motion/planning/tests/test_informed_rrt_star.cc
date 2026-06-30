/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/motion/planning/informed_rrt_star.h"
namespace motion {
namespace planning {
namespace ompl {

using system_conf_t = std::map<std::string, Eigen::VectorXd>;

TEST(InformedRRTStarPlanner, Plan) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  auto robot_constraints {
      std::make_shared<RobotConstraints>(*robot_model, constraints_adapter)};
  // number of problems to solve
  const size_t n_problems {1};
  drake::RandomGenerator gen {0};
  // generate a vector of valid start samples using
  // robot_constraints->GenerateValidSamples
  const auto q_start_eigen {
      robot_constraints->GenerateSamples(&gen, n_problems)[0]};
  // generate a vector of valid goal samples using
  // robot_constraints->GenerateValidSamples
  const auto q_goal_eigen {
      robot_constraints->GenerateSamples(&gen, n_problems)[0]};

  // Create rrt star planner
  InformedRRTStarPlanner informed_rrt_star_planner(*robot_constraints);
  // solve the planning problem
  const auto solution_vec {
      informed_rrt_star_planner.Solve(q_start_eigen, q_goal_eigen, 5)};
  EXPECT_TRUE(solution_vec.has_value());
}

TEST(InformedRRTStarPlanner, PlanWithMimicJoints) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  auto robot_constraints {
      std::make_shared<RobotConstraints>(*robot_model, constraints_adapter)};
  // number of problems to solve
  const size_t n_problems {1};
  drake::RandomGenerator gen {0};
  // generate a vector of valid start samples using
  // robot_constraints->GenerateValidSamples
  const auto q_start_eigen {
      robot_constraints->GenerateSamples(&gen, n_problems)[0]};  // 8D
  // generate a vector of valid goal samples using
  // robot_constraints->GenerateValidSamples
  const auto q_goal_eigen {
      robot_constraints->GenerateSamples(&gen, n_problems)[0]};  // 8D
  // Create rrt star planner
  InformedRRTStarPlanner informed_rrt_star_planner(*robot_constraints);
  // solve the planning problem
  const auto solution_vec {
      informed_rrt_star_planner.Solve(q_start_eigen, q_goal_eigen, 5)};
  EXPECT_TRUE(solution_vec.has_value());
}

TEST(InformedRRTStarPlanner, TimeOut) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  // Testing projection of a conf on a constrained set
  const std::string plan_adapter_file {
      "planning_service/test_data/plan_hotel_pan_third_6in_000.yaml"};
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};

  const auto n_threads {std::thread::hardware_concurrency()};
  auto robot_constraints {std::make_shared<RobotConstraints>(
      *robot_model, plan_adapter, n_threads)};

  CheckSatisfiedOptions options;
  options.parallel = true;
  options.verbose = false;
  options.num_threads = n_threads;

  system_conf_t q_start;
  system_conf_t q_goal;

  const auto aa_name {"ancillary_arm"};
  const auto franka_name {"franka"};
  const auto singulator_name {"singulator"};

  // Define a valid start/goal planning problem in the current robot model and
  // constraints
  q_start[aa_name] = Eigen::VectorXd::Zero(2);
  q_start[aa_name] << 0.9414, -2.0900;
  q_start[franka_name] = Eigen::VectorXd::Zero(7);
  q_start[franka_name] << -1.2620, 0.9311, 0.2120, -0.7009, -0.1422, 1.4454,
      -0.3013;
  q_start[singulator_name] = Eigen::VectorXd::Zero(1);
  q_start[singulator_name] << -2.7;

  q_goal[aa_name] = Eigen::VectorXd::Zero(2);
  q_goal[aa_name] << -4.11, 1.9742;
  q_goal[franka_name] = Eigen::VectorXd::Zero(7);
  q_goal[franka_name] << -0.6846, 1.4172, -1.5138, -1.5433, 1.9714, 1.1208,
      0.2666;
  q_goal[singulator_name] = Eigen::VectorXd::Zero(1);
  q_goal[singulator_name] << -2.7;

  // Write start and goal as Eigen vectors
  const auto q_start_eigen {robot_model->ToGeneralizedPosition(q_start)};
  const auto q_goal_eigen {robot_model->ToGeneralizedPosition(q_goal)};
  // Create rrt star planner
  InformedRRTStarPlanner informed_rrt_star_planner(*robot_constraints);
  // solve the planning problem
  const auto solution_vec {
      informed_rrt_star_planner.Solve(q_start_eigen, q_goal_eigen, 0.1)};
  EXPECT_FALSE(solution_vec.has_value()) << "Solution found!";
}

TEST(InformedRRTStarPlanner, IncorrectVectorSizes) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  // Testing projection of a conf on a constrained set
  const std::string plan_adapter_file {
      "planning_service/test_data/plan_hotel_pan_third_6in_000.yaml"};
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};

  const auto n_threads {std::thread::hardware_concurrency()};
  auto robot_constraints {std::make_shared<RobotConstraints>(
      *robot_model, plan_adapter, n_threads)};

  CheckSatisfiedOptions options;
  options.parallel = true;
  options.verbose = false;
  options.num_threads = n_threads;

  // get num_positions() from plant()
  const auto num_positions {robot_model->plant().num_positions()};

  // define a start of the correct size, and a goal of a different size, with
  // arbitrary entries
  Eigen::VectorXd q_start_eigen = Eigen::VectorXd::Zero(num_positions);
  Eigen::VectorXd q_goal_eigen = Eigen::VectorXd::Zero(num_positions + 1);

  // Create rrt star planner
  InformedRRTStarPlanner informed_rrt_star_planner(*robot_constraints);
  // solve the planning problem and expect throw
  EXPECT_THROW(informed_rrt_star_planner.Solve(q_start_eigen, q_goal_eigen),
               std::runtime_error);
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
