/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>

#include "planning_service/motion/robot_constraints.h"

namespace motion {

namespace {
const std::string kCobotName {"franka"};
const std::string kAncillaryArmName {"ancillary_arm"};
const std::string kSingulatorName {"singulator"};
}  // namespace

using CollisionType = RobotConstraints::CollisionType;

TEST(TestConstraintGeneration, ScoopCompositeConstraints) {
  // Make and save scoop composite constraints
  CompositeConstraintsAdapter composite_constraints;
  composite_constraints.composite_plan_name = "scoop_hotel_pan_third_6in_000";
  // Approach to get stuff
  ConstraintsAdapter plan_1;
  plan_1.plan_name = "view_conf_to_getstuff_approach";
  plan_1.collision_checker = CollisionCheckerAdapter {};
  PositionConstraintAdapter column_and_height_disher_2oz;
  column_and_height_disher_2oz.frame_A = "hotel_pan_third_6in_000";
  column_and_height_disher_2oz.frame_B = "disher_2oz";
  const Eigen::Vector3d container_size {0.33, 0.53, 0.152};
  const double margin {0.015};
  const double height {0.5};  // 50 cm
  column_and_height_disher_2oz.position_AQ_lower = Eigen::Vector3d(
      -container_size(0) + margin, -container_size(1) + margin, 0);
  column_and_height_disher_2oz.position_AQ_upper = Eigen::Vector3d(
      container_size(0) - margin, container_size(1) - margin, height);
  column_and_height_disher_2oz.position_BQ = Eigen::Vector3d(0, 0, 0);
  PositionConstraintAdapter column_and_height_disher_2oz_tip {
      column_and_height_disher_2oz};
  column_and_height_disher_2oz_tip.frame_B = "disher_2oz_tip";
  plan_1.position_constraints = {column_and_height_disher_2oz,
                                 column_and_height_disher_2oz_tip};
  // Getstuff approach to getstuff finish
  ConstraintsAdapter plan_2;
  plan_2.plan_name = "getstuff_approach_to_getstuff_finish";
  plan_2.collision_checker = CollisionCheckerAdapter {};
  plan_2.position_constraints = plan_1.position_constraints;
  // Getstuff finish to dispense start
  ConstraintsAdapter plan_3;
  plan_3.plan_name = "getstuff_finish_to_dispense_start";
  plan_3.collision_checker = CollisionCheckerAdapter {};
  plan_3.position_constraints = plan_1.position_constraints;
  AngleBetweenVectorsConstraintAdapter angles_disher_2oz;
  angles_disher_2oz.frame_A = "world";
  angles_disher_2oz.frame_B = "disher_2oz_face";
  angles_disher_2oz.angle_lower = -0.1;  // ~6 degrees
  angles_disher_2oz.angle_upper = -0.1;  // ~6 degrees
  angles_disher_2oz.a_A = Eigen::Vector3d(0, 0, 1);
  angles_disher_2oz.b_B = Eigen::Vector3d(0, 1, 0);
  plan_3.angle_constraints = {angles_disher_2oz};
  // Dispense start to dispense finish
  ConstraintsAdapter plan_4;
  plan_4.plan_name = "dispense_start_to_dispense_finish";
  plan_4.collision_checker = CollisionCheckerAdapter {};
  PositionConstraintAdapter disher_above_bowl;
  disher_above_bowl.frame_A = "bowl_center_location";
  disher_above_bowl.frame_B = "disher_2oz_face";
  const double xy_margin {0.03};
  const double z_margin {0.2};
  const Eigen::Vector3d threshold {xy_margin, xy_margin, z_margin};
  disher_above_bowl.position_AQ_lower = -threshold;
  disher_above_bowl.position_AQ_upper = threshold;
  disher_above_bowl.position_BQ = Eigen::Vector3d(0, 0, 0.0);
  // Now add all of these constraints to the composite
  composite_constraints.constraints_vec = {plan_1, plan_2, plan_3, plan_4};
  // Now serialize the composite plan constraints
  logging::log()->info("composite_constraints: \n{}",
                       drake::yaml::SaveYamlString(composite_constraints));
}

TEST(TestRobotConstraints, CheckSatisfied) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter dut;
  dut.plan_name = "test";
  dut.collision_checker = CollisionCheckerAdapter {};
  const auto dut_string {drake::yaml::SaveYamlString(dut)};
  logging::log()->info("dut_string: \n{}", dut_string);
  // CI only has 1 core. But the test will pass anyways
  const int n_threads {16};
  auto robot_constraints {RobotConstraints(*robot_model, dut, n_threads)};
  CheckSatisfiedOptions options;
  options.parallel = false;
  options.verbose = true;
  options.num_threads = n_threads;
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
  EXPECT_TRUE(robot_constraints.CheckSatisfied(q_vec, options));
  options.parallel = true;
  EXPECT_TRUE(robot_constraints.CheckSatisfied(q_vec, options));
  // Check the same thing with edge
  const auto& q1 {q_vec.front()};
  const auto& q2 {q_vec.back()};
  EXPECT_TRUE(robot_constraints.CheckSatisfiedEdge(q1, q2, step, options));
  EXPECT_TRUE(robot_constraints.CheckSatisfiedEdge(q2, q1, step, options));
}

TEST(TestRobotConstraints, CheckSatisfiedMultiThread) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter dut;
  dut.plan_name = "test";
  dut.collision_checker = CollisionCheckerAdapter {};
  const int n_threads {16};
  auto robot_constraints {RobotConstraints(*robot_model, dut, n_threads)};
  CheckSatisfiedOptions options;
  options.parallel = false;
  options.verbose = true;
  options.num_threads = n_threads;
  Eigen::VectorXd q {Eigen::VectorXd::Zero(10)};
  std::vector<Eigen::VectorXd> q_vec {};
  const double step {0.001};
  for (int i = 0; i < 100; ++i) {
    q(0) = 2.5 + step * i;
    q(2) = 1.0 + step * i;
    q(3) = -1.0 + step * i;
    q(5) = 1.5 + step * i;
    q(7) = -4.28;
    q(8) = 2.0;
    q(9) = -3.5;
    q_vec.push_back(q);
  }
  // now we want to run check satisfy asynchronously on the same thread
  std::vector<std::thread> threads;
  int num_jobs = std::ssize(q_vec);
  threads.reserve(num_jobs);
  for (int i = 0; i < num_jobs; ++i) {
    threads.emplace_back(
        [&robot_constraints, &q_vec, &options, i, n_threads]() {
          const auto& q {q_vec[i]};
          EXPECT_TRUE(robot_constraints.CheckSatisfied(q, 0, options));
        });
  }
  for (auto& thread : threads) {
    thread.join();
  }
}

TEST(TestRobotConstraints, TestJointPositionsBoxConstraintAdapter) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.joint_position_box_constraints = {
      {kAncillaryArmName, Eigen::Vector2d {-1.0, 1.0},
       Eigen::Vector2d {0.0, 1.5}}};
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  Eigen::VectorXd q {Eigen::VectorXd::Zero(10)};
  q(7) = -0.5;
  q(8) = 1.2;
  // only franka joint with an upper bound < 0
  q(3) = -0.5;
  // only franka joint with a lower bound > 0
  q(5) = 1.2;
  EXPECT_TRUE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  // change q(8) to be out of bounds
  q(8) = 1.6;
  EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  const auto projected_q_opt {
      robot_constraints.ProjectConfToNonCollisionConstraints(q, 0, q, 1.0)};
  EXPECT_TRUE(projected_q_opt.has_value());
  const auto projected_q {projected_q_opt.value()};
  logging::log()->info("q: {}, projected_q: {}", q.transpose(),
                       projected_q.transpose());
  EXPECT_TRUE(robot_constraints.CheckSatisfied(projected_q, 0,
                                               check_satisfied_options));
  // Check hash
  const drake::DefaultHash hash_func;
  const auto hash_constraints = hash_func(robot_constraints);
  // now change the constraints and check the hash
  constraints_adapter.joint_position_box_constraints = {
      {kAncillaryArmName, Eigen::Vector2d {-1.1, 1.0},
       Eigen::Vector2d {0.0, 1.5}}};
  const auto robot_constraints_new {
      RobotConstraints(*robot_model, constraints_adapter)};
  const auto hash_constraints_new = hash_func(robot_constraints_new);
  EXPECT_NE(hash_constraints, hash_constraints_new);
}

TEST(TestRobotConstraints, CalcPenaltyJointPosition) {
  // test with joint position box constraints
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.joint_position_box_constraints = {
      {"robot", Eigen::Vector2d {-1.0, -1.0}, Eigen::Vector2d {1.0, 1.0}}};
  const double zero_penalty {1.0};
  const double gamma {0.5};
  const double influence {0.1};
  MinimumValuePenaltyParams params {
      .m = zero_penalty, .gamma = gamma, .x0 = influence};
  constraints_adapter.joint_position_box_constraints.value()
      .front()
      .minimum_value_penalty_params = params;
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  // no constraint nearly violated, zero penalty
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  // Reminder that joint limits are [-7, 0.2] to [7, 0.4]
  const auto q_1 {Eigen::Vector2d {0.0, 0.3}};
  EXPECT_NEAR(robot_constraints.CalcPenalty(q_1, 0).first, 0.0, 1e-6);
  EXPECT_TRUE(
      robot_constraints.CheckSatisfied(q_1, 0, check_satisfied_options));
  EXPECT_TRUE(robot_constraints.CalcPenalty(q_1, 0).second);
  // Getting close to the constraint triggers the penalty
  const auto q_2 {Eigen::Vector2d {-0.98, 0.3}};
  // the first joint gets close to the lower bound with a distance of 0.02
  // the influence is 0.1. So the penalty would be
  // zero_penalty * (exp(-gamma * 0.02) - exp(-gamma * influence)) / (1 -
  // exp(-gamma * influence))
  double expected_penalty = zero_penalty
                            * (exp(-gamma * 0.02) - exp(-gamma * influence))
                            / (1 - exp(-gamma * influence));
  logging::log()->info("expected_penalty: {}", expected_penalty);
  EXPECT_NEAR(robot_constraints.CalcPenalty(q_2, 0).first, expected_penalty,
              1e-6);
  // the checksatisfied is passed
  EXPECT_TRUE(
      robot_constraints.CheckSatisfied(q_2, 0, check_satisfied_options));
  EXPECT_TRUE(robot_constraints.CalcPenalty(q_2, 0).second);
}

TEST(TestRobotConstraints, CalcPenaltyCollision) {
  // Test with collision constraints
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  const double zero_penalty {1.0};
  const double gamma {0.5};
  const double influence {0.1};
  MinimumValuePenaltyParams params {
      .m = zero_penalty, .gamma = gamma, .x0 = influence};
  constraints_adapter.collision_checker.value().minimum_value_penalty_params =
      params;
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  const auto q {Eigen::Vector2d {0.0, 0.38}};
  // Find the robot clearance
  const auto clearance =
      robot_constraints.collision_checker().CalcRobotClearance(q, influence);
  double penalty = robot_constraints.CalcPenalty(q, 0).first;
  for (const auto& distance : clearance.distances()) {
    penalty -= zero_penalty * (exp(-gamma * distance) - exp(-gamma * influence))
               / (1 - exp(-gamma * influence));
  }
  EXPECT_NEAR(penalty, 0.0, 1e-6);
}

TEST(TestRobotConstraints, CalcPenaltyParallelized) {
  // Test the parallel overload
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  const double zero_penalty {1.0};
  const double gamma {0.5};
  const double influence {0.1};
  MinimumValuePenaltyParams params {
      .m = zero_penalty, .gamma = gamma, .x0 = influence};
  constraints_adapter.collision_checker.value().minimum_value_penalty_params =
      params;
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  std::vector<Eigen::VectorXd> q_vec {};
  const double step {0.0001};
  for (int i = 0; i < 1000; ++i) {
    Eigen::VectorXd q {Eigen::VectorXd::Zero(2)};
    q(0) = 0.0;
    q(1) = 0.3 + step * i;
    q_vec.push_back(q);
  }
  // Add the penalties in serial
  double expected_penalty = 0.0;
  const auto time_now = std::chrono::high_resolution_clock::now();
  for (const auto& q : q_vec) {
    expected_penalty += robot_constraints.CalcPenalty(q, 0).first;
  }
  const auto time_after_serial = std::chrono::high_resolution_clock::now();
  logging::log()->info("Calculating in parallel");
  const auto penalty_agg_opt {
      robot_constraints.CalcPenaltyVecAggregated(q_vec, kSum)};
  EXPECT_TRUE(penalty_agg_opt.has_value());
  // Now use the parallel version
  const auto& [parallel_penalty, valid] {penalty_agg_opt.value()};
  EXPECT_NEAR(expected_penalty, parallel_penalty, 1e-6);
  const auto time_after_parallel = std::chrono::high_resolution_clock::now();
  logging::log()->info("Time serial: {} ms vs parallel: {} ms",
                       std::chrono::duration_cast<std::chrono::milliseconds>(
                           time_after_serial - time_now)
                           .count(),
                       std::chrono::duration_cast<std::chrono::milliseconds>(
                           time_after_parallel - time_after_serial)
                           .count());
  // valid should be the same as CheckSatisfied parallel
  EXPECT_EQ(valid, robot_constraints.CheckSatisfied(q_vec));
}

TEST(TestRobotConstraints, Projection) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // Testing projection of a conf on a constrained set
  const std::string plan_adapter_file {
      "planning_service/test_data/plan_hotel_pan_third_6in_000.yaml"};
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};
  const auto robot_constraints {RobotConstraints(*robot_model, plan_adapter)};
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  Eigen::VectorXd q {Eigen::VectorXd::Zero(10)};
  q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6, 3.5, 1.9, -3.5;
  const auto is_satisfied {
      robot_constraints.CheckSatisfied(q, 0, check_satisfied_options)};
  EXPECT_FALSE(is_satisfied);
  logging::log()->info("is_satisfied: {}", is_satisfied);
  const auto projected_q_opt {
      robot_constraints.ProjectConfToNonCollisionConstraints(q, 0, q, 100.0)};
  logging::log()->info("projected_q_opt.has_value(): {}",
                       projected_q_opt.has_value());
  EXPECT_TRUE(projected_q_opt.has_value());
  const auto projected_q {projected_q_opt.value()};
  logging::log()->info("q: {}, projected_q: {}", q.transpose(),
                       projected_q.transpose());
  EXPECT_TRUE(robot_constraints.CheckSatisfied(projected_q, 0,
                                               check_satisfied_options));
}

TEST(TestRobotConstraints, CollisionFreeWithFilter) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // test if collision with filtered pairs is ignored
  Eigen::VectorXd franka_conf(7), aa_conf(2), singulator_conf(1);
  franka_conf << -1.0, 1.0, -1.0, -2.0, 1.4, 1.8, 0.0;
  aa_conf << -2.88, 0.0;
  singulator_conf << -3.5;
  system_conf_t system_conf;
  system_conf.emplace(kCobotName, franka_conf);
  system_conf.emplace(kAncillaryArmName, aa_conf);
  system_conf.emplace(kSingulatorName, singulator_conf);
  const auto q {robot_model->ToGeneralizedPosition(system_conf)};
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  {
    // aa will collide with the sneeze guard
    ConstraintsAdapter adapter;
    EXPECT_TRUE(RobotConstraints(*robot_model, adapter, 1)
                    .CheckSatisfied(q, 0, check_satisfied_options))
        << "No collision checking is enabled, so should be satisfied";
  }
  {
    ConstraintsAdapter adapter;
    CollisionCheckerAdapter default_collision_checker;
    adapter.collision_checker = default_collision_checker;
    EXPECT_FALSE(RobotConstraints(*robot_model, adapter, 1)
                     .CheckSatisfied(q, 0, check_satisfied_options))
        << "Collision checking should catch q in collision";
  }
  {
    // use no_aa_container_ramp that filters aa/ramp collisions
    ConstraintsAdapter adapter;
    std::vector<collision_group_pair_t> aa_ramp {
        {"ancillary_arm", "container_ramp"}};
    // get the collision_filter_groups
    adapter.collision_checker = {.filtered_pairs = aa_ramp};
    EXPECT_TRUE(RobotConstraints(*robot_model, adapter, 1)
                    .CheckSatisfied(q, 0, check_satisfied_options))
        << "Filtering aa-ramp pair should nullify the collision";
  }
  {
    // remove aa from the collision checker
    ConstraintsAdapter adapter;
    std::vector<std::string> no_aa {"ancillary_arm"};
    adapter.collision_checker = {.filtered_groups = no_aa};
    EXPECT_TRUE(RobotConstraints(*robot_model, adapter, 1)
                    .CheckSatisfied(q, 0, check_satisfied_options))
        << "Filtering aa as a group should nullify the collision";
  }
  {
    // not a valid collision group
    ConstraintsAdapter adapter;
    std::vector<collision_group_pair_t> not_existing {
        {"not_existing", "container_ramp"}};
    adapter.collision_checker = {.filtered_pairs = not_existing};
    EXPECT_THROW(RobotConstraints(*robot_model, adapter, 1)
                     .CheckSatisfied(q, 0, check_satisfied_options),
                 std::runtime_error);
  }
}

TEST(TestRobotConstraints, CollisionWithPading) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // test if collision with filtered pairs is ignored
  Eigen::VectorXd franka_conf(7), aa_conf(2), singulator_conf(1);
  franka_conf << -2.4, -0.12, 0.48, -2.2, 0.17, 1.7, 0.0;
  aa_conf << -1.7, 0;
  singulator_conf << -3.5;
  system_conf_t system_conf;
  system_conf.emplace(kCobotName, franka_conf);
  system_conf.emplace(kAncillaryArmName, aa_conf);
  system_conf.emplace(kSingulatorName, singulator_conf);
  const auto q {robot_model->ToGeneralizedPosition(system_conf)};
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  ConstraintsAdapter adapter;
  adapter.plan_name = "scoop";
  auto collision_checker_adapter = CollisionCheckerAdapter();
  adapter.collision_checker = collision_checker_adapter;
  EXPECT_FALSE(RobotConstraints(*robot_model, adapter, 1)
                   .CheckSatisfied(q, 0, check_satisfied_options));
  // add padding to the collision checker
  PaddingAdapter disher_aa_1, disher_aa_2;
  disher_aa_1.pair[0] = "group_1_disher_2oz";
  disher_aa_1.pair[1] = "group_3_ancillary_arm_l2";
  disher_aa_1.distance = -0.031;
  disher_aa_2.pair[0] = "group_2_disher_2oz_tip";
  disher_aa_2.pair[1] = "group_3_ancillary_arm_l2";
  disher_aa_2.distance = -0.012;
  std::vector<PaddingAdapter> padding_adapters {disher_aa_1, disher_aa_2};
  collision_checker_adapter.paddings = padding_adapters;
  adapter.collision_checker = collision_checker_adapter;
  EXPECT_TRUE(RobotConstraints(*robot_model, adapter, 1)
                  .CheckSatisfied(q, 0, check_satisfied_options));
  // Save the adapter as string
  logging::log()->info("adapter: \n{}", drake::yaml::SaveYamlString(adapter));
}

TEST(TestRobotConstraints, GenerateValidSamples) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  const std::string plan_adapter_file {
      "planning_service/test_data/plan_hotel_pan_third_6in_000.yaml"};
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};
  const auto robot_constraints {RobotConstraints(*robot_model, plan_adapter)};
  drake::RandomGenerator generator {0};
  const auto q_vec_rejection {robot_constraints.GenerateSamples(&generator, 3)};
  SampleOptions sample_options;
  sample_options.use_projection = true;
  sample_options.max_num_samples = 2000;
  const auto q_vec_projection {
      robot_constraints.GenerateSamples(&generator, 3, sample_options)};
  EXPECT_TRUE(!q_vec_rejection.empty());
  EXPECT_TRUE(!q_vec_projection.empty());
  // Check that all samples are valid
  for (const auto& q : q_vec_rejection) {
    for (auto constraint_ptr :
         robot_constraints.get_non_collision_constraints()) {
      Eigen::VectorXd constraint_val;
      constraint_ptr->Eval(q, &constraint_val);
    }
    EXPECT_TRUE(robot_constraints.CheckSatisfied(q));
  }
  for (const auto& q : q_vec_projection) {
    for (auto constraint_ptr :
         robot_constraints.get_non_collision_constraints()) {
      Eigen::VectorXd constraint_val;
      constraint_ptr->Eval(q, &constraint_val);
    }
    EXPECT_TRUE(robot_constraints.CheckSatisfied(q));
  }
}

TEST(TestRobotConstraints, GenerateValidSamplesWithHolonomicMapping) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  const std::string plan_adapter_file {
      "planning_service/test_data/franka/constraints.yaml"};
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};
  const auto robot_constraints {RobotConstraints(*robot_model, plan_adapter)};
  drake::RandomGenerator generator {0};
  const auto q_vec_rejection {robot_constraints.GenerateSamples(&generator, 3)};
  SampleOptions sample_options;
  sample_options.use_projection = true;
  sample_options.max_num_samples = 2000;
  const auto q_vec_projection {
      robot_constraints.GenerateSamples(&generator, 3, sample_options)};
  EXPECT_TRUE(!q_vec_rejection.empty());
  EXPECT_TRUE(!q_vec_projection.empty());
  // Check that all samples are valid
  for (const auto& q : q_vec_rejection) {
    EXPECT_TRUE(robot_constraints.CheckSatisfied(q));
  }
  for (const auto& q : q_vec_projection) {
    EXPECT_TRUE(robot_constraints.CheckSatisfied(q));
  }
}

TEST(TestRobotConstraints, CalcAndClassifyCollisions) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/wallflower/dual.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // Make a default collision adapter
  ConstraintsAdapter adapter;
  CollisionCheckerAdapter default_collision_checker;
  adapter.collision_checker = default_collision_checker;
  const auto robot_constraints = RobotConstraints(*robot_model, adapter, 1);
  Eigen::VectorXd q {Eigen::VectorXd::Zero(4)};
  q << M_PI, 0.35, 0.01, 0.35;
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  // We know it involves multiple arms.
  EXPECT_EQ(robot_constraints.CalcAndClassifyCollisions(q, 0),
            CollisionType::kAcrossArmsOnly);
  EXPECT_TRUE(robot_constraints.DoArmsCollide(q, 0));
  // Check a conf that flower hits the wall.
  q << 0.0, 0.35, M_PI, 0.35;
  EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  EXPECT_EQ(robot_constraints.CalcAndClassifyCollisions(q, 0),
            CollisionType::kArmEnvOnly);
  EXPECT_FALSE(robot_constraints.DoArmsCollide(q, 0));
  // A conf that both collisions exist. Should return false.
  q << 0, 0.35, 0, 0.35;
  EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  EXPECT_EQ(robot_constraints.CalcAndClassifyCollisions(q, 0),
            CollisionType::kMixed);
  EXPECT_TRUE(robot_constraints.DoArmsCollide(q, 0));
  // A conf that is collision free. Should return false.
  q << M_PI / 2, 0.35, -M_PI / 2, 0.35;
  EXPECT_TRUE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  EXPECT_EQ(robot_constraints.CalcAndClassifyCollisions(q, 0),
            CollisionType::kNone);
  EXPECT_FALSE(robot_constraints.DoArmsCollide(q, 0));
}

TEST(TestRobotConstraints, CalcClosestSatisfyingConfigurationCollisionOnly) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // test if collision with filtered pairs is ignored
  Eigen::VectorXd franka_conf(7), aa_conf(2), singulator_conf(1);
  franka_conf << -1.0, 1.0, -1.0, -2.0, 1.4, 1.8, 0.0;
  aa_conf << 3.4, 0.1;
  singulator_conf << -3.5;
  system_conf_t system_conf;
  system_conf.emplace(kCobotName, franka_conf);
  system_conf.emplace(kAncillaryArmName, aa_conf);
  system_conf.emplace(kSingulatorName, singulator_conf);
  const auto q {robot_model->ToGeneralizedPosition(system_conf)};
  ConstraintsAdapter adapter;
  CollisionCheckerAdapter default_collision_checker;
  adapter.collision_checker = default_collision_checker;
  const auto robot_constraints = RobotConstraints(*robot_model, adapter, 1);
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  const auto closest_q_opt =
      robot_constraints.CalcClosestSatisfyingConfiguration(q);
  EXPECT_TRUE(closest_q_opt.has_value());
  const auto& closest_q = closest_q_opt.value();
  logging::log()->info(" q_closest - q: {}", (closest_q - q).transpose());
  // Check that the closest q satisfies the position constraint
  EXPECT_TRUE(
      robot_constraints.CheckSatisfied(closest_q, 0, check_satisfied_options));
  // We have something better for q_closest
  const auto polyhedron_closest =
      robot_constraints.FindSatisfactionHPolyhedron(closest_q, 0);
  EXPECT_FALSE(polyhedron_closest.PointInSet(closest_q));
  EXPECT_FALSE(polyhedron_closest.IsEmpty());
}

TEST(TestRobotConstraints, CalcClosestSatisfyingConfigurationWithFixedModels) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/wallflower/dual.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // Make a default collision adapter
  ConstraintsAdapter adapter;
  CollisionCheckerAdapter default_collision_checker;
  adapter.collision_checker = default_collision_checker;
  const auto robot_constraints = RobotConstraints(*robot_model, adapter, 1);
  Eigen::VectorXd q {Eigen::VectorXd::Zero(4)};
  Eigen::VectorXd q_1(2), q_2(2);
  q_1 << 2.0, 0.35;
  q_2 << 1.6, 0.35;
  q << q_1, q_2;
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  // De-conflict the collision but keep flower 1 fixed
  auto model_idx = robot_model->plant().GetModelInstanceByName("flower1");
  const auto closest_q_opt =
      robot_constraints.CalcClosestSatisfyingConfiguration(q, 0, {model_idx});
  EXPECT_TRUE(closest_q_opt.has_value());
  auto q_sol = closest_q_opt.value();
  EXPECT_TRUE(
      robot_constraints.CheckSatisfied(q_sol, 0, check_satisfied_options));
  // The first part of q_sol should be the same as q_1
  EXPECT_TRUE(q_sol.head(2).isApprox(q_1));
  // Let's solve this time with flower_2 fixed
  model_idx = robot_model->plant().GetModelInstanceByName("flower2");
  const auto closest_q_opt_2 =
      robot_constraints.CalcClosestSatisfyingConfiguration(q, 0, {model_idx});
  EXPECT_TRUE(closest_q_opt_2.has_value());
  auto q_sol_2 = closest_q_opt_2.value();
  EXPECT_TRUE(
      robot_constraints.CheckSatisfied(q_sol_2, 0, check_satisfied_options));
  // The second part of q_sol should be the same as q_2
  EXPECT_TRUE(q_sol_2.tail(2).isApprox(q_2));
}

TEST(TestRobotConstraints,
     CalcClosestSatisfyingConfigurationWithPositionConstraint) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // test if collision with filtered pairs is ignored
  Eigen::VectorXd franka_conf(7), aa_conf(2), singulator_conf(1);
  franka_conf << -1.0, 0.8, -0.3, -1.0, 0.3, 1.5, -0.6;
  aa_conf << 3.4, 0.1;
  singulator_conf << -3.5;
  system_conf_t system_conf;
  system_conf.emplace(kCobotName, franka_conf);
  system_conf.emplace(kAncillaryArmName, aa_conf);
  system_conf.emplace(kSingulatorName, singulator_conf);
  const auto q {robot_model->ToGeneralizedPosition(system_conf)};
  const std::string plan_adapter_file {
      "planning_service/test_data/plan_hotel_pan_third_6in_000.yaml"};
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};
  const auto robot_constraints =
      RobotConstraints(*robot_model, plan_adapter, 1);
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0, check_satisfied_options));
  const auto closest_q_opt =
      robot_constraints.CalcClosestSatisfyingConfiguration(q);
  EXPECT_TRUE(closest_q_opt.has_value());
  const auto& closest_q = closest_q_opt.value();
  logging::log()->info(" q_closest - q: {}", (closest_q - q).transpose());
  // Check that the closest q satisfies the position constraint
  EXPECT_TRUE(
      robot_constraints.CheckSatisfied(closest_q, 0, check_satisfied_options));
  // We have something better for q_closest
  const auto polyhedron_closest =
      robot_constraints.FindSatisfactionHPolyhedron(closest_q, 0);
  EXPECT_FALSE(polyhedron_closest.PointInSet(closest_q));
  EXPECT_FALSE(polyhedron_closest.IsEmpty());
}

TEST(TestRobotConstraints, CalcClosestSatisfyingConfigurationOnEdge) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // Make a default collision adapter
  ConstraintsAdapter adapter;
  CollisionCheckerAdapter default_collision_checker;
  adapter.collision_checker = default_collision_checker;
  const auto robot_constraints = RobotConstraints(*robot_model, adapter, 1);
  Eigen::VectorXd q(2), q_valid(2);
  q << 0.0, 0.4;
  q_valid << 0.0, 0.2;
  const auto closest_q_opt =
      robot_constraints.CalcClosestSatisfyingConfigurationOnEdge(q, q_valid);
  EXPECT_TRUE(closest_q_opt.has_value());
  const auto& closest_q = closest_q_opt.value();
  // Ensure it is on the edge between q and q_valid
  const auto closest_q_vec = closest_q - q_valid;
  const auto edge_vec = q - q_valid;
  const auto dot_product = closest_q_vec.dot(edge_vec);
  EXPECT_NEAR(dot_product, closest_q_vec.norm() * edge_vec.norm(), 1e-6);

  ConstraintsAdapter joint_constrained_adapter;
  joint_constrained_adapter.joint_position_box_constraints = {
      {"robot", Eigen::Vector2d {-M_PI / 2, 0.4},
       Eigen::Vector2d {M_PI / 2, 0.4}}};
  const auto robot_constraints_joint_constrained {
      RobotConstraints(*robot_model, joint_constrained_adapter, 1)};
  EXPECT_THROW(robot_constraints_joint_constrained
                   .CalcClosestSatisfyingConfigurationOnEdge(q, q_valid),
               std::runtime_error)
      << "q_valid is not valid for the joint constraints, so should throw";
}

TEST(TestRobotConstraints, PointDistanceToMesh) {
  // test with joint position box constraints
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/wallflower/dmd_with_mesh.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  CollisionCheckerAdapter collision_checker;
  constraints_adapter.collision_checker = collision_checker;
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  // Let's compute the distance from a point to the mesh
  const auto& body = robot_model->plant().GetBodyByName("banana");
  const auto& frame = robot_model->plant().GetFrameByName("A");
  // Let's get the distance to the mesh at every point. It will
  // collide with the convex-hull of the mesh, but the mesh itself.
  for (double q_2 = 0.25; q_2 < 0.4; q_2 += 0.01) {
    Eigen::Vector2d q {0.0, q_2};
    auto distance_opt = robot_constraints.CalcPointDistanceToBody(
        q, frame, Eigen::Vector3d {0.0, 0.0, 0.0}, body, 0.5);
    EXPECT_TRUE(distance_opt.has_value());
    EXPECT_GE(distance_opt.value(), 0.0);
    // But CheckSatisfied should return false
    EXPECT_FALSE(robot_constraints.CheckSatisfied(q, 0));
  }
}

TEST(TestRobotConstraints, Hash) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  const std::string plan_adapter_file {
      "planning_service/test_data/plan_hotel_pan_third_6in_000.yaml"};
  ConstraintsAdapter no_constraints_adapter;
  const auto robot_constraints_null {
      RobotConstraints(*robot_model, no_constraints_adapter)};
  const drake::DefaultHash hash_func;
  const auto hash_constraints_null = hash_func(robot_constraints_null);
  // Now let's do with another set of constraints, only collision this time
  CollisionCheckerAdapter collision_checker;
  ConstraintsAdapter constraints_adapter_collision {.collision_checker =
                                                        collision_checker};
  const auto robot_constraints_only_collision {
      RobotConstraints(*robot_model, constraints_adapter_collision)};
  const auto hash_constraints_collision =
      hash_func(robot_constraints_only_collision);
  const size_t kHashCollisionChecker {2171024290876027270};
  EXPECT_EQ(hash_constraints_collision, kHashCollisionChecker);
  EXPECT_NE(hash_constraints_null, hash_constraints_collision);
  // hash should be different with a collision checker and plan constraints
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};
  const auto robot_constraints_column_collision {
      RobotConstraints(*robot_model, plan_adapter)};
  const auto hash_constraints = hash_func(robot_constraints_column_collision);
  EXPECT_NE(hash_constraints_null, hash_constraints);
  logging::log()->info("hash_constraints: {}", hash_constraints);
  // The following number changes when the collision filters change.
  const size_t kHashCollisionCheckerAndPlanConstraints {11315862405437023893U};
  EXPECT_EQ(hash_constraints, kHashCollisionCheckerAndPlanConstraints);
  // constraints_hash
  EXPECT_EQ(hash_constraints_null, robot_constraints_null.constraints_hash());
  EXPECT_EQ(hash_constraints,
            robot_constraints_column_collision.constraints_hash());
}

TEST(TestCheckSatisfiedResult, FailedConstraintStrings) {
  // Verify that failed_constraint_strings is populated and a non-collision
  // constraint is violated.
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_wallflowers/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.joint_position_box_constraints = {
      {"flower1", Eigen::Vector2d {-2.0, 0.25}, Eigen::Vector2d {2.0, 0.3}}};
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  Eigen::VectorXd q(4);
  q << M_PI / 2.0, 0.35, 0.0,
      0.35;  // flower1 violates the joint position box constraint
  {
    CheckSatisfiedOptions opts;
    const auto result {robot_constraints.CheckSatisfied(q, 0, opts)};
    EXPECT_FALSE(result);
    EXPECT_FALSE(result.failed_constraint_strings().has_value());
  }
  {
    CheckSatisfiedOptions opts;
    opts.collect_failed_constraint_strings = true;
    const auto result {robot_constraints.CheckSatisfied(q, 0, opts)};
    EXPECT_FALSE(result);
    ASSERT_TRUE(result.failed_constraint_strings().has_value());
    EXPECT_FALSE(result.failed_constraint_strings()->empty());
    logging::log()->info("failed_constraint_strings: {}",
                         fmt::join(*result.failed_constraint_strings(), ", "));
  }
  // A valid q should have an empty failed_constraint_strings list
  {
    Eigen::VectorXd q_valid {Eigen::VectorXd::Zero(4)};
    q_valid << M_PI / 2.0, 0.3, 0.0, 0.2;  // valid configuration
    CheckSatisfiedOptions opts;
    opts.collect_failed_constraint_strings = true;
    opts.verbose = true;
    const auto result {robot_constraints.CheckSatisfied(q_valid, 0, opts)};
    EXPECT_TRUE(result);
    ASSERT_TRUE(result.failed_constraint_strings().has_value());
    EXPECT_TRUE(result.failed_constraint_strings()->empty());
  }
}

TEST(TestCheckSatisfiedResult, OffendingModelNames) {
  // Verify that offending_model_names is populated when a collision is detected
  // and collect_offending_model_names=true
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_wallflowers/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter adapter;
  adapter.collision_checker = CollisionCheckerAdapter {};
  const auto robot_constraints {RobotConstraints(*robot_model, adapter, 1)};
  Eigen::VectorXd q(4);
  q << M_PI / 2.0, 0.35, 0.0,
      0.35;  // known to be in collision based on previous tests
  CheckSatisfiedOptions opts;
  opts.collect_offending_model_names = true;
  const auto result {robot_constraints.CheckSatisfied(q, 0, opts)};
  EXPECT_TRUE(result);
  ASSERT_TRUE(result.offending_model_names().has_value());
  EXPECT_TRUE(result.offending_model_names()->empty());
  // Now a bad q that is in collision
  q << 0.1, 0.3, -0.1, 0.3;  // known to be in collision based on previous tests
  const auto result_collision {robot_constraints.CheckSatisfied(q, 0, opts)};
  EXPECT_FALSE(result_collision);
  ASSERT_TRUE(result_collision.offending_model_names().has_value());
  EXPECT_EQ(result_collision.offending_model_names()->size(), 1);
  // Only flower1 should be offending
  EXPECT_EQ(result_collision.offending_model_names()->front(), "flower1");
  // Names should be sorted and deduplicated
  const auto& names {*result_collision.offending_model_names()};
  EXPECT_TRUE(std::is_sorted(names.begin(), names.end()));
  const bool has_duplicates {std::adjacent_find(names.begin(), names.end())
                             != names.end()};
  EXPECT_FALSE(has_duplicates);
  // A config that has both arms colliding with each other. Both flower1 and
  // flower2 should be in the offending_model_names.
  q << M_PI, 0.35, 0.1,
      0.35;  // known to be in collision based on previous tests
  const auto result_both_collision {
      robot_constraints.CheckSatisfied(q, 0, opts)};
  EXPECT_FALSE(result_both_collision);
  ASSERT_TRUE(result_both_collision.offending_model_names().has_value());
  EXPECT_EQ(result_both_collision.offending_model_names()->size(), 2);
  EXPECT_EQ(result_both_collision.offending_model_names()->front(), "flower1");
  EXPECT_EQ(result_both_collision.offending_model_names()->back(), "flower2");
}

TEST(TestCheckSatisfiedResult, CalcPenaltyViaOptions) {
  // Verify that calc_penalty=true in CheckSatisfiedOptions produces the same
  // penalty as the dedicated CalcPenalty function.
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  // Joint position constraint with penalty params
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.joint_position_box_constraints = {
      {"robot", Eigen::Vector2d {-1.0, -1.0}, Eigen::Vector2d {1.0, 1.0}}};
  const double zero_penalty {1.0};
  const double gamma {0.5};
  const double influence {0.1};
  MinimumValuePenaltyParams params {
      .m = zero_penalty, .gamma = gamma, .x0 = influence};
  constraints_adapter.joint_position_box_constraints.value()
      .front()
      .minimum_value_penalty_params = params;
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  // A q that is near the constraint boundary (penalty > 0)
  const auto q {Eigen::Vector2d {-0.98, 0.3}};
  // Use CalcPenalty as the reference
  const auto [expected_penalty,
              expected_valid] {robot_constraints.CalcPenalty(q, 0)};
  // Use CheckSatisfied with calc_penalty=true
  CheckSatisfiedOptions opts;
  opts.calc_penalty = true;
  const auto result {robot_constraints.CheckSatisfied(q, 0, opts)};
  EXPECT_EQ(result.satisfied(), expected_valid);
  ASSERT_TRUE(result.penalty().has_value());
  EXPECT_NEAR(*result.penalty(), expected_penalty, 1e-6);
  // With calc_penalty=false (default): penalty should not be populated
  {
    CheckSatisfiedOptions opts_no_penalty;
    const auto result2 {
        robot_constraints.CheckSatisfied(q, 0, opts_no_penalty)};
    EXPECT_FALSE(result2.penalty().has_value());
  }
}

TEST(TestCheckSatisfiedResult, CalcPenaltyViaOptionsWithCollision) {
  // Verify that calc_penalty=true also accumulates the collision penalty,
  // matching the dedicated CalcPenalty function for a collision setup.
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  const double zero_penalty {1.0};
  const double gamma {0.5};
  const double influence {0.1};
  MinimumValuePenaltyParams params {
      .m = zero_penalty, .gamma = gamma, .x0 = influence};
  constraints_adapter.collision_checker.value().minimum_value_penalty_params =
      params;
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  const auto q {Eigen::Vector2d {0.0, 0.38}};
  // Reference penalty from dedicated CalcPenalty
  const auto [expected_penalty,
              expected_valid] {robot_constraints.CalcPenalty(q, 0)};
  // Verify via CheckSatisfied with calc_penalty=true
  CheckSatisfiedOptions opts;
  opts.calc_penalty = true;
  const auto result {robot_constraints.CheckSatisfied(q, 0, opts)};
  EXPECT_EQ(result.satisfied(), expected_valid);
  ASSERT_TRUE(result.penalty().has_value());
  EXPECT_NEAR(*result.penalty(), expected_penalty, 1e-6);
}

}  // namespace motion
