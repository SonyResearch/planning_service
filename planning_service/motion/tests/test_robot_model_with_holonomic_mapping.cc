/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include "planning_service/motion/robot_model.h"

namespace motion {

TEST(HolonomicMapping, ArmWithGripper) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot = std::make_unique<RobotModel>(xml_file, dmd);
  auto hm = robot->holonomic_mapping();
  EXPECT_EQ(hm.full_dim(), 13);
  EXPECT_EQ(hm.minimal_dim(), 8);
  // Test Reduce
  auto q_full = Eigen::VectorXd::LinSpaced(13, 0.0, 1.2);
  auto q_reduced = hm.Reduce(q_full);
  auto q_reduced_expected = Eigen::VectorXd::LinSpaced(8, 0.0, 0.7);
  EXPECT_TRUE(q_reduced.isApprox(q_reduced_expected, 1e-6));
  // Test Lift
  auto q_lifted = hm.Lift(q_reduced);
  // Test reduce again: it will be a configuration satisfying the coupler
  // constraints
  auto q_reduced_2 = hm.Reduce(q_lifted);
  EXPECT_TRUE(q_reduced_2.isApprox(q_reduced, 1e-6));
  // Test LiftedIndex and ReducedIndex
  for (int i = 0; i < hm.minimal_dim(); ++i) {
    EXPECT_EQ(hm.ReducedIndex(hm.LiftedIndex(i)), i);
  }
  // Test ReducedIndex for 8 to 12 should throw, and IsMimickingJoint would be
  // true
  for (int i = 8; i < 13; ++i) {
    EXPECT_TRUE(hm.IsMimickingJoint(i));
    EXPECT_THROW(hm.ReducedIndex(i), std::runtime_error);
  }
  // Test instance Reduce and Lift
  auto system_conf = robot->ToSystemConf(q_lifted);
  // Print system conf
  logging::log()->info("System conf: {}", system_conf);
  // Let's go through the system conf and reduce/lift each model instance
  for (const auto& [robot_name, q_instance_full] : system_conf) {
    auto model_instance = robot->plant().GetModelInstanceByName(robot_name);
    auto q_instance_reduced =
        hm.ReduceInstance(model_instance, q_instance_full);
    auto q_instance_lifted =
        hm.LiftInstance(model_instance, q_instance_reduced);
    logging::log()->info(
        "Robot name: {}, q_instance_full: {}, q_instance_reduced: {}, "
        "q_instance_lifted: {}",
        robot_name, q_instance_full.transpose(), q_instance_reduced.transpose(),
        q_instance_lifted.transpose());
    EXPECT_TRUE(q_instance_lifted.isApprox(q_instance_full, 1e-6));
  }
  // Test system conf reduce
  auto reduced_sysconf = robot->ReduceSystemConf(system_conf);
  // Print reduced system conf
  logging::log()->info("Reduced system conf: {}", reduced_sysconf);
  // Check that the reduced system conf for gripper is of size 1
  EXPECT_EQ(reduced_sysconf.at("gripper").size(), 1);
  // Check that the reduced system conf for franka is of size 7
  EXPECT_EQ(reduced_sysconf.at("franka").size(), 7);
  // Test system conf lift
  auto lifted_sysconf = robot->LiftSystemConf(reduced_sysconf);
  // Print lifted system conf
  logging::log()->info("Lifted system conf: {}", lifted_sysconf);
  // The lifted and original system conf should be the same
  EXPECT_EQ(lifted_sysconf.size(), system_conf.size());
  for (const auto& [robot_name, q_instance_full] : system_conf) {
    EXPECT_TRUE(lifted_sysconf.count(robot_name) == 1);
    logging::log()->info("Robot name: {}, original q: {}, lifted q: {}",
                         robot_name, q_instance_full.transpose(),
                         lifted_sysconf.at(robot_name).transpose());
    EXPECT_TRUE(lifted_sysconf.at(robot_name).isApprox(q_instance_full, 1e-6));
  }
  // Additional checks: the drake plant is discrete and the contact solver is
  // SAP
  EXPECT_GT(robot->plant().time_step(), 0.0);
  auto discre_contact_solver = robot->plant().get_discrete_contact_solver();
  EXPECT_EQ(discre_contact_solver,
            drake::multibody::DiscreteContactSolver::kSap);
  // Check the function returned by GetDoubleParameterization
  auto param_func = hm.iris_parameterization_function();
  // Ok let's test the parameterization function on a sample reduced vector
  Eigen::VectorXd sample_reduced(8);
  sample_reduced << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8;
  // Allow some grad;
  Eigen::MatrixXd grad(8, 2);
  grad.col(0) << 1, 0, 0, 0, 0, 0, 0, 1;
  grad.col(1) << 0, 1, 0, 0, 0, 0, 0, -2;
  auto sample_auto_diff = drake::math::InitializeAutoDiff(sample_reduced, grad);
  auto param_auto_diff = param_func.get_parameterization_autodiff();
  auto lifted_auto_diff = param_auto_diff(sample_auto_diff);
  Eigen::VectorXd lifted_d = drake::math::ExtractValue(lifted_auto_diff);
  Eigen::MatrixXd lifted_ad = drake::math::ExtractGradient(lifted_auto_diff);
  EXPECT_TRUE(lifted_d.isApprox(hm.Lift(sample_reduced), 1e-6));
  EXPECT_EQ(lifted_ad.cols(), 2);
  EXPECT_EQ(lifted_ad.rows(), hm.full_dim());
  EXPECT_TRUE(lifted_ad.col(0).isApprox(hm.Lift(grad.col(0)), 1e-6));
  EXPECT_TRUE(lifted_ad.col(1).isApprox(hm.Lift(grad.col(1)), 1e-6));
}

TEST(HolonomicMapping, DualArmWithGripper) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dual_arm.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot = std::make_unique<RobotModel>(xml_file, dmd);
  auto hm = robot->holonomic_mapping();
  EXPECT_EQ(hm.full_dim(), 26);
  EXPECT_EQ(hm.minimal_dim(), 16);
  // Test Reduce on an unrealistic full vector
  auto q_full = Eigen::VectorXd::LinSpaced(26, 0.0, 2.5);
  auto q_reduced = hm.Reduce(q_full);
  // Test Lift
  auto q_lifted = hm.Lift(q_reduced);
  EXPECT_TRUE(q_lifted.head(7).isApprox(q_full.head(7), 1e-6));
  EXPECT_TRUE(q_lifted.segment(13, 7).isApprox(q_full.segment(13, 7), 1e-6));
  // Test reduce again: it will be a configuration satisfying the coupler
  // constraints
  auto q_reduced_2 = hm.Reduce(q_lifted);
  EXPECT_TRUE(q_reduced_2.isApprox(q_reduced, 1e-6));
  // Test LiftedIndex and ReducedIndex
  for (int i = 0; i < hm.minimal_dim(); ++i) {
    EXPECT_EQ(hm.ReducedIndex(hm.LiftedIndex(i)), i);
  }
  // Test applying reduce/lift for system conf
  auto system_conf = robot->ToSystemConf(q_lifted);
  auto system_conf_reduced = robot->ReduceSystemConf(system_conf);
  auto system_conf_lifted = robot->LiftSystemConf(system_conf_reduced);
  EXPECT_EQ(system_conf.size(), system_conf_lifted.size());
  for (const auto& [robot_name, q_instance_full] : system_conf) {
    EXPECT_TRUE(system_conf_lifted.count(robot_name) == 1);
    EXPECT_TRUE(
        system_conf_lifted.at(robot_name).isApprox(q_instance_full, 1e-6));
  }
  // log the system confs
  logging::log()->info("Original system conf: {}", system_conf);
  logging::log()->info("Reduced system conf: {}", system_conf_reduced);
  // Check the holonomic mapping for each arm
  EXPECT_EQ(robot->num_arms(), 2);
  for (int i = 0; i < robot->num_arms(); ++i) {
    const auto& arm = robot->GetArm(ArmIndex(i));
    EXPECT_EQ(arm.model_instances().size(), 2);
    EXPECT_EQ(arm.plant().num_positions(), 13);
    EXPECT_EQ(arm.arm_holonomic_mapping().full_dim(), 13);
    EXPECT_EQ(arm.arm_holonomic_mapping().minimal_dim(), 8);
  }
}

}  // namespace motion
