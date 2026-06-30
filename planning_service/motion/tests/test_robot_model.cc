/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include "planning_service/motion/robot_model.h"

namespace motion {

const std::string kCobotName {"franka"};
const std::string kAncillaryArmName {"ancillary_arm"};
const std::string kSingulatorName {"singulator"};

TEST(TestRobotModel, Basic) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  std::vector<std::pair<std::string, int>> continuous_revolute_joints;
  continuous_revolute_joints.emplace_back("ancillary_arm", 0);
  const auto robot = std::make_unique<RobotModel>(xml_file, dmd, std::nullopt,
                                                  continuous_revolute_joints);
  auto cobot_index = robot->plant().GetModelInstanceByName(kCobotName);
  auto ancillary_arm_index =
      robot->plant().GetModelInstanceByName(kAncillaryArmName);
  auto singulator_index =
      robot->plant().GetModelInstanceByName(kSingulatorName);
  // Test getting the start position of the robot.
  // They follow the order in the DMD file.
  EXPECT_EQ(robot->GetModelStartIndex(cobot_index), 0);
  EXPECT_EQ(robot->GetModelStartIndex(ancillary_arm_index), 7);
  EXPECT_EQ(robot->GetModelStartIndex(singulator_index), 9);
  // But what we specify the arms.
  EXPECT_EQ(
      robot->get_arm_index(robot->plant().GetModelInstanceByName(kCobotName)),
      ArmIndex(0));
  EXPECT_EQ(robot->get_arm_index(
                robot->plant().GetModelInstanceByName(kAncillaryArmName)),
            ArmIndex(1));
  EXPECT_EQ(robot->get_arm_index(
                robot->plant().GetModelInstanceByName(kSingulatorName)),
            ArmIndex(2));
  // Querying the wrong arm index should throw an exception
  // Joint 7 is the one that is continuous revolute
  EXPECT_EQ(robot->continuous_revolute_joint_indices().size(), 1);
  EXPECT_EQ(robot->continuous_revolute_joint_indices().front(), 7);
  // Some meshes are not encapsulated by collision shapes!
  EXPECT_FALSE(robot->AreAllVisualShapesEncapsulatedByCollisionShapes());
  // ToDo: Find a tolerance that the test passes.
}

TEST(TestRobotModel, ToGeneralizedPosition1) {
  // Setup the robot and convert system conf to generalized position and back
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot {std::make_unique<RobotModel>(xml_file, dmd)};
  // No joint is declared as continuous revolute
  EXPECT_EQ(robot->continuous_revolute_joint_indices().size(), 0);
  // test conversion between generalized and system configuration
  Eigen::VectorXd franka_conf(7), aa_conf(2), singulator_conf(1);
  franka_conf << -1.0, 1.0, -1.0, -2.0, 1.4, 1.8, 0.0;
  aa_conf << 1.57, 2.1;
  singulator_conf << -3.5;
  system_conf_t system_conf;
  system_conf.emplace(kCobotName, franka_conf);
  system_conf.emplace(kAncillaryArmName, aa_conf);
  system_conf.emplace(kSingulatorName, singulator_conf);
  const auto q_gen {robot->ToGeneralizedPosition(system_conf)};
  const auto system_conf_2 {robot->ToSystemConf(q_gen)};
  for (const auto& [robot_name, conf] : system_conf) {
    EXPECT_TRUE(system_conf_2.count(robot_name) > 0);
    EXPECT_TRUE((system_conf_2.at(robot_name) - conf).array().abs().maxCoeff()
                < 1e-6);
  }
  // the other way around
  for (const auto& [robot_name, conf] : system_conf_2) {
    EXPECT_TRUE(system_conf.count(robot_name) > 0);
    EXPECT_TRUE((system_conf.at(robot_name) - conf).array().abs().maxCoeff()
                < 1e-6);
  }
  // back to generalized conf
  const auto q_gen_2 {robot->ToGeneralizedPosition(system_conf_2)};
  EXPECT_TRUE((q_gen - q_gen_2).array().abs().maxCoeff() < 1e-6);
  // Test an incomplete system conf. It should throw PartialSysconfError
  system_conf_t partial_system_conf;
  partial_system_conf.emplace(kCobotName, franka_conf);
  partial_system_conf.emplace(kAncillaryArmName, aa_conf);
  EXPECT_THROW(robot->ToGeneralizedPosition(partial_system_conf),
               RobotModel::PartialSysconfError);
}

TEST(TestRobotModel, ToGeneralizedPosition2) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_aa.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot {std::make_unique<RobotModel>(xml_file, dmd)};
  // The robot has only the ancillary arm
  EXPECT_THROW(robot->plant().GetModelInstanceByName(kCobotName),
               std::logic_error);
  EXPECT_THROW(robot->plant().GetModelInstanceByName(kSingulatorName),
               std::logic_error);
  EXPECT_NO_THROW(robot->plant().GetModelInstanceByName(kAncillaryArmName));
  EXPECT_EQ(robot->plant().num_positions(), 2);
  system_conf_t bad_system_conf;
  bad_system_conf.emplace(kCobotName, Eigen::VectorXd::Zero(7));
  bad_system_conf.emplace(kAncillaryArmName, Eigen::VectorXd::Zero(2));
  // such a system conf is not valid
  EXPECT_THROW(robot->ToGeneralizedPosition(bad_system_conf), std::exception);
}

TEST(TestRobotModel, Hash) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/alfred/sp_disher_2oz_000.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot {std::make_unique<RobotModel>(xml_file, dmd)};
  // let's do it again
  const auto dmd2 {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot2 {std::make_unique<RobotModel>(xml_file, dmd2)};
  const drake::DefaultHash hash_func;
  // let's hash both robots
  const auto hash1 = hash_func(*robot);
  const auto hash2 = hash_func(*robot2);
  logging::log()->info("Hash1: {}", hash1);
  logging::log()->info("Hash2: {}", hash2);
  EXPECT_EQ(hash1, hash2);
}

TEST(TestRobotModel, Arms) {
  // Here we create a many chains dmd file. The model has 3 flowers (each 2dof).
  // However, 2 flowers are serially connected. There are also 3 walls.
  // One connected to world, one connected to the first wall, and one connected
  // to a flower.
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/wallflower/many_chains.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot {std::make_unique<RobotModel>(xml_file, dmd)};
  EXPECT_EQ(robot->plant().num_positions(), 6);
  EXPECT_EQ(robot->num_arms(), 2);
  // The first arm has 2 dofs and 1 model (flower1)
  EXPECT_EQ(robot->GetArm(ArmIndex(0)).plant().num_positions(), 2);
  EXPECT_EQ(robot->GetArm(ArmIndex(0)).model_instances().size(), 1);
  // The second arm has 4 dofs and 3 models (flower2, flower3, wall2)
  EXPECT_EQ(robot->GetArm(ArmIndex(1)).plant().num_positions(), 4);
  EXPECT_EQ(robot->GetArm(ArmIndex(1)).model_instances().size(), 3);
  // Test SetIdleArmsConfigToRef
  Eigen::VectorXd q = Eigen::VectorXd::Zero(6);
  q << 0.1, 0.2, 0.3, 0.4, 0.5, 0.6;
  Eigen::VectorXd q_expected(q);
  // arm 1: flower1_idx, arm 2: flower2_idx,, flower3_idx
  auto flower1_idx {robot->plant().GetModelInstanceByName("flower1")};
  auto flower2_idx {robot->plant().GetModelInstanceByName("flower2")};
  auto flower3_idx {robot->plant().GetModelInstanceByName("flower3")};
  // Test GetPositionFromOriginalPlant
  const auto& arm_1 = robot->GetArm(ArmIndex(0));
  const auto& arm_2 = robot->GetArm(ArmIndex(1));
  auto q_arm_1 = robot->GetArm(ArmIndex(0)).GetPositionFromOriginalPlant(q);
  auto q_arm_2 = robot->GetArm(ArmIndex(1)).GetPositionFromOriginalPlant(q);
  EXPECT_TRUE(q_arm_1.isApprox(q.head(2), 1e-6));
  EXPECT_TRUE(q_arm_2.isApprox(q.tail(4), 1e-6));
  // Test IsGeneralIndexInArm
  for (int i = 0; i < 2; ++i) {
    EXPECT_TRUE(arm_1.IsGeneralIndexInArm(i));
  }
  for (int i = 2; i < 6; ++i) {
    EXPECT_FALSE(arm_1.IsGeneralIndexInArm(i));
  }
  for (int i = 0; i < 2; ++i) {
    EXPECT_FALSE(arm_2.IsGeneralIndexInArm(i));
  }
  for (int i = 2; i < 6; ++i) {
    EXPECT_TRUE(arm_2.IsGeneralIndexInArm(i));
  }
  // Test 1: all arms active. Nothing should change.
  robot->SetIdleArmsConfigToRef(&q, q, {flower1_idx, flower2_idx, flower3_idx});
  EXPECT_TRUE(q.isApprox(q_expected, 1e-6));
  // Test 2: if flower2 is active, then flower3 would also be active.
  // Only flower1 would be inactive.
  const auto q_ref = Eigen::VectorXd::Zero(6);
  robot->SetIdleArmsConfigToRef(&q, q_ref, {flower2_idx});
  q_expected(0) = 0.0;  // flower1(0)
  q_expected(1) = 0.0;  // flower2(1)
  EXPECT_TRUE(q.isApprox(q_expected, 1e-6));
  // Test 3: no active arm. Everything should be set to reference.
  robot->SetIdleArmsConfigToRef(&q, q_ref, {});
  EXPECT_TRUE(q.isApprox(q_ref, 1e-6));
  // Test instance_dof_masks
  EXPECT_EQ(robot->instance_dof_masks().size(), 3);
  EXPECT_EQ(robot->instance_dof_masks().at(flower1_idx).count(), 2);
  EXPECT_EQ(robot->instance_dof_masks().at(flower2_idx).count(), 2);
  EXPECT_EQ(robot->instance_dof_masks().at(flower3_idx).count(), 2);
  EXPECT_TRUE(robot->instance_dof_masks().at(flower1_idx)[0]);
  EXPECT_TRUE(robot->instance_dof_masks().at(flower1_idx)[1]);
  EXPECT_FALSE(robot->instance_dof_masks().at(flower1_idx)[2]);
  // Also test flower3_idx, and not all other combinations
  EXPECT_FALSE(robot->instance_dof_masks().at(flower3_idx)[0]);
  EXPECT_TRUE(robot->instance_dof_masks().at(flower3_idx)[4]);
  EXPECT_TRUE(robot->instance_dof_masks().at(flower3_idx)[5]);
}

TEST(TestRobotModel, IsSystemConfComplete) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);

  Eigen::VectorXd conf_right {7};
  conf_right << -0.89, 1.42, 1.47, -1.54, -0.05, 2.09, -0.27;
  Eigen::VectorXd conf_left {7};
  conf_left << -1.78, -1.20, 0.32, -0.43, 0.38, 1.90, 2.40;

  system_conf_t empty_sysconf;

  EXPECT_FALSE(robot_model->IsSysconfComplete(empty_sysconf))
      << "Empty system conf should not be complete";

  system_conf_t partial_sysconf_right;
  partial_sysconf_right["franka_right"] = conf_right;

  EXPECT_FALSE(robot_model->IsSysconfComplete(partial_sysconf_right))
      << "Partial system conf should not be complete";

  system_conf_t partial_sysconf_left;
  partial_sysconf_left["franka_left"] = conf_left;
  EXPECT_FALSE(robot_model->IsSysconfComplete(partial_sysconf_left))
      << "Partial system conf should not be complete";

  system_conf_t complete_sysconf;
  complete_sysconf["franka_right"] = conf_right;
  complete_sysconf["franka_left"] = conf_left;
  EXPECT_TRUE(robot_model->IsSysconfComplete(complete_sysconf))
      << "Complete system conf should be complete";
}

TEST(TestRobotModel, AddFrameDmd) {
  // Test that adding frames with fixed offsets does not change the hash
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const std::string calibrated_dmd_file {
      "planning_service/test_data/wallflower/calibrated_dmd.yaml"};
  auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto dmd_calibrated {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          calibrated_dmd_file)};
  const auto robot = std::make_unique<RobotModel>(xml_file, dmd);
  const auto calibrated_robot =
      std::make_unique<RobotModel>(xml_file, dmd_calibrated);
  const drake::DefaultHash hash_func;
  const auto hash = hash_func(*robot);
  auto calibrated_hash = hash_func(*calibrated_robot);
  EXPECT_EQ(hash, calibrated_hash);
  // The number of bodies should be the same
  EXPECT_EQ(robot->plant().num_bodies(),
            calibrated_robot->plant().num_bodies());
}

TEST(TestRobotModel, SetFixedOffsetFramePoseInParentFrame) {
  // Test that setting pose works, and does not change hash
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/wallflower/calibrated_dmd.yaml"};
  auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot = std::make_unique<RobotModel>(xml_file, dmd);
  const drake::DefaultHash hash_func;
  auto hash_1 = hash_func(*robot);
  // Add a fixed offset frame to the ball frame
  Eigen::Vector2d q {1.0, 0.4};
  const auto& ball_frame = robot->GetScopedFrameByName("ball");
  const auto& calibrated_ball_frame =
      robot->GetScopedFrameByName("calibrated_ball");
  const auto& shaft_frame = robot->GetScopedFrameByName("shaft");
  const auto& world_frame = robot->plant().world_frame();
  const auto pose_ball {
      robot->CalcRelativeTransform(q, world_frame, ball_frame)};
  const auto pose_ball_offset =
      robot->CalcRelativeTransform(q, world_frame, calibrated_ball_frame);
  EXPECT_TRUE(pose_ball_offset.IsExactlyEqualTo(pose_ball));
  drake::math::RigidTransformd offset(
      drake::math::RollPitchYaw(M_PI / 2, M_PI, 0.01),
      Eigen::Vector3d(0.1, 0.2, 0.3));
  robot->SetFixedOffsetFramePoseInParentFrame(calibrated_ball_frame, offset);
  const auto& pose_ball_offset_2 =
      robot->CalcRelativeTransform(q, world_frame, calibrated_ball_frame);
  const auto X_ball_to_ball_offset_2 = pose_ball.inverse() * pose_ball_offset_2;
  EXPECT_TRUE(X_ball_to_ball_offset_2.IsNearlyEqualTo(offset, 1e-6));
  // Check the transformation between shaft and ball
  const auto X_shaft_ball =
      robot->CalcRelativeTransform(q, shaft_frame, ball_frame);
  // log it
  const auto X_shaft_ball_calibrated =
      robot->CalcRelativeTransform(q, shaft_frame, calibrated_ball_frame);
  // also log offset
  // they should be related by the offset
  auto expected_offset = X_shaft_ball.inverse() * X_shaft_ball_calibrated;
  EXPECT_TRUE(expected_offset.IsNearlyEqualTo(offset, 1e-6));
  const auto expected_X_shaft_ball_calibrated = X_shaft_ball * offset;
  EXPECT_TRUE(X_shaft_ball_calibrated.IsNearlyEqualTo(
      expected_X_shaft_ball_calibrated, 1e-6));
  // With not-fixed-frame, we will have throw an error
  EXPECT_THROW(robot->SetFixedOffsetFramePoseInParentFrame(
                   robot->GetScopedFrameByName("piston"), offset),
               std::runtime_error);
  // Recompute kinematics hash
  auto hash_2 = hash_func(*robot);
  EXPECT_EQ(hash_1, hash_2) << "Setting a fixed offset frame pose in parent "
                               "frame should not change the hash";
}

TEST(TestRobotModel, GhostRobotHash) {
  // URDF 1: ghost_arm.urdf
  // URDF 2: ghost_arm_modified_unimportant_link.urdf
  // That has an unimportant link modified (no downstream collision geo)
  // And some colors change.
  // URDF 3: ghost_arm_modified_important_link.urdf
  // That has an important link modified (with downstream collision geo)
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/ghost/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  const drake::DefaultHash hash_func;
  const auto robot_1 = RobotModel(xml_file, dmd);
  auto hash_1 = hash_func(robot_1);
  // Now change the urdf.
  EXPECT_TRUE(dmd.directives[0].add_model.has_value());
  dmd.directives[0].add_model->file =
      "package://test_models/ghost/ghost_arm_modified_unimportant_link.urdf";
  const auto robot_2 = RobotModel(xml_file, dmd);
  auto hash_2 = hash_func(robot_2);
  EXPECT_EQ(hash_1, hash_2);
  // Now change to the important link modified urdf
  dmd.directives[0].add_model->file =
      "package://test_models/ghost/ghost_arm_modified_important_link.urdf";
  const auto robot_3 = RobotModel(xml_file, dmd);
  auto hash_3 = hash_func(robot_3);
  EXPECT_NE(hash_1, hash_3);
  // Let's move the wall.
  EXPECT_TRUE(dmd.directives[3].add_weld.has_value());
  dmd.directives[3].add_weld->X_PC->translation =
      Eigen::Vector3d(0.5, 0.0, 0.0);
  const auto robot_4 = RobotModel(xml_file, dmd);
  auto hash_4 = hash_func(robot_4);
  EXPECT_NE(hash_3, hash_4);
}

}  // namespace motion
