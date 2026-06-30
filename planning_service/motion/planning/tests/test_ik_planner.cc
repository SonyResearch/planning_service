/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/multibody/parsing/scoped_names.h>
#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/motion/planning/ik_planner.h"

namespace motion {
namespace planning {

using FailureType = IkPlanner::IkResult::FailureStatus::FailureType;

class IkPlannerStub final : public IkPlanner {
 public:
  IkPlannerStub(const RobotConstraints& robot_constraints,
                const std::vector<Eigen::VectorXd>& cache_configs)
      : IkPlanner(robot_constraints, cache_configs) {}
  FRIEND_TEST(TestIkPlanner, GlobalIkUsingCache);
};

TEST(TestIkPlanner, SolveIK) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  // get_frame:
  const auto& frame = robot_model->GetScopedFrameByName("franka_tool_location");
  auto robot_constraints =
      RobotConstraints(*robot_model, ConstraintsAdapter(), 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  const auto& constraints_adapter =
      ik_planner->robot_constraints().constraints_adapter();
  // Check the constraints adapter
  EXPECT_TRUE(constraints_adapter.joint_position_box_constraints.has_value());
  const auto& joint_limits_adapter_vec =
      constraints_adapter.joint_position_box_constraints.value();
  EXPECT_EQ(joint_limits_adapter_vec.at(0).lower_bounds.size(), 2);
  EXPECT_EQ(joint_limits_adapter_vec.at(0).upper_bounds.size(), 2);
  EXPECT_EQ(joint_limits_adapter_vec.at(1).lower_bounds.size(), 7);
  EXPECT_EQ(joint_limits_adapter_vec.at(1).upper_bounds.size(), 7);
  EXPECT_EQ(joint_limits_adapter_vec.at(2).lower_bounds.size(), 1);
  EXPECT_EQ(joint_limits_adapter_vec.at(2).upper_bounds.size(), 1);
  // Create a pose at some location
  Eigen::VectorXd q(10);
  q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6, -2.2, 1.25, -3.0;
  auto plant_context = robot_model->plant().CreateDefaultContext();
  robot_model->plant().SetPositions(plant_context.get(), q);
  const auto& world_frame = robot_model->plant().world_frame();
  drake::math::RigidTransformd pose =
      robot_model->plant().CalcRelativeTransform(*plant_context, world_frame,
                                                 frame);
  // Change the pose a little
  pose.set_translation(pose.translation() + Eigen::Vector3d(0.1, 0.1, 0.1));
  pose.set_rotation(pose.rotation()
                    * drake::math::RotationMatrixd::MakeZRotation(0.1));
  // set the frame and solve IK
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  auto ik_result =
      ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  EXPECT_TRUE(ik_result.is_valid());
  const auto& q_sol = ik_result.value();
  // Check that the solution is close to the desired pose
  robot_model->plant().SetPositions(plant_context.get(), q_sol);
  const auto new_pose = robot_model->plant().CalcRelativeTransform(
      *plant_context, world_frame, frame);
  const auto delta_pose = pose.inverse() * new_pose;
  const auto delta_pose_translation =
      pose.translation() - new_pose.translation();
  const auto eps = 1e-5;  // small tolerance for snopt solver
  EXPECT_LE(std::abs(delta_pose_translation(0)),
            ik_options.position_tolerance(0) + eps);
  EXPECT_LE(std::abs(delta_pose_translation(1)),
            ik_options.position_tolerance(1) + eps);
  EXPECT_LE(std::abs(delta_pose_translation(2)),
            ik_options.position_tolerance(2) + eps);
  const auto angle = delta_pose.rotation().ToAngleAxis().angle();
  EXPECT_GT(std::cos(angle), std::cos(ik_options.orientation_tolerance) - eps);
  EXPECT_LE(angle, ik_options.orientation_tolerance + std::sqrt(eps));
  // it is infeasible for the bowl to reach the franka pose
  const auto& frame_bowl =
      robot_model->GetScopedFrameByName("bowl_center_location");
  EXPECT_FALSE(
      ik_planner->SolveIk(world_frame, frame_bowl, pose, q, 0, ik_options)
          .is_valid());
}

TEST(TestIkPlanner, SolveIKCollisonAvoidance) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  // empty constraints adapter
  auto empty_constraints_adapter = ConstraintsAdapter();
  // Add collision checker
  empty_constraints_adapter.collision_checker = CollisionCheckerAdapter();
  // get_frame:
  const auto& frame = robot_model->GetScopedFrameByName("franka_tool_location");
  auto robot_constraints =
      RobotConstraints(*robot_model, empty_constraints_adapter, 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  // Create a pose at some location
  Eigen::VectorXd q(10);
  q << -0.5, 0.4, 1.2, -1.8, 0, 1.7, 0.2, 0, 0, 0;
  CheckSatisfiedOptions options;
  options.verbose = true;
  EXPECT_FALSE(ik_planner->robot_constraints().CheckSatisfied(q, 0, options));
  auto plant_context = robot_model->plant().CreateDefaultContext();
  robot_model->plant().SetPositions(plant_context.get(), q);
  const auto& world_frame = robot_model->plant().world_frame();
  // Checksatisfied will fail, it is in collision
  const drake::math::RigidTransformd pose =
      robot_model->plant().CalcRelativeTransform(*plant_context, world_frame,
                                                 frame);
  EXPECT_FALSE(ik_planner->SolveIk(world_frame, frame, pose, q).is_valid());
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 10.0;
  ik_options.fix_idle_joints = false;
  ik_options.add_seed_distance_cost = false;
  auto ik_result =
      ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  EXPECT_TRUE(ik_result.is_valid());
  const auto& q_sol = ik_result.value();
  // Check that the solution is close to the desired pose
  robot_model->plant().SetPositions(plant_context.get(), q_sol);
  const auto new_pose = robot_model->plant().CalcRelativeTransform(
      *plant_context, robot_model->plant().world_frame(), frame);
  const auto delta_pose_translation =
      pose.translation() - new_pose.translation();
  const auto eps = 1e-5;  // small tolerance for snopt solver
  EXPECT_LE(std::abs(delta_pose_translation(0)),
            ik_options.position_tolerance(0) + eps);
  EXPECT_LE(std::abs(delta_pose_translation(1)),
            ik_options.position_tolerance(1) + eps);
  EXPECT_LE(std::abs(delta_pose_translation(2)),
            ik_options.position_tolerance(2) + eps);
}

TEST(TestIkPlanner, SolveIkSafeJointLimits) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/franka/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  const auto robot_constraints =
      RobotConstraints(*robot_model, ConstraintsAdapter(), 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  // get_frame:
  const auto& frame = robot_model->GetScopedFrameByName("franka_tool_location");
  // Create a pose at some location with joints very close to joint limits
  Eigen::VectorXd q(7);
  q << -0.5, 0.4, 2.85, -1.2, 0.9, 1.7,
      0;  // joint 3 is super close to joint limit
  CheckSatisfiedOptions options;
  options.verbose = true;
  EXPECT_TRUE(ik_planner->robot_constraints().CheckSatisfied(q, 0, options));
  const auto& world_frame = robot_model->plant().world_frame();
  const auto pose = robot_model->CalcRelativeTransform(q, world_frame, frame);
  auto ik_options = IkPlannerOptions();
  ik_options.joint_limits_safety_margin = 0.0;
  auto ik_result =
      ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  EXPECT_TRUE(ik_result.is_valid());
  // It must be the same solution as the seed, since the joint limits are
  // already satisfied.
  EXPECT_TRUE(ik_result.value().isApprox(q));
  // Now increase the margin
  ik_options.joint_limits_safety_margin = 0.1;
  ik_result = ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  EXPECT_TRUE(ik_result.is_valid());
  // The solution should be different, since the joint limits are now
  // satisfied with a margin.
  EXPECT_FALSE(ik_result.value().isApprox(q));
  logging::log()->info("IkPlanner: SolveIkSafeJointLimits: q_opt = {}",
                       ik_result.value().transpose());
}

TEST(TestIkPlanner, SolveIKWithGripper) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  const auto robot_constraints =
      RobotConstraints(*robot_model, ConstraintsAdapter(), 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  // get_frame:
  const auto& frame =
      robot_model->GetScopedFrameByName("panda_east__panda_link7");
  // Create a pose at some location with joints very close to joint limits
  Eigen::VectorXd q(8);
  q << -0.5, 0.4, 2.7, -1.2, 0.9, 1.7, 0.1, 0.0;
  CheckSatisfiedOptions options;
  options.verbose = true;
  EXPECT_TRUE(ik_planner->robot_constraints().CheckSatisfied(q, 0, options));
  const auto& world_frame = robot_model->plant().world_frame();
  const auto pose = robot_model->CalcRelativeTransform(q, world_frame, frame);
  auto ik_options = IkPlannerOptions();
  ik_options.joint_limits_safety_margin = 0.0;
  logging::log()->critical("Before solve IK");
  auto ik_result =
      ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  EXPECT_TRUE(ik_result.is_valid());
  // It must be the same solution as the seed, since the joint limits are
  // already satisfied.
  EXPECT_TRUE(ik_result.value().isApprox(q));
}

TEST(TestIkPlanner, SolveIKDualArm) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  auto constraints_adapter = ConstraintsAdapter();
  const auto& frame {
      robot_model->GetScopedFrameByName("franka_left::franka_tool_location")};
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  auto robot_constraints =
      RobotConstraints(*robot_model, constraints_adapter, 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  Eigen::VectorXd q(14);
  // clang-format off
  q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6,
       -2.2, 1.25, -2.5, -2.0, 1.5, -0.6, -1.5;
  // clang-format on
  system_conf_t sys_conf_seed = robot_model->ToSystemConf(q);
  const auto& world_frame = robot_model->plant().world_frame();
  auto pose = robot_model->CalcRelativeTransform(q, world_frame, frame);
  // change the pose a little
  pose.set_translation(pose.translation() + Eigen::Vector3d(0.1, 0.1, 0.1));
  // Solve IK
  auto ik_result =
      ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  EXPECT_TRUE(ik_result.is_valid());
  const auto& q_sol = ik_result.value();
  // Check that the solution is close to the desired pose
  system_conf_t sys_conf_sol = robot_model->ToSystemConf(q_sol);
  // for franka_right, the solution should be the same as the seed
  const std::string kRightCobotName {"franka_right"};
  const std::string kLeftCobotName {"franka_left"};
  EXPECT_TRUE(sys_conf_sol.at(kRightCobotName)
                  .isApprox(sys_conf_seed.at(kRightCobotName)));
  EXPECT_FALSE(sys_conf_sol.at(kLeftCobotName)
                   .isApprox(sys_conf_seed.at(kLeftCobotName)));
  // Another test: when franka_right is very close to joint limits: it should
  // not affect franka_left's IK solution + right will not be affected because
  // it is not part of the IK problem
  Eigen::VectorXd upper_limit = robot_model->plant().GetPositionUpperLimits();
  upper_limit -= Eigen::VectorXd::Ones(upper_limit.size()) * 1e-3;
  sys_conf_seed[kRightCobotName] =
      robot_model->ToSystemConf(upper_limit).at(kRightCobotName);
  auto q_right_close_to_limits =
      robot_model->ToGeneralizedPosition(sys_conf_seed);
  auto ik_result_2 = ik_planner->SolveIk(
      world_frame, frame, pose, q_right_close_to_limits, 0, ik_options);
  EXPECT_TRUE(ik_result_2.is_valid());
  const auto& q_sol_2 = ik_result_2.value();
  auto sys_conf_sol_2 = robot_model->ToSystemConf(q_sol_2);
  // for franka_right, the solution should be the same as the seed
  EXPECT_TRUE(sys_conf_sol_2.at(kRightCobotName)
                  .isApprox(sys_conf_seed.at(kRightCobotName)));
  // Nothing has changed for franka_left
  EXPECT_TRUE(sys_conf_sol_2.at(kLeftCobotName)
                  .isApprox(sys_conf_sol.at(kLeftCobotName)));
}

TEST(TestIkPlanner, MultipleFrameIK) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  auto constraints_adapter = ConstraintsAdapter();
  const auto& frame_1 {
      robot_model->GetScopedFrameByName("franka_left::franka_tool_location")};
  const auto& frame_2 {
      robot_model->GetScopedFrameByName("franka_right::franka_tool_location")};
  const auto& world_frame {robot_model->GetScopedFrameByName("world")};
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  auto robot_constraints =
      RobotConstraints(*robot_model, constraints_adapter, 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  Eigen::VectorXd q(14);
  // clang-format off
  q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6,
       -2.2, 1.25, -2.5, -2.0, 1.5, -0.6, -1.5;
  // clang-format on

  // get frame 1 pose
  auto plant_context = robot_model->plant().CreateDefaultContext();
  robot_model->plant().SetPositions(plant_context.get(), q);
  drake::math::RigidTransformd pose_1 =
      robot_model->plant().CalcRelativeTransform(*plant_context, world_frame,
                                                 frame_1);
  // get frame 2 pose
  drake::math::RigidTransformd pose_2 =
      robot_model->plant().CalcRelativeTransform(*plant_context, world_frame,
                                                 frame_2);

  // change the pose a little
  pose_1.set_translation(pose_1.translation() + Eigen::Vector3d(0.1, 0.1, 0.1));
  pose_2.set_translation(pose_2.translation() + Eigen::Vector3d(0.1, 0.1, 0.1));

  // Make FrameRelativePoses for frame 1 and frame 2
  FrameRelativePoses target_frame_relative_poses {
      {&world_frame, &frame_1, pose_1}, {&world_frame, &frame_2, pose_2}};

  // Solve the IK problem
  auto q_sol =
      ik_planner->SolveIk(target_frame_relative_poses, q, 0, ik_options)
          .value();

  // Check that the solution is close to the desired pose
  robot_model->plant().SetPositions(plant_context.get(), q_sol);
  const auto new_pose_1 = robot_model->plant().CalcRelativeTransform(
      *plant_context, robot_model->plant().world_frame(), frame_1);
  const auto delta_pose_translation_1 =
      pose_1.translation() - new_pose_1.translation();
  const auto eps = 1e-5;  // small tolerance for snopt solver
  EXPECT_LE(std::abs(delta_pose_translation_1(0)),
            ik_options.position_tolerance(0) + eps);
  EXPECT_LE(std::abs(delta_pose_translation_1(1)),
            ik_options.position_tolerance(1) + eps);
  EXPECT_LE(std::abs(delta_pose_translation_1(2)),
            ik_options.position_tolerance(2) + eps);

  const auto new_pose_2 = robot_model->plant().CalcRelativeTransform(
      *plant_context, robot_model->plant().world_frame(), frame_2);
  const auto delta_pose_translation_2 =
      pose_2.translation() - new_pose_2.translation();
  EXPECT_LE(std::abs(delta_pose_translation_2(0)),
            ik_options.position_tolerance(0) + eps);
  EXPECT_LE(std::abs(delta_pose_translation_2(1)),
            ik_options.position_tolerance(1) + eps);
  EXPECT_LE(std::abs(delta_pose_translation_2(2)),
            ik_options.position_tolerance(2) + eps);
}

TEST(TestIkPlanner, FixedArmIK) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  auto constraints_adapter = ConstraintsAdapter();
  const auto& frame_1 {
      robot_model->GetScopedFrameByName("franka_left::franka_tool_location")};
  const auto& world_frame {robot_model->GetScopedFrameByName("world")};
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  ik_options.fix_idle_joints = true;
  auto robot_constraints =
      RobotConstraints(*robot_model, constraints_adapter, 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  Eigen::VectorXd q(14);
  // clang-format off
  q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6,
       -2.2, 1.25, -2.5, -2.0, 1.5, -0.6, -1.5;
  // clang-format on

  // get frame 1 pose
  auto plant_context = robot_model->plant().CreateDefaultContext();
  robot_model->plant().SetPositions(plant_context.get(), q);
  drake::math::RigidTransformd pose_1 =
      robot_model->plant().CalcRelativeTransform(*plant_context, world_frame,
                                                 frame_1);

  // change the pose a little
  pose_1.set_translation(pose_1.translation() + Eigen::Vector3d(0.1, 0.1, 0.1));

  // Make FrameRelativePoses for frame 1 and frame 2
  FrameRelativePoses target_frame_relative_poses {
      {&world_frame, &frame_1, pose_1}};

  // Solve the IK problem
  auto q_sol =
      ik_planner->SolveIk(target_frame_relative_poses, q, 0, ik_options)
          .value();

  // Check that the solution is close to the desired pose
  robot_model->plant().SetPositions(plant_context.get(), q_sol);
  const auto new_pose_1 = robot_model->plant().CalcRelativeTransform(
      *plant_context, robot_model->plant().world_frame(), frame_1);
  const auto delta_pose_translation_1 =
      pose_1.translation() - new_pose_1.translation();
  const auto eps = 1e-5;  // small tolerance for snopt solver
  EXPECT_LE(std::abs(delta_pose_translation_1(0)),
            ik_options.position_tolerance(0) + eps);
  EXPECT_LE(std::abs(delta_pose_translation_1(1)),
            ik_options.position_tolerance(1) + eps);
  EXPECT_LE(std::abs(delta_pose_translation_1(2)),
            ik_options.position_tolerance(2) + eps);

  // Get the active model instances for frame 1
  const auto& model_instance_1 = frame_1.model_instance();
  std::vector<drake::multibody::ModelInstanceIndex> fixed_model_instances;
  for (int i = 0; i < robot_model->plant().num_model_instances(); ++i) {
    auto model_instance = drake::multibody::ModelInstanceIndex(i);
    if (model_instance != model_instance_1
        && robot_model->plant().num_positions(model_instance) > 0) {
      fixed_model_instances.push_back(model_instance);
    }
  }
  for (const auto& model_instance : fixed_model_instances) {
    // q_instance and q_sol_instance should be the same
    const int start_idx = robot_model->GetModelStartIndex(model_instance);
    Eigen::VectorXd q_instance = q.segment(
        start_idx, robot_model->plant().num_positions(model_instance));
    Eigen::VectorXd q_sol_instance = q_sol.segment(
        start_idx, robot_model->plant().num_positions(model_instance));
    EXPECT_TRUE(q_instance.isApprox(q_sol_instance));
  }
}

TEST(TestIkPlanner, IgnoreMultiArmCollision) {
  // This test is to check if the dual arm ik planner ignores collisions
  // between the two arms if ignore_multi_arm_collision is set to true,
  // and if it considers them otherwise.
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
  const auto& frame {robot_model->GetScopedFrameByName("flower1::ball")};
  auto robot_constraints = RobotConstraints(*robot_model, adapter, 2);
  auto ik_planner = std::make_unique<IkPlanner>(robot_constraints,
                                                std::vector<Eigen::VectorXd>());
  Eigen::VectorXd q(4);
  q << -M_PI, 0.3, 0, 0.3;
  // This is a conf that flower1 hits flower 2, regardless of the position of
  // the latter.
  CheckSatisfiedOptions options;
  options.verbose = true;
  EXPECT_FALSE(ik_planner->robot_constraints().CheckSatisfied(q, 0, options));
  EXPECT_EQ(ik_planner->robot_constraints().CalcAndClassifyCollisions(q),
            RobotConstraints::CollisionType::kAcrossArmsOnly);
  auto plant_context = robot_model->plant().CreateDefaultContext();
  robot_model->plant().SetPositions(plant_context.get(), q);
  const auto& world_frame = robot_model->plant().world_frame();
  drake::math::RigidTransformd pose =
      robot_model->plant().CalcRelativeTransform(*plant_context, world_frame,
                                                 frame);
  // Fix the z, but let's move x and y. We don't care about the rotation as
  // well.
  pose.set_translation(pose.translation() + Eigen::Vector3d(0.05, -0.03, 0));
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = M_PI;
  ik_options.make_collision_avoidance_constraint = false;
  ik_options.resolve_with_collision_avoidance = true;
  // Solve the IK problem
  auto ik_result =
      ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  // This should fail, since the two arms are in collision.
  EXPECT_FALSE(ik_result.is_valid());
  EXPECT_FALSE(ik_result.multiarm_collision());
  // Check if the message contains the collision information
  logging::log()->info("IkPlanner: SolveIk failed with error: {}",
                       ik_result.failure_status_message());
  EXPECT_TRUE(
      ik_result.failure_status_message().find("Minimum distance constraint")
      != std::string::npos);
  ik_options.fix_idle_joints = false;
  ik_options.ignore_multi_arm_collision = true;
  // Solve the IK problem again. This time it should be succeed.
  ik_result = ik_planner->SolveIk(world_frame, frame, pose, q, 0, ik_options);
  EXPECT_FALSE(ik_result.is_valid());
  EXPECT_TRUE(ik_result.multiarm_collision());
  const auto& q_sol = ik_result.value();
  EXPECT_FALSE(
      ik_planner->robot_constraints().CheckSatisfied(q_sol, 0, options));
  EXPECT_EQ(ik_planner->robot_constraints().CalcAndClassifyCollisions(q_sol, 0),
            RobotConstraints::CollisionType::kAcrossArmsOnly);
}

TEST(TestIkPlanner, GlobalIkUsingCache) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  auto constraints_adapter = ConstraintsAdapter();
  const auto& frame_B {
      robot_model->GetScopedFrameByName("franka_left::franka_tool_location")};
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  // Let's make a random ik cache adapter
  std::vector<Eigen::VectorXd> cache_configs;
  drake::RandomGenerator gen(0);
  int num_samples = 100;
  const auto& lb = robot_model->plant().GetPositionLowerLimits();
  const auto& ub = robot_model->plant().GetPositionUpperLimits();
  auto rectangle = drake::geometry::optimization::Hyperrectangle(lb, ub);
  for (int i = 0; i < num_samples; ++i) {
    Eigen::VectorXd q = rectangle.UniformSample(&gen);
    cache_configs.push_back(q);
  }
  auto robot_constraints =
      RobotConstraints(*robot_model, constraints_adapter, 2);
  auto ik_planner_with_random_cache =
      std::make_unique<IkPlannerStub>(robot_constraints, cache_configs);
  Eigen::VectorXd q(14);
  // clang-format off
  q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6,
       -2.2, 1.25, -2.5, -2.0, 1.5, -0.6, -1.5;
  // clang-format on
  auto plant_context = robot_model->plant().CreateDefaultContext();
  robot_model->plant().SetPositions(plant_context.get(), q);
  drake::math::RigidTransformd pose =
      robot_model->plant().CalcRelativeTransform(
          *plant_context, robot_model->plant().world_frame(), frame_B);
  // change the pose a little
  pose.set_translation(pose.translation() + Eigen::Vector3d(0.1, 0.1, 0.1));
  // Solve IK
  FrameRelativePoses frps {
      {&robot_model->plant().world_frame(), &frame_B, pose}};
  EXPECT_TRUE(ik_planner_with_random_cache
                  ->SolveGlobalIkUsingCache(frps, q, 0, ik_options)
                  .is_valid());
  std::vector<Eigen::VectorXd> cache_configs_out = {};
  auto ik_planner_no_cache =
      std::make_unique<IkPlannerStub>(robot_constraints, cache_configs_out);
  FrameRelativePoses frps_no_cache {
      {&robot_model->plant().world_frame(), &frame_B, pose}};
  // Without cache but with q as q_ref, the IK should succeed.
  EXPECT_TRUE(ik_planner_no_cache
                  ->SolveGlobalIkUsingCache(frps_no_cache, q, 0, ik_options)
                  .is_valid());
  // With empty cache, and non-sense q_ref, it should fail.
  auto q_zero = Eigen::VectorXd::Zero(14);
  EXPECT_FALSE(
      ik_planner_no_cache
          ->SolveGlobalIkUsingCache(frps_no_cache, q_zero, 0, ik_options)
          .is_valid());
}

TEST(TestIkPlanner, GlobalIk) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  const auto& frame {
      robot_model->GetScopedFrameByName("franka_left::franka_tool_location")};
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  // Let's make a random ik cache adapter
  std::vector<Eigen::VectorXd> random_configs;
  drake::RandomGenerator gen(0);
  int num_samples = 100;
  const auto& lb = robot_model->plant().GetPositionLowerLimits();
  const auto& ub = robot_model->plant().GetPositionUpperLimits();
  auto rectangle = drake::geometry::optimization::Hyperrectangle(lb, ub);
  for (int i = 0; i < num_samples; ++i) {
    Eigen::VectorXd q = rectangle.UniformSample(&gen);
    random_configs.push_back(q);
  }
  auto robot_constraints =
      RobotConstraints(*robot_model, ConstraintsAdapter(), 2);
  auto ik_planner_with_cache =
      std::make_unique<IkPlanner>(robot_constraints, random_configs);
  Eigen::VectorXd q(14);
  // clang-format off
  q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6,
       -2.2, 1.25, -2.5, -2.0, 1.5, -0.6, -1.5;
  // clang-format on
  auto plant_context = robot_model->plant().CreateDefaultContext();
  robot_model->plant().SetPositions(plant_context.get(), q);
  drake::math::RigidTransformd pose =
      robot_model->plant().CalcRelativeTransform(
          *plant_context, robot_model->plant().world_frame(), frame);
  // change the pose a little
  pose.set_translation(pose.translation() + Eigen::Vector3d(0.1, 0.1, 0.1));
  FrameRelativePoses frps {{&robot_model->plant().world_frame(), &frame, pose}};
  // Solve IK
  EXPECT_TRUE(
      ik_planner_with_cache->SolveGlobalIk(frps, q, ik_options).is_valid());
  // If we have 1 thread, it should throw.
  auto robot_constraints_1_thread =
      RobotConstraints(*robot_model, ConstraintsAdapter(), 1);
  auto ik_planner_one_thread =
      std::make_unique<IkPlanner>(robot_constraints_1_thread, random_configs);
  EXPECT_THROW(ik_planner_one_thread->SolveGlobalIk(frps, q, ik_options),
               std::runtime_error);
  // Let's try a ik_planner with zero cache but with random seeds.
  auto ik_planner_no_cache = std::make_unique<IkPlanner>(
      robot_constraints, std::vector<Eigen::VectorXd>());
  ik_options.num_random_seeds = 0;
  ik_options.fix_idle_joints = false;
  // Make the q_current to be the lower bounds, which is far from the actual
  // solution.
  q = robot_model->plant().GetPositionLowerLimits();
  auto sol = ik_planner_no_cache->SolveGlobalIk(frps, q, ik_options);
  EXPECT_FALSE(sol.is_valid());
  EXPECT_TRUE(sol.failure_status().failure_type
                  == FailureType::kOptimizationNearSingularity
              || sol.failure_status().failure_type
                     == FailureType::kOptimizationNearJointLimits);
  EXPECT_GE(ik_planner_no_cache->ik_cache().size(), 0);
  logging::log()->info("IkPlanner: SolveGlobalIk failed with error: {}",
                       sol.failure_status_message());
  // Let's try a ik_planner with zero cache but with random seeds.
  ik_options.num_random_seeds = 10;
  ik_options.insert_random_seed_into_cache = true;
  EXPECT_TRUE(
      ik_planner_no_cache->SolveGlobalIk(frps, q, ik_options).is_valid());
  // Now we actually have a cache.
  EXPECT_GE(ik_planner_no_cache->ik_cache().size(), 1);
  // Resolving the same IK problem should succeed.
  logging::log()->info(
      "IkPlanner: SolveGlobalIk with random seeds succeeded. Now retrying to "
      "see if cache was used.");
  ik_options.num_random_seeds = 0;
  auto sol2 = ik_planner_no_cache->SolveGlobalIk(frps, q, ik_options);
  EXPECT_TRUE(sol2.is_valid());
  if (!sol2.is_valid()) {
    logging::log()->error("IkPlanner: SolveGlobalIk failed with error: {}",
                          sol2.failure_status_message());
  }
}

TEST(TestIkPlanner, GlobalIk_SelfCollision) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/franka/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  const auto& frame =
      robot_model->GetScopedFrameByName("franka::franka_tool_location");
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  ik_options.self_collision_resolve_with_constraint = false;
  ik_options.resolve_with_collision_avoidance = false;
  // Let's make a random ik cache adapter
  std::vector<Eigen::VectorXd> random_configs;
  drake::RandomGenerator gen(0);
  int num_samples = 10;
  const auto& lb = robot_model->plant().GetPositionLowerLimits();
  const auto& ub = robot_model->plant().GetPositionUpperLimits();
  auto rectangle = drake::geometry::optimization::Hyperrectangle(lb, ub);
  for (int i = 0; i < num_samples; ++i) {
    Eigen::VectorXd q = rectangle.UniformSample(&gen);
    random_configs.push_back(q);
  }
  ConstraintsAdapter adapter;
  adapter.collision_checker = CollisionCheckerAdapter();
  auto robot_constraints = RobotConstraints(*robot_model, adapter, 2);
  auto ik_planner =
      std::make_unique<IkPlanner>(robot_constraints, random_configs);
  // A q that is in self-collision
  Eigen::VectorXd q(7);
  q << 0.43, 1.73, -1.1, -2.9, -0.1, 2.12, -0.3;
  // See if this conf is in self-collision
  auto collision_type = robot_constraints.CalcAndClassifyCollisions(q);
  EXPECT_EQ(collision_type, RobotConstraints::CollisionType::kArmSelfOnly);
  // Get pose of frame
  auto pose = robot_model->CalcRelativeTransform(
      q, robot_model->plant().world_frame(), frame);
  // Solve Global IK with Cache
  FrameRelativePoses frps {{&robot_model->plant().world_frame(), &frame, pose}};
  // Solve IK with seed will result in self-collision, and invalid
  auto result = ik_planner->SolveIk(frps, q, 0, ik_options);
  EXPECT_FALSE(result.is_valid());
  EXPECT_TRUE(result.self_collision());
  // Give an absurd seed
  const auto q_absurd_seed = ub;
  EXPECT_FALSE(ik_planner->SolveIk(frps, q_absurd_seed, 0, ik_options)
                   .optimization_success());
  // Now try global IK
  auto ik_global_result =
      ik_planner->SolveGlobalIk(frps, q_absurd_seed, ik_options);
  EXPECT_TRUE(ik_global_result.is_valid());
}

TEST(TestIkPlanner, GlobalDualFrankaWithGripper) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dual_arm.dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.collision_checker = CollisionCheckerAdapter();
  auto robot_constraints =
      RobotConstraints(*robot_model, ConstraintsAdapter(), 2);
  const auto& frame {robot_model->GetScopedFrameByName(
      "franka_left::panda_east__panda_link7")};
  auto ik_options = IkPlannerOptions();
  ik_options.position_tolerance = Eigen::Vector3d::Ones() * 1e-4;
  ik_options.orientation_tolerance = 5e-3;
  // Let's make a random ik cache adapter
  drake::RandomGenerator gen(0);
  SampleOptions sample_options {.parallel = false};
  auto random_configs =
      robot_constraints.GenerateSamples(&gen, 10, sample_options);
  auto ik_planner_without_cache = std::make_unique<IkPlanner>(
      robot_constraints, std::vector<Eigen::VectorXd>());
  Eigen::VectorXd q_sample = random_configs.back();
  drake::math::RigidTransformd pose = robot_model->CalcRelativeTransform(
      q_sample, robot_model->plant().world_frame(), frame);
  // change the pose a little
  pose.set_translation(pose.translation() + Eigen::Vector3d(0.01, 0.01, 0.01));
  FrameRelativePoses frps {{&robot_model->plant().world_frame(), &frame, pose}};
  // Solve Global IK
  auto ik_result =
      ik_planner_without_cache->SolveGlobalIk(frps, q_sample, ik_options);
  EXPECT_TRUE(ik_result.is_valid());
  EXPECT_EQ(ik_result.value().size(), 14 + 2);
  // Give it a very far seed
  Eigen::VectorXd q_far_lifted = robot_model->plant().GetPositionUpperLimits();
  auto q_far = robot_model->holonomic_mapping().Reduce(q_far_lifted);
  // clang-format on
  auto ik_result_2 =
      ik_planner_without_cache->SolveGlobalIk(frps, q_far, ik_options);
  EXPECT_FALSE(ik_result_2.is_valid());
  // Build one with cache
  auto ik_planner_with_cache =
      std::make_unique<IkPlanner>(robot_constraints, random_configs);
  auto ik_result_3 =
      ik_planner_with_cache->SolveGlobalIk(frps, q_far, ik_options);
  EXPECT_TRUE(ik_result_3.is_valid());
  // Check the value
  logging::log()->info("IkPlanner: Global IK with cache succeeded. q_sol: {}",
                       ik_result_3.value().transpose());
  // The idle models should be the same as q_far
  EXPECT_EQ(ik_result_3.value().size(), 16);
  EXPECT_EQ(ik_result_3.value()(7), q_far(7));
  EXPECT_EQ(ik_result_3.value().block(7, 0, 9, 1), q_far.block(7, 0, 9, 1));
}

TEST(TestIkPlanner, CalcInterpolatedPose) {
  const auto pose_1 = drake::math::RigidTransformd::Identity();
  const auto pose_2 = drake::math::RigidTransformd(
      drake::math::RotationMatrixd::MakeZRotation(0.6),
      Eigen::Vector3d(0.1, 0.1, 0.1));
  const auto pose_start = IkPlanner::CalcInterpolatedPose(pose_1, pose_2, 0.0);
  const auto pose_between =
      IkPlanner::CalcInterpolatedPose(pose_1, pose_2, 0.75);
  const auto pose_end = IkPlanner::CalcInterpolatedPose(pose_1, pose_2, 1.0);
  EXPECT_TRUE(pose_start.GetAsMatrix4().isApprox(pose_1.GetAsMatrix4()));
  EXPECT_TRUE(pose_end.GetAsMatrix4().isApprox(pose_2.GetAsMatrix4()));
  EXPECT_TRUE(pose_between.translation().isApprox(
      Eigen::Vector3d(0.075, 0.075, 0.075)));
  EXPECT_TRUE(pose_between.rotation().matrix().isApprox(
      drake::math::RotationMatrixd::MakeZRotation(0.45).matrix()));
}

TEST(TestIkPlanner, EvalRobotLimitsString) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_wallflowers/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  auto ik_planner =
      IkPlanner(RobotConstraints(*robot_model, ConstraintsAdapter(), 2),
                std::vector<Eigen::VectorXd>());
  auto lower_limit = robot_model->plant().GetPositionLowerLimits();
  auto upper_limit = robot_model->plant().GetPositionUpperLimits();
  const auto q_center = 0.5 * (lower_limit + upper_limit);
  Eigen::VectorXd q {q_center};
  double eps = 5e-4;
  q(0) = lower_limit(0) + eps;
  // Check if string contains flower1[0], eps, and lower keywords
  auto dut = ik_planner.EvalRobotLimits(q);
  EXPECT_EQ(dut.failure_type, FailureType::kOptimizationNearJointLimits);
  EXPECT_TRUE(dut.message.find("flower1[0]") != std::string::npos);
  EXPECT_TRUE(dut.message.find("lower limit") != std::string::npos);
  EXPECT_TRUE(dut.message.find("0.0005") != std::string::npos);
  // Test upper limit for flower2.
  q = q_center;
  q(3) = upper_limit(3) - eps;
  dut = ik_planner.EvalRobotLimits(q);
  EXPECT_EQ(dut.failure_type, FailureType::kOptimizationNearJointLimits);
  EXPECT_TRUE(dut.message.find("flower2[1]") != std::string::npos);
  EXPECT_TRUE(dut.message.find("upper limit") != std::string::npos);
  EXPECT_TRUE(dut.message.find("0.0005") != std::string::npos);
}

TEST(TestIkPlanner, CalcConfigDeltaFromSpatialDelta) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  auto robot_constraints =
      RobotConstraints(*robot_model, ConstraintsAdapter(), 2);
  auto ik_planner =
      IkPlanner(robot_constraints, std::vector<Eigen::VectorXd>());
  Eigen::VectorXd q(14);
  // clang-format off
    q << -2.0, 0.5, -0.6, -1.5, 0.5, 1.6, -0.6,
          -2.2, 1.25, -2.5, -2.0, 1.5, -0.6, -1.5;
  // clang-format on
  std::string frame_A_name = "world";
  std::string frame_B_name = "franka_left::franka_link8";
  std::string frame_E_name = "world";
  const auto& frame_A = robot_model->GetScopedFrameByName(frame_A_name);
  const auto& frame_B = robot_model->GetScopedFrameByName(frame_B_name);
  const auto& frame_E = robot_model->GetScopedFrameByName(frame_E_name);
  Eigen::Vector3d translation_delta(0.01, -0.02, 0.03);
  Eigen::Vector3d rotation_delta(0.05, 0.01, -0.03);
  auto spatial_delta = drake::multibody::SpatialVelocity<double>(
      rotation_delta, translation_delta);
  auto config_delta = ik_planner.CalcConfigDeltaFromSpatialDelta(
      q, frame_A, frame_B, frame_E, spatial_delta);
  // Log config delta for debugging
  logging::log()->info("Config delta: {}", config_delta.transpose());
  // Check the size of config_delta
  EXPECT_EQ(config_delta.size(), q.size());
  auto sys_delta = robot_model->ToSystemConf(config_delta);
  // Check that the spatial delta is zero for franka_right
  EXPECT_TRUE(sys_delta.at("franka_right").isApprox(Eigen::VectorXd::Zero(7)));
  // Check that config+delta the pose of frame_B in frame_E is approximately the
  // same as the original pose plus the spatial delta.
  const auto pose = robot_model->CalcRelativeTransform(q, frame_A, frame_B);
  const auto pose_with_delta =
      robot_model->CalcRelativeTransform(q + config_delta, frame_A, frame_B);
  const auto delta_pose_translation =
      pose_with_delta.translation() - pose.translation();
  // Compute the rotation delta as an angle-axis vector (rotation vector),
  // consistent with the angular component of SpatialVelocity.
  const auto R_delta = pose_with_delta.rotation() * pose.rotation().inverse();
  const Eigen::AngleAxisd angle_axis(R_delta.matrix());
  const Eigen::Vector3d delta_pose_rotvec =
      angle_axis.angle() * angle_axis.axis();
  // Log the deltas for debugging
  logging::log()->info("Translation delta: {}",
                       delta_pose_translation.transpose());
  logging::log()->info("Rotation delta (rot vec): {}",
                       delta_pose_rotvec.transpose());
  // Errors up to second order are expected, so we use 0.1 relative tolerance
  // here.
  EXPECT_TRUE(delta_pose_translation.isApprox(translation_delta, 1e-1));
  EXPECT_TRUE(delta_pose_rotvec.isApprox(rotation_delta, 1e-1));
}

}  // namespace planning
}  // namespace motion
