#include <gtest/gtest.h>

#include "planning_service/common/string_utils.h"
#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

using motion::system_conf_t;

TEST(TestDracoPlanner, TestGeneralizedMultimodalPlan) {
  const auto planner = DracoPlanner(test::DualPandas());
  logging::log()->info("Defining generalized multimodal planning problem...");
  // Define start and goal system configurations
  planning_service_client::SystemConf system_conf_start;
  system_conf_t sysconf_start;
  Eigen::VectorXd q_right(7), q_left_start(7), q_left_goal(7);
  q_right << -0.44, 0.501, -0.227, -1.822, -1.635, 2.006, -0.028;
  q_left_start << 2.077, 0.664, 1.083, -2.460, 1.839, 1.579, 0.164;
  q_left_goal << -1.0, 0.664, 1.083, -2.460, 1.839, 1.579, 0.164;
  sysconf_start["franka_right"] = q_right;
  sysconf_start["franka_left"] = q_left_start;
  for (const auto& [key, value] : sysconf_start) {
    system_conf_start[key] = value;
  }
  planning_service_client::SystemConf system_conf_goal;
  system_conf_t sysconf_goal;
  sysconf_goal["franka_right"] = q_right;
  sysconf_goal["franka_left"] = q_left_goal;

  for (const auto& [key, value] : sysconf_goal) {
    system_conf_goal[key] = value;
  }
  const auto& world_frame = planner.robot_model().GetScopedFrameByName("world");
  // Define wayposes for left arm
  const std::string left_ik_frame = "franka_left::franka_tool_location";
  auto seed_pose_left_start = planner.robot_model().CalcRelativeTransform(
      planner.robot_model().ToGeneralizedPosition(sysconf_start), world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  auto seed_pose_left_goal = planner.robot_model().CalcRelativeTransform(
      planner.robot_model().ToGeneralizedPosition(sysconf_goal), world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  Eigen::Vector3d seed_translation_left_start =
      seed_pose_left_start.translation();
  Eigen::Quaterniond seed_quaternion_left_start =
      seed_pose_left_start.rotation().ToQuaternion();
  Eigen::Vector3d seed_translation_left_goal =
      seed_pose_left_goal.translation();
  Eigen::Quaterniond seed_quaternion_left_goal =
      seed_pose_left_goal.rotation().ToQuaternion();
  // Start wayposes
  std::vector<planning_service_client::FrameRelativePose> start_wayposes_1 = {
      planning_service_client::FrameRelativePose("world", left_ik_frame,
                                                 seed_translation_left_start,
                                                 seed_quaternion_left_start)};
  std::vector<planning_service_client::FrameRelativePose> start_wayposes_2 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0, 0.01),
          seed_quaternion_left_start)};
  std::vector<planning_service_client::FrameRelativePose> start_wayposes_3 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0.01, 0),
          seed_quaternion_left_start)};
  std::vector<planning_service_client::FrameRelativePose> start_wayposes_4 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0, -0.01),
          seed_quaternion_left_start)};
  std::vector<planning_service_client::FrameRelativePose> start_wayposes_5 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, -0.01, 0),
          seed_quaternion_left_start)};
  planning_service_client::FrameRelativePosesVec start_wayposes_vec_1(
      start_wayposes_1);
  planning_service_client::FrameRelativePosesVec start_wayposes_vec_2(
      start_wayposes_2);
  planning_service_client::FrameRelativePosesVec start_wayposes_vec_3(
      start_wayposes_3);
  planning_service_client::FrameRelativePosesVec start_wayposes_vec_4(
      start_wayposes_4);
  planning_service_client::FrameRelativePosesVec start_wayposes_vec_5(
      start_wayposes_5);
  std::vector<planning_service_client::planner::Anchor> start_anchors;
  planning_service_client::SystemConf fixed_right_sysconf;
  fixed_right_sysconf["franka_right"] = q_right;
  planning_service_client::planner::Anchor start_anchor_1 {fixed_right_sysconf,
                                                           start_wayposes_1};
  planning_service_client::planner::Anchor start_anchor_2 {fixed_right_sysconf,
                                                           start_wayposes_2};
  planning_service_client::planner::Anchor start_anchor_3 {fixed_right_sysconf,
                                                           start_wayposes_3};
  planning_service_client::planner::Anchor start_anchor_4 {fixed_right_sysconf,
                                                           start_wayposes_4};
  planning_service_client::planner::Anchor start_anchor_5 {fixed_right_sysconf,
                                                           start_wayposes_5};
  start_anchors.push_back(start_anchor_1);
  start_anchors.push_back(start_anchor_2);
  start_anchors.push_back(start_anchor_3);
  start_anchors.push_back(start_anchor_4);
  start_anchors.push_back(start_anchor_5);
  // Goal wayposes
  std::vector<planning_service_client::FrameRelativePose> goal_wayposes_1 = {
      planning_service_client::FrameRelativePose("world", left_ik_frame,
                                                 seed_translation_left_goal,
                                                 seed_quaternion_left_goal)};
  std::vector<planning_service_client::FrameRelativePose> goal_wayposes_2 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_goal + Eigen::Vector3d(0, 0, 0.02),
          seed_quaternion_left_goal)};
  std::vector<planning_service_client::FrameRelativePose> goal_wayposes_3 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_goal + Eigen::Vector3d(0.02, 0, 0),
          seed_quaternion_left_goal)};
  planning_service_client::FrameRelativePosesVec goal_wayposes_vec_1(
      goal_wayposes_1);
  planning_service_client::FrameRelativePosesVec goal_wayposes_vec_2(
      goal_wayposes_2);
  planning_service_client::FrameRelativePosesVec goal_wayposes_vec_3(
      goal_wayposes_3);
  std::vector<planning_service_client::planner::Anchor> goal_anchors;
  planning_service_client::planner::Anchor goal_anchor_1 {fixed_right_sysconf,
                                                          goal_wayposes_1};
  planning_service_client::planner::Anchor goal_anchor_2 {fixed_right_sysconf,
                                                          goal_wayposes_2};
  planning_service_client::planner::Anchor goal_anchor_3 {fixed_right_sysconf,
                                                          goal_wayposes_3};
  goal_anchors.push_back(goal_anchor_1);
  goal_anchors.push_back(goal_anchor_2);
  goal_anchors.push_back(goal_anchor_3);
  std::vector<bool> fast_estimate_options = {false, true};
  for (const auto& fast_estimate : fast_estimate_options) {
    auto time_now = std::chrono::high_resolution_clock::now();
    // Build the problem
    planning_service_client::planner::GeneralizedMultimodalPlanningProblem
        gen_multimodal_plan {start_anchors, goal_anchors, fast_estimate};
    // Solve the plan
    auto result = planner.SolvePlan(gen_multimodal_plan, "", std::nullopt,
                                    system_conf_start);
    EXPECT_TRUE(result.is_success());
    auto sys_traj = result.system_timed_trajectory();
    EXPECT_TRUE(sys_traj.has_key("franka_right"));
    EXPECT_TRUE(sys_traj.has_key("franka_left"));
    auto traj_left = sys_traj.at("franka_left");
    auto traj_right = sys_traj.at("franka_right");
    EXPECT_NEAR(traj_right.start_time(), 0.0, 1e-4);
    EXPECT_TRUE(traj_right.Value(0).isApprox(q_right));
    EXPECT_TRUE(traj_right.Value(traj_right.end_time())
                    .isApprox(q_right));  // because the first waypose is the
    EXPECT_NEAR(traj_left.start_time(), 0.0, 1e-4);
    EXPECT_TRUE(traj_left.Value(0).isApprox(
        q_left_start));  // because the first waypose is the FK of the start
                         // conf
    // Check that the end pose matches the goal waypose
    system_conf_t end_sysconf;
    end_sysconf["franka_left"] = traj_left.Value(traj_left.end_time());
    end_sysconf["franka_right"] = traj_right.Value(traj_right.end_time());
    auto evaluated_goal_pose_left = planner.robot_model().CalcRelativeTransform(
        planner.robot_model().ToGeneralizedPosition(end_sysconf), world_frame,
        planner.robot_model().GetScopedFrameByName(left_ik_frame));
    drake::math::RigidTransformd left_goal_frp_pose(
        goal_wayposes_3[0].X_AB_quaternion(),
        goal_wayposes_3[0].X_AB_translation());
    auto X_to_goal_ik_end =
        left_goal_frp_pose.inverse() * evaluated_goal_pose_left;
    EXPECT_LT(X_to_goal_ik_end.translation().norm(), 1e-3);
    EXPECT_LT(X_to_goal_ik_end.rotation().ToAngleAxis().angle(), 1e-3);
    logging::log()->info(
        "Fast estimate: {}: Planning took {} seconds.", fast_estimate,
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now()
                                      - time_now)
            .count());
  }
  // Test with unreachable waypose (should fail)
  std::vector<planning_service_client::FrameRelativePose> bad_wayposes = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_goal + Eigen::Vector3d(0, 0, 3.0),
          seed_quaternion_left_goal)};
  planning_service_client::planner::Anchor goal_anchor_bad {fixed_right_sysconf,
                                                            bad_wayposes};
  std::vector<planning_service_client::planner::Anchor> goal_anchors_bad;
  goal_anchors_bad.push_back(goal_anchor_bad);
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      bad_plan {start_anchors, goal_anchors_bad};
  auto bad_result =
      planner.SolvePlan(bad_plan, "", std::nullopt, system_conf_start);
  EXPECT_FALSE(bad_result.is_success());
  EXPECT_TRUE(bad_result.system_timed_trajectory().size() == 0);
  // Expect the error message to contain "kOptimization"
  EXPECT_TRUE(
      common::utils::string_includes(bad_result.message(), "kOptimization"));
}

TEST(GeneralizedMultimodalPlan, TestSolveSequentialAnchors) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Define start and goal system configurations
  planning_service_client::SystemConf system_conf_start;
  system_conf_t sysconf_start;
  Eigen::VectorXd q_right(7), q_left_start(7), q_left_goal(7);
  q_right << -0.44, 0.501, -0.227, -1.822, -1.635, 2.006, -0.028;
  q_left_start << 2.077, 0.664, 1.083, -2.46, 1.84, 1.58, 0.16;
  q_left_goal << -1.0, 0.664, 1.083, -2.46, 1.84, 1.58, 0.16;
  sysconf_start["franka_right"] = q_right;
  sysconf_start["franka_left"] = q_left_start;
  for (const auto& [key, value] : sysconf_start) {
    system_conf_start[key] = value;
  }
  planning_service_client::SystemConf system_conf_goal;
  system_conf_t sysconf_goal;
  sysconf_goal["franka_right"] = q_right;
  sysconf_goal["franka_left"] = q_left_goal;
  for (const auto& [key, value] : sysconf_goal) {
    system_conf_goal[key] = value;
  }
  Eigen::VectorXd q_start {
      planner.robot_model().ToGeneralizedPosition(sysconf_start)};

  const auto& world_frame = planner.robot_model().GetScopedFrameByName("world");
  // IK frame for left arm
  const std::string left_ik_frame = "franka_left::franka_tool_location";
  auto seed_pose_left_start = planner.robot_model().CalcRelativeTransform(
      q_start, world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  Eigen::Vector3d seed_translation_left_start =
      seed_pose_left_start.translation();
  Eigen::Quaterniond seed_quaternion_left_start =
      seed_pose_left_start.rotation().ToQuaternion();
  // Create a sequence of anchors for the left arm
  std::vector<planning_service_client::planner::Anchor> anchors;
  planning_service_client::SystemConf right_sysconf_1;
  right_sysconf_1["franka_right"] =
      q_right + Eigen::VectorXd::Constant(7, 0.01);
  planning_service_client::SystemConf right_sysconf_2;
  right_sysconf_2["franka_right"] =
      q_right + Eigen::VectorXd::Constant(7, 0.02);
  planning_service_client::SystemConf right_sysconf_3;
  right_sysconf_3["franka_right"] =
      q_right + Eigen::VectorXd::Constant(7, 0.03);
  // Anchor 1: initial pose
  std::vector<planning_service_client::FrameRelativePose> wayposes_1 = {
      planning_service_client::FrameRelativePose("world", left_ik_frame,
                                                 seed_translation_left_start,
                                                 seed_quaternion_left_start)};
  anchors.emplace_back(right_sysconf_1, wayposes_1);
  // Anchor 2: move up in z
  std::vector<planning_service_client::FrameRelativePose> wayposes_2 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0, 0.02),
          seed_quaternion_left_start)};
  anchors.emplace_back(right_sysconf_2, wayposes_2);
  // Anchor 3: move right in x
  std::vector<planning_service_client::FrameRelativePose> wayposes_3 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0.02, 0, 0),
          seed_quaternion_left_start)};
  anchors.emplace_back(right_sysconf_3, wayposes_3);
  // Solve the sequence
  auto configs = planner.SolveSequentialAnchors(anchors, q_start);
  EXPECT_TRUE(configs.has_value());
  EXPECT_EQ(configs.value().size(), anchors.size());
  // Check that each config matches the expected pose
  for (size_t i = 0; i < configs.value().size(); ++i) {
    auto pose = planner.robot_model().CalcRelativeTransform(
        configs.value()[i], world_frame,
        planner.robot_model().GetScopedFrameByName(left_ik_frame));
    drake::math::RigidTransformd expected_pose(
        anchors[i].poses()[0].X_AB_quaternion(),
        anchors[i].poses()[0].X_AB_translation());
    auto error = expected_pose.inverse() * pose;
    EXPECT_LT(error.translation().norm(), 2e-3);
    EXPECT_LT(error.rotation().ToAngleAxis().angle(), 2e-3);
  }
  // Test with unreachable anchor (should fail)
  std::vector<planning_service_client::planner::Anchor> bad_anchors = anchors;
  // Add an unreachable anchor (move far in z)
  std::vector<planning_service_client::FrameRelativePose> bad_wayposes = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0, 3.0),
          seed_quaternion_left_start)};
  bad_anchors.emplace_back(right_sysconf_1, bad_wayposes);
  auto bad_configs = planner.SolveSequentialAnchors(bad_anchors, q_start, true);
  EXPECT_FALSE(bad_configs.has_value());
  // Expect the error message to contain "kOptimization"
  logging::log()->info("Bad configs error message: {}", bad_configs.error());
  EXPECT_TRUE(
      common::utils::string_includes(bad_configs.error(), "kOptimization"));
}

TEST(GeneralizedMultimodalPlan, TestSolveSequentialAnchorsIncomplete) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Define start system configuration
  planning_service_client::SystemConf system_conf_start;
  system_conf_t sysconf_start;
  Eigen::VectorXd q_right(7), q_left_start(7);
  q_right << -0.44, 0.501, -0.227, -1.822, -1.635, 2.006, -0.028;
  q_left_start << 2.077, 0.664, 1.083, -2.46, 1.84, 1.58, 0.16;
  sysconf_start["franka_right"] = q_right;
  sysconf_start["franka_left"] = q_left_start;
  for (const auto& [key, value] : sysconf_start) {
    system_conf_start[key] = value;
  }

  Eigen::VectorXd q_start {
      planner.robot_model().ToGeneralizedPosition(sysconf_start)};

  const auto& world_frame = planner.robot_model().GetScopedFrameByName("world");
  const std::string left_ik_frame = "franka_left::franka_tool_location";
  auto seed_pose_left_start = planner.robot_model().CalcRelativeTransform(
      q_start, world_frame,
      planner.robot_model().GetScopedFrameByName(left_ik_frame));
  Eigen::Vector3d seed_translation_left_start =
      seed_pose_left_start.translation();
  Eigen::Quaterniond seed_quaternion_left_start =
      seed_pose_left_start.rotation().ToQuaternion();

  // Create a sequence of anchors: first 3 are solvable, 4th is impossible
  std::vector<planning_service_client::planner::Anchor> anchors;
  planning_service_client::SystemConf right_sysconf_1;
  right_sysconf_1["franka_right"] =
      q_right + Eigen::VectorXd::Constant(7, 0.01);
  planning_service_client::SystemConf right_sysconf_2;
  right_sysconf_2["franka_right"] =
      q_right + Eigen::VectorXd::Constant(7, 0.02);
  planning_service_client::SystemConf right_sysconf_3;
  right_sysconf_3["franka_right"] =
      q_right + Eigen::VectorXd::Constant(7, 0.03);
  planning_service_client::SystemConf right_sysconf_impossible;
  right_sysconf_impossible["franka_right"] =
      q_right + Eigen::VectorXd::Constant(7, 0.04);

  // Anchor 1: small z offset (solvable)
  std::vector<planning_service_client::FrameRelativePose> wayposes_1 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0, 0.01),
          seed_quaternion_left_start)};
  anchors.emplace_back(right_sysconf_1, wayposes_1);

  // Anchor 2: small y offset (solvable)
  std::vector<planning_service_client::FrameRelativePose> wayposes_2 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0.01, 0),
          seed_quaternion_left_start)};
  anchors.emplace_back(right_sysconf_2, wayposes_2);

  // Anchor 3: small x offset (solvable)
  std::vector<planning_service_client::FrameRelativePose> wayposes_3 = {
      planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0.01, 0, 0),
          seed_quaternion_left_start)};
  anchors.emplace_back(right_sysconf_3, wayposes_3);

  // Anchor 4: very large z offset (impossible to reach - beyond arm reach)
  std::vector<planning_service_client::FrameRelativePose> wayposes_impossible =
      {planning_service_client::FrameRelativePose(
          "world", left_ik_frame,
          seed_translation_left_start + Eigen::Vector3d(0, 0, 3.0),
          seed_quaternion_left_start)};
  anchors.emplace_back(right_sysconf_impossible, wayposes_impossible);

  // --- Test 1: Call with return_longest_incomplete_solution = false (default)
  // --- Expect failure
  logging::log()->info(
      "TestSolveSequentialAnchorsIncomplete: Testing with "
      "return_longest_incomplete_solution=false");
  auto configs_fail =
      planner.SolveSequentialAnchors(anchors, q_start, true, false);
  EXPECT_FALSE(configs_fail.has_value())
      << "Expected failure when return_longest_incomplete_solution=false";
  EXPECT_TRUE(
      common::utils::string_includes(configs_fail.error(), "kOptimization"))
      << "Expected error to mention optimization failure, got: "
      << configs_fail.error();

  // --- Test 2: Call with return_longest_incomplete_solution = true
  // --- Expect success with partial solution
  logging::log()->info(
      "TestSolveSequentialAnchorsIncomplete: Testing with "
      "return_longest_incomplete_solution=true");
  auto configs_partial =
      planner.SolveSequentialAnchors(anchors, q_start, true, true);
  EXPECT_TRUE(configs_partial.has_value())
      << "Expected success when return_longest_incomplete_solution=true";

  // Verify that the returned solution is a partial solution
  const auto& partial_configs = configs_partial.value();
  EXPECT_GT(partial_configs.size(), 0u)
      << "Partial solution should have at least one configuration";
  EXPECT_LT(partial_configs.size(), anchors.size())
      << "Partial solution should be shorter than full anchor sequence";
  logging::log()->info(
      "TestSolveSequentialAnchorsIncomplete: Returned partial solution with "
      "{} configurations out of {} anchors",
      partial_configs.size(), anchors.size());

  // Verify that the partial solution actually satisfies the solvable anchors
  // We expect the first 3 anchors to be solvable, so size should be 3
  EXPECT_EQ(partial_configs.size(), 3u)
      << "Expected to solve the first 3 anchors before hitting impossible "
         "anchor";

  // Verify each partial config matches the expected pose
  for (size_t i = 0; i < partial_configs.size(); ++i) {
    auto pose = planner.robot_model().CalcRelativeTransform(
        partial_configs[i], world_frame,
        planner.robot_model().GetScopedFrameByName(left_ik_frame));
    drake::math::RigidTransformd expected_pose(
        anchors[i].poses()[0].X_AB_quaternion(),
        anchors[i].poses()[0].X_AB_translation());
    auto error = expected_pose.inverse() * pose;
    EXPECT_LT(error.translation().norm(), 2e-3)
        << "Configuration " << i << " does not match expected pose";
    EXPECT_LT(error.rotation().ToAngleAxis().angle(), 2e-3)
        << "Configuration " << i << " rotation does not match expected pose";
  }
}

TEST(GeneralizedMultimodalPlan, ConstructWaypointAnchorsFromWayposesVec_Basic) {
  const auto planner = DracoPlanner(test::DualPandas());

  // Build some simple frame-relative poses
  const std::string frame_A = "world";
  const std::string frame_B = "franka_left::franka_tool_location";
  Eigen::Vector3d t1(0.0, 0.0, 0.0);
  Eigen::Quaterniond q1 = Eigen::Quaterniond::Identity();
  Eigen::Vector3d t2(0.0, 0.0, 0.05);
  Eigen::Quaterniond q2 = Eigen::Quaterniond::Identity();

  planning_service_client::FrameRelativePose frp1(frame_A, frame_B, t1, q1);
  planning_service_client::FrameRelativePose frp2(frame_A, frame_B, t2, q2);

  // Two FrameRelativePosesVec inputs: first has one pose, second has two poses
  planning_service_client::FrameRelativePosesVec vec1(std::vector {frp1});
  planning_service_client::FrameRelativePosesVec vec2(std::vector {frp2});
  std::vector<planning_service_client::FrameRelativePosesVec> wayposes = {vec1,
                                                                          vec2};

  // Provide a fixed system config (only need a valid key used in tests)
  planning_service_client::SystemConf fixed_sysconf;
  Eigen::VectorXd q_right(7);
  q_right << -0.44, 0.50, -0.23, -1.82, -1.63, 2.00, -0.03;
  fixed_sysconf["franka_right"] = q_right;

  // Call the function under test
  auto anchors =
      planner.ConstructWaypointAnchorsFromWayposesVec(wayposes, fixed_sysconf);

  // Expect same number of anchors as wayposes vecs
  ASSERT_EQ(anchors.size(), wayposes.size());

  // Check each anchor contains the expected FRPs in order and the fixed sysconf
  for (size_t i = 0; i < anchors.size(); ++i) {
    const auto& anchor = anchors[i];
    const auto& expected_frps = wayposes[i].FrameRelativePoses();
    const auto& actual_frps = anchor.poses();
    EXPECT_EQ(actual_frps.size(), expected_frps.size());

    for (size_t j = 0; j < expected_frps.size(); ++j) {
      EXPECT_EQ(actual_frps[j].frame_A(), expected_frps[j].frame_A());
      EXPECT_EQ(actual_frps[j].frame_B(), expected_frps[j].frame_B());
      EXPECT_TRUE(actual_frps[j].X_AB_translation().isApprox(
          expected_frps[j].X_AB_translation(), 1e-9));
      Eigen::Quaterniond qa(actual_frps[j].X_AB_quaternion());
      Eigen::Quaterniond qe(expected_frps[j].X_AB_quaternion());
      EXPECT_NEAR(std::abs(qa.dot(qe)), 1.0,
                  1e-9);  // quaternion equality up to sign
    }

    // Check fixed system configuration was copied into the anchor
    EXPECT_EQ(anchor.system_conf().size(), fixed_sysconf.size());
    for (const auto& [k, v] : anchor.system_conf()) {
      const auto& expected_v = fixed_sysconf.at(k);
      EXPECT_TRUE(v.q().isApprox(expected_v.q(), 1e-9));
    }
  }
}

TEST(GeneralizedMultimodalPlan,
     ConstructWaypointAnchorsFromRelativeTransforms_Interpolation) {
  const auto planner = DracoPlanner(test::DualPandas());

  // Prepare a known start system configuration (right + left)
  planning_service_client::SystemConf start_sysconf;
  Eigen::VectorXd q_right(7), q_left(7);
  q_right << -0.44111, 0.500755, -0.22664, -1.82248, -1.63484, 2.00563,
      -0.027943;
  q_left << 2.07683, 0.664395, 1.0827, -2.46029, 1.83885, 1.57929, 0.1637;
  start_sysconf["franka_right"] = q_right;
  start_sysconf["franka_left"] = q_left;

  // Create a start anchor that contains the start pose for the left tool frame
  const std::string left_tool = "franka_left::franka_tool_location";
  Eigen::VectorXd q_start =
      conversions::ToGeneralizedPosition(planner.robot_model(), start_sysconf);
  auto first_rt = planner.CalcRelativePose(q_start, left_tool);
  planning_service_client::FrameRelativePose start_frp(
      "world", left_tool, first_rt.translation(),
      first_rt.rotation().ToQuaternion());

  planning_service_client::planner::Anchor start_anchor(start_sysconf, {});

  // Create a goal offset FRP: translate +0.10 in z, no rotation change
  Eigen::Vector3d offset_t(0.0, 0.0, 0.10);
  Eigen::Quaterniond offset_q = Eigen::Quaterniond::Identity();
  planning_service_client::FrameRelativePose goal_offset_frp(
      "world", left_tool, offset_t, offset_q);

  // Construct waypoint anchors with spacing small enough to force >=3 steps
  const double max_translation_spacing = 0.05;
  const double max_rotation_spacing =
      0.5;  // large so rotation doesn't dominate
  auto anchors = planner.ConstructWaypointAnchorsFromRelativeTransforms(
      start_anchor, std::vector {goal_offset_frp}, std::nullopt,
      max_translation_spacing, max_rotation_spacing);

  // There should be at least start + 2 interpolated anchors (per
  // implementation)
  ASSERT_GE(anchors.size(), 3u);

  // Extract first and last constructed anchor's pose transforms
  auto to_rt = [](const planning_service_client::FrameRelativePose& frp) {
    return drake::math::RigidTransformd(frp.X_AB_quaternion(),
                                        frp.X_AB_translation());
  };

  // last anchor's first FRP (goal)
  ASSERT_FALSE(anchors.back().poses().empty());
  const auto last_rt = to_rt(anchors.back().poses().front());

  // last translation is first translation + offset translation
  const auto expected_last_translation = first_rt.translation() + offset_t;
  EXPECT_NEAR((last_rt.translation() - expected_last_translation).norm(), 0.0,
              1e-6);

  // Verify linear cartesian interpolation for each interpolated anchor
  // Note: function appends interpolated anchors after the initial start_anchor.
  // The appended anchors correspond to t = i/(N-1) for i in [0..N-1].
  const size_t appended_count =
      anchors.size() - 1;  // excluding the original start_anchor
  // start and goal for interpolation should be the appended anchors at indices
  // 1 and last
  const auto appended_start_rt = to_rt(anchors[1].poses().front());
  const auto appended_goal_rt = to_rt(anchors.back().poses().front());

  for (size_t idx = 0; idx < appended_count; ++idx) {
    const double t =
        static_cast<double>(idx) / static_cast<double>(appended_count - 1);
    const auto& anchor = anchors[idx + 1];  // appended anchors start at index 1
    ASSERT_EQ(anchor.poses().size(), 1u);
    const auto cur_rt = to_rt(anchor.poses().front());

    // Expected translation = lerp between appended_start_rt and
    // appended_goal_rt
    const Eigen::Vector3d expected_trans =
        (1.0 - t) * appended_start_rt.translation()
        + t * appended_goal_rt.translation();
    EXPECT_NEAR((cur_rt.translation() - expected_trans).norm(), 0.0, 1e-6);

    // Expected rotation = slerp between appended_start_rt and appended_goal_rt
    Eigen::Quaterniond q_start(appended_start_rt.rotation().ToQuaternion());
    Eigen::Quaterniond q_goal(appended_goal_rt.rotation().ToQuaternion());
    Eigen::Quaterniond q_expected = q_start.slerp(t, q_goal);
    Eigen::Quaterniond q_cur(cur_rt.rotation().ToQuaternion());
    // Compare quaternions up to sign
    EXPECT_NEAR(std::abs(q_expected.dot(q_cur)), 1.0, 1e-6);
  }
}

TEST(GeneralizedMultimodalPlan,
     ConstructWaypointAnchorsFromAbsoluteTransforms_Interpolation) {
  const auto planner = DracoPlanner(test::DualPandas());

  // Prepare a known start system configuration (right + left)
  planning_service_client::SystemConf start_sysconf;
  Eigen::VectorXd q_right(7), q_left(7);
  q_right << -0.44111, 0.500755, -0.22664, -1.82248, -1.63484, 2.00563,
      -0.027943;
  q_left << 2.07683, 0.664395, 1.0827, -2.46029, 1.83885, 1.57929, 0.1637;
  start_sysconf["franka_right"] = q_right;
  start_sysconf["franka_left"] = q_left;

  // Generalized position for transforms
  Eigen::VectorXd q_start =
      conversions::ToGeneralizedPosition(planner.robot_model(), start_sysconf);

  // Frames and compute start transform of left tool in world
  const std::string left_tool = "franka_left::franka_tool_location";
  const auto& world_frame = planner.robot_model().GetScopedFrameByName("world");
  auto start_rt = planner.robot_model().CalcRelativeTransform(
      q_start, world_frame,
      planner.robot_model().GetScopedFrameByName(left_tool));

  // Create start anchor containing the start FRP explicitly
  //   planning_service_client::FrameRelativePose start_frp(
  //       world_frame.name(), left_tool, start_rt.translation(),
  //       start_rt.rotation().ToQuaternion());
  planning_service_client::planner::Anchor start_anchor(start_sysconf, {});

  // Create a goal absolute FRP: translate +0.10 in z, no rotation change
  Eigen::Vector3d offset_t(0.0, 0.0, 0.10);
  //   Eigen::Quaterniond offset_q = Eigen::Quaterniond::Identity();
  drake::math::RigidTransformd goal_rt = start_rt;
  goal_rt.set_translation(start_rt.translation() + offset_t);
  planning_service_client::FrameRelativePose goal_frp(
      world_frame.name(), left_tool, goal_rt.translation(),
      goal_rt.rotation().ToQuaternion());

  // Construct waypoint anchors with spacing small enough to force multiple
  // steps
  const double max_translation_spacing = 0.05;
  const double max_rotation_spacing =
      0.5;  // large so rotation doesn't dominate
  auto anchors_result = planner.ConstructWaypointAnchorsFromAbsoluteTransforms(
      start_anchor, std::vector {goal_frp}, std::nullopt,
      max_translation_spacing, max_rotation_spacing);
  ASSERT_TRUE(anchors_result.has_value())
      << "Failed to construct waypoint anchors: " << anchors_result.error();
  auto anchors = anchors_result.value();

  // There should be at least start + 3 anchors
  ASSERT_GT(anchors.size(), 2u);

  // Helpers to convert FRP -> RigidTransform
  auto to_rt = [](const planning_service_client::FrameRelativePose& frp) {
    return drake::math::RigidTransformd(frp.X_AB_quaternion(),
                                        frp.X_AB_translation());
  };

  // Last anchor's first FRP should match goal_frp
  ASSERT_FALSE(anchors.back().poses().empty());
  const auto last_rt = to_rt(anchors.back().poses().front());
  EXPECT_NEAR((last_rt.translation() - goal_rt.translation()).norm(), 0.0,
              1e-6);
  EXPECT_NEAR(last_rt.rotation().ToAngleAxis().angle(),
              goal_rt.rotation().ToAngleAxis().angle(), 1e-6);

  // Verify linear cartesian interpolation for each interpolated anchor
  const size_t appended_count =
      anchors.size() - 1;  // excluding the original start_anchor
  const auto appended_start_rt = to_rt(anchors[1].poses().front());
  const auto appended_goal_rt = to_rt(anchors.back().poses().front());

  for (size_t idx = 0; idx < appended_count; ++idx) {
    const double t =
        static_cast<double>(idx) / static_cast<double>(appended_count - 1);
    const auto& anchor = anchors[idx + 1];  // appended anchors start at index 1
    ASSERT_EQ(anchor.poses().size(), 1u);
    const auto cur_rt = to_rt(anchor.poses().front());

    // Expected translation = lerp between appended_start_rt and
    // appended_goal_rt
    const Eigen::Vector3d expected_trans =
        (1.0 - t) * appended_start_rt.translation()
        + t * appended_goal_rt.translation();
    EXPECT_NEAR((cur_rt.translation() - expected_trans).norm(), 0.0, 1e-6);

    // Expected rotation = slerp between appended_start_rt and appended_goal_rt
    Eigen::Quaterniond q_start(appended_start_rt.rotation().ToQuaternion());
    Eigen::Quaterniond q_goal(appended_goal_rt.rotation().ToQuaternion());
    Eigen::Quaterniond q_expected = q_start.slerp(t, q_goal);
    Eigen::Quaterniond q_cur(cur_rt.rotation().ToQuaternion());
    EXPECT_NEAR(std::abs(q_expected.dot(q_cur)), 1.0, 1e-6);
  }
}

TEST(GeneralizedMultimodalPlan, SolvePlanWithActiveTrajectory) {
  auto dual_pandas_adapter = test::DualPandas();
  bool DEBUG_MESHCAT =
      false;  // Set to true to visualize in Meshcat for debugging
  if (DEBUG_MESHCAT) {
    dual_pandas_adapter.robot_meshcat_params = motion::RobotMeshcatParams();
    dual_pandas_adapter.robot_meshcat_params->port = 7000;
    dual_pandas_adapter.robot_meshcat_params->visual = true;
    dual_pandas_adapter.robot_meshcat_params->collision = true;
    dual_pandas_adapter.options.visualizer_options.mode =
        VisualizerMode::kDraco;
  }
  const auto planner = DracoPlanner(dual_pandas_adapter);
  // Define a multimodal planning problem for arm left
  planning_service_client::SystemConf system_conf_start, system_conf_goal;
  Eigen::VectorXd q_right(7), q_left_start(7);
  Eigen::VectorXd q_left_goal_1(7), q_left_goal_2(7), q_left_goal_3(7);
  q_right << -0.4, 0.5, -0.2, -1.8, -1.6, 2.0, 0.0;
  q_left_start << 2.0, 0.6, 1.0, -2.4, 1.8, 1.5, 0.1;
  q_left_goal_1 << -1.0, 0.8, 1.0, -2.4, 1.8, 1.5, 1.1;
  q_left_goal_2 << -1.0, 0.6, 1.2, -2.6, 1.6, 1.7, 0.9;
  q_left_goal_3 << -1.2, 0.7, 1.1, -2.5, 1.7, 1.6, 1.0;
  system_conf_start["franka_right"] = q_right;
  system_conf_start["franka_left"] = q_left_start;
  // Goal system config
  system_conf_goal["franka_left"] = q_left_goal_1;
  // Construct goal anchor
  planning_service_client::planner::Anchor goal_anchor_1(system_conf_goal, {});
  system_conf_goal["franka_left"] = q_left_goal_2;
  planning_service_client::planner::Anchor goal_anchor_2(system_conf_goal, {});
  system_conf_goal["franka_left"] = q_left_goal_3;
  planning_service_client::planner::Anchor goal_anchor_3(system_conf_goal, {});
  // Build the problem
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      gen_multimodal_plan {
          {}, std::vector {goal_anchor_1, goal_anchor_2, goal_anchor_3}};
  // Solve the plan.
  auto result =
      planner.SolvePlan(gen_multimodal_plan, "left alone", std::nullopt,
                        system_conf_start, std::nullopt);
  // The plan should be successful
  EXPECT_TRUE(result.is_success());
  // It should also include both arms in the trajectory
  const auto& sys_traj = result.system_timed_trajectory();
  logging::log()->info("System timed trajectory with size {}", sys_traj.size());
  EXPECT_FALSE(sys_traj.has_key("franka_right"));
  EXPECT_TRUE(sys_traj.has_key("franka_left"));
  // Now, let's plan for the right arm but send the current left trajectory as
  // active
  planning_service_client::SystemTimedTrajectory active_sys_traj;
  active_sys_traj["franka_left"] = sys_traj.at("franka_left");
  // Get time since epoch
  std::chrono::duration<double> epoch_duration_now =
      std::chrono::system_clock::now().time_since_epoch();
  double time_global_now = epoch_duration_now.count();
  time_global_now -= 0.3;  // Pretend the trajectory started 0.3s ago
  active_sys_traj["franka_left"].SetGlobalTimeOffset(time_global_now);
  // Now make some goal anchors for the right arm
  planning_service_client::SystemConf right_goal_sysconf;
  right_goal_sysconf["franka_right"] =
      q_right - Eigen::VectorXd::Constant(7, 1.0);
  planning_service_client::planner::Anchor right_goal_anchor_1(
      right_goal_sysconf, {});
  right_goal_sysconf["franka_right"] =
      q_right - Eigen::VectorXd::Constant(7, 0.6);
  planning_service_client::planner::Anchor right_goal_anchor_2(
      right_goal_sysconf, {});
  right_goal_sysconf["franka_right"] =
      q_right - Eigen::VectorXd::Constant(7, 1.0);
  planning_service_client::planner::Anchor right_goal_anchor_3(
      right_goal_sysconf, {});
  right_goal_sysconf["franka_right"] =
      q_right - Eigen::VectorXd::Constant(7, 0.6);
  planning_service_client::planner::Anchor right_goal_anchor_4(
      right_goal_sysconf, {});
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      right_arm_plan {{},
                      std::vector {right_goal_anchor_1, right_goal_anchor_2,
                                   right_goal_anchor_3, right_goal_anchor_4}};
  // Setup options
  auto result_async =
      planner.SolvePlan(right_arm_plan, "async left and right", std::nullopt,
                        system_conf_start, active_sys_traj);
  EXPECT_TRUE(result_async.is_success());
  // The plan should include both arms -- UPDATE: SEE BELOW
  auto sys_traj_async = result_async.system_timed_trajectory();
  EXPECT_TRUE(sys_traj_async.has_key("franka_right"));
  // For now, we do not expect the left arm to be replanned. So we have
  // Not included it in the async trajectory.
  EXPECT_FALSE(sys_traj_async.has_key("franka_left"));
  // The time starts from t_now for both arms
  EXPECT_NEAR(sys_traj_async.at("franka_right").start_time(), 0.0, 1e-4);
  // ToDo(@Sadra) bring back this check when we switch to update with sliced
  // trajectory
  //   EXPECT_NEAR(sys_traj_async.at("franka_left").start_time(),
  //               traj_update_options.t_now, 1e-4);
  // The trajectory of left must be exactly the same as the active one from
  // t_now
  //   auto left_traj_async = sys_traj_async.at("franka_left");
  //   auto left_traj_active = active_sys_traj.at("franka_left");
  //   // Check values from t_now to the end
  //   for (double t = traj_update_options.t_now; t <=
  //   left_traj_active.end_time();
  //        t += 0.1) {
  //     EXPECT_TRUE(
  //         left_traj_async.Value(t).isApprox(left_traj_active.Value(t),
  //         1e-6));
  //   }
  // The values of trajectory of right arm between t_now and
  // t_now + dt_switch should be constant (holding the first waypoint)
  auto right_traj_async = sys_traj_async.at("franka_right");
  auto first_waypoint_right = right_traj_async.GlobalValue(time_global_now);
  double delta_latency = 0.055;  // ToDo(@Sadra) get from options
  for (double t = time_global_now; t <= time_global_now + delta_latency;
       t += 0.01) {
    EXPECT_TRUE(
        right_traj_async.GlobalValue(t).isApprox(first_waypoint_right, 1e-6));
  }
  // To see meshcat visualization, uncomment the following lineee
  while (DEBUG_MESHCAT) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

TEST(GeneralizedMultimodalPlan, BestApproachTrajectory_Valid) {
  auto dual_pandas_adapter = test::DualPandas();
  bool DEBUG_MESHCAT =
      false;  // Set to true to visualize in Meshcat for debugging
  if (DEBUG_MESHCAT) {
    dual_pandas_adapter.robot_meshcat_params = motion::RobotMeshcatParams();
    dual_pandas_adapter.robot_meshcat_params->port = 7000;
    dual_pandas_adapter.robot_meshcat_params->visual = true;
    dual_pandas_adapter.robot_meshcat_params->collision = true;
    dual_pandas_adapter.options.visualizer_options.mode =
        VisualizerMode::kDraco;
  }
  const auto planner = DracoPlanner(dual_pandas_adapter);
  // Define a multimodal planning problem for arm left
  planning_service_client::SystemConf system_conf_start, system_conf_goal;
  Eigen::VectorXd q_right(7), q_left_start(7);
  Eigen::VectorXd q_left_goal_1(7), q_left_goal_2(7), q_left_goal_3(7);
  q_right << -0.4, 0.5, -0.2, -1.8, -1.6, 2.0, 0.0;
  q_left_start << 2.0, 0.6, 1.0, -2.4, 1.8, 1.5, 0.1;
  q_left_goal_1 << -1.0, 0.8, 1.0, -2.4, 1.8, 1.5, 1.1;
  q_left_goal_2 << -1.0, 0.6, 1.2, -2.6, 1.6, 1.7, 0.9;
  q_left_goal_3 << -1.2, 0.7, 1.1, -2.5, 1.7, 1.6, 1.0;
  system_conf_start["franka_right"] = q_right;
  system_conf_start["franka_left"] = q_left_start;
  // Goal system config
  system_conf_goal["franka_left"] = q_left_goal_1;
  // Construct goal anchor
  planning_service_client::planner::Anchor goal_anchor_1(system_conf_goal, {});
  system_conf_goal["franka_left"] = q_left_goal_2;
  planning_service_client::planner::Anchor goal_anchor_2(system_conf_goal, {});
  system_conf_goal["franka_left"] = q_left_goal_3;
  planning_service_client::planner::Anchor goal_anchor_3(system_conf_goal, {});
  // Build the problem
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      gen_multimodal_plan {
          {},    std::vector {goal_anchor_1, goal_anchor_2, goal_anchor_3},
          false, false,
          false, true};
  // Solve the plan.
  const auto& left_alone_plan = planner.SolveGeneralizedMultiModalPlan(
      gen_multimodal_plan, system_conf_start, std::nullopt);
  EXPECT_TRUE(left_alone_plan.has_value());
  const auto& timed_path_left_alone =
      drake::trajectories::PathParameterizedTrajectory<double>(
          left_alone_plan.value().trajectory.first,
          left_alone_plan.value().trajectory.second);
  const auto& sys_traj = conversions::ToSystemTimedTrajectory(
      planner.time_optimal_spliner(), timed_path_left_alone);
  logging::log()->info("System timed trajectory with size {}", sys_traj.size());
  EXPECT_TRUE(sys_traj.has_key("franka_left"));
  // Now, let's plan for the right arm but send the current left trajectory as
  // active
  planning_service_client::SystemTimedTrajectory active_sys_traj;
  active_sys_traj["franka_left"] = sys_traj.at("franka_left");
  // Get time since epoch
  std::chrono::duration<double> epoch_duration_now =
      std::chrono::system_clock::now().time_since_epoch();
  double time_global_now = epoch_duration_now.count();
  double t_now = 0.3;
  double time_global_active_traj =
      time_global_now
      - t_now;  // Pretend the trajectory started t_now seconds ago
  active_sys_traj["franka_left"].SetGlobalTimeOffset(time_global_active_traj);
  // Now make some goal anchors for the right arm
  planning_service_client::SystemConf right_goal_sysconf;
  right_goal_sysconf["franka_right"] =
      q_right - Eigen::VectorXd::Constant(7, 1.0);
  planning_service_client::planner::Anchor right_goal_anchor_1(
      right_goal_sysconf, {});
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      right_arm_plan {
          {}, std::vector {right_goal_anchor_1}, false, false, false, true};

  std::string movable_system = "franka_right";

  auto model_idx =
      planner.robot_model().plant().GetModelInstanceByName(movable_system);
  auto movable_arm_index = planner.robot_model().get_arm_index(model_idx);

  auto right_arm_plan_gen_mm = planner.SolveGeneralizedMultiModalPlan(
      right_arm_plan, system_conf_start, active_sys_traj);
  EXPECT_TRUE(right_arm_plan_gen_mm.has_value());

  logging::log()->info("Transition start time: {}, transition end time: {}",
                       right_arm_plan_gen_mm.value().transit_start_time,
                       right_arm_plan_gen_mm.value().transit_end_time);

  double collision_checking_step = 0.01;  // seconds
  double min_partial_solution_time =
      right_arm_plan_gen_mm.value().transit_start_time;
  double max_partial_solution_time =
      right_arm_plan_gen_mm.value().transit_end_time;

  const auto& timed_path_gen_mm =
      drake::trajectories::PathParameterizedTrajectory<double>(
          right_arm_plan_gen_mm.value().trajectory.first,
          right_arm_plan_gen_mm.value().trajectory.second);
  const auto& sys_traj_gen_mm = conversions::ToSystemTimedTrajectory(
      planner.time_optimal_spliner(), timed_path_gen_mm);
  EXPECT_TRUE(sys_traj_gen_mm.has_key("franka_right"));
  EXPECT_TRUE(sys_traj_gen_mm.has_key("franka_left"));
  // Check that the output trajectory does not have a collision
  auto collision_conf_gen_mm_opt =
      planner.TimeOfArmsCollision(sys_traj_gen_mm, collision_checking_step);
  EXPECT_FALSE(collision_conf_gen_mm_opt.has_value())
      << "Expected no collision in gen mm plan trajectory";
  // Get best approach trajectory from gen mm plan
  auto best_approach_trajectory_gen_mm_opt = planner.BestApproachTrajectory(
      sys_traj_gen_mm, {movable_arm_index}, 10 /* num_slices */,
      collision_checking_step, min_partial_solution_time,
      max_partial_solution_time);
  EXPECT_TRUE(best_approach_trajectory_gen_mm_opt.has_value());
  const auto& best_approach_trajectory_gen_mm =
      best_approach_trajectory_gen_mm_opt.value();
  EXPECT_TRUE(best_approach_trajectory_gen_mm.has_key("franka_right"));
  EXPECT_TRUE(best_approach_trajectory_gen_mm.has_key("franka_left"));
  const auto& collision_conf_best_approach_opt = planner.TimeOfArmsCollision(
      best_approach_trajectory_gen_mm, collision_checking_step);
  EXPECT_FALSE(collision_conf_best_approach_opt.has_value())
      << "Expected no collision in best approach trajectory from gen mm plan";
  // Check that the best approach trajectory matches the planned trajectory for
  // the movable ressource and the active trajectory for the non-movable arm
  auto best_approach_traj_right =
      best_approach_trajectory_gen_mm.at("franka_right");
  auto best_approach_traj_left =
      best_approach_trajectory_gen_mm.at("franka_left");
  auto planned_traj_right = sys_traj_gen_mm.at("franka_right");
  auto active_traj_left = active_sys_traj.at("franka_left");
  for (double t = time_global_now; t <= best_approach_traj_left.end_time();
       t += 0.1) {
    EXPECT_TRUE(best_approach_traj_left.GlobalValue(t).isApprox(
        active_traj_left.GlobalValue(t), 1e-6));
  }
  for (double t = best_approach_traj_right.start_time();
       t <= best_approach_traj_right.end_time(); t += 0.1) {
    EXPECT_TRUE(best_approach_traj_right.GlobalValue(t).isApprox(
        planned_traj_right.GlobalValue(t), 1e-6));
  }

  // Use SolvePlan to solve the right arm plan with the left arm active
  // trajectory, and check that the output trajectory matches the best approach
  // trajectory from gen mm plan
  auto right_arm_plan_solve_plan =
      planner.SolvePlan(right_arm_plan, "right arm with left active",
                        std::nullopt, system_conf_start, active_sys_traj);
  EXPECT_TRUE(right_arm_plan_solve_plan.is_success());
  const auto& sys_traj_solve_plan =
      right_arm_plan_solve_plan.system_timed_trajectory();
  EXPECT_TRUE(sys_traj_solve_plan.has_key("franka_right"));
  EXPECT_FALSE(sys_traj_solve_plan.has_key(
      "franka_left"));  // We do not expect the left arm to be replanned, so it
                        // should not be included in the output trajectory
  auto traj_solve_plan_right = sys_traj_solve_plan.at("franka_right");
  for (double t = time_global_now; t <= traj_solve_plan_right.end_time();
       t += 0.1) {
    EXPECT_TRUE(traj_solve_plan_right.GlobalValue(t).isApprox(
        best_approach_traj_right.GlobalValue(t), 1e-6));
  }

  // To see meshcat visualization, uncomment the following line
  while (DEBUG_MESHCAT) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

TEST(GeneralizedMultimodalPlan, BestApproachTrajectory_Invalid) {
  auto dual_pandas_adapter = test::DualPandas();
  bool DEBUG_MESHCAT =
      false;  // Set to true to visualize in Meshcat for debugging
  if (DEBUG_MESHCAT) {
    dual_pandas_adapter.robot_meshcat_params = motion::RobotMeshcatParams();
    dual_pandas_adapter.robot_meshcat_params->port = 7001;
    dual_pandas_adapter.robot_meshcat_params->visual = true;
    dual_pandas_adapter.robot_meshcat_params->collision = true;
    dual_pandas_adapter.options.visualizer_options.mode =
        VisualizerMode::kDraco;
  }
  auto planner = DracoPlanner(dual_pandas_adapter);
  // Define a multimodal planning problem for arm left
  planning_service_client::SystemConf system_conf_start, system_conf_goal,
      system_conf_combined_goal;
  Eigen::VectorXd q_left_start(7), q_left_goal(7), q_left_pre_goal_3(7),
      q_left_pre_goal_2(7), q_left_pre_goal_1(7);
  Eigen::VectorXd q_right_goal_1(7), q_right_goal_2(7), q_right_goal_3(7),
      q_right_start(7);
  q_left_start << -1.0, 0.8, 1.5, -1.5, -0.75, 1.75, 1.2;
  q_left_goal << -2.1, 0.055, 0.573, -1.75, 0.236, 1.53, 0.0;
  q_left_pre_goal_3 << -2., 0.055, 0.573, -1.75, 0.236, 1.53, 0.0;
  q_left_pre_goal_2 << -1.9, 0.055, 0.573, -1.75, 0.236, 1.53, 0.0;
  q_left_pre_goal_1 << -1.8, 0.055, 0.573, -1.75, 0.236, 1.53, 0.0;
  q_right_start << -1.0, 0.8, 1.0, -2.4, 1.8, 1.5, 1.1;
  q_right_goal_1 << -1.5, 0.4, 0.14, -1.4, 0.0, 2.0, 0.4;
  q_right_goal_2 << -1.5, 0.45, 0.14, -1.4, 0.0, 2.0, 0.4;
  q_right_goal_3 << -1.5, 0.5, 0.14, -1.4, 0.0, 2.0, 0.4;
  system_conf_start["franka_right"] = q_right_start;
  system_conf_start["franka_left"] = q_left_start;
  // Goal system config
  system_conf_goal["franka_right"] = q_right_goal_1;
  system_conf_combined_goal["franka_right"] = q_left_goal;
  system_conf_combined_goal["franka_left"] = q_right_goal_1;
  // Expect collision goal system config
  Eigen::VectorXd goal_config_1 = conversions::ToGeneralizedPosition(
      planner.robot_model(), system_conf_combined_goal);
  EXPECT_FALSE(planner.robot_constraints().CheckSatisfied({goal_config_1}));
  // Construct goal anchor
  planning_service_client::planner::Anchor goal_anchor_1(system_conf_goal, {});
  system_conf_goal["franka_right"] = q_right_goal_2;
  planning_service_client::planner::Anchor goal_anchor_2(system_conf_goal, {});
  system_conf_goal["franka_right"] = q_right_goal_3;
  planning_service_client::planner::Anchor goal_anchor_3(system_conf_goal, {});
  // Build the problem
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      gen_multimodal_plan {
          {},    std::vector {goal_anchor_1, goal_anchor_2, goal_anchor_3},
          false, false,
          false, true};
  // Solve the plan.
  auto right_alone_plan = planner.SolveGeneralizedMultiModalPlan(
      gen_multimodal_plan, system_conf_start, std::nullopt);
  // The plan should be successful
  EXPECT_TRUE(right_alone_plan.has_value());
  const auto& timed_right_alone_path =
      drake::trajectories::PathParameterizedTrajectory<double>(
          right_alone_plan.value().trajectory.first,
          right_alone_plan.value().trajectory.second);
  const auto& sys_traj = conversions::ToSystemTimedTrajectory(
      planner.time_optimal_spliner(), timed_right_alone_path);
  logging::log()->info("System timed trajectory with size {}", sys_traj.size());
  EXPECT_TRUE(sys_traj.has_key("franka_left"));
  EXPECT_TRUE(sys_traj.has_key("franka_right"));
  // Now, let's plan for the right arm but send the current left trajectory as
  // active
  planning_service_client::SystemTimedTrajectory active_sys_traj;
  active_sys_traj["franka_right"] = sys_traj.at("franka_right");
  // Get time since epoch
  std::chrono::duration<double> epoch_duration_now =
      std::chrono::system_clock::now().time_since_epoch();
  double time_global_now = epoch_duration_now.count();
  double t_now = 0.3;  // seconds
  double time_global_active_traj_start =
      time_global_now - t_now;  // Pretend the trajectory started t_now s ago
  active_sys_traj["franka_right"].SetGlobalTimeOffset(
      time_global_active_traj_start);
  // Now make some goal anchors for the left arm
  planning_service_client::SystemConf left_goal_sysconf,
      left_pre_goal_sysconf_1, left_pre_goal_sysconf_2, left_pre_goal_sysconf_3;
  left_pre_goal_sysconf_1["franka_left"] = q_left_pre_goal_1;
  left_pre_goal_sysconf_2["franka_left"] = q_left_pre_goal_2;
  left_pre_goal_sysconf_3["franka_left"] = q_left_pre_goal_3;
  left_goal_sysconf["franka_left"] = q_left_goal;
  planning_service_client::planner::Anchor left_pre_goal_anchor_1(
      left_pre_goal_sysconf_1, {});
  planning_service_client::planner::Anchor left_pre_goal_anchor_2(
      left_pre_goal_sysconf_2, {});
  planning_service_client::planner::Anchor left_pre_goal_anchor_3(
      left_pre_goal_sysconf_3, {});
  planning_service_client::planner::Anchor left_goal_anchor(left_goal_sysconf,
                                                            {});
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      left_arm_plan {
          {},
          std::vector {left_pre_goal_anchor_1, left_pre_goal_anchor_2,
                       left_pre_goal_anchor_3, left_goal_anchor},
          false,
          false,
          false,
          true};

  std::string movable_system = {"franka_left"};
  // Get movable arm index
  auto model_idx =
      planner.robot_model().plant().GetModelInstanceByName(movable_system);
  auto movable_arm_index = planner.robot_model().get_arm_index(model_idx);
  EXPECT_TRUE(movable_arm_index.is_valid())
      << "Failed to get movable arm index for system " << movable_system;
  // Log the movable arm name and index

  const auto& movable_arm = planner.robot_model().GetArm(movable_arm_index);
  const auto& arm_name = movable_arm.name();
  logging::log()->info(
      "Movable system: {}, model index: {}, arm index: {}, arm name: {}",
      movable_system, model_idx, movable_arm_index, arm_name);

  auto left_arm_plan_gen_mm = planner.SolveGeneralizedMultiModalPlan(
      left_arm_plan, system_conf_start, active_sys_traj);
  EXPECT_TRUE(left_arm_plan_gen_mm.has_value());

  logging::log()->info("Transition start time: {}, transition end time: {}",
                       left_arm_plan_gen_mm.value().transit_start_time,
                       left_arm_plan_gen_mm.value().transit_end_time);

  double collision_checking_step = 0.01;  // seconds
  double min_partial_solution_time =
      left_arm_plan_gen_mm.value().transit_start_time;
  double max_partial_solution_time =
      left_arm_plan_gen_mm.value().transit_end_time;

  const auto& timed_path_gen_mm =
      drake::trajectories::PathParameterizedTrajectory<double>(
          left_arm_plan_gen_mm.value().trajectory.first,
          left_arm_plan_gen_mm.value().trajectory.second);
  auto sys_traj_gen_mm = conversions::ToSystemTimedTrajectory(
      planner.time_optimal_spliner(), timed_path_gen_mm);
  EXPECT_TRUE(sys_traj_gen_mm.has_key("franka_right"));
  EXPECT_TRUE(sys_traj_gen_mm.has_key("franka_left"));
  // Set the global time offset of the gen mm trajectory to time_global_now
  for (auto& [system_name, traj] : sys_traj_gen_mm) {
    traj.SetGlobalTimeOffset(time_global_now);
  }

  // Check that the output result does not have a collision
  const auto& collision_conf_gen_mm_opt =
      planner.TimeOfArmsCollision(sys_traj_gen_mm, collision_checking_step);
  EXPECT_FALSE(collision_conf_gen_mm_opt.has_value())
      << "Expected no collision in gen mm plan trajectory";

  const auto planned_traj {sys_traj_gen_mm};

  // Replace the systems in the active trajectory by their active path
  for (const auto& [system_name, traj] : active_sys_traj) {
    sys_traj_gen_mm[system_name] = traj;
  }

  // Check that now there is a collision
  const auto& collision_conf_gen_mm_opt_after_replace =
      planner.TimeOfArmsCollision(sys_traj_gen_mm, collision_checking_step);
  EXPECT_TRUE(collision_conf_gen_mm_opt_after_replace.has_value())
      << "Expected collision in gen mm plan trajectory after replacing active "
         "trajectories";

  //   Get best approach trajectory from gen mm plan
  auto best_approach_trajectory_gen_mm_opt = planner.BestApproachTrajectory(
      sys_traj_gen_mm, {movable_arm_index}, 10 /* num_slices */,
      collision_checking_step, min_partial_solution_time,
      max_partial_solution_time);
  EXPECT_TRUE(best_approach_trajectory_gen_mm_opt.has_value());
  const auto& best_approach_trajectory_gen_mm =
      best_approach_trajectory_gen_mm_opt.value();
  EXPECT_TRUE(best_approach_trajectory_gen_mm.has_key("franka_right"));
  EXPECT_TRUE(best_approach_trajectory_gen_mm.has_key("franka_left"));
  const auto& collision_conf_best_approach_opt = planner.TimeOfArmsCollision(
      best_approach_trajectory_gen_mm, collision_checking_step);
  EXPECT_FALSE(collision_conf_best_approach_opt.has_value())
      << "Expected no collision in best approach trajectory from gen mm plan";

  // Check that the trajectory is constant for the movable ressource after
  // collision_conf_gen_mm_opt_after_replace
  auto best_approach_traj_movable =
      best_approach_trajectory_gen_mm.at(movable_system);
  double t_collision = collision_conf_gen_mm_opt_after_replace.value();
  auto traj_value_before_collision =
      best_approach_traj_movable.Value(t_collision - 1e-4);
  // Sample the time interval [t_collision, t_end] with a few samples and check
  // that the trajectory for the movable arm is constant
  for (double t = t_collision; t <= best_approach_traj_movable.end_time();
       t += 0.1) {
    auto traj_value = best_approach_traj_movable.Value(t);
    EXPECT_TRUE(traj_value.isApprox(traj_value_before_collision, 1e-6))
        << "Expected trajectory for movable system to be constant after "
           "collision time. Trajectory value at time "
        << t << " is " << traj_value.transpose() << " but expected "
        << traj_value_before_collision.transpose();
  }

  // Check that the trajectory for the movable arm is the same from the start up
  // to the collision time (- some buffer)
  for (double t = time_global_now;
       t < t_collision - 3 * collision_checking_step; t += 0.1) {
    auto traj_value = best_approach_traj_movable.Value(t);
    auto gen_mm_traj_value = sys_traj_gen_mm.at(movable_system).Value(t);
    EXPECT_TRUE(traj_value.isApprox(gen_mm_traj_value, 1e-6))
        << "Expected trajectory for movable system to be the same as gen mm "
           "trajectory before collision time. Trajectory value at time "
        << t << " is " << traj_value.transpose() << " but expected "
        << gen_mm_traj_value.transpose();
  }

  // Check that the trajectory for the non-movable system is the same as the
  // active trajectory
  auto best_approach_traj_non_movable =
      best_approach_trajectory_gen_mm.at("franka_right");
  auto active_traj_non_movable = active_sys_traj.at("franka_right");
  for (double t = time_global_now;
       t <= best_approach_traj_non_movable.end_time(); t += 0.1) {
    auto traj_value = best_approach_traj_non_movable.Value(t);
    auto active_traj_value = active_traj_non_movable.Value(t);
    EXPECT_TRUE(traj_value.isApprox(active_traj_value, 1e-6))
        << "Expected trajectory for non-movable system to be the same as "
           "active trajectory. Trajectory value at time "
        << t << " is " << traj_value.transpose() << " but expected "
        << active_traj_value.transpose();
  }

  // // Use SolvePlan to solve the right arm plan with the left arm active
  // trajectory, and check that the output trajectory matches the best approach
  // trajectory from gen mm plan
  auto right_arm_plan_solve_plan =
      planner.SolvePlan(left_arm_plan, "left arm with right active",
                        std::nullopt, system_conf_start, active_sys_traj);
  EXPECT_TRUE(right_arm_plan_solve_plan.is_success());
  const auto& sys_traj_solve_plan =
      right_arm_plan_solve_plan.system_timed_trajectory();
  EXPECT_TRUE(sys_traj_solve_plan.has_key("franka_left"));
  EXPECT_FALSE(sys_traj_solve_plan.has_key(
      "franka_right"));  // Removed by SolvePlan as it is not targeted in the
                         // plan
  auto traj_solve_plan_left = sys_traj_solve_plan.at("franka_left");
  for (double t = time_global_now; t <= traj_solve_plan_left.end_time();
       t += 0.1) {
    EXPECT_TRUE(traj_solve_plan_left.GlobalValue(t).isApprox(
        best_approach_traj_movable.GlobalValue(t), 1e-6));
  }

  logging::log()->info(
      "Best approach trajectory from gen mm plan is collision free and has "
      "expected properties");
  if (planner.has_draco_visualizer()) {
    planner.mutable_draco_visualizer().Add(best_approach_trajectory_gen_mm,
                                           "Best Approach Trajectory");
    planner.mutable_draco_visualizer().Add(planned_traj, "Planned Trajectory");
    planner.mutable_draco_visualizer().Add(sys_traj_gen_mm,
                                           "Planned on top of active");
    planner.mutable_draco_visualizer().Add(sys_traj_solve_plan,
                                           "SolvePlan output trajectory");
  }

  // To see meshcat visualization, uncomment the following lineee
  while (DEBUG_MESHCAT) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

TEST(GeneralizedMultimodalPlan, BestApproachTrajectory_Invalid2) {
  auto dual_pandas_adapter = test::DualPandas();
  bool DEBUG_MESHCAT =
      false;  // Set to true to visualize in Meshcat for debugging
  if (DEBUG_MESHCAT) {
    dual_pandas_adapter.robot_meshcat_params = motion::RobotMeshcatParams();
    dual_pandas_adapter.robot_meshcat_params->port = 7001;
    dual_pandas_adapter.robot_meshcat_params->visual = true;
    dual_pandas_adapter.robot_meshcat_params->collision = true;
    dual_pandas_adapter.options.visualizer_options.mode =
        VisualizerMode::kDraco;
  }
  auto planner = DracoPlanner(dual_pandas_adapter);
  // Define a multimodal planning problem for arm left
  planning_service_client::SystemConf system_conf_start, system_conf_start_1,
      system_conf_start_2, system_conf_start_3, system_conf_goal,
      system_conf_combined_goal;
  Eigen::VectorXd q_left_start_1(7), q_left_start_2(7), q_left_start_3(7),
      q_left_goal(7);
  Eigen::VectorXd q_right_goal_1(7), q_right_goal_2(7), q_right_goal_3(7),
      q_right_start(7);
  q_left_start_1 << -1.0, 0.8, 1.5, -1.5, -0.75, 1.75, 1.2;
  q_left_start_2 << -1.1, 0.8, 1.5, -1.5, -0.75, 1.75, 1.2;
  q_left_start_3 << -1.2, 0.8, 1.5, -1.5, -0.75, 1.75, 1.2;
  q_left_goal << -2.1, 0.055, 0.573, -1.75, 0.236, 1.53, 0.0;
  q_right_start << -1.0, 0.8, 1.0, -2.4, 1.8, 1.5, 1.1;
  q_right_goal_1 << -1.5, 0.4, 0.14, -1.4, 0.0, 2.0, 0.4;
  q_right_goal_2 << -1.5, 0.45, 0.14, -1.4, 0.0, 2.0, 0.4;
  q_right_goal_3 << -1.5, 0.5, 0.14, -1.4, 0.0, 2.0, 0.4;
  system_conf_start["franka_right"] = q_right_start;
  system_conf_start["franka_left"] = q_left_start_1;
  //   system_conf_start_1["franka_right"] = q_right_start;
  system_conf_start_1["franka_left"] = q_left_start_1;
  //   system_conf_start_2["franka_right"] = q_right_start;
  system_conf_start_2["franka_left"] = q_left_start_2;
  //   system_conf_start_3["franka_right"] = q_right_start;
  system_conf_start_3["franka_left"] = q_left_start_3;
  // Goal system config
  system_conf_goal["franka_right"] = q_right_goal_1;
  system_conf_combined_goal["franka_right"] = q_left_goal;
  system_conf_combined_goal["franka_left"] = q_right_goal_1;
  // Expect collision goal system config
  Eigen::VectorXd goal_config_1 = conversions::ToGeneralizedPosition(
      planner.robot_model(), system_conf_combined_goal);
  EXPECT_FALSE(planner.robot_constraints().CheckSatisfied({goal_config_1}));
  // Construct goal anchor
  planning_service_client::planner::Anchor goal_anchor_1(system_conf_goal, {});
  system_conf_goal["franka_right"] = q_right_goal_2;
  planning_service_client::planner::Anchor goal_anchor_2(system_conf_goal, {});
  system_conf_goal["franka_right"] = q_right_goal_3;
  planning_service_client::planner::Anchor goal_anchor_3(system_conf_goal, {});
  // Build the problem
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      gen_multimodal_plan {
          {},    std::vector {goal_anchor_1, goal_anchor_2, goal_anchor_3},
          false, false,
          false, true};
  // Solve the plan.
  auto right_alone_plan = planner.SolveGeneralizedMultiModalPlan(
      gen_multimodal_plan, system_conf_start, std::nullopt);
  // The plan should be successful
  EXPECT_TRUE(right_alone_plan.has_value());
  const auto& timed_right_alone_path =
      drake::trajectories::PathParameterizedTrajectory<double>(
          right_alone_plan.value().trajectory.first,
          right_alone_plan.value().trajectory.second);
  const auto& sys_traj = conversions::ToSystemTimedTrajectory(
      planner.time_optimal_spliner(), timed_right_alone_path);
  logging::log()->info("System timed trajectory with size {}", sys_traj.size());
  EXPECT_TRUE(sys_traj.has_key("franka_left"));
  EXPECT_TRUE(sys_traj.has_key("franka_right"));
  // Now, let's plan for the right arm but send the current left trajectory as
  // active
  planning_service_client::SystemTimedTrajectory active_sys_traj;
  active_sys_traj["franka_right"] = sys_traj.at("franka_right");
  // Get time since epoch
  std::chrono::duration<double> epoch_duration_now =
      std::chrono::system_clock::now().time_since_epoch();
  double time_global_now = epoch_duration_now.count();
  double t_now = 0.3;  // seconds
  double time_global_active_traj_start =
      time_global_now - t_now;  // Pretend the trajectory started t_now s ago
  logging::log()->info("Setting global time offset for active trajectory to {}",
                       time_global_now);
  active_sys_traj["franka_right"].SetGlobalTimeOffset(
      time_global_active_traj_start);
  // Now make some goal anchors for the left arm
  planning_service_client::SystemConf left_goal_sysconf;
  left_goal_sysconf["franka_left"] = q_left_goal;
  planning_service_client::planner::Anchor left_start_anchor_1(
      system_conf_start_1, {});
  planning_service_client::planner::Anchor left_start_anchor_2(
      system_conf_start_2, {});
  planning_service_client::planner::Anchor left_start_anchor_3(
      system_conf_start_3, {});
  planning_service_client::planner::Anchor left_goal_anchor(left_goal_sysconf,
                                                            {});
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      left_arm_plan {{std::vector {left_start_anchor_1, left_start_anchor_2,
                                   left_start_anchor_3}},
                     std::vector {left_goal_anchor},
                     false,
                     false,
                     false,
                     true};

  std::string movable_system = {"franka_left"};
  // Get movable arm index
  auto model_idx =
      planner.robot_model().plant().GetModelInstanceByName(movable_system);
  auto movable_arm_index = planner.robot_model().get_arm_index(model_idx);
  EXPECT_TRUE(movable_arm_index.is_valid())
      << "Failed to get movable arm index for system " << movable_system;
  // Log the movable arm name and index

  const auto& movable_arm = planner.robot_model().GetArm(movable_arm_index);
  const auto& arm_name = movable_arm.name();
  logging::log()->info(
      "Movable system: {}, model index: {}, arm index: {}, arm name: {}",
      movable_system, model_idx, movable_arm_index, arm_name);

  auto left_arm_plan_gen_mm = planner.SolveGeneralizedMultiModalPlan(
      left_arm_plan, system_conf_start, active_sys_traj);
  EXPECT_TRUE(left_arm_plan_gen_mm.has_value());
  const auto& timed_path_gen_mm =
      drake::trajectories::PathParameterizedTrajectory<double>(
          left_arm_plan_gen_mm.value().trajectory.first,
          left_arm_plan_gen_mm.value().trajectory.second);
  auto sys_traj_gen_mm = conversions::ToSystemTimedTrajectory(
      planner.time_optimal_spliner(), timed_path_gen_mm);
  EXPECT_TRUE(sys_traj_gen_mm.has_key("franka_right"));
  EXPECT_TRUE(sys_traj_gen_mm.has_key("franka_left"));

  // log the start and end time of sys_traj_gen_mm
  for (const auto& [system_name, traj] : sys_traj_gen_mm) {
    logging::log()->info("System {} trajectory start time: {}, end time: {}",
                         system_name, traj.start_time(), traj.end_time());
    // Set the global time offset for the trajectory to time_global_now to be
    // able to compare with active trajectory
    sys_traj_gen_mm[system_name].SetGlobalTimeOffset(time_global_now);
  }

  logging::log()->info("Transition start time: {}, transition end time: {}",
                       left_arm_plan_gen_mm.value().transit_start_time,
                       left_arm_plan_gen_mm.value().transit_end_time);

  double min_partial_solution_time =
      left_arm_plan_gen_mm.value().transit_start_time;
  double max_partial_solution_time =
      left_arm_plan_gen_mm.value().transit_end_time;
  logging::log()->info("Min and max collision times: [{}, {}]",
                       min_partial_solution_time, max_partial_solution_time);
  double collision_checking_step = 0.01;  // seconds

  // Check that the output result does not have a collision
  const auto& collision_conf_gen_mm_opt =
      planner.TimeOfArmsCollision(sys_traj_gen_mm, collision_checking_step);
  EXPECT_FALSE(collision_conf_gen_mm_opt.has_value())
      << "Expected no collision in gen mm plan trajectory";

  const auto planned_traj {sys_traj_gen_mm};

  // Replace the systems in the active trajectory by their active path
  for (const auto& [system_name, traj] : active_sys_traj) {
    sys_traj_gen_mm[system_name] = traj;
  }
  logging::log()->info(
      "Replaced active trajectories in gen mm plan trajectory");
  // Check that now there is a collision
  const auto& collision_conf_gen_mm_opt_after_replace =
      planner.TimeOfArmsCollision(sys_traj_gen_mm, collision_checking_step);
  EXPECT_TRUE(collision_conf_gen_mm_opt_after_replace.has_value())
      << "Expected collision in gen mm plan trajectory after replacing active "
         "trajectories";

  double t_collision = collision_conf_gen_mm_opt_after_replace.value();

  //   Get best approach trajectory from gen mm plan
  auto best_approach_trajectory_gen_mm_opt = planner.BestApproachTrajectory(
      sys_traj_gen_mm, {movable_arm_index}, 2, collision_checking_step,
      min_partial_solution_time, max_partial_solution_time);
  EXPECT_TRUE(best_approach_trajectory_gen_mm_opt.has_value());
  const auto& best_approach_trajectory_gen_mm =
      best_approach_trajectory_gen_mm_opt.value();
  EXPECT_TRUE(best_approach_trajectory_gen_mm.has_key("franka_right"));
  EXPECT_TRUE(best_approach_trajectory_gen_mm.has_key("franka_left"));
  const auto& collision_conf_best_approach_opt = planner.TimeOfArmsCollision(
      best_approach_trajectory_gen_mm, collision_checking_step);
  EXPECT_FALSE(collision_conf_best_approach_opt.has_value())
      << "Expected no collision in best approach trajectory from gen mm plan";

  // Check that the trajectory is constant for the movable ressource after
  // collision_conf_gen_mm_opt_after_replace
  auto best_approach_traj_movable =
      best_approach_trajectory_gen_mm.at(movable_system);
  //   double t_collision = collision_conf_gen_mm_opt_after_replace.value();
  auto traj_value_before_collision =
      best_approach_traj_movable.Value(t_collision - 1e-4);
  // Sample the time interval [t_collision, t_end] with a few samples and check
  // that the trajectory for the movable arm is constant
  for (double t = t_collision; t <= best_approach_traj_movable.end_time();
       t += 0.1) {
    auto traj_value = best_approach_traj_movable.Value(t);
    EXPECT_TRUE(traj_value.isApprox(traj_value_before_collision, 1e-6))
        << "Expected trajectory for movable system to be constant after "
           "collision time. Trajectory value at time "
        << t << " is " << traj_value.transpose() << " but expected "
        << traj_value_before_collision.transpose();
  }

  // Check that the trajectory for the movable arm is the same from the start up
  // to the collision time (- some buffer)
  for (double t = time_global_now;
       t < t_collision - 3 * collision_checking_step; t += 0.1) {
    auto traj_value = best_approach_traj_movable.Value(t);
    auto gen_mm_traj_value = sys_traj_gen_mm.at(movable_system).Value(t);
    EXPECT_TRUE(traj_value.isApprox(gen_mm_traj_value, 1e-6))
        << "Expected trajectory for movable system to be the same as gen mm "
           "trajectory before collision time. Trajectory value at time "
        << t << " is " << traj_value.transpose() << " but expected "
        << gen_mm_traj_value.transpose();
  }

  // Check that the trajectory for the non-movable system is the same as the
  // active trajectory
  auto best_approach_traj_non_movable =
      best_approach_trajectory_gen_mm.at("franka_right");
  auto active_traj_non_movable = active_sys_traj.at("franka_right");
  for (double t = time_global_now;
       t <= best_approach_traj_non_movable.end_time(); t += 0.1) {
    auto traj_value = best_approach_traj_non_movable.Value(t);
    auto active_traj_value = active_traj_non_movable.Value(t);
    EXPECT_TRUE(traj_value.isApprox(active_traj_value, 1e-6))
        << "Expected trajectory for non-movable system to be the same as "
           "active trajectory. Trajectory value at time "
        << t << " is " << traj_value.transpose() << " but expected "
        << active_traj_value.transpose();
  }

  logging::log()->info(
      "Best approach trajectory from gen mm plan has expected properties "
      "around collision time {}",
      t_collision);

  // Use SolvePlan to solve the right arm plan with the left arm active
  //   trajectory, and check that the output trajectory matches the best
  //   approach trajectory from gen mm plan
  auto right_arm_plan_solve_plan =
      planner.SolvePlan(left_arm_plan, "left arm with right active",
                        std::nullopt, system_conf_start, active_sys_traj);
  EXPECT_TRUE(right_arm_plan_solve_plan.is_success());
  const auto& sys_traj_solve_plan =
      right_arm_plan_solve_plan.system_timed_trajectory();
  EXPECT_TRUE(sys_traj_solve_plan.has_key("franka_left"));
  auto traj_solve_plan_left = sys_traj_solve_plan.at("franka_left");
  for (double t = time_global_now; t <= traj_solve_plan_left.end_time();
       t += 0.1) {
    EXPECT_TRUE(traj_solve_plan_left.GlobalValue(t).isApprox(
        best_approach_traj_movable.GlobalValue(t), 1e-6));
  }

  logging::log()->info(
      "Best approach trajectory from gen mm plan is collision free and has "
      "expected properties");
  if (planner.has_draco_visualizer()) {
    planner.mutable_draco_visualizer().Add(best_approach_trajectory_gen_mm,
                                           "Best Approach Trajectory");
    planner.mutable_draco_visualizer().Add(planned_traj, "Planned Trajectory");
    planner.mutable_draco_visualizer().Add(sys_traj_gen_mm,
                                           "Planned on top of active");
    planner.mutable_draco_visualizer().Add(sys_traj_solve_plan,
                                           "SolvePlan output trajectory");
  }

  //   To see meshcat visualization, uncomment the following lineee
  while (DEBUG_MESHCAT) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

}  // namespace planner
}  // namespace draco
