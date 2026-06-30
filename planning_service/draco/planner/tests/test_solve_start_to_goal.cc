#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

namespace psc = planning_service_client;
using motion::system_conf_t;

TEST(TestDracoPlanner, ResolveAnchorConf) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Step 0️⃣: Setup the problem:
  // 1- a reference conf, slightly different from the ik_seed
  // 2- a fixed partial sysconf for the right arm (different from ik_seed)
  // 3- a target FrameRelativePose for the left arm
  psc::SystemConf q_sys_ref;
  Eigen::VectorXd q_ref_right(7);
  q_ref_right << -0.9, 1.4, 1.5, -1.5, -0.05, 2.1, -0.3;
  Eigen::VectorXd q_ref_left(7);
  q_ref_left << -1.8, -1.2, 0.3, -0.4, 0.4, 1.9, 2.4;
  q_sys_ref["franka_right"] = q_ref_right;
  q_sys_ref["franka_left"] = q_ref_left;
  // Also make a reference conf out of the q_sys_ref
  Eigen::VectorXd q_ref =
      conversions::ToGeneralizedPosition(planner.robot_model(), q_sys_ref);
  q_ref += Eigen::VectorXd::Ones(q_ref.size()) * 0.1;
  // Provide fixed partial sysconf for the right arm
  Eigen::VectorXd q_right(7);
  q_right << -2.3, 1.6, 0.9, -2.5, -0.8, 2.8, 0.6;
  std::string world_frame_name = "world";
  std::string left_tool_frame_name = "franka_left::franka_tool_location";
  const auto& world_frame =
      planner.robot_model().GetScopedFrameByName(world_frame_name);
  const auto& left_tool_frame =
      planner.robot_model().GetScopedFrameByName(left_tool_frame_name);
  psc::FrameRelativePose left_goal {
      world_frame_name, left_tool_frame_name, Eigen::Vector3d(0.7, 0.5, 0.9),
      Eigen::Quaterniond(0.4312, -0.7235, 0.4251, 0.3316)};
  psc::SystemConf partial_right;
  partial_right["franka_right"] = q_right;
  auto X_world_left_tool = drake::math::RigidTransformd(
      left_goal.X_AB_quaternion(), left_goal.X_AB_translation());
  // Problem 1️⃣: Solve the Frame Relative Pose problem with a fixed partial
  // sysconf. The expected outcome for right is to remain at its place.
  psc::planner::Anchor anchor_1 {partial_right, {left_goal}};
  auto q_1_opt = planner.ResolveAnchorConf(anchor_1, q_ref);
  EXPECT_TRUE(q_1_opt.has_value());
  const auto& q_1 = q_1_opt.value();
  EXPECT_TRUE(planner.robot_constraints().CheckSatisfied(q_1));
  // The solution is supposed to agree with the target FRP
  auto pose_1 = planner.robot_model().CalcRelativeTransform(q_1, world_frame,
                                                            left_tool_frame);
  auto X_actual_expected = X_world_left_tool.inverse() * pose_1;
  // The error must be small
  EXPECT_LT(X_actual_expected.translation().norm(), 2e-3);
  EXPECT_LT(X_actual_expected.rotation().ToAngleAxis().angle(), 2e-3);
  // The solution is supposed to agree with the fixed partial sysconf
  auto q_right_goal =
      planner.robot_model().ToSystemConf(q_1).at("franka_right");
  EXPECT_TRUE(q_right_goal.isApprox(q_right));
  // Problem 4️⃣: Solve the Frame Relative Pose problem with empty sysconf, but
  // have a reference conf provided.
  // The expected outcome for right is to the reference.
  logging::log()->info(
      "Problem 4️⃣: Solve the Frame Relative Pose problem with empty sysconf, "
      "but have an ik_seed provided, and a reference conf provided.");
  psc::planner::Anchor anchor_4 {psc::SystemConf(), {left_goal}};
  logging::log()->info("Anchor 4 is setup");
  logging::log()->info("Reference conf = {}", q_ref.transpose());
  // Solve the fixed partial sysconf problem without any FRP but reference
  auto q_4_opt = planner.ResolveAnchorConf(anchor_4, q_ref);
  EXPECT_TRUE(q_4_opt.has_value());
  const auto& q_4 = q_4_opt.value();
  EXPECT_TRUE(planner.robot_constraints().CheckSatisfied(q_4));
  // The solution is not going to be q_ref, but right is expected to be
  // exactly q_right
  auto pose_4 = planner.robot_model().CalcRelativeTransform(q_4, world_frame,
                                                            left_tool_frame);
  auto X_actual_expected_4 = X_world_left_tool.inverse() * pose_4;
  // The error must be small
  EXPECT_LT(X_actual_expected_4.translation().norm(), 1e-3);
  EXPECT_LT(X_actual_expected_4.rotation().ToAngleAxis().angle(), 1e-3);
  // The solution is supposed to agree with the fixed partial sysconf
  auto q_right_goal_frp_ref =
      planner.robot_model().ToSystemConf(q_4).at("franka_right");
  auto q_right_ref =
      planner.robot_model().ToSystemConf(q_ref).at("franka_right");
  EXPECT_TRUE(q_right_goal_frp_ref.isApprox(q_right_ref));
  // Problem 5️⃣: Solve the anchor without any FRP, but with a fixed partial
  // sysconf and reference conf. The expected outcome for right is to become the
  // reference.
  psc::planner::Anchor anchor_5 {partial_right, {}};
  auto q_5_opt = planner.ResolveAnchorConf(anchor_5, q_ref);
  EXPECT_TRUE(q_5_opt.has_value());
  const auto& q_5 = q_5_opt.value();
  EXPECT_TRUE(planner.robot_constraints().CheckSatisfied(q_5));
  // For right, we already have the conf.
  auto q_right_goal_frp_ref_5 =
      planner.robot_model().ToSystemConf(q_5).at("franka_right");
  EXPECT_TRUE(q_right_goal_frp_ref_5.isApprox(q_right));
  // For left, we become the reference conf
  auto q_left_ref = planner.robot_model().ToSystemConf(q_ref).at("franka_left");
  EXPECT_TRUE(planner.robot_model()
                  .ToSystemConf(q_5)
                  .at("franka_left")
                  .isApprox(q_left_ref));
  // Problem 6️⃣: Solve the empty anchor, but with a reference conf. We can not
  // solve this!
  psc::planner::Anchor anchor_6 {psc::SystemConf(), {}};
  auto q_6_opt = planner.ResolveAnchorConf(anchor_6, q_ref);
  EXPECT_FALSE(q_6_opt.has_value());
  // Problem 7️⃣ : An invalid full anchor can not be resovled.
  Eigen::VectorXd q_invalid(14);
  q_invalid << 0.5, -0.3, -0.1, -1.2, -0.9, 2.5, -1.0, 1.6, 1.6, 0.9, -3.0, 0.7,
      2.5, 2.7;
  system_conf_t invalid_conf_sysconf {
      planner.robot_model().ToSystemConf(q_invalid)};
  psc::SystemConf invalid_conf_client =
      conversions::DracoToClientSystemConf(invalid_conf_sysconf);
  const auto anchor_7 = psc::planner::Anchor(invalid_conf_client, {});
  const auto q_7_opt = planner.ResolveAnchorConf(anchor_7);
  // The anchor is invalid, so the resolution must fail.
  EXPECT_FALSE(q_7_opt.has_value());
}

TEST(TestDracoPlanner, ResolveStartToGoalProblem_ValidCompleteStart) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Start is defined by a complete robot conf
  psc::SystemConf start;
  Eigen::VectorXd q_start_right(7);
  q_start_right << -0.936, 0.516, -0.758, -0.217, -2.07, 3.205, -0.152;
  Eigen::VectorXd q_start_left(7);
  q_start_left << 1.732, 0.0714, 1.03, -0.917, 0.472, 1.982, 1.488;
  start["franka_right"] = q_start_right;
  start["franka_left"] = q_start_left;
  psc::planner::Anchor start_anchor {start, {}};

  // Goal is defined by a Frame relative pose
  psc::FrameRelativePose goal_frp {
      "WorldModelInstance::world", "franka_left::franka_tool_location",
      Eigen::Vector3d(0.7699, -0.2482, 1.0616),
      Eigen::Quaterniond(0.2985, -0.495, -0.0013, 0.816)};
  psc::planner::Anchor goal_anchor {psc::SystemConf(), {goal_frp}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // Resolve the StartToGoalProblem
  const auto start_conf = planner.ResolveAnchorConf(def.start());
  EXPECT_TRUE(start_conf.has_value());
  const auto goal_conf =
      planner.ResolveAnchorConf(def.goal(), start_conf.value());
  EXPECT_TRUE(goal_conf.has_value());
  // The start and goal must be satisfied
  const auto& q_start {start_conf.value()};
  const auto& q_goal {goal_conf.value()};
  EXPECT_TRUE(planner.robot_constraints().CheckSatisfied(q_start, 0))
      << "The resolved start configuration is not satisfied";
  EXPECT_TRUE(planner.robot_constraints().CheckSatisfied(q_goal, 0))
      << "The resolved goal configuration is not satisfied";
  logging::log()->info("Checking proximity of resolved start and goal");
  // The start and goal must be close to the provided start and goal
  system_conf_t start_sysconf {planner.robot_model().ToSystemConf(q_start)};
  system_conf_t goal_sysconf {planner.robot_model().ToSystemConf(q_goal)};
  EXPECT_TRUE(start_sysconf.at("franka_right").isApprox(q_start_right));
  logging::log()->info("Start is close to the provided start");
  auto evaluated_pose = planner.CalcRelativePose(
      goal_sysconf, "franka_left::franka_tool_location");
  logging::log()->info("Evaluated pose");
  drake::math::RigidTransformd left_goal_frp_pose {goal_frp.X_AB_quaternion(),
                                                   goal_frp.X_AB_translation()};
  logging::log()->info("Left goal frp pose");
  auto X_to_goal = left_goal_frp_pose.inverse() * evaluated_pose;
  logging::log()->info("Checking proximity of resolved start and goal");
  // The error must be small
  EXPECT_LT(X_to_goal.translation().norm(), 1e-3);
  EXPECT_LT(X_to_goal.rotation().ToAngleAxis().angle(), 1e-3);
  logging::log()->info("Finished ResolveStartToGoalProblem");
}

TEST(TestDracoPlanner, ResolveStartToGoalProblem_InvalidStart) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Start is defined by a complete robot conf that's invalid
  psc::SystemConf start;
  Eigen::VectorXd q_start_right = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd q_start_left = Eigen::VectorXd::Zero(7);
  start["franka_right"] = q_start_right;
  start["franka_left"] = q_start_left;
  psc::planner::Anchor start_anchor {start, {}};

  // Goal is defined by a Frame relative pose
  psc::FrameRelativePose goal_frp {
      "WorldModelInstance::world", "franka_left::franka_tool_location",
      Eigen::Vector3d(0.7699, -0.2482, 1.0616),
      Eigen::Quaterniond(0.2985, -0.495, -0.0013, 0.816)};
  psc::planner::Anchor goal_anchor {psc::SystemConf(), {goal_frp}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // Resolve the StartToGoalProblem
  auto result = planner.SolvePlan(def, "", std::nullopt, start);
  EXPECT_FALSE(result.is_success());
}

TEST(TestDracoPlanner, ResolveStartToGoalProblem_InvalidGoal) {
  // We ask to replace goal with a valid one!
  const auto planner = DracoPlanner(test::DualPandas());
  // Start is defined by a complete robot conf
  psc::SystemConf start;
  Eigen::VectorXd q_start_right(7);
  q_start_right << -0.936, 0.516, -0.758, -0.217, -2.07, 3.205, -0.152;
  Eigen::VectorXd q_start_left(7);
  q_start_left << 1.732, 0.0714, 1.03, -0.917, 0.472, 1.982, 1.488;
  start["franka_right"] = q_start_right;
  start["franka_left"] = q_start_left;
  psc::planner::Anchor start_anchor {start, {}};
  // generate valid samples to use as random ik seeds
  Eigen::VectorXd q_invalid(14);
  q_invalid << 0.471016, -0.298477, -0.145606, -1.20496, -0.932201, 2.48721,
      -1.05193, 1.60177, 1.56702, 0.935277, -3.0116, 0.70693, 2.4832, 2.71585;
  // Make sure the invalid configuration is invalid
  EXPECT_FALSE(planner.robot_constraints().CheckSatisfied(q_invalid, 0))
      << "Invalid configuration is valid";
  system_conf_t invalid_sysconf {planner.robot_model().ToSystemConf(q_invalid)};
  psc::SystemConf invalid_sysconf_client;
  for (const auto& [key, value] : invalid_sysconf) {
    invalid_sysconf_client[key] = value;
  }
  psc::planner::Anchor invalid_conf_anchor {invalid_sysconf_client, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor,
                                              invalid_conf_anchor};
  // Resolve the StartToGoalProblem
  auto result = planner.SolvePlan(def, "", std::nullopt, start);
  EXPECT_FALSE(result.is_success());
  // The message contains invalid goal
  logging::log()->info("Result message: {}", result.message());
  EXPECT_TRUE(
      result.message().find("Failed to resolve goal") != std::string::npos
      || result.message().find("Goal resolution failed") != std::string::npos);
}

TEST(TestDracoPlanner, SolveStartToGoalProblem_SysconfToFrp) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Start is defined by a complete robot conf
  psc::SystemConf start;
  Eigen::VectorXd q_start_right(7);
  q_start_right << -0.936, 0.516, -0.758, -0.217, -2.07, 3.205, -0.152;
  Eigen::VectorXd q_start_left(7);
  q_start_left << 1.732, 0.0714, 1.03, -0.917, 0.472, 1.982, 1.488;
  start["franka_right"] = q_start_right;
  start["franka_left"] = q_start_left;
  psc::planner::Anchor start_anchor {start, {}};
  // Goal is defined by a Frame relative pose
  psc::FrameRelativePose goal_frp {
      "WorldModelInstance::world", "franka_left::franka_tool_location",
      Eigen::Vector3d(0.7699, -0.2482, 1.0616),
      Eigen::Quaterniond(0.2985, -0.495, -0.0013, 0.816)};
  psc::planner::Anchor goal_anchor {psc::SystemConf(), {goal_frp}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // Solve the problem
  auto motion_plan_result = planner.SolvePlan(def, "", std::nullopt, start);
  EXPECT_TRUE(motion_plan_result.is_success());
  //   Let's check the trajectory
  auto new_trajectory = motion_plan_result.system_timed_trajectory();
  // Let's inspect the trajectory
  EXPECT_FALSE(new_trajectory.has_key("franka_right"));
  EXPECT_TRUE(new_trajectory.has_key("franka_left"));
  auto new_time_scaling = new_trajectory.at("franka_left").time_scaling();
  EXPECT_NEAR(new_time_scaling.start_time(), 0.0, 1e-4);
  // Make sure the start of the trajectory is the same as the start conf
  auto franka_left_path = new_trajectory.at("franka_left").path();
  auto franka_left_start = franka_left_path.Value(0);
  EXPECT_TRUE(franka_left_start.isApprox(q_start_left));
  // Make sure the end of the trajectory has the same Frame Relative Pose as the
  // goal FRP
  system_conf_t goal_sysconf;
  goal_sysconf["franka_right"] = q_start_right;
  goal_sysconf["franka_left"] =
      franka_left_path.Value(franka_left_path.end_time());
  auto evaluated_pose = planner.CalcRelativePose(
      goal_sysconf, "franka_left::franka_tool_location");
  drake::math::RigidTransformd left_goal_frp_pose {goal_frp.X_AB_quaternion(),
                                                   goal_frp.X_AB_translation()};
  auto X_to_goal = left_goal_frp_pose.inverse() * evaluated_pose;
  // The error must be small
  EXPECT_LT(X_to_goal.translation().norm(), 1e-3);
  EXPECT_LT(X_to_goal.rotation().ToAngleAxis().angle(), 1e-3);
  // log the message
  logging::log()->info("Message: {}", motion_plan_result.message());
}

TEST(TestDracoPlanner, SolveStartToGoalProblem_SysconfToSysconf1) {
  const auto planner = DracoPlanner(test::DualPandas());
  // Start is defined by a complete robot conf
  psc::SystemConf start_sysconf;
  Eigen::VectorXd q_start_right(7);
  q_start_right << -0.936, 0.516, -0.758, -0.217, -2.07, 3.205, -0.152;
  Eigen::VectorXd q_start_left(7);
  q_start_left << 1.732, 0.0714, 1.03, -0.917, 0.472, 1.982, 1.488;
  start_sysconf["franka_right"] = q_start_right;
  start_sysconf["franka_left"] = q_start_left;
  psc::planner::Anchor start_anchor {start_sysconf, {}};

  psc::SystemConf goal_sysconf;
  Eigen::VectorXd q_goal_right(7);
  q_goal_right << -0.886, 1.419, 1.471, -1.544, -0.0526, 2.093, -0.272;
  Eigen::VectorXd q_goal_left(7);
  q_goal_left << -1.779, -1.201, 0.322, -0.429, 0.382, 1.896, 2.392;
  goal_sysconf["franka_right"] = q_goal_right;
  goal_sysconf["franka_left"] = q_goal_left;
  psc::planner::Anchor goal_anchor {goal_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // Solve the problem
  auto motion_plan_result =
      planner.SolvePlan(def, "", std::nullopt, start_sysconf);
  EXPECT_TRUE(motion_plan_result.is_success());
  // Get the start of the returned trajectory
  auto new_trajectory = motion_plan_result.system_timed_trajectory();
  EXPECT_TRUE(new_trajectory.has_key("franka_right"));
  EXPECT_TRUE(new_trajectory.has_key("franka_left"));
  auto new_time_scaling = new_trajectory.at("franka_right").time_scaling();
  EXPECT_NEAR(new_time_scaling.start_time(), 0.0, 1e-4);
  // Make sure the start of the trajectory is the same as the start conf
  auto franka_right_path = new_trajectory.at("franka_right").path();
  auto franka_right_start = franka_right_path.Value(0);
  EXPECT_TRUE(franka_right_start.isApprox(q_start_right));
  auto franka_left_path = new_trajectory.at("franka_left").path();
  auto franka_left_start = franka_left_path.Value(0);
  EXPECT_TRUE(franka_left_start.isApprox(q_start_left));
  // Get the end of the returned trajectory
  auto franka_right_end = franka_right_path.Value(franka_right_path.end_time());
  auto franka_left_end = franka_left_path.Value(franka_left_path.end_time());
  EXPECT_TRUE(franka_right_end.isApprox(q_goal_right));
  EXPECT_TRUE(franka_left_end.isApprox(q_goal_left));
  // log the message
  logging::log()->info("Message: {}", motion_plan_result.message());
}

TEST(TestDracoPlanner, SolveStartToGoalProblem_SysconfToSysconf2) {
  // We want a test to use GCS.
  const auto planner = DracoPlanner(test::Wallflower());
  // q2 at 0.4 means the flower will hit the wall if not retracted.
  // Straight line from (1, 0.4) to (-1, 0.4) is NOT valid. GCS
  // is needed to retract the flower.
  Eigen::VectorXd q_start = Eigen::Vector2d(1.0, 0.4);
  Eigen::VectorXd q_end = Eigen::Vector2d(-1.0, 0.4);
  psc::SystemConf start_sysconf, end_sysconf;
  start_sysconf["robot"] = q_start;
  end_sysconf["robot"] = q_end;
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {end_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // Solve the problem
  auto motion_plan_result =
      planner.SolvePlan(def, "", std::nullopt, start_sysconf);
  EXPECT_TRUE(motion_plan_result.is_success());
  // Get the start of the returned trajectory
  auto traj = motion_plan_result.system_timed_trajectory();
  EXPECT_TRUE(traj.has_key("robot"));
  auto robot_trj = traj.at("robot");
  // Let's inspect the value at start and end
  EXPECT_TRUE(robot_trj.Value(0.0).isApprox(q_start));
  EXPECT_TRUE(robot_trj.Value(robot_trj.end_time()).isApprox(q_end));
  // log the message
  logging::log()->info("Message: {}", motion_plan_result.message());
}

TEST(TestDracoPlanner, SolveStartToGoalProblem_SysconfToSsyconf_Deconflict) {
  const auto planner = DracoPlanner(test::DualWallflowers());
  // log the number of vertices in the PRM
  logging::log()->info(
      "DracoPlanner:ResolveAnchorConf: Number of vertices in the PRM: {}",
      planner.thunder_planner().vertices_confs().size());
  psc::SystemConf start_sysconf, end_sysconf;
  start_sysconf["flower1"] = Eigen::Vector2d(1.0, 0.3);
  start_sysconf["flower2"] = Eigen::Vector2d(0.0, 0.3);
  end_sysconf["flower1"] = Eigen::Vector2d(M_PI, 0.35);
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {end_sysconf, {}};
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // Solve the problem
  auto motion_plan_result =
      planner.SolvePlan(def, "", std::nullopt, start_sysconf);
  // print the message
  logging::log()->info("Message: {}", motion_plan_result.message());
  EXPECT_TRUE(motion_plan_result.is_success());
  auto traj = motion_plan_result.system_timed_trajectory();
  EXPECT_TRUE(traj.has_key("flower1"));
  EXPECT_TRUE(traj.has_key("flower2"));
}

TEST(TestDracoPlanner, FastEstimateSolution1) {
  // Staright line from (1, 0.2) to (2, 0.3) is valid.
  const auto planner = DracoPlanner(test::Wallflower());
  Eigen::VectorXd q_start = Eigen::Vector2d(1.0, 0.2);
  Eigen::VectorXd q_end = Eigen::Vector2d(2.5, 0.3);
  psc::SystemConf start_sysconf, end_sysconf;
  start_sysconf["robot"] = q_start;
  end_sysconf["robot"] = q_end;
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {end_sysconf, {}};
  // Define the problem
  bool fast_estimate_solution = true;
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor, false,
                                              fast_estimate_solution};
  // Solve the problem
  auto motion_plan_result =
      planner.SolvePlan(def, "", std::nullopt, start_sysconf);
  // log the message
  logging::log()->info("Message: {}", motion_plan_result.message());
  EXPECT_TRUE(motion_plan_result.is_success());
  // Get the start of the returned trajectory
  auto traj = motion_plan_result.system_timed_trajectory();
  EXPECT_TRUE(traj.has_key("robot"));
  auto robot_trj = traj.at("robot");
  // Let's inspect the value at start and end
  EXPECT_TRUE(robot_trj.Value(0.0).isApprox(q_start));
  EXPECT_TRUE(robot_trj.Value(robot_trj.end_time()).isApprox(q_end));
  // We know what the duration should be: path with maximum velocity bound.
  auto duration = robot_trj.end_time() - robot_trj.start_time();
  auto max_vel =
      planner.time_optimal_spliner().joint_dynamic_limits().velocity_bound;
  // find the maximum element-wise division of the path
  double expected_duration = 0;
  for (int i = 0; i < q_start.size(); ++i) {
    double path_length = (q_end[i] - q_start[i]);
    double duration_i = std::abs(path_length / max_vel[i]);
    if (duration_i > expected_duration) {
      expected_duration = duration_i;
    }
  }
  EXPECT_NEAR(duration, expected_duration, 1e-4)
      << "The duration of the trajectory is not as expected.";
}

TEST(TestDracoPlanner, FastEstimateSolution2) {
  // We want a test to use GCS.
  const auto planner = DracoPlanner(test::Wallflower());
  // q2 at 0.4 means the flower will hit the wall if not retracted.
  // Straight line from (1, 0.4) to (-1, 0.4) is NOT valid. GCS
  // is needed to retract the flower.
  Eigen::VectorXd q_start = Eigen::Vector2d(1.0, 0.4);
  Eigen::VectorXd q_end = Eigen::Vector2d(-1.0, 0.4);
  psc::SystemConf start_sysconf, end_sysconf;
  start_sysconf["robot"] = q_start;
  end_sysconf["robot"] = q_end;
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {end_sysconf, {}};
  // Define the problem
  bool fast_estimate_solution = true;
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor, false,
                                              fast_estimate_solution};
  // Solve the problem
  auto motion_plan_result =
      planner.SolvePlan(def, "", std::nullopt, start_sysconf);
  logging::log()->info("Message: {}", motion_plan_result.message());
  EXPECT_TRUE(motion_plan_result.is_success());
  // Get the start of the returned trajectory
  auto traj = motion_plan_result.system_timed_trajectory();
  EXPECT_TRUE(traj.has_key("robot"));
  auto robot_trj = traj.at("robot");
  // Let's inspect the value at start and end
  EXPECT_TRUE(robot_trj.Value(0.0).isApprox(q_start));
  EXPECT_TRUE(robot_trj.Value(robot_trj.end_time()).isApprox(q_end));
}

TEST(TestDracoPlanner, FastEstimateSolution3) {
  // Same start and goal
  const auto planner = DracoPlanner(test::Wallflower());
  Eigen::VectorXd q_start = Eigen::Vector2d(1.0, 0.4);
  Eigen::VectorXd q_end = Eigen::Vector2d(1.0, 0.4);
  psc::SystemConf start_sysconf, end_sysconf;
  start_sysconf["robot"] = q_start;
  end_sysconf["robot"] = q_end;
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {end_sysconf, {}};
  // Define the problem
  bool fast_estimate_solution = true;
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor, false,
                                              fast_estimate_solution};
  // Solve the problem
  auto motion_plan_result =
      planner.SolvePlan(def, "", std::nullopt, start_sysconf);
  logging::log()->info("Message: {}", motion_plan_result.message());
  EXPECT_TRUE(motion_plan_result.is_success());
  // Get the start of the returned trajectory
  auto traj = motion_plan_result.system_timed_trajectory();
  EXPECT_TRUE(traj.has_key("robot"));
  auto robot_trj = traj.at("robot");
  // Let's inspect the value at start and end
  EXPECT_TRUE(robot_trj.Value(0.0).isApprox(q_start));
  EXPECT_TRUE(robot_trj.Value(robot_trj.end_time()).isApprox(q_end));
  // EXPECT the duration to be very small
  EXPECT_NEAR(robot_trj.end_time() - robot_trj.start_time(), 0.0, 1e-4);
}

}  // namespace planner
}  // namespace draco
