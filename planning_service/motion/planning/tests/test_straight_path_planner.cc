#include <gtest/gtest.h>

#include "planning_service/common/logging.h"
#include "planning_service/motion/planning/straight_path_planner.h"

namespace motion {
namespace planning {

TEST(TestStraightPathPlanner, StraightLine) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = RobotModel(xml_file, dmd);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter;
  constraints_adapter.collision_checker = collision_checker_adapter;
  auto robot_constraints = RobotConstraints(robot_model, constraints_adapter);
  auto start_point = Eigen::Vector2d(0, 0.3);
  auto end_point = Eigen::Vector2d(1.0, 0.3);
  const double step_size = 0.01;
  auto traj_opt = MaybeValidStraightLinePath(robot_constraints, start_point,
                                             end_point, step_size);
  EXPECT_TRUE(
      robot_constraints.CheckSatisfiedEdge(start_point, end_point, step_size));
  EXPECT_TRUE(traj_opt.has_value());
  const auto& traj = traj_opt.value();
  EXPECT_EQ(traj.rows(), 2);
  EXPECT_TRUE(traj.value(0).isApprox(start_point));
  EXPECT_TRUE(traj.value(1).isApprox(end_point));
}

TEST(TestStraightPathPlanner, StraightLineWithWrapping) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  std::pair<std::string, int> continuous_revolute_joint_indices = {"robot", 0};
  std::vector<std::pair<std::string, int>>
      continuous_revolute_joint_indices_vector = {
          continuous_revolute_joint_indices};
  auto robot_model = RobotModel(xml_file, dmd, std::nullopt,
                                continuous_revolute_joint_indices_vector);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter;
  constraints_adapter.collision_checker = collision_checker_adapter;

  auto robot_constraints = RobotConstraints(robot_model, constraints_adapter);
  auto start_point = Eigen::Vector2d(0, 0.3);
  auto end_point = Eigen::Vector2d(1.0 + 6 * M_PI, 0.3);
  const double step_size = 0.01;
  auto traj_opt = MaybeValidStraightLinePath(robot_constraints, start_point,
                                             end_point, step_size);
  EXPECT_TRUE(
      robot_constraints.CheckSatisfiedEdge(start_point, end_point, step_size));
  EXPECT_TRUE(traj_opt.has_value());
  const auto& traj = traj_opt.value();
  EXPECT_EQ(traj.rows(), 2);
  EXPECT_TRUE(traj.value(0).isApprox(start_point));
  const Eigen::Vector2d end_wrapped(1.0, 0.3);
  EXPECT_TRUE(traj.value(1).isApprox(end_wrapped));
}

TEST(TestStraightPathPlanner, StraightLineWithMimicJoints) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = RobotModel(xml_file, dmd);
  const auto& hm = robot_model.holonomic_mapping();
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter;
  constraints_adapter.collision_checker = collision_checker_adapter;
  auto robot_constraints = RobotConstraints(robot_model, constraints_adapter);
  system_conf_t system_conf_reduced_start, system_conf_reduced_goal;
  system_conf_reduced_start["franka"] = Eigen::VectorXd(7);
  system_conf_reduced_start["franka"] << -1.8, 0.9, 0.5, -1.2, -0.1, 2.7, 0.8;
  system_conf_reduced_start["gripper"] = Eigen::VectorXd::Ones(1) * 0.0;
  system_conf_reduced_goal["franka"] = Eigen::VectorXd(7);
  system_conf_reduced_goal["franka"] << -2.0, -0.1, 0.3, -2.2, -0.1, 2.5, 1.1;
  system_conf_reduced_goal["gripper"] = Eigen::VectorXd::Ones(1) * 0.8;
  // We need to convert them to reduced q_start and q_goal
  auto q_full_start = robot_model.ToGeneralizedPosition(
      robot_model.LiftSystemConf(system_conf_reduced_start));
  auto q_full_goal = robot_model.ToGeneralizedPosition(
      robot_model.LiftSystemConf(system_conf_reduced_goal));
  auto start_point = hm.Reduce(q_full_start);
  auto end_point = hm.Reduce(q_full_goal);
  // start and goal should be 8D, first 7 are franka, last is gripper
  EXPECT_EQ(start_point.size(), 8);
  EXPECT_EQ(end_point.size(), 8);
  EXPECT_TRUE(
      start_point.head(7).isApprox(system_conf_reduced_start["franka"]));
  EXPECT_TRUE(end_point.head(7).isApprox(system_conf_reduced_goal["franka"]));
  EXPECT_TRUE(
      start_point.tail(1).isApprox(system_conf_reduced_start["gripper"]));
  EXPECT_TRUE(end_point.tail(1).isApprox(system_conf_reduced_goal["gripper"]));
  const double step_size = 0.01;
  auto traj_opt = MaybeValidStraightLinePath(robot_constraints, start_point,
                                             end_point, step_size);
  EXPECT_TRUE(
      robot_constraints.CheckSatisfiedEdge(start_point, end_point, step_size));
  EXPECT_TRUE(traj_opt.has_value());
  const auto& traj = traj_opt.value();
  EXPECT_EQ(traj.rows(), 8);
  // Check the trajectory starts and ends at the right points
  EXPECT_TRUE(traj.value(0).col(0).isApprox(start_point));
  EXPECT_TRUE(traj.value(traj.end_time()).col(0).isApprox(end_point));
}

TEST(TestStraightPathPlanner, MaybeOutOfViolationPath) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = RobotModel(xml_file, dmd);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter;
  constraints_adapter.collision_checker = collision_checker_adapter;
  auto robot_constraints = RobotConstraints(robot_model, constraints_adapter);
  // Let's sample a point that violates the constraints
  auto start_violating_point = Eigen::Vector2d(0.01, 0.4);
  double collision_influence_distance = 0.1;
  double clearance = 0.05;
  auto traj_opt =
      MaybeOutOfViolationPath(robot_constraints, start_violating_point,
                              collision_influence_distance, clearance);
  EXPECT_TRUE(traj_opt.has_value());
  const auto& traj = traj_opt.value();
  EXPECT_EQ(traj.rows(), 2);
  EXPECT_TRUE(traj.value(0).isApprox(start_violating_point));
  // A unit circle * epsilon around the final point is still valid
  const Eigen::Vector2d end_point = traj.value(traj.end_time());
  double epsilon = clearance * 0.7;  // ~sqrt(2)/2
  std::vector<Eigen::VectorXd> points = {end_point};
  // Sample a unit circle around the end point
  int n_samples = 20;
  for (int i = 0; i < 20; i++) {
    points.push_back(
        end_point
        + Eigen::Vector2d(epsilon * cos(i * 2 * M_PI / n_samples),
                          epsilon * sin(i * 2 * M_PI / n_samples)));
  }
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  EXPECT_TRUE(
      robot_constraints.CheckSatisfied(points, check_satisfied_options));
}

}  // namespace planning
}  // namespace motion
