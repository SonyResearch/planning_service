#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

namespace psc = planning_service_client;

class PlanOptionsTest : public ::testing::Test {
 protected:
  std::unique_ptr<DracoPlanner> planner;
  psc::SystemConf start_sysconf, valid_goal_sysconf, colliding_goal_sysconf;

  void SetUp() override {
    planner = std::make_unique<DracoPlanner>(test::Wallflower());
    start_sysconf["robot"] = Eigen::Vector2d(1.0, 0.4);
    valid_goal_sysconf["robot"] = Eigen::Vector2d(-1.0, 0.4);
    colliding_goal_sysconf["robot"] = Eigen::Vector2d(0.0, 0.4);
  }
};

TEST_F(PlanOptionsTest, DynamicLimits) {
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {valid_goal_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // First, solve without dynamic limits
  const auto no_limits_result =
      planner->SolvePlan(def, "test_label", std::nullopt, start_sysconf);
  EXPECT_TRUE(no_limits_result.is_success());
  psc::planner::DynamicLimits limits;
  limits.safety_factor_velocity = 0.495;
  limits.safety_factor_acceleration = 0.495;
  limits.cartesian_velocity_limits = {{"robot", 10.0}};
  psc::planner::PlanOptions options;
  options.set_dynamic_limits(limits);
  // Solve the problem
  const auto limits_result =
      planner->SolvePlan(def, "test_label", options, start_sysconf);
  EXPECT_TRUE(limits_result.is_success());
  // compare durations
  const auto no_limits_duration =
      no_limits_result.system_timed_trajectory().at("robot").duration();
  const auto limits_duration =
      limits_result.system_timed_trajectory().at("robot").duration();
  EXPECT_LT(limits_duration / no_limits_duration, 2.0);
}

TEST_F(PlanOptionsTest, CollisionOptionsFiltered) {
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {colliding_goal_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // First, solve without collision checking
  const auto result =
      planner->SolvePlan(def, "test_label", std::nullopt, start_sysconf);
  EXPECT_FALSE(result.is_success());
  // Now, reattempt with filtering
  psc::planner::CollisionOptions collision_options;
  collision_options.filtered_pairs.emplace_back("robot::ball", "wall::main");
  psc::planner::PlanOptions options;
  options.set_collision_options(collision_options);
  const auto filtered_result =
      planner->SolvePlan(def, "test_label", options, start_sysconf);
  EXPECT_TRUE(filtered_result.is_success());
  // Finally, expect another failure when trying without options
  const auto no_options_result =
      planner->SolvePlan(def, "test_label", std::nullopt, start_sysconf);
  EXPECT_FALSE(no_options_result.is_success());
}

TEST_F(PlanOptionsTest, CollisionOptionsPadded) {
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {colliding_goal_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // First, solve without collision checking
  const auto result =
      planner->SolvePlan(def, "test_label", std::nullopt, start_sysconf);
  EXPECT_FALSE(result.is_success());
  // Now, reattempt with padding
  psc::planner::CollisionOptions collision_options;
  // Collision at this configuration has a penetration of ~55 mm
  collision_options.paddings.emplace_back("robot::ball", "wall::main", -0.06);
  psc::planner::PlanOptions options;
  options.set_collision_options(collision_options);
  const auto padded_result =
      planner->SolvePlan(def, "test_label", options, start_sysconf);
  EXPECT_TRUE(padded_result.is_success());
  // Reattempt with a shallower padding, which should fail
  collision_options.paddings.clear();
  collision_options.paddings.emplace_back("robot::ball", "wall::main", -0.02);
  options.set_collision_options(collision_options);
  const auto barely_padded_result =
      planner->SolvePlan(def, "test_label", options, start_sysconf);
  EXPECT_FALSE(barely_padded_result.is_success());
}

TEST_F(PlanOptionsTest, CollisionOptionsTrajectory) {
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {colliding_goal_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  psc::planner::CollisionOptions collision_options;
  collision_options.filtered_pairs.emplace_back("robot::ball", "wall::main");
  psc::planner::PlanOptions options;
  options.set_collision_options(collision_options);
  const auto result =
      planner->SolvePlan(def, "test_label", options, start_sysconf);
  EXPECT_TRUE(result.is_success());
  std::vector<psc::SystemConf> sys_conf_vec;
  const auto trajectory {result.system_timed_trajectory().at("robot")};
  for (double t {0}; t < trajectory.end_time(); t += 0.001) {
    psc::SystemConf sys_conf;
    sys_conf["robot"] = trajectory.Value(t);
    sys_conf_vec.push_back(sys_conf);
  }
  EXPECT_FALSE(planner->CheckSatisfied(sys_conf_vec).satisfied());
  EXPECT_TRUE(
      planner->CheckSatisfied(sys_conf_vec, collision_options).satisfied());
}

TEST(PlanOptionsTestFranka, CollisionOptionsPoseGoal) {
  const auto planner {DracoPlanner(test::Franka())};
  // This configuration constitutes a self collision
  // between the end-effector and base.
  Eigen::VectorXd q(7);
  q << 0, 0.743, 0, -2.223, 0, 1.115, 0;
  EXPECT_FALSE(
      planner.CheckSatisfied({psc::SystemConf {{"franka", q}}}).satisfied());
  auto X_AB {planner.CalcRelativePose(q, "franka::franka_head_fangs")};
  const auto xyz {X_AB.translation()};
  const auto quat {X_AB.rotation().ToQuaternion()};
  psc::FrameRelativePose colliding_pose {"world", "franka::franka_head_fangs",
                                         xyz, quat};
  psc::SystemConf sysconf;
  sysconf["franka"] = q;
  psc::planner::Anchor start {sysconf, {}};
  psc::planner::Anchor goal {{}, {colliding_pose}};
  const psc::planner::StartToGoalProblem def {start, goal};
  const auto result =
      planner.SolvePlan(def, "test_label", std::nullopt, sysconf);
  EXPECT_FALSE(result.is_success());
  psc::planner::CollisionOptions collision_options;
  collision_options.filtered_pairs.emplace_back("franka::franka_robot_base",
                                                "franka::franka_head_fangs");
  psc::planner::PlanOptions options;
  options.set_collision_options(collision_options);
  const auto filtered_result =
      planner.SolvePlan(def, "test_label", options, sysconf);
  EXPECT_TRUE(filtered_result.is_success());
}

TEST_F(PlanOptionsTest, CollisionOptionsNonexistentFrames) {
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {colliding_goal_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  psc::planner::CollisionOptions collision_options;
  collision_options.filtered_pairs.emplace_back("robot::random_frame",
                                                "wall::main");
  psc::planner::PlanOptions options;
  options.set_collision_options(collision_options);
  EXPECT_ANY_THROW(
      planner->SolvePlan(def, "test_label", options, start_sysconf));
  collision_options.filtered_pairs.clear();
  // repeat for padding matrix
  collision_options.paddings.emplace_back("robot::ball", "wall::random_frame",
                                          -0.10);
  options.set_collision_options(collision_options);
  EXPECT_ANY_THROW(
      planner->SolvePlan(def, "test_label", options, start_sysconf));
}

TEST_F(PlanOptionsTest, CollisionShape) {
  psc::planner::Anchor start_anchor {start_sysconf, {}};
  psc::planner::Anchor goal_anchor {valid_goal_sysconf, {}};
  // Define the problem
  const psc::planner::StartToGoalProblem def {start_anchor, goal_anchor};
  // Verify valid problem is solvable
  const auto result =
      planner->SolvePlan(def, "test_label", std::nullopt, start_sysconf);
  EXPECT_TRUE(result.is_success());
  // Now, reattempt with a large collision shape in the middle of the path
  psc::planner::CollisionOptions collision_options;
  auto cylinder {psc::ShapeInFrame::MakeCylinder(0.1, 0.5)};
  cylinder.set_translation(Eigen::Vector3d(0.5, 0.0, 0.0));
  collision_options.shapes.push_back(cylinder);
  logging::log()->critical("Options: {}", collision_options.ToString());
  psc::planner::PlanOptions options;
  options.set_collision_options(collision_options);
  const auto result_with_shape =
      planner->SolvePlan(def, "test_label", options, start_sysconf);
  EXPECT_FALSE(result_with_shape.is_success());
  // Finally, expect it to succeed again
  const auto result_no_shape =
      planner->SolvePlan(def, "test_label", std::nullopt, start_sysconf);
  EXPECT_TRUE(result_no_shape.is_success());
}

class PlanOptionsTestDualRobot : public ::testing::Test {
 protected:
  std::unique_ptr<DracoPlanner> planner;
  psc::planner::UpdateTrajTowardWaypointsProblem problem;

  void SetUp() override {
    planner = std::make_unique<DracoPlanner>(test::DualWallflowers());
    psc::SystemTimedTrajectory sys_timed_trajectory;
    // Flower 1 has some meaningful trajectory
    auto s_samples_path = std::vector<double> {0.3, 0.8, 1.2, 1.6, 2.2};
    std::vector<Eigen::MatrixXd> q_samples_path;
    q_samples_path.reserve(5);
    q_samples_path.push_back(Eigen::Vector2d {1.0, 0.2});
    q_samples_path.push_back(Eigen::Vector2d {1.2, 0.22});
    q_samples_path.push_back(Eigen::Vector2d {1.4, 0.24});
    q_samples_path.push_back(Eigen::Vector2d {1.6, 0.27});
    q_samples_path.push_back(Eigen::Vector2d {1.8, 0.3});
    auto path = drake::trajectories::PiecewisePolynomial<
        double>::CubicWithContinuousSecondDerivatives(s_samples_path,
                                                      q_samples_path);
    auto t_samples = Eigen::VectorXd::LinSpaced(4, 0, 1.3);
    auto s_samples_time = Eigen::MatrixXd(1, 4);
    s_samples_time.row(0) << 0.3, 0.8, 1.5, 2.2;
    auto time_scaling = drake::trajectories::PiecewisePolynomial<
        double>::CubicWithContinuousSecondDerivatives(t_samples,
                                                      s_samples_time);
    sys_timed_trajectory["flower1"] = psc::TimedTrajectory(
        conversions::DrakePiecewisePolynomialToClient(path),
        conversions::DrakePiecewisePolynomialToClient(time_scaling));
    psc::SystemConf sysconf_flower2;
    sys_timed_trajectory["flower2"] =
        psc::TimedTrajectory::Constant(Eigen::Vector2d(0.3, 0.3));
    psc::SystemConf sys_conf_1, sys_conf_2, sys_conf_3, sys_conf_4;
    sys_conf_1["flower1"] = Eigen::Vector2d(2.0, 0.34);
    sys_conf_2["flower1"] = Eigen::Vector2d(2.1, 0.36);
    sys_conf_3["flower1"] = Eigen::Vector2d(2.3, 0.37);
    sys_conf_4["flower1"] = Eigen::Vector2d(2.5, 0.35);
    std::vector<psc::SystemConf> waypoints = {sys_conf_1, sys_conf_2,
                                              sys_conf_3, sys_conf_4};
    double time_now = 1.1;
    std::vector<double> segment_durations = {};  // Let the planner decide
    double search_step_size = 0.01;
    problem = psc::planner::UpdateTrajTowardWaypointsProblem(
        sys_timed_trajectory, waypoints, time_now, segment_durations,
        search_step_size);
  }
};

TEST_F(PlanOptionsTestDualRobot, ArmSpliners) {
  psc::planner::DynamicLimits limits;
  limits.safety_factor_velocity = 0.495;
  limits.safety_factor_acceleration = 0.495;
  psc::planner::PlanOptions options;
  options.set_dynamic_limits(limits);
  // Solve the problem
  const auto limits_result =
      planner->SolvePlan(problem, "speed_limited", options);
  EXPECT_TRUE(limits_result.is_success());
  const auto no_limits_result = planner->SolvePlan(problem, "no_limits");
  EXPECT_TRUE(no_limits_result.is_success());
  // compare durations
  const auto limits_duration =
      limits_result.system_timed_trajectory().at("flower1").duration();
  const auto no_limits_duration =
      no_limits_result.system_timed_trajectory().at("flower1").duration();
  EXPECT_LT(limits_duration / no_limits_duration, 2.0);
}

}  // namespace planner
}  // namespace draco
