#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

TEST(SolveUpdateTrajTowardWaypoints, Waypoints) {
  // When waypoints are given.
  planning_service_client::planner::UpdateTrajTowardWaypointsProblem def;
  const auto planner = DracoPlanner(test::Wallflower());
  // Let's create a simple trajectory and update it with new waypoints
  planning_service_client::SystemTimedTrajectory sys_timed_trajectory;
  auto s_samples_path = Eigen::VectorXd::LinSpaced(5, 0, 1);
  auto q_samples_path = Eigen::MatrixXd(2, 5);
  // Let's make the samples on the line
  q_samples_path.row(0) << 0, 0.1, 0.2, 0.3, 0.4;
  q_samples_path.row(1) << 0.25, 0.26, 0.27, 0.28, 0.29;
  auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_samples_path,
                                                    q_samples_path);
  // let's make the time scaling
  auto t_samples_time = Eigen::VectorXd::LinSpaced(4, 0, 2.0);
  auto s_samples_time = Eigen::MatrixXd(1, 4);
  s_samples_time.row(0) << 0, 0.3, 0.7, 1.0;
  auto time_scaling = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(t_samples_time,
                                                    s_samples_time);
  // Let's make the system timed trajectory
  sys_timed_trajectory["robot"] = planning_service_client::TimedTrajectory(
      conversions::DrakePiecewisePolynomialToClient(path),
      conversions::DrakePiecewisePolynomialToClient(time_scaling));
  double time_now = 1.0;
  // Let's make some waypoints to update the trajectory toward
  planning_service_client::SystemConf sys_conf_1, sys_conf_2, sys_conf_3;
  sys_conf_1["robot"] = Eigen::Vector2d(0.2, 0.28);
  sys_conf_2["robot"] = Eigen::Vector2d(0.31, 0.32);
  sys_conf_3["robot"] = Eigen::Vector2d(0.42, 0.35);
  std::vector<planning_service_client::SystemConf> waypoints = {
      sys_conf_1, sys_conf_2, sys_conf_3};
  def = planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
      sys_timed_trajectory, waypoints, time_now);
  // Solve the problem
  auto motion_plan_result = planner.SolvePlan(def);
  logging::log()->info("Message: {}", motion_plan_result.message());
  EXPECT_EQ(motion_plan_result.is_success(), true);
  // Let's check the trajectory
  auto new_trajectory = motion_plan_result.system_timed_trajectory();
  // Let's inspect the trajectory
  EXPECT_TRUE(new_trajectory.has_key("robot"));
  auto new_time_scaling = new_trajectory.at("robot").time_scaling();
  logging::log()->info("Start time: {}", new_time_scaling.start_time());
  logging::log()->info("End time: {}", new_time_scaling.end_time());
  logging::log()->info("time_now: {}", time_now);
  EXPECT_NEAR(new_time_scaling.start_time(), time_now, 1e-4);
  // log the message
  logging::log()->info("Message: {}", motion_plan_result.message());
}

TEST(SolveUpdateTrajTowardWaypoints, Wayposes) {
  // When wayposes are given.
  const auto planner = DracoPlanner(test::Wallflower());
  // Let's create a simple trajectory and update it with new waypoints
  planning_service_client::SystemTimedTrajectory sys_timed_trajectory;
  auto s_samples_path = Eigen::VectorXd::LinSpaced(5, 0, 1);
  auto q_samples_path = Eigen::MatrixXd(2, 5);
  // Let's make the samples on the line
  q_samples_path.row(0) << 0, 0.1, 0.2, 0.3, 0.4;
  q_samples_path.row(1) << 0.25, 0.26, 0.27, 0.28, 0.29;
  auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_samples_path,
                                                    q_samples_path);
  // let's make the time scaling
  auto t_samples_time = Eigen::VectorXd::LinSpaced(4, 0, 2.0);
  auto s_samples_time = Eigen::MatrixXd(1, 4);
  s_samples_time.row(0) << 0, 0.3, 0.7, 1.0;
  auto time_scaling = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(t_samples_time,
                                                    s_samples_time);
  // Let's make the system timed trajectory
  sys_timed_trajectory["robot"] = planning_service_client::TimedTrajectory(
      conversions::DrakePiecewisePolynomialToClient(path),
      conversions::DrakePiecewisePolynomialToClient(time_scaling));
  double time_now = 1.0;
  // Let's make some poses to update the trajectory toward.
  // First, let's get the current pose
  std::map<std::string, Eigen::VectorXd> sys_conf;
  sys_conf["robot"] = Eigen::Vector2d(0.3, 0.3);
  auto pose = planner.CalcRelativePose(sys_conf, "ball", "world");
  planning_service_client::FrameRelativePose frp(
      "world", "ball", pose.translation(), pose.rotation().ToQuaternion());
  // Let's say we want to update the trajectory toward the same pose.
  std::vector<planning_service_client::FrameRelativePose> wayposes = {frp, frp,
                                                                      frp};
  auto def = planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
      sys_timed_trajectory, wayposes, time_now);
  // Solve the problem
  auto motion_plan_result = planner.SolvePlan(def);
  logging::log()->info("Message: {}", motion_plan_result.message());
  EXPECT_EQ(motion_plan_result.is_success(), true);
  // Let's check the trajectory
  auto new_trajectory = motion_plan_result.system_timed_trajectory();
  // Let's inspect the trajectory
  EXPECT_TRUE(new_trajectory.has_key("robot"));
  auto new_time_scaling = new_trajectory.at("robot").time_scaling();
  EXPECT_NEAR(new_time_scaling.start_time(), time_now, 1e-4);
}

TEST(SolveUpdateTrajTowardWaypoints, ConstantTrajectory) {
  // From a constant trajectory.
  const auto planner = DracoPlanner(test::Wallflower());
  // Let's create a simple trajectory and update it with new waypoints
  planning_service_client::SystemConf sysconf;
  sysconf["robot"] = Eigen::Vector2d(0.3, 0.3);
  auto sys_timed_trajectory =
      planning_service_client::ConstantSystemTimedTrajectory(sysconf);
  auto sys_conf_1 = sysconf;
  auto sys_conf_2 = sysconf;
  auto sys_conf_3 = sysconf;
  sys_conf_1["robot"] = Eigen::Vector2d(0.2, 0.28);
  sys_conf_2["robot"] = Eigen::Vector2d(0.1, 0.2);
  sys_conf_3["robot"] = Eigen::Vector2d(0.0, 0.0);
  std::vector<planning_service_client::SystemConf> waypoints = {
      sys_conf_1, sys_conf_2, sys_conf_3};
  double time_now = 1.0;

  std::vector<double> segment_durations = {0.5, 0.5};
  double search_step_size = 0.01;
  auto def = planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
      sys_timed_trajectory, waypoints, time_now, segment_durations,
      search_step_size);
  // Solve the problem
  auto motion_plan_result = planner.SolvePlan(def);
  logging::log()->info("Message: {}", motion_plan_result.message());
  EXPECT_EQ(motion_plan_result.is_success(), true);
  // Let's check the trajectory
  auto new_trajectory = motion_plan_result.system_timed_trajectory();
  // Let's inspect the trajectory
  auto robot_trj = new_trajectory.at("robot");
  // Verify the start trajectory and the start velocity
  EXPECT_TRUE(robot_trj.Value(0.0).isApprox(sysconf["robot"].q()));
  EXPECT_TRUE(robot_trj.EvalDerivative(0.0).isApprox(Eigen::Vector2d::Zero()));
}

TEST(SolveUpdateTrajTowardWaypoints, DualArmWaypoints) {
  const auto planner = DracoPlanner(test::DualWallflowers());
  // Let's check the number of arms
  EXPECT_EQ(planner.robot_model().num_arms(), 2);
  // Let's construct a SystemTimedTrajectory with two arms for flower1 and
  // flower2
  planning_service_client::SystemTimedTrajectory sys_timed_trajectory;
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
  // Let's make the time scaling
  auto t_samples = Eigen::VectorXd::LinSpaced(4, 0, 1.3);
  auto s_samples_time = Eigen::MatrixXd(1, 4);
  s_samples_time.row(0) << 0.3, 0.8, 1.5, 2.2;
  auto time_scaling = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(t_samples, s_samples_time);
  // Let's make the system timed trajectory for flower1
  sys_timed_trajectory["flower1"] = planning_service_client::TimedTrajectory(
      conversions::DrakePiecewisePolynomialToClient(path),
      conversions::DrakePiecewisePolynomialToClient(time_scaling));
  // Let's now make a constant trajectory for flower2
  planning_service_client::SystemConf sysconf_flower2;
  sys_timed_trajectory["flower2"] =
      planning_service_client::TimedTrajectory::Constant(
          Eigen::Vector2d(0.3, 0.3));
  // Setup the problem with waypoints only for flower 1
  planning_service_client::SystemConf sys_conf_1, sys_conf_2, sys_conf_3,
      sys_conf_4;
  sys_conf_1["flower1"] = Eigen::Vector2d(2.0, 0.34);
  sys_conf_2["flower1"] = Eigen::Vector2d(2.1, 0.36);
  sys_conf_3["flower1"] = Eigen::Vector2d(2.3, 0.37);
  sys_conf_4["flower1"] = Eigen::Vector2d(2.5, 0.35);
  std::vector<planning_service_client::SystemConf> waypoints = {
      sys_conf_1, sys_conf_2, sys_conf_3, sys_conf_4};
  double time_now = 1.1;
  std::vector<double> segment_durations = {};  // Let the planner decide
  double search_step_size = 0.01;
  Eigen::VectorXd wiggle_room_conf(2);
  wiggle_room_conf << 0.05,
      0.02;  // Allow some wiggle room for flower1's waypoints (smoothing them)
  planning_service_client::SystemConf wiggle_room;
  wiggle_room["flower1"] = wiggle_room_conf;
  // Case 1: Without Toppra and Smoothing
  auto def_1 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, waypoints, time_now, segment_durations,
          search_step_size);
  auto motion_plan_result_1 = planner.SolvePlan(def_1);
  logging::log()->info("Message 1: {} \n\n\n", motion_plan_result_1.message());
  EXPECT_EQ(motion_plan_result_1.is_success(), true);
  double end_time_1 =
      motion_plan_result_1.system_timed_trajectory().at("flower1").duration();
  // Case 2: With Toppra but without Smoothing
  auto def_2 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, waypoints, time_now, segment_durations,
          search_step_size, true);
  auto motion_plan_result_2 = planner.SolvePlan(def_2);
  logging::log()->info("Message 2: {} \n\n\n", motion_plan_result_2.message());
  EXPECT_EQ(motion_plan_result_2.is_success(), true);
  double end_time_2 =
      motion_plan_result_2.system_timed_trajectory().at("flower1").duration();
  // Case 3: Without Toppra but without smoothing
  auto def_3 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, waypoints, time_now, segment_durations,
          search_step_size, false, wiggle_room);
  auto motion_plan_result_3 = planner.SolvePlan(def_3);
  logging::log()->info("Message 3: {}  \n\n\n", motion_plan_result_3.message());
  EXPECT_EQ(motion_plan_result_3.is_success(), true);
  double end_time_3 =
      motion_plan_result_3.system_timed_trajectory().at("flower1").duration();
  // Case 4: With Toppra and Smoothing
  auto def_4 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, waypoints, time_now, segment_durations,
          search_step_size, true, wiggle_room);
  auto motion_plan_result_4 = planner.SolvePlan(def_4);
  logging::log()->info("Message 4: {} \n\n\n", motion_plan_result_4.message());
  EXPECT_EQ(motion_plan_result_4.is_success(), true);
  double end_time_4 =
      motion_plan_result_4.system_timed_trajectory().at("flower1").duration();
  // Log all end times
  logging::log()->info("End time without Toppra or Smoothing: {}", end_time_1);
  logging::log()->info("End time with Toppra, no Smoothing: {}", end_time_2);
  logging::log()->info("End time without Toppra, with Smoothing: {}",
                       end_time_3);
  logging::log()->info("End time with Toppra and Smoothing: {}", end_time_4);
}

TEST(SolveUpdateTrajTowardWaypoints, DualArmWayposes) {
  const auto planner = DracoPlanner(test::DualWallflowers());
  // Let's check the number of arms
  EXPECT_EQ(planner.robot_model().num_arms(), 2);
  // Let's construct a SystemTimedTrajectory with two arms for flower1 and
  // flower2
  planning_service_client::SystemTimedTrajectory sys_timed_trajectory;
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
  // Let's make the time scaling
  auto t_samples = Eigen::VectorXd::LinSpaced(4, 0, 1.3);
  auto s_samples_time = Eigen::MatrixXd(1, 4);
  s_samples_time.row(0) << 0.3, 0.8, 1.5, 2.2;
  auto time_scaling = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(t_samples, s_samples_time);
  // Let's make the system timed trajectory for flower1
  sys_timed_trajectory["flower1"] = planning_service_client::TimedTrajectory(
      conversions::DrakePiecewisePolynomialToClient(path),
      conversions::DrakePiecewisePolynomialToClient(time_scaling));
  // Let's now make a constant trajectory for flower2
  planning_service_client::SystemConf sysconf_flower2;
  sys_timed_trajectory["flower2"] =
      planning_service_client::TimedTrajectory::Constant(
          Eigen::Vector2d(0.3, 0.3));
  // Setup the problem with wayposes only for flower 1
  std::vector<planning_service_client::FrameRelativePose> wayposes;
  for (int i = 0; i < 10; ++i) {
    motion::system_conf_t sysconf;
    // Some noise
    double w_x = std::sin(i * 5.0) * 0.05;
    double w_y = std::cos(i * 5.0) * 0.01;
    sysconf["flower1"] =
        Eigen::Vector2d(1.72 + i * 0.05 + w_x, 0.25 + i * 0.01 + w_y);
    sysconf["flower2"] = Eigen::Vector2d(-1.5, 0.3);
    logging::log()->info("sysconf: {}", sysconf["flower1"].transpose());
    auto pose = planner.CalcRelativePose(sysconf, "flower1::ball", "world");
    planning_service_client::FrameRelativePose frp(
        "world", "flower1::ball", pose.translation(),
        pose.rotation().ToQuaternion());
    wayposes.push_back(frp);
  }
  double time_now = 1.1;
  std::vector<double> segment_durations = {};  // Let the planner decide
  double search_step_size = 0.01;
  planning_service_client::SystemConf wiggle_room;
  wiggle_room["flower1"] = Eigen::Vector2d(0.02, 0.001);
  // Case 1: Without Toppra and Smoothing
  auto def_1 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, wayposes, time_now, segment_durations,
          search_step_size);
  auto motion_plan_result_1 = planner.SolvePlan(def_1);
  logging::log()->info("Message 1: {} \n\n\n", motion_plan_result_1.message());
  EXPECT_EQ(motion_plan_result_1.is_success(), true);
  double end_time_1 =
      motion_plan_result_1.system_timed_trajectory().at("flower1").duration();
  // Case 2: With Toppra but without Smoothing
  auto def_2 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, wayposes, time_now, segment_durations,
          search_step_size, true);
  auto motion_plan_result_2 = planner.SolvePlan(def_2);
  logging::log()->info("Message 2: {} \n\n\n", motion_plan_result_2.message());
  EXPECT_EQ(motion_plan_result_2.is_success(), true);
  double end_time_2 =
      motion_plan_result_2.system_timed_trajectory().at("flower1").duration();
  // Case 3: Without Toppra but without smoothing
  auto def_3 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, wayposes, time_now, segment_durations,
          search_step_size, false, wiggle_room);
  auto motion_plan_result_3 = planner.SolvePlan(def_3);
  logging::log()->info("Message 3: {}  \n\n\n", motion_plan_result_3.message());
  EXPECT_EQ(motion_plan_result_3.is_success(), true);
  double end_time_3 =
      motion_plan_result_3.system_timed_trajectory().at("flower1").duration();
  // Case 4: With Toppra and Smoothing
  auto def_4 =
      planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
          sys_timed_trajectory, wayposes, time_now, segment_durations,
          search_step_size, true, wiggle_room);
  auto motion_plan_result_4 = planner.SolvePlan(def_4);
  logging::log()->info("Message 4: {} \n\n\n", motion_plan_result_4.message());
  EXPECT_EQ(motion_plan_result_4.is_success(), true);
  double end_time_4 =
      motion_plan_result_4.system_timed_trajectory().at("flower1").duration();
  // Log all end times
  logging::log()->info("End time without Toppra or Smoothing: {}", end_time_1);
  logging::log()->info("End time with Toppra, no Smoothing: {}", end_time_2);
  logging::log()->info("End time without Toppra, with Smoothing: {}",
                       end_time_3);
  logging::log()->info("End time with Toppra and Smoothing: {}", end_time_4);
}

}  // namespace planner
}  // namespace draco
