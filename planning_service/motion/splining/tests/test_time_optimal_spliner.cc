/*
 * Copyright © 2025 Sony Research. All rights reserved.
 */

#include <gtest/gtest.h>

#include "planning_service/motion/splining/time_optimal_spliner.h"

namespace motion {
namespace splining {

// ToDo(@sadra): organize these into test utilities

using RobotAndTimeOptimalSpliner =
    std::pair<std::unique_ptr<RobotModel>, std::unique_ptr<TimeOptimalSpliner>>;

namespace {

RobotAndTimeOptimalSpliner MakeRobotModelAndSpliner(
    const std::string& xml_file, const std::string& dmd_file,
    const std::string& dynamic_limits_file,
    const std::string& time_optimal_spline_params_file,
    const ArmIndex arm_index = ArmIndex()) {
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  const auto joint_dynamic_limits_map =
      drake::yaml::LoadYamlFile<joint_dynamic_limits_map_t>(dynamic_limits_file,
                                                            "joint_limits");
  const auto cartesian_dynamic_limits_map =
      drake::yaml::LoadYamlFile<cartesian_dynamic_limits_map_t>(
          dynamic_limits_file, "cartesian_limits");
  const auto time_optimal_spline_params =
      drake::yaml::LoadYamlFile<TimeOptimalSplineParams>(
          time_optimal_spline_params_file);
  return std::make_pair(
      std::move(robot_model),
      std::make_unique<TimeOptimalSpliner>(
          *robot_model, joint_dynamic_limits_map, cartesian_dynamic_limits_map,
          time_optimal_spline_params, arm_index));
}

RobotAndTimeOptimalSpliner MakeAlfredRobotModelAndSpliner() {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/alfred/sp.dmd.yaml"};
  const std::string dynamic_limits_file {
      "planning_service/test_data/dynamic_limits.yaml"};
  const std::string time_optimal_spline_params_file {
      "planning_service/test_data/time_optimal_spline_params.yaml"};
  return MakeRobotModelAndSpliner(xml_file, dmd_file, dynamic_limits_file,
                                  time_optimal_spline_params_file);
}

RobotAndTimeOptimalSpliner MakeDualWallflowerRobotModelAndSpliner(
    ArmIndex arm_index = ArmIndex()) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_wallflowers/dmd.yaml"};
  const std::string dynamic_limits_file {
      "planning_service/test_data/dual_wallflowers/dynamic_limits.yaml"};
  const std::string time_optimal_spline_params_file {
      "planning_service/test_data/time_optimal_spline_params.yaml"};
  return MakeRobotModelAndSpliner(xml_file, dmd_file, dynamic_limits_file,
                                  time_optimal_spline_params_file, arm_index);
}

RobotAndTimeOptimalSpliner Make2DPrismaticRobotModelAndSpliner() {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
  const std::string dynamic_limits_file {
      "planning_service/test_data/2d_prismatic_robot/dynamic_limits.yaml"};
  const std::string time_optimal_spline_params_file {
      "planning_service/test_data/time_optimal_spline_params.yaml"};
  return MakeRobotModelAndSpliner(xml_file, dmd_file, dynamic_limits_file,
                                  time_optimal_spline_params_file);
}

RobotAndTimeOptimalSpliner MakeFrankaWithGripperRobotModelAndSpliner() {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const std::string dynamic_limits_file {
      "planning_service/test_data/franka_with_gripper/dynamic_limits.yaml"};
  const std::string time_optimal_spline_params_file {
      "planning_service/test_data/time_optimal_spline_params.yaml"};
  return MakeRobotModelAndSpliner(xml_file, dmd_file, dynamic_limits_file,
                                  time_optimal_spline_params_file);
}

drake::trajectories::CompositeTrajectory<double> MakeTrajectoryFromAdapter(
    const CompositeBezierTrajectoryAdapter& trajectory_adapter) {
  std::vector<traj_ptr_t> trajectory_vec {};
  for (size_t i {0}; i < trajectory_adapter.start_times_vec.size(); ++i) {
    const auto control_points = trajectory_adapter.control_points_vec[i];
    const auto start_time = trajectory_adapter.start_times_vec[i];
    const auto end_time = trajectory_adapter.end_times_vec[i];
    const auto bezier_curve = drake::trajectories::BezierCurve<double>(
        start_time, end_time, control_points);
    trajectory_vec.push_back(static_cast<traj_ptr_t>(bezier_curve.Clone()));
  }
  return drake::trajectories::CompositeTrajectory<double>(trajectory_vec);
}

void CheckTrajectoryC1Continuity(
    const drake::trajectories::PathParameterizedTrajectory<double>& traj,
    double dt = 1e-8, double tol = 1e-4) {
  const auto* path =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &traj.path());
  DRAKE_THROW_UNLESS(path != nullptr);
  const auto* time_scaling =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &traj.time_scaling());
  for (double t : time_scaling->get_segment_times()) {
    Eigen::VectorXd left_derivative = traj.EvalDerivative(t - dt, 1);
    Eigen::VectorXd right_derivative = traj.EvalDerivative(t + dt, 1);
    EXPECT_TRUE((left_derivative - right_derivative).norm() < tol)
        << "Trajectory is not C1 continuous at t = " << t
        << ". Left derivative: " << left_derivative.transpose()
        << ", Right derivative: " << right_derivative.transpose();
  }
}

}  // namespace

TEST(TimeOptimalSpliner, CalcDynamicsAwareSpline1) {
  // Test the first overload, with timings specified.
  auto [robot_model, time_optimal_spliner] =
      Make2DPrismaticRobotModelAndSpliner();
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(Eigen::Vector2d(0.0, 0.0));
  waypoints.push_back(Eigen::Vector2d(1.0, 0.1));
  waypoints.push_back(Eigen::Vector2d(2.0, 0.2));
  waypoints.push_back(Eigen::Vector2d(3.0, -0.8));
  Eigen::VectorXd start_tangent = Eigen::Vector2d(0.98, -2.95);
  Eigen::VectorXd end_tangent = Eigen::Vector2d(0.98, 1.0);
  std::vector<double> segment_durations {1.0, 1.0, 1.0};
  const auto spline_opt = time_optimal_spliner->CalcDynamicsAwareSpline(
      waypoints, start_tangent, end_tangent, segment_durations);
  ASSERT_TRUE(spline_opt.has_value());
  const auto& spline = spline_opt.value();
  EXPECT_NEAR(spline.start_time(), 0.0, 1e-6);
  EXPECT_TRUE(spline.value(0.0).isApprox(waypoints[0], 1e-6));
  EXPECT_TRUE(spline.value(spline.end_time()).isApprox(waypoints.back(), 1e-6));
  EXPECT_TRUE(spline.EvalDerivative(0.0, 1).isApprox(start_tangent, 1e-6));
  EXPECT_TRUE(
      spline.EvalDerivative(spline.end_time(), 1).isApprox(end_tangent, 1e-6));
  EXPECT_TRUE(time_optimal_spliner->CheckSatisfied(spline));
}

TEST(TimeOptimalSpliner, CalcDynamicsAwareSpline2) {
  // Test when segment durations are not provided
  auto [robot_model, time_optimal_spliner] =
      Make2DPrismaticRobotModelAndSpliner();
  std::vector<Eigen::VectorXd> waypoints;
  waypoints.push_back(Eigen::Vector2d(0.0, 0.0));
  waypoints.push_back(Eigen::Vector2d(1.0, 0.3));
  waypoints.push_back(Eigen::Vector2d(2.0, 1.5));
  waypoints.push_back(Eigen::Vector2d(2.0, 3.2));
  waypoints.push_back(Eigen::Vector2d(3.0, 2.2));
  Eigen::VectorXd start_tangent = Eigen::Vector2d(0.98, 1.0);
  Eigen::VectorXd end_tangent = Eigen::Vector2d(0.98, -1.0);
  std::vector<double> segment_durations {1.0, 1.0, 1.0, 1.0};
  const auto spline_opt = time_optimal_spliner->CalcDynamicsAwareSpline(
      waypoints, start_tangent, end_tangent, segment_durations);
  ASSERT_TRUE(spline_opt.has_value());
  const auto& spline = spline_opt.value();
  EXPECT_NEAR(spline.start_time(), 0.0, 1e-6);
  EXPECT_TRUE(spline.value(0.0).isApprox(waypoints[0], 1e-6));
  EXPECT_TRUE(spline.value(spline.end_time()).isApprox(waypoints.back(), 1e-6));
  EXPECT_TRUE(spline.EvalDerivative(0.0, 1).isApprox(start_tangent, 1e-6));
  EXPECT_TRUE(
      spline.EvalDerivative(spline.end_time(), 1).isApprox(end_tangent, 1e-6));
  EXPECT_TRUE(time_optimal_spliner->CheckSatisfied(spline));
  // Test left and right derivatives being equal.
  double eps = 1e-5;
  for (double s : spline.get_segment_times()) {
    EXPECT_TRUE((spline.value(s - eps) - spline.value(s + eps)).norm() < 1e-3);
    EXPECT_TRUE(spline.EvalDerivative(s - eps, 1)
                    .isApprox(spline.EvalDerivative(s + eps, 1), 1e-3));
  }
}

TEST(TimeOptimalSpliner, CalcTrajectoryToStop) {
  // Test when segment durations are not provided
  auto [robot_model, time_optimal_spliner] =
      Make2DPrismaticRobotModelAndSpliner();
  Eigen::VectorXd start_position = Eigen::Vector2d(1.0, 0.0);
  Eigen::VectorXd start_velocity = Eigen::Vector2d(-2.0, 3.0);
  auto stop_trajectory = time_optimal_spliner->CalcTrajectoryToStop(
      start_position, start_velocity);
  EXPECT_NEAR(stop_trajectory.start_time(), 0.0, 1e-6);
  EXPECT_TRUE(stop_trajectory.value(0.0).isApprox(start_position, 1e-6));
  EXPECT_TRUE(
      stop_trajectory.EvalDerivative(0.0, 1).isApprox(start_velocity, 1e-6));
  double end_time = stop_trajectory.end_time();
  // Larger of 2.0/3.0 = 0.6666666666666666 and 3.0/4.0 = 0.75
  EXPECT_NEAR(end_time, 0.75, 1e-6);
  // stop trajectory should be at rest at the end
  EXPECT_TRUE(stop_trajectory.EvalDerivative(end_time, 1)
                  .isApprox(Eigen::Vector2d::Zero(), 1e-6));
  // Stop trajectory has only one segment
  EXPECT_EQ(stop_trajectory.get_number_of_segments(), 1);
}

TEST(TimeOptimalSpliner, RunToppraOnPiecewiseTrajectory) {
  auto [robot_model, time_optimal_spliner] = MakeAlfredRobotModelAndSpliner();
  logging::log()->info("Loading Trajectory");
  const auto trajectory_file {
      "planning_service/test_data/optimized_trajectory.yaml"};
  auto trajectory_adapter =
      drake::yaml::LoadYamlFile<CompositeBezierTrajectoryAdapter>(
          trajectory_file);
  const auto trajectory = MakeTrajectoryFromAdapter(trajectory_adapter);
  // Check the trajectory values
  std::vector<double> check_times;
  for (double s : trajectory.get_segment_times()) {
    if (s == 0 || s == trajectory.end_time()) {
      check_times.push_back(s);
      continue;
    }
    const double dt = 0.01;
    check_times.push_back(s - dt);
    check_times.push_back(s);
    check_times.push_back(s + dt);
  }
  for (double s : check_times) {
    logging::log()->debug("\n segment s: {}", s);
    // show q
    logging::log()->debug("q: {}", trajectory.value(s).transpose());
    const auto q_prime = trajectory.EvalDerivative(s, 1);
    logging::log()->debug("q_prime: {}", q_prime.transpose());
    const auto q_double_prime = trajectory.EvalDerivative(s, 2);
    logging::log()->debug("q_double_prime: {}", q_double_prime.transpose());
  }
  // now make time_optimal_spliner run toppra.
  // We use the piecewise trajectory version because it incorporates the
  // breakpoints of the trajectory as the grid points for Toppra.
  const auto time_parameterization_opt =
      time_optimal_spliner->RunToppraOnPiecewiseTrajectory(trajectory);
  EXPECT_TRUE(time_parameterization_opt.has_value());
  const auto& time_parameterization {time_parameterization_opt.value()};
  EXPECT_EQ(time_parameterization.rows(), 1);
  // show the breaks
  for (const auto break_time : time_parameterization.get_segment_times()) {
    logging::log()->debug("break time: {}, s ={}, sdot = {}, sddot = {}",
                          break_time, time_parameterization.value(break_time),
                          time_parameterization.EvalDerivative(break_time, 1),
                          time_parameterization.EvalDerivative(break_time, 2));
  }
  // run check satisfied
  EXPECT_TRUE(
      time_optimal_spliner->CheckSatisfied(trajectory, time_parameterization));
  // run check satisfied without time parameterization
  const auto trajectory_poly =
      time_optimal_spliner->ConvertToPoly(trajectory, time_parameterization);
  EXPECT_TRUE(time_optimal_spliner->CheckSatisfied(trajectory_poly));
  // now convert the trajectory to a system_poly_t object
  const auto system_poly = time_optimal_spliner->ConvertToSystemPolynomial(
      trajectory, time_parameterization);
}

TEST(TimeOptimalSpliner, C1FailsToppra) {
  auto [robot_model, time_optimal_spliner] = MakeAlfredRobotModelAndSpliner();
  const auto trajectory_file {
      "planning_service/test_data/optimized_trajectory.yaml"};
  const auto trajectory =
      CompositeBezierTrajectoryAdapter::LoadYamlFile(trajectory_file);
  const auto time_parameterization_opt_without_pieces =
      time_optimal_spliner->RunToppra(trajectory);
  const auto time_parameterization_opt_with_pieces =
      time_optimal_spliner->RunToppraOnPiecewiseTrajectory(trajectory);
  EXPECT_TRUE(time_parameterization_opt_without_pieces.has_value());
  EXPECT_TRUE(time_parameterization_opt_with_pieces.has_value());
  const auto& time_parameterization_with_pieces {
      time_parameterization_opt_with_pieces.value()};
  const auto& time_parameterization_without_pieces {
      time_parameterization_opt_without_pieces.value()};
  // run check satisfied
  EXPECT_TRUE(time_optimal_spliner->CheckSatisfied(
      trajectory, time_parameterization_with_pieces));
  EXPECT_FALSE(time_optimal_spliner->CheckSatisfied(
      trajectory, time_parameterization_without_pieces));
}

TEST(TimeOptimalSpliner, CalcTrajTowardNewWaypoints) {
  auto [robot_model, time_optimal_spliner] =
      Make2DPrismaticRobotModelAndSpliner();
  // Let's make a fake path
  std::vector<double> times {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> samples;
  samples.push_back(Eigen::Vector2d(0.0, 0.0));
  samples.push_back(Eigen::Vector2d(1.0, 1.0));
  samples.push_back(Eigen::Vector2d(2.1, 0.0));
  const auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, samples);
  const auto time_parameterization_opt_without_pieces =
      time_optimal_spliner->RunToppra(path);
  auto traj = drake::trajectories::PathParameterizedTrajectory<double>(
      path, time_parameterization_opt_without_pieces.value());
  // Let's find the trailing path towards the waypoints.
  std::vector<Eigen::VectorXd> waypoints;
  double time_now = 1.5;
  double delta_switch = 0.1;
  waypoints.push_back(traj.value(time_now + 0.2) + Eigen::Vector2d(0.00, 0.02));
  waypoints.push_back(traj.value(time_now + 0.4) + Eigen::Vector2d(0.05, 0.05));
  waypoints.push_back(traj.value(time_now + 0.6) + Eigen::Vector2d(0.00, 0.1));
  waypoints.push_back(traj.value(time_now + 0.8) + Eigen::Vector2d(0.05, 0.17));
  waypoints.push_back(traj.value(time_now + 1.0) + Eigen::Vector2d(0.00, 0.3));
  waypoints.push_back(traj.value(time_now + 1.2) + Eigen::Vector2d(0.05, 0.38));
  auto new_traj = time_optimal_spliner->CalcTrajTowardNewWaypoints(
      traj, time_now, delta_switch, waypoints, {}, 0.01);
  EXPECT_TRUE(new_traj.has_value());
  // Todo: Remove this when the tests are done.
  // ------------------------- Start of commented section
  // ------------------------- std::string q_s_file
  // {"planning_service/test_data/traj_q_s.yaml"}; std::string q_s_new_file
  // {"planning_service/test_data/traj_q_s_new.yaml"}; std::string s_t_file
  // {"planning_service/test_data/traj_s_t.yaml"}; std::string s_t_new_file
  // {"planning_service/test_data/traj_s_t_new.yaml"};
  // drake::yaml::SaveYamlFile(q_s_file, *dynamic_cast<const
  // drake::trajectories::PiecewisePolynomial<double>*>(&traj.path()));
  // drake::yaml::SaveYamlFile(q_s_new_file, *dynamic_cast<const
  // drake::trajectories::PiecewisePolynomial<double>*>(&new_traj.value().path()));
  // drake::yaml::SaveYamlFile(s_t_file, *dynamic_cast<const
  // drake::trajectories::PiecewisePolynomial<double>*>(&traj.time_scaling()));
  // drake::yaml::SaveYamlFile(s_t_new_file, *dynamic_cast<const
  // drake::trajectories::PiecewisePolynomial<double>*>(&new_traj.value().time_scaling()));
  // ------------------------- End of commented section
  // -------------------------
}

TEST(TimeOptimalSpliner, CalcTrajTowardNewWaypoints2) {
  // Use the new overload of CalcTrajTowardNewWaypoints, with no parameters
  auto [robot_model, time_optimal_spliner] =
      Make2DPrismaticRobotModelAndSpliner();
  // Let's make a fake path
  std::vector<double> times {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> samples;
  samples.push_back(Eigen::Vector2d(0.0, 0.0));
  samples.push_back(Eigen::Vector2d(1.0, 1.0));
  samples.push_back(Eigen::Vector2d(2.1, 0.0));
  const auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, samples);
  const auto time_parameterization_opt_without_pieces =
      time_optimal_spliner->RunToppra(path);
  auto traj = drake::trajectories::PathParameterizedTrajectory<double>(
      path, time_parameterization_opt_without_pieces.value());
  // Let's find the trailing path towards the waypoints.
  std::vector<Eigen::VectorXd> waypoints;
  double time_now = 1.5;
  waypoints.push_back(traj.value(time_now + 0.2) + Eigen::Vector2d(0.00, 0.02));
  waypoints.push_back(traj.value(time_now + 0.4) + Eigen::Vector2d(0.05, 0.05));
  waypoints.push_back(traj.value(time_now + 0.6) + Eigen::Vector2d(-0.2, 0.1));
  waypoints.push_back(traj.value(time_now + 0.8) + Eigen::Vector2d(0.05, 0.17));
  waypoints.push_back(traj.value(time_now + 1.0) + Eigen::Vector2d(-0.3, 0.3));
  waypoints.push_back(traj.value(time_now + 1.2) + Eigen::Vector2d(0.05, 0.38));
  double delta_switch = 0.1;
  auto new_traj_opt = time_optimal_spliner->CalcTrajTowardNewWaypoints(
      traj, time_now, delta_switch, waypoints, {}, 0.01);
  EXPECT_TRUE(new_traj_opt.has_value());
  // Check if the new traj ends at zero velocity
  const auto& new_traj = new_traj_opt.value();
  auto end_velocity = new_traj.EvalDerivative(new_traj.end_time(), 1);
  EXPECT_LE(end_velocity.norm(), 1e-6);
  // Let's inspect the time parameterization and the path
  for (double t = new_traj.start_time(); t <= time_now + delta_switch;
       t += 0.02) {
    EXPECT_TRUE(new_traj.value(t).isApprox(traj.value(t), 1e-6));
  }
  CheckTrajectoryC1Continuity(new_traj);
  // Test the case delta_switch is large and switch time falls after the end of
  // the trajectory.
  delta_switch = 2.0;
  new_traj_opt = time_optimal_spliner->CalcTrajTowardNewWaypoints(
      traj, time_now, delta_switch, waypoints, {}, 0.01);
  EXPECT_TRUE(new_traj_opt.has_value());
  const auto& new_traj2 = new_traj_opt.value();
  // The new trajectory should be a constant between traj.end_time() and
  // time_switch.
  double time_switch = time_now + delta_switch;
  EXPECT_TRUE(new_traj2.value(time_switch)
                  .isApprox(new_traj2.value(traj.end_time()), 1e-6));
  EXPECT_TRUE(new_traj2.value(new_traj2.start_time())
                  .isApprox(traj.value(time_now), 1e-6));
  EXPECT_TRUE(
      new_traj2.value(new_traj2.end_time()).isApprox(waypoints.back(), 1e-6));
  CheckTrajectoryC1Continuity(new_traj2);
}

TEST(TimeOptimalSpliner, CalcTrajTowardNewWaypoints_single_waypoint) {
  // Single waypoint
  auto [robot_model, time_optimal_spliner] =
      Make2DPrismaticRobotModelAndSpliner();
  // Let's make a fake path
  std::vector<double> times {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> samples;
  samples.push_back(Eigen::Vector2d(0.0, 0.0));
  samples.push_back(Eigen::Vector2d(1.0, 1.0));
  samples.push_back(Eigen::Vector2d(2.1, 0.0));
  const auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, samples);
  const auto time_parameterization_opt_without_pieces =
      time_optimal_spliner->RunToppra(path);
  auto traj = drake::trajectories::PathParameterizedTrajectory<double>(
      path, time_parameterization_opt_without_pieces.value());
  // Let's find the trailing path towards the waypoints.
  std::vector<Eigen::VectorXd> waypoints;
  double time_now = 2.0;
  waypoints.push_back(traj.value(time_now + 0.2) + Eigen::Vector2d(0.00, 0.02));
  double delta_switch = 0.1;
  auto new_traj_opt = time_optimal_spliner->CalcTrajTowardNewWaypoints(
      traj, time_now, delta_switch, waypoints, {}, 0.01);
  EXPECT_TRUE(new_traj_opt.has_value());
  // Check if the new traj ends at zero velocity
  const auto& new_traj = new_traj_opt.value();
  auto end_velocity = new_traj.EvalDerivative(new_traj.end_time(), 1);
  EXPECT_LE(end_velocity.norm(), 1e-6);
  // Let's inspect the time parameterization and the path
  for (double t = new_traj.start_time(); t <= time_now + delta_switch;
       t += 0.02) {
    EXPECT_TRUE(new_traj.value(t).isApprox(traj.value(t), 1e-6));
  }
  // expect traj to arrive at the single waypoint at the end of the trajectory
  EXPECT_TRUE(new_traj.value(new_traj.end_time()).isApprox(waypoints[0], 1e-6));
  CheckTrajectoryC1Continuity(new_traj);
  // Another test: when delta_switch is large and switch time falls after the
  // end of the trajectory.
  delta_switch = 1.0;
  new_traj_opt = time_optimal_spliner->CalcTrajTowardNewWaypoints(
      traj, time_now, delta_switch, waypoints, {}, 0.01);
  EXPECT_TRUE(new_traj_opt.has_value());
  const auto& new_traj2 = new_traj_opt.value();
  // It should start at time time_now.
  EXPECT_TRUE(new_traj2.value(new_traj2.start_time())
                  .isApprox(traj.value(time_now), 1e-6));
  // Log the start value and end value of the new traj for debugging.
  logging::log()->info("new_traj2 start value: {}",
                       new_traj2.value(new_traj2.start_time()).transpose());
  logging::log()->info("new_traj2 end value: {}",
                       new_traj2.value(new_traj2.end_time()).transpose());
  logging::log()->info("waypoint: {}", waypoints[0].transpose());
  logging::log()->info("Old traj end value: {}",
                       traj.value(traj.end_time()).transpose());
  EXPECT_TRUE(
      new_traj2.value(new_traj2.end_time()).isApprox(waypoints[0], 1e-6));
  CheckTrajectoryC1Continuity(new_traj2);
}

TEST(TimeOptimalSpliner, ConvertToPathParameterizedTrajectory1) {
  // The case when both models have a trajectory
  auto [robot_model, time_optimal_spliner] =
      MakeDualWallflowerRobotModelAndSpliner();
  // Let's make a toy path
  std::vector<double> s_values {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> q_values;
  q_values.push_back(Eigen::Vector2d(-1.0, 0.0));
  q_values.push_back(Eigen::Vector2d(1.5, 1.0));
  q_values.push_back(Eigen::Vector2d(3.5, 1.5));
  const auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_values, q_values);
  // Let's make a time parameterization
  std::vector<double> times {0.0, 0.5, 1.0};
  std::vector<Eigen::MatrixXd> s_values_scaling;
  s_values_scaling.push_back(drake::Vector1d(0.0));
  s_values_scaling.push_back(drake::Vector1d(1.3));
  s_values_scaling.push_back(drake::Vector1d(2.0));
  const auto time_parameterization = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, s_values_scaling);
  std::map<std::string,
           std::pair<drake::trajectories::PiecewisePolynomial<double>,
                     drake::trajectories::PiecewisePolynomial<double>>>
      sys_traj;
  auto path2 = path + Eigen::Vector2d {5.0, 3.0};
  sys_traj["flower1"] = std::make_pair(path, time_parameterization);
  sys_traj["flower2"] = std::make_pair(path2, time_parameterization);
  // Now convert to path parameterized trajectory
  auto ppt =
      time_optimal_spliner->ConvertToPathParameterizedTrajectory(sys_traj);
  // Let's now inspect the path parameterized trajectory
  EXPECT_EQ(ppt.rows(), 4);
  EXPECT_EQ(ppt.cols(), 1);
  EXPECT_TRUE(ppt.value(0.0).isApprox(Eigen::Vector4d(-1.0, 0.0, 4.0, 3.0)));
  EXPECT_TRUE(ppt.value(1.0).isApprox(Eigen::Vector4d(3.5, 1.5, 8.5, 4.5)));
}

TEST(TimeOptimalSpliner, ConvertToPathParameterizedTrajectory2) {
  // The case when only one model is present is the map
  auto [robot_model, time_optimal_spliner] =
      MakeDualWallflowerRobotModelAndSpliner();
  // Let's make a toy path
  std::vector<double> s_values {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> q_values;
  q_values.push_back(Eigen::Vector2d(-1.0, 0.0));
  q_values.push_back(Eigen::Vector2d(1.5, 1.0));
  q_values.push_back(Eigen::Vector2d(3.5, 1.5));
  const auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_values, q_values);
  // Let's make a time parameterization
  std::vector<double> times {0.0, 0.5, 1.0};
  std::vector<Eigen::MatrixXd> s_values_scaling;
  s_values_scaling.push_back(drake::Vector1d(0.0));
  s_values_scaling.push_back(drake::Vector1d(1.3));
  s_values_scaling.push_back(drake::Vector1d(2.0));
  const auto time_parameterization = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, s_values_scaling);
  std::map<std::string,
           std::pair<drake::trajectories::PiecewisePolynomial<double>,
                     drake::trajectories::PiecewisePolynomial<double>>>
      sys_traj;
  sys_traj["flower2"] = std::make_pair(path, time_parameterization);
  // Now convert to path parameterized trajectory
  auto ppt =
      time_optimal_spliner->ConvertToPathParameterizedTrajectory(sys_traj);
  EXPECT_EQ(ppt.rows(), 4);
  EXPECT_EQ(ppt.cols(), 1);
  // The absent robot would be zeros.
  EXPECT_TRUE(ppt.value(0.0).isApprox(Eigen::Vector4d(0.0, 0.0, -1.0, 0.0)));
  EXPECT_TRUE(ppt.value(1.0).isApprox(Eigen::Vector4d(0.0, 0.0, 3.5, 1.5)));
}

TEST(TimeOptimalSpliner, OneArmSpliner) {
  // Let's build the spliner only for one arm.
  auto [robot_model, time_optimal_spliner] =
      MakeDualWallflowerRobotModelAndSpliner(ArmIndex(0));
  EXPECT_EQ(time_optimal_spliner->arm_index(), ArmIndex(0));
  // The spliner plan has 2 dimensions, whereas the robot model has 4.
  EXPECT_EQ(time_optimal_spliner->plant().num_positions(), 2);
  EXPECT_EQ(robot_model->plant().num_positions(), 4);
  // We can spline in 2 dimensions.
  Eigen::Vector2d start_position {0.0, 0.3};
  Eigen::Vector2d start_velocity {0.98, 0.0};
  Eigen::Vector2d end_position {1.0, 0.3};
  Eigen::Vector2d end_velocity {0.0, 0.0};
  std::vector<Eigen::VectorXd> waypoints {start_position, end_position};
  auto spline_opt = time_optimal_spliner->CalcDynamicsAwareSpline(
      waypoints, start_velocity, end_velocity, {1.0});
  ASSERT_TRUE(spline_opt.has_value());
  const auto& spline = spline_opt.value();
  EXPECT_EQ(spline.rows(), 2);
  EXPECT_NEAR(spline.start_time(), 0.0, 1e-6);
  EXPECT_TRUE(spline.value(0.0).isApprox(start_position, 1e-6));
  EXPECT_TRUE(spline.value(spline.end_time()).isApprox(end_position, 1e-6));
  EXPECT_TRUE(spline.EvalDerivative(0.0, 1).isApprox(start_velocity, 1e-6));
  EXPECT_LE((spline.EvalDerivative(spline.end_time(), 1) - end_velocity).norm(),
            1e-6);
}

TEST(TimeOptimalSpliner, FrankaWithGripperToppra) {
  auto [robot_model, time_optimal_spliner] =
      MakeFrankaWithGripperRobotModelAndSpliner();
  EXPECT_EQ(robot_model->plant().num_positions(), 13);
  // Check time_optimal_spliner
  EXPECT_EQ(time_optimal_spliner->holonomic_mapping().minimal_dim(), 8);
  // Test Toppra on a simple path
  std::vector<double> s_values {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> q_values;
  Eigen::VectorXd q0(8), q1(8), q2(8);
  q0 << 0.0, -0.5, 0.0, -2.2, 0.0, 1.0, 0.0, 0.0;
  q1 << 0.1, -0.4, 0.2, -2.0, 0.1, 1.5, 0.1, 0.5;
  q2 << 0.2, -0.3, 0.4, -1.8, 0.2, 1.2, 0.6, 0.7;
  q_values.push_back(q0);
  q_values.push_back(q1);
  q_values.push_back(q2);
  const auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_values, q_values);
  // Let's solve a time parameterization problem
  const auto time_parameterization_opt =
      time_optimal_spliner->RunToppraOnPiecewiseTrajectory(path);
  EXPECT_TRUE(time_parameterization_opt.has_value());
}

TEST(TimeOptimalSpliner, FrankaWithGripperCalcDynamicsAwareSpline) {
  auto [robot_model, time_optimal_spliner] =
      MakeFrankaWithGripperRobotModelAndSpliner();
  std::vector<Eigen::VectorXd> waypoints;
  Eigen::VectorXd wp0(8), wp1(8), wp2(8);
  wp0 << 0.0, -0.5, 0.0, -2.2, 0.0, 1.0, 0.0, 0.0;
  wp1 << 0.1, -0.4, 0.2, -2.0, 0.1, 1.5, 0.1, 0.5;
  wp2 << 0.2, -0.3, 0.4, -1.8, 0.2, 1.2, 0.6, 0.7;
  waypoints.push_back(wp0);
  waypoints.push_back(wp1);
  waypoints.push_back(wp2);
  Eigen::VectorXd start_tangent(8), end_tangent(8);
  start_tangent << 0.98, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5;
  end_tangent << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -0.2;
  std::vector<double> segment_durations {1.0, 1.0};
  auto spline_opt = time_optimal_spliner->CalcDynamicsAwareSpline(
      waypoints, start_tangent, end_tangent, segment_durations);
  EXPECT_TRUE(spline_opt.has_value());
  const auto& spline = spline_opt.value();
  EXPECT_EQ(spline.rows(), 8);
  EXPECT_NEAR(spline.start_time(), 0.0, 1e-6);
  EXPECT_TRUE(spline.value(0.0).isApprox(waypoints[0], 1e-6));
  EXPECT_TRUE(spline.value(spline.end_time()).isApprox(waypoints.back(), 1e-6));
  EXPECT_TRUE(spline.EvalDerivative(0.0, 1).isApprox(start_tangent, 1e-6));
  EXPECT_TRUE(
      spline.EvalDerivative(spline.end_time(), 1).isApprox(end_tangent, 1e-6));
  EXPECT_TRUE(time_optimal_spliner->CheckSatisfied(spline));
}

}  // namespace splining
}  // namespace motion
