#include "time_optimal_spliner.h"

#include <drake/common/trajectories/bezier_curve.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/tree/multibody_forces.h>
#include <drake/planning/dof_mask.h>
#include <drake/solvers/constraint.h>
#include <drake/solvers/cost.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

#include "planning_service/motion/splining/internal/cubic_trajectory.h"

namespace motion {
namespace splining {

using internal::CalcCubicPath;
using internal::CalcSamplesAndTangentFromNewWaypts;
using internal::CalcTrailingPathTowardWaypts;
using internal::MergeTrajectory;

namespace {
JointDynamicLimits MakePlantJointDynamicLimits(
    const drake::multibody::MultibodyPlant<double>& plant,
    const joint_dynamic_limits_map_t& joint_dynamic_limits_map,
    const HolonomicMapping& holonomic_mapping) {
  JointDynamicLimits result;
  int n_dofs = holonomic_mapping.minimal_dim();
  int n_plant = plant.num_positions();
  DRAKE_THROW_UNLESS(n_plant == holonomic_mapping.full_dim());
  Eigen::VectorXd plant_vel_bound = Eigen::VectorXd::Zero(n_plant);
  Eigen::VectorXd acc_vel_bound = Eigen::VectorXd::Zero(n_plant);
  Eigen::VectorXd acc_torque_bound = Eigen::VectorXd::Zero(n_plant);
  for (const auto& [model_name, joint_dynamic_limits] :
       joint_dynamic_limits_map) {
    if (!plant.HasModelInstanceNamed(model_name)) {
      logging::log()->debug(
          "MakePlantJointDynamicLimits: Model instance {} not found in plant",
          model_name);
      continue;
    }
    const auto model_instance = plant.GetModelInstanceByName(model_name);
    plant.SetPositionsInArray(
        model_instance,
        holonomic_mapping.LiftInstance(model_instance,
                                       joint_dynamic_limits.velocity_bound),
        &plant_vel_bound);
    plant.SetPositionsInArray(
        model_instance,
        holonomic_mapping.LiftInstance(model_instance,
                                       joint_dynamic_limits.acceleration_bound),
        &acc_vel_bound);
    plant.SetPositionsInArray(
        model_instance,
        holonomic_mapping.LiftInstance(model_instance,
                                       joint_dynamic_limits.torque_bound),
        &acc_torque_bound);
  }
  result.velocity_bound = holonomic_mapping.Reduce(plant_vel_bound);
  result.acceleration_bound = holonomic_mapping.Reduce(acc_vel_bound);
  result.torque_bound = holonomic_mapping.Reduce(acc_torque_bound);
  DRAKE_DEMAND(result.velocity_bound.size() == n_dofs);
  DRAKE_DEMAND(result.acceleration_bound.size() == n_dofs);
  DRAKE_DEMAND(result.torque_bound.size() == n_dofs);
  return result;
}

bool IsConstant(drake::trajectories::PathParameterizedTrajectory<double> traj,
                double tolerance = 1e-3) {
  // Get the value at all breaks of the path, and check if all velocities are
  // zero
  bool is_constant = true;
  const auto& path = traj.path();
  const drake::trajectories::PiecewisePolynomial<double>* pp_path =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &path);
  DRAKE_THROW_UNLESS(pp_path != nullptr);
  for (double time_break : pp_path->get_segment_times()) {
    Eigen::VectorXd velocity = traj.EvalDerivative(time_break, 1);
    if (velocity.norm() > tolerance) {
      is_constant = false;
      break;
    }
  }
  logging::log()->info("IsConstant: Trajectory is {}constant",
                       is_constant ? "" : "not ");
  return is_constant;
}

std::optional<drake::trajectories::PathParameterizedTrajectory<double>>
BuildSingleWaypointTrajectory(
    drake::trajectories::PiecewisePolynomial<double> cubic_path,
    double s_switch, double time_switch, bool time_now_after_traj,
    const drake::trajectories::PathParameterizedTrajectory<double>&
        current_traj,
    double time_now) {
  cubic_path.shiftRight(s_switch);
  auto cubic_path_timing =
      internal::MakeUniformTimingForPath(cubic_path, time_switch);
  auto cubic_traj = drake::trajectories::PathParameterizedTrajectory<double>(
      cubic_path, cubic_path_timing);
  if (time_now_after_traj) {
    return cubic_traj;
  }
  return MergeTrajectory(current_traj, time_now, cubic_traj);
}

}  // namespace

TimeOptimalSpliner::TimeOptimalSpliner(
    const RobotModel& robot_model,
    const joint_dynamic_limits_map_t& joint_dynamic_limits_map,
    const cartesian_dynamic_limits_map_t& cartesian_dynamic_limits_map,
    const TimeOptimalSplineParams& time_optimal_spline_params,
    ArmIndex arm_index)
    : robot_model_ {robot_model},
      plant_ {arm_index.is_valid() ? robot_model_.GetArm(arm_index).plant()
                                   : robot_model.plant()},
      holonomic_mapping_ {
          arm_index.is_valid()
              ? robot_model_.GetArm(arm_index).arm_holonomic_mapping()
              : robot_model.holonomic_mapping()},
      joint_dynamic_limits_ {MakePlantJointDynamicLimits(
          plant_, joint_dynamic_limits_map, holonomic_mapping_)},
      cartesian_dynamic_limits_map_ {
          ValidateCartesianDynamicLimits(cartesian_dynamic_limits_map)},
      time_optimal_spline_params_ {time_optimal_spline_params},
      plant_context_ {plant_.CreateDefaultContext()},
      arm_index_(arm_index) {}

cartesian_dynamic_limits_map_t
TimeOptimalSpliner::ValidateCartesianDynamicLimits(
    const cartesian_dynamic_limits_map_t& cartesian_dynamic_limits_map) const {
  cartesian_dynamic_limits_map_t result;
  for (const auto& [frame_name, limits] : cartesian_dynamic_limits_map) {
    const auto* frame = drake::multibody::parsing::GetScopedFrameByNameMaybe(
        plant_, frame_name);
    if (!frame) {
      logging::log()->warn(
          "TimeOptimalSpliner:AdoptCartesianDynamicLimits: Frame {} not found "
          "in plant",
          frame_name);
      continue;
    }
    result[frame_name] = limits;
  }
  return result;
}

std::optional<drake::trajectories::PiecewisePolynomial<double>>
TimeOptimalSpliner::RunToppra(
    const drake::trajectories::Trajectory<double>& traj) const {
  const auto grid_points = drake::multibody::Toppra::CalcGridPoints(
      traj, time_optimal_spline_params_.calc_grid_points_options);
  return DoRunToppra(traj, grid_points);
}

std::optional<drake::trajectories::PiecewisePolynomial<double>>
TimeOptimalSpliner::CalcDynamicsAwareSpline(
    const std::vector<Eigen::VectorXd>& waypoints,
    const std::optional<Eigen::VectorXd>& start_velocity,
    const std::optional<Eigen::VectorXd>& end_velocity,
    const std::vector<double>& segment_durations, int max_num_iterations,
    double minimum_time_scaling) const {
  logging::log()->info(
      "Calculating a dynamics-aware spline with {} iterations and minimum "
      "time scaling of {}",
      max_num_iterations, minimum_time_scaling);
  int num_waypoints = std::ssize(waypoints);
  DRAKE_THROW_UNLESS(num_waypoints >= 2);
  int num_segments = num_waypoints - 1;
  DRAKE_THROW_UNLESS(std::ssize(segment_durations) == num_segments);
  const int n = holonomic_mapping_.minimal_dim();
  for (int i = 0; i < num_waypoints; ++i) {
    DRAKE_THROW_UNLESS(waypoints[i].rows() == n);
  }
  auto joint_limits_low =
      holonomic_mapping_.Reduce(plant_.GetPositionLowerLimits());
  auto joint_limits_high =
      holonomic_mapping_.Reduce(plant_.GetPositionUpperLimits());
  auto velocity_limits = joint_dynamic_limits_.velocity_bound;
  auto acceleration_limits = joint_dynamic_limits_.acceleration_bound;
  auto prog = drake::solvers::MathematicalProgram();
  // We will need infinity and zero vectors
  Eigen::VectorXd inf_n =
      Eigen::VectorXd::Constant(n, std::numeric_limits<double>::infinity());
  Eigen::VectorXd zero_n = Eigen::VectorXd::Zero(n);
  // Add two intermediate waypoints between each pair of waypoints
  auto h = prog.NewContinuousVariables(1, "h");
  std::map<int, drake::solvers::VectorXDecisionVariable> q_1_map;
  std::map<int, drake::solvers::VectorXDecisionVariable> q_2_map;
  double total_duration =
      std::accumulate(segment_durations.begin(), segment_durations.end(), 0.0);
  std::map<std::pair<int, int>,
           drake::solvers::Binding<drake::solvers::LinearConstraint>*>
      acc_con_map;
  for (int i = 0; i < num_segments; ++i) {
    const auto& q_start = waypoints[i];
    const auto& q_end = waypoints[i + 1];
    double time_fraction = segment_durations[i] / total_duration;
    auto vel_lim = time_fraction
                   * time_optimal_spline_params_.safety_factor_velocity
                   * velocity_limits.sparseView();
    q_1_map[i] = prog.NewContinuousVariables(n, "q1_" + std::to_string(i));
    q_2_map[i] = prog.NewContinuousVariables(n, "q2_" + std::to_string(i));
    auto& q_1 = q_1_map[i];
    auto& q_2 = q_2_map[i];
    prog.AddBoundingBoxConstraint(joint_limits_low, joint_limits_high, q_1);
    prog.AddBoundingBoxConstraint(joint_limits_low, joint_limits_high, q_2);
    // Adding velocity and acceleration constraints. We take advantage of the
    // fact that the constraints are linear in the control points as
    // velocity and acceleration profiles becomes quadratic and linear Bezier
    // curves, respectively. See.
    // https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves
    // Velocity limit: 3*(q_1 - q_start) <= v_max * h
    Eigen::SparseMatrix<double> A_1(n, n + 1);
    A_1.reserve(2 * n);
    for (int j = 0; j < n; ++j) {
      A_1.insert(j, j) = 3.0;
    }
    A_1.rightCols<1>() = -vel_lim;
    prog.AddLinearConstraint(A_1, -inf_n, 3 * q_start, {q_1, h});
    // Velocity limit: 3*(q_1 - q_start) >= -v_max * time_fraction
    A_1.rightCols<1>() = vel_lim;
    prog.AddLinearConstraint(A_1, 3 * q_start, inf_n, {q_1, h});
    // Velocity limit: 3*(q_2 - q_1) <= v_max * h * time_fraction
    Eigen::SparseMatrix<double> A_2(n, 2 * n + 1);
    A_2.reserve(3 * n);
    for (int j = 0; j < n; ++j) {
      A_2.insert(j, j) = -3.0;
      A_2.insert(j, j + n) = 3.0;
    }
    A_2.rightCols<1>() = -vel_lim;
    prog.AddLinearConstraint(A_2, -inf_n, zero_n, {q_1, q_2, h});
    // Velocity limit: 3(q_2 - q_1) >= -v_max * h * time_fraction
    A_2.rightCols<1>() = vel_lim;
    prog.AddLinearConstraint(A_2, zero_n, inf_n, {q_1, q_2, h});
    // Velocity limit: 3*(q_end - q_2) <= v_max * h * time_fraction
    A_1.rightCols<1>() = vel_lim;
    prog.AddLinearConstraint(A_1, 3 * q_end, inf_n, {q_2, h});
    // Velocity limit: 3 * (q_end - q_2) >= -v_max * h * time_fraction
    A_1.rightCols<1>() = -vel_lim;
    prog.AddLinearConstraint(A_1, -inf_n, 3 * q_end, {q_2, h});
    // Control points of the Acceleration Bezier curve:
    // 6 * (q_2 - 2 * q_1 + q_start)  and 6 * (q_end - 2 * q_2 + q_1).
    // Both must lie within the acceleration limit of a_max * (time_fraction
    // *h)^2 However, we can write the h^2 because it's a quadratic constraint.
    // Instead, we approximate the h^2  > h_min * h.
    auto acc_lim = time_fraction * time_fraction
                   * (total_duration * 0.99
                      * time_optimal_spline_params_.safety_factor_acceleration)
                   * acceleration_limits.sparseView();
    // Acceleration limits: 6 (-2 * q_1 + q_2 + q_start) <= acc_coeff * h
    Eigen::SparseMatrix<double> A_3(n, 2 * n + 1);
    A_3.reserve(3 * n);
    for (int j = 0; j < n; ++j) {
      A_3.insert(j, j) = -12.0;
      A_3.insert(j, j + n) = 6.0;
    }
    A_3.rightCols<1>() = -acc_lim;
    // acc_con_map.insert({0, i}, prog.AddLinearConstraint(A_3, -inf_n, -6 *
    // q_start,
    //                                            {q_1, q_2, h}));
    acc_con_map[{0, i}] =
        new drake::solvers::Binding<drake::solvers::LinearConstraint>(
            prog.AddLinearConstraint(A_3, -inf_n, -6 * q_start, {q_1, q_2, h}));
    // prog.AddLinearConstraint(A_3, -inf_n, -6 * q_start, {q_1, q_2, h});
    // Acceleration limits: 6(-2 * q_1 + q_2 + q_start) >= -acc_coeff * h
    A_3.rightCols<1>() = acc_lim;
    // acc_con_map.insert({1, i}, prog.AddLinearConstraint(A_3, -6 * q_start,
    // inf_n,
    //                                            {q_1, q_2, h}));
    acc_con_map[{1, i}] =
        new drake::solvers::Binding<drake::solvers::LinearConstraint>(
            prog.AddLinearConstraint(A_3, -6 * q_start, inf_n, {q_1, q_2, h}));
    // Acceleration limits: 6(-2 * q_2 + q_1 + q_end) <= acc_coeff * h
    A_3.rightCols<1>() = -acc_lim;
    // acc_con_map.insert({2, i}, prog.AddLinearConstraint(A_3, -inf_n, -6 *
    // q_end,
    //                                            {q_2, q_1, h}));
    acc_con_map[{2, i}] =
        new drake::solvers::Binding<drake::solvers::LinearConstraint>(
            prog.AddLinearConstraint(A_3, -inf_n, -6 * q_end, {q_2, q_1, h}));
    // Acceleration limits: 6(-2 * q_2 + q_1 + q_end) >= -acc_coeff * h
    A_3.rightCols<1>() = acc_lim;
    // acc_con_map.insert({3, i}, prog.AddLinearConstraint(A_3, -6 * q_end,
    // inf_n,
    //                                            {q_2, q_1, h}));
    acc_con_map[{3, i}] =
        new drake::solvers::Binding<drake::solvers::LinearConstraint>(
            prog.AddLinearConstraint(A_3, -6 * q_end, inf_n, {q_2, q_1, h}));
    // Add a quadratic energy cost between q_start and q_1
    auto Q_1 = Eigen::MatrixXd::Identity(n, n);
    prog.AddQuadraticErrorCost(Q_1, q_start, q_1);
    // Add a quadratic energy cost between q_1 and q_2
    Eigen::MatrixXd Q_2 = Eigen::MatrixXd::Identity(2, 2);
    Q_2(0, 1) = -1.0;
    Q_2(1, 0) = -1.0;
    for (int j = 0; j < n; ++j) {
      prog.AddQuadraticCost(2 * Q_2, Eigen::VectorXd::Zero(2),
                            {q_1.row(j), q_2.row(j)}, true);
    }
    // Add a quadratic energy cost between q_2 and q_end
    prog.AddQuadraticErrorCost(Q_1, q_end, q_2);
  }
  // Now we need to impose speed continuity constraints.
  // (q_1-q_start)_{i+1} / h_{i+1} = (q_end - q_2)_{i} / {h_i}
  // h_i * q_1_{i+1} - h_i * q_start_{i+1} = h_{i+1} * q_end_{i} - h_{i+1} *
  // q_2_{i} Which becomes a linear constraint, and q_start_{i+1} = q_end_{i}
  // h_i * q_1_{i+1} + h_{i+1} * q_2_{i} = h_i + h_{i+1} * q_end_{i}
  for (int i = 0; i < num_segments - 1; ++i) {
    auto& q_2 = q_2_map[i];
    auto& q_1_next = q_1_map[i + 1];
    Eigen::SparseMatrix<double> A_4(n, 2 * n);
    A_4.reserve(2 * n);
    for (int j = 0; j < n; ++j) {
      A_4.insert(j, j) = segment_durations[i];
      A_4.insert(j, j + n) = segment_durations[i + 1];
    }
    Eigen::VectorXd b =
        (segment_durations[i] + segment_durations[i + 1]) * waypoints[i + 1];
    prog.AddLinearEqualityConstraint(A_4, b, {q_1_next, q_2});
  }
  // If start and end velocitys are provided, add them as constraints.
  if (start_velocity.has_value()) {
    // First, verify that the start velocity is feasible
    for (int i = 0; i < n; ++i) {
      if (std::abs(start_velocity.value()(i)) > velocity_limits(i)) {
        logging::log()->error(
            "The start velocity is {} and the velocity limits are: {}",
            start_velocity.value()(i), velocity_limits(i));
        return std::nullopt;
      }
    }
    // That means 3*(q_1- q_start) = start_velocity * h * s_0
    // Or q_1 - start_velocity * h * s_i = q_start
    auto A_5 = Eigen::SparseMatrix<double>(n, n + 1);
    A_5.reserve(2 * n);
    for (int i = 0; i < n; ++i) {
      A_5.insert(i, i) = 1.0;
    }
    A_5.rightCols<1>() = -1.0 / 3 * segment_durations[0] / total_duration
                         * start_velocity.value().sparseView();
    prog.AddLinearEqualityConstraint(A_5, waypoints[0], {q_1_map[0], h});
  }
  if (end_velocity.has_value()) {
    // That means q_end - q_2 = end_velocity * h * s_i
    // Or q_2 + end_velocity * h * s_i = q_end
    auto A_6 = Eigen::SparseMatrix<double>(n, n + 1);
    A_6.reserve(2 * n);
    for (int i = 0; i < n; ++i) {
      A_6.insert(i, i) = 1.0;
    }
    A_6.rightCols<1>() = 1.0 / 3 * segment_durations[num_segments - 1]
                         / total_duration * end_velocity.value().sparseView();
    prog.AddLinearEqualityConstraint(A_6, waypoints[num_segments],
                                     {q_2_map[num_segments - 1], h});
  }
  // The timing cost and constraints
  auto h_min_con_binding =
      prog.AddLinearConstraint(Eigen::VectorXd::Ones(1), total_duration,
                               std::numeric_limits<double>::infinity(), h);
  prog.AddLinearCost(Eigen::VectorXd::Ones(1), 0, h);
  // Solve the program
  int num_iterations = 0;
  drake::solvers::MathematicalProgramResult result;
  while (num_iterations < max_num_iterations) {
    result = drake::solvers::Solve(prog);
    if (result.is_success()) {
      logging::log()->info("iteration {} succeeded with h: {} >= h_min: {}",
                           num_iterations, result.GetSolution(h)(0),
                           h_min_con_binding.evaluator()->lower_bound());
      break;
    }
    logging::log()->info("iteration {} failed with h_min: {}", num_iterations,
                         h_min_con_binding.evaluator()->lower_bound());
    // Edit the constraints
    for (int i = 0; i < num_segments; ++i) {
      for (int j = 0; j < 4; ++j) {
        const auto& con = acc_con_map[{j, i}]->evaluator();
        auto A = con->get_sparse_A();
        A.rightCols<1>() = A.rightCols<1>() * minimum_time_scaling;
        con->UpdateCoefficients(A, con->lower_bound(), con->upper_bound());
      }
    }
    auto h_min_con = h_min_con_binding.evaluator();
    auto h_min = h_min_con->lower_bound() * minimum_time_scaling;
    h_min_con->UpdateLowerBound(h_min);
    num_iterations++;
  }
  if (!result.is_success()) {
    logging::log()->error(
        "Failed to solve the optimization problem after {} "
        "iterations. All failed.",
        num_iterations);
    return std::nullopt;
  }
  // Extract the solution as a Bezier curve
  std::vector<Eigen::VectorXd> control_points;
  for (int i = 0; i < num_segments; ++i) {
    control_points.push_back(result.GetSolution(q_1_map[i]));
    control_points.push_back(result.GetSolution(q_2_map[i]));
  }
  for (const auto& control_point : control_points) {
    logging::log()->debug("control_point: {}", control_point.transpose());
  }
  // Also log h
  logging::log()->info("h: {}", result.GetSolution(h));
  // Now extract a piecewise polynomial. The Bernstein form has to be converted
  // to the power form.
  std::vector<
      drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix>
      poly_matrix_vec;
  std::vector<double> breaks;
  double t = 0.0;
  breaks.push_back(t);
  for (int i = 0; i < num_segments; ++i) {
    double optimal_duration =
        segment_durations[i] / total_duration * result.GetSolution(h)(0);
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(n, 4);
    coefficients.col(0) = waypoints[i];
    coefficients.col(1) =
        3 * (-waypoints[i] + result.GetSolution(q_1_map[i])) / optimal_duration;
    coefficients.col(2) = 3
                          * (waypoints[i] - 2 * result.GetSolution(q_1_map[i])
                             + result.GetSolution(q_2_map[i]))
                          / (optimal_duration * optimal_duration);
    coefficients.col(3) =
        (-waypoints[i] + 3 * result.GetSolution(q_1_map[i])
         - 3 * result.GetSolution(q_2_map[i]) + waypoints[i + 1])
        / (optimal_duration * optimal_duration * optimal_duration);
    Eigen::MatrixX<drake::Polynomiald> poly_matrix(n, 1);
    for (int j = 0; j < n; ++j) {
      poly_matrix(j, 0) = drake::Polynomiald(coefficients.row(j));
    }
    t += optimal_duration;
    breaks.push_back(t);
    poly_matrix_vec.push_back(poly_matrix);
  }
  drake::trajectories::PiecewisePolynomial<double> piecewise_polynomial(
      poly_matrix_vec, breaks);
  return piecewise_polynomial;
}

drake::trajectories::PiecewisePolynomial<double>
TimeOptimalSpliner::CalcTrajectoryToStop(
    const Eigen::VectorXd& start_position,
    const Eigen::VectorXd& start_velocity) const {
  // Let's find the minimum time that we can break.
  double t_min = 0.0;
  int n = holonomic_mapping_.minimal_dim();
  for (int i = 0; i < n; ++i) {
    t_min = std::max(t_min, std::abs(start_velocity(i))
                                / joint_dynamic_limits_.acceleration_bound(i));
  }
  logging::log()->info(
      "TimeOptimalSpliner:CalcTrajectoryToStop: Minimum time to stop: {}",
      t_min);
  // Now let's get the breaking maximum acceleration vector.
  Eigen::VectorXd breaking_acceleration = Eigen::VectorXd::Zero(n);
  for (int i = 0; i < n; ++i) {
    breaking_acceleration(i) = -start_velocity(i) / t_min;
  }
  // The trajectory is a quadratic polynomial q = q_0 + v_0 * t + 0.5 * a * t^2
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(n, 3);
  coefficients.col(0) = start_position;
  coefficients.col(1) = start_velocity;
  coefficients.col(2) = 0.5 * breaking_acceleration;
  Eigen::MatrixX<drake::Polynomiald> poly_matrix(n, 1);
  for (int j = 0; j < n; ++j) {
    poly_matrix(j, 0) = drake::Polynomiald(coefficients.row(j));
  }
  std::vector<Eigen::MatrixX<drake::Polynomiald>> poly_matrix_vec {poly_matrix};
  std::vector<double> breaks = {0.0, t_min};
  drake::trajectories::PiecewisePolynomial<double> piecewise_polynomial(
      poly_matrix_vec, breaks);
  return piecewise_polynomial;
}

std::optional<drake::trajectories::PiecewisePolynomial<double>>
TimeOptimalSpliner::CalcOptimalCubicPath(
    const Eigen::VectorXd& start_position,
    const Eigen::VectorXd& start_velocity, const Eigen::VectorXd& end_position,
    const Eigen::VectorXd& end_velocity) const {
  auto minimum_h_opt = internal::CalcCubicTrajectoryMinimumTime(
      start_position, start_velocity, end_position, end_velocity,
      joint_dynamic_limits_.velocity_bound
          * time_optimal_spline_params_.safety_factor_velocity,
      joint_dynamic_limits_.acceleration_bound
          * time_optimal_spline_params_.safety_factor_acceleration);
  if (!minimum_h_opt.has_value()) {
    logging::log()->error(
        "TimeOptimalSpliner::CalcOptimalCubicPath: Failed to find minimum time "
        "cubic trajectory");
    return std::nullopt;
  }
  double h = minimum_h_opt.value();
  auto q1 = start_position + start_velocity * h / 3;
  auto q2 = end_position - end_velocity * h / 3;
  // Check if q1 and q2 are within the joint limits
  const auto q_upper =
      holonomic_mapping_.Reduce(plant().GetPositionUpperLimits());
  const auto q_lower =
      holonomic_mapping_.Reduce(plant().GetPositionLowerLimits());
  if ((q1.array() > q_upper.array()).any()
      || (q1.array() < q_lower.array()).any()) {
    logging::log()->info(
        "TimeOptimalSpliner::CalcOptimalCubicPath: q1 is out of bounds");
    return std::nullopt;
  }
  if ((q2.array() > q_upper.array()).any()
      || (q2.array() < q_lower.array()).any()) {
    logging::log()->info(
        "TimeOptimalSpliner::CalcOptimalCubicPath: q2 is out of bounds");
    return std::nullopt;
  }
  // Note: we will do a final CheckSatisfied check before running on robot, but
  // checking the cheap joint limits here is a good idea.
  return internal::CubicTrajectoryFromControlPoints(start_position, q1, q2,
                                                    end_position, h);
}

std::optional<drake::trajectories::PiecewisePolynomial<double>>
TimeOptimalSpliner::RunToppraOnPiecewiseTrajectory(
    const drake::trajectories::PiecewiseTrajectory<double>& piecewise_traj,
    double s_dot_start, double s_dot_end) const {
  const Eigen::VectorXd grid_points = drake::multibody::Toppra::CalcGridPoints(
      piecewise_traj, time_optimal_spline_params_.calc_grid_points_options);
  const std::vector<double> segment_times = piecewise_traj.get_segment_times();
  for (int i {0}; i < std::ssize(segment_times); ++i) {
    logging::log()->debug("RunToppraOnPiecewiseTrajectory: Segment {}/{}: {}",
                          i, piecewise_traj.get_number_of_segments(),
                          segment_times[i]);
  }
  // mix the grid points with the segment times
  std::vector<double> new_grid_points_vec {segment_times};
  for (int i {0}; i < grid_points.size(); ++i) {
    new_grid_points_vec.push_back(grid_points(i));
  }
  // sort the grid points
  std::sort(new_grid_points_vec.begin(), new_grid_points_vec.end());
  // remove duplicates, keeping the earlier (smaller) element so that
  // time_start is never dropped; use a while-loop to avoid size_t underflow
  {
    size_t i = 0;
    while (i + 1 < new_grid_points_vec.size()) {
      if (std::abs(new_grid_points_vec[i] - new_grid_points_vec[i + 1])
          < 1e-9) {
        new_grid_points_vec.erase(new_grid_points_vec.begin() + i + 1);
      } else {
        ++i;
      }
    }
  }
  // convert to Eigen::VectorXd
  Eigen::VectorXd new_grid_points {
      static_cast<int>(new_grid_points_vec.size())};
  for (size_t i {0}; i < new_grid_points_vec.size(); ++i) {
    new_grid_points(i) = new_grid_points_vec[i];
  }
  logging::log()->trace("Old Grid points: {}", grid_points.transpose());
  logging::log()->trace("New Grid points: {}", new_grid_points.transpose());
  return DoRunToppra(piecewise_traj, new_grid_points, s_dot_start, s_dot_end,
                     true);
}

drake::trajectories::PiecewisePolynomial<double>
TimeOptimalSpliner::ConvertToPoly(
    const drake::trajectories::Trajectory<double>& traj,
    const drake::trajectories::PiecewisePolynomial<double> path_time) const {
  DRAKE_DEMAND(path_time.rows() == 1);
  DRAKE_DEMAND(path_time.cols() == 1);
  logging::log()->debug(
      "TimeOptimalSpliner:ConvertToPoly: Converting q(s) and s(t) to q(t)");
  auto time_now = std::chrono::system_clock::now();
  system_poly_t system_poly;
  const auto break_times = path_time.get_segment_times();
  std::vector<Eigen::MatrixXd> q_vec;
  std::vector<Eigen::MatrixXd> q_dot_vec;
  // Get derivatives ahead of time
  const auto traj_prime = traj.MakeDerivative(1);
  const auto s_dot = path_time.MakeDerivative(1);
  for (double t : break_times) {
    const double s = path_time.value(t)(0, 0);
    const Eigen::VectorXd q = traj.value(s);
    const double sdot = s_dot->value(t)(0, 0);
    const Eigen::VectorXd q_dot = traj_prime->value(s) * sdot;
    q_vec.push_back(q);
    q_dot_vec.push_back(q_dot);
  }
  const auto& poly {
      drake::trajectories::PiecewisePolynomial<double>::CubicHermite(
          break_times, q_vec, q_dot_vec)};
  logging::log()->debug(
      "TimeOptimalSpliner:ConvertToPoly: Computed polynomial with duration "
      "{:.3f} s in {} ms",
      poly.end_time() - poly.start_time(),
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now() - time_now)
          .count());
  return poly;
}

system_poly_t TimeOptimalSpliner::ConvertToSystemPolynomial(
    const drake::trajectories::Trajectory<double>& traj,
    const drake::trajectories::PiecewisePolynomial<double> path_time) const {
  DRAKE_DEMAND(path_time.rows() == 1);
  DRAKE_DEMAND(path_time.cols() == 1);
  const auto full_poly {ConvertToPoly(traj, path_time)};
  logging::log()->debug(
      "TimeOptimalSpliner:ConvertToSystemPolynomial: Converting to system "
      "poly");
  return SlicePerEntities(full_poly);
}

system_poly_t TimeOptimalSpliner::SlicePerEntities(
    const drake::trajectories::PiecewisePolynomial<double>& poly) const {
  system_poly_t system_poly;
  for (int i {0}; i < plant().num_model_instances(); ++i) {
    const auto model_instance {drake::multibody::ModelInstanceIndex(i)};
    if (plant().num_positions(model_instance) > 0) {
      const auto start {
          RobotModel::GetModelStartIndex(plant(), model_instance)};
      system_poly[plant().GetModelInstanceName(model_instance)] =
          poly.Block(start, 0, plant().num_positions(model_instance), 1);
    }
  }
  return system_poly;
}

bool TimeOptimalSpliner::CheckSatisfied(
    const drake::trajectories::Trajectory<double>& traj,
    const drake::trajectories::PiecewisePolynomial<double>
        time_parameterization) const {
  DRAKE_DEMAND(time_parameterization.rows() == 1);
  DRAKE_DEMAND(time_parameterization.cols() == 1);
  const auto traj_prime = traj.MakeDerivative(1);
  const auto traj_second = traj.MakeDerivative(2);
  const auto s_dot = time_parameterization.MakeDerivative(1);
  const auto s_second = time_parameterization.MakeDerivative(2);
  const auto velocity_bound = joint_dynamic_limits_.velocity_bound;
  const auto acceleration_bound = joint_dynamic_limits_.acceleration_bound;
  const auto torque_bound = joint_dynamic_limits_.torque_bound;
  std::vector<double> check_times;
  double dt = 0.005;
  for (double t = 0; t < traj.end_time(); t += dt) {
    check_times.push_back(t);
  }
  for (double t : check_times) {
    const double s = time_parameterization.value(t)(0, 0);
    const Eigen::VectorXd q = traj.value(s);
    const Eigen::VectorXd q_prime = traj_prime->value(s);
    const Eigen::VectorXd q_second = traj_second->value(s);
    const auto v = q_prime * s_dot->value(t)(0, 0);
    const auto a = q_second * s_dot->value(t)(0, 0) * s_dot->value(t)(0, 0)
                   + q_prime * s_second->value(t)(0, 0);
    const auto torque = CalcTorque(q, v, a);
    // print the velocity, acceleration, and torque
    logging::log()->debug("Velocity at time {}: {}, \nq_prime = {}", t,
                          v.transpose(), q_prime.transpose());
    logging::log()->debug("Acceleration at time {}: {}, \n q_second={} \n", t,
                          a.transpose(), q_second.transpose());
    logging::log()->debug("Torque at time {}: {}", t, torque.transpose());
    // check velcoity
    const double alpha {1.05};
    for (int i {0}; i < velocity_bound.size(); ++i) {
      if (std::abs(v(i)) > alpha * velocity_bound(i)) {
        logging::log()->error(
            "Velocity bound violated at time {} for joint {}: |{}| > {}", t, i,
            v(i), velocity_bound(i));
        return false;
      }
    }
    // check acceleration
    for (int i {0}; i < acceleration_bound.size(); ++i) {
      if (std::abs(a(i)) > alpha * acceleration_bound(i)) {
        logging::log()->error(
            "Acceleration bound violated at time {} for joint {}: |{}| > {}", t,
            i, a(i), acceleration_bound(i));
        return false;
      }
    }
    // check torque
    for (int i {0}; i < torque_bound.size(); ++i) {
      if (std::abs(torque(i)) > alpha * torque_bound(i)) {
        logging::log()->error(
            "Torque bound violated at time {} for joint {}: |{}| > {}", t, i,
            torque(i), torque_bound(i));
        return false;
      }
    }
    // compute power
    double power = 0;
    for (int i {0}; i < torque_bound.size(); ++i) {
      power += std::abs(torque(i)) * std::abs(v(i));
    }
    // logging::log()->info("Power at time {}: {} watts", t, power);
  }
  return true;
}

bool TimeOptimalSpliner::CheckSatisfied(
    const drake::trajectories::Trajectory<double>& traj) const {
  // we treat the trajectory as q(t)
  // make s(t) as linear between 0 and end_time
  std::vector<double> breaks {0, traj.end_time()};
  std::vector<Eigen::MatrixXd> s_vec;
  s_vec.push_back(Eigen::MatrixXd::Zero(1, 1));
  s_vec.push_back(Eigen::MatrixXd::Ones(1, 1) * traj.end_time());
  const auto time_parameterization {
      drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(breaks,
                                                                       s_vec)};
  return CheckSatisfied(traj, time_parameterization);
}

std::optional<drake::trajectories::PathParameterizedTrajectory<double>>
TimeOptimalSpliner::CalcTrajTowardNewWaypoints(
    drake::trajectories::PathParameterizedTrajectory<double> traj,
    double time_now, double delta_switch,
    const std::vector<Eigen::VectorXd>& waypoints,
    const std::vector<double>& segment_durations,
    double merge_point_search_step_size, int max_num_iterations,
    double minimum_time_scale, bool run_toppra,
    const std::optional<Eigen::VectorXd>& wiggle_room,
    bool constrain_end_velocity_to_zero) const {
  auto current_traj {traj};  // copy as we may edit.
  // Get the path parameterized trajectory and the current time.
  auto& path_base = current_traj.path();
  auto* path_ptr =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &path_base);
  DRAKE_THROW_UNLESS(path_ptr != nullptr);
  auto& ts_base = current_traj.time_scaling();
  auto* time_scaling_ptr =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &ts_base);
  DRAKE_THROW_UNLESS(time_scaling_ptr != nullptr);
  auto path = *path_ptr;                  // copy as we may edit.
  auto time_scaling = *time_scaling_ptr;  // copy as we may edit.
  // Let's find the trailing path towards the waypoints.
  // double s_now = time_parameterization.value(time_now)(0, 0);
  double time_switch = time_now + delta_switch;
  double s_switch = time_scaling.value(time_switch)(0, 0);
  logging::log()->info(
      "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Finding trailing path "
      "towards waypoints from time_now: {}, time_switch: {}, s_switch: {}"
      " start/end of current trajectory: {}/{}",
      time_now, time_switch, s_switch, traj.start_time(), traj.end_time());
  // If the time_switch is after the end of the current trajectory,
  // we flag it as such and clamp the time_switch to the end of the trajectory.
  bool time_switch_after_traj = time_switch > traj.end_time();
  bool time_now_after_traj = time_now > traj.end_time();
  logging::log()->info(
      "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: time_now_after_traj: {}, "
      "time_switch_after_traj: {}",
      time_now_after_traj, time_switch_after_traj);
  // If time_switch is after the end of the trajectory, we can just append the
  // trajectory by constant.
  if (time_switch_after_traj && !time_now_after_traj) {
    logging::log()->info(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: time_switch is after "
        "traj "
        "end time but time_now is not. Returning a trajectory that holds the "
        "last configuration. path.start_time: {}, path.end_time: {}, "
        "time_scaling.start_time: {}"
        "time_scaling.end_time: {}",
        path.start_time(), path.end_time(), time_scaling.start_time(),
        time_scaling.end_time());
    auto last_conf = current_traj.value(current_traj.end_time());
    s_switch +=
        1.0;  // add a small buffer to ensure it's after the end of the path.
    path.AppendFirstOrderSegment(s_switch, last_conf);
    drake::Vector1d time_scaling_end(s_switch);
    DRAKE_DEMAND(time_switch > time_scaling.end_time());
    time_scaling.AppendFirstOrderSegment(time_switch, time_scaling_end);
    current_traj = drake::trajectories::PathParameterizedTrajectory<double>(
        path, time_scaling);
    logging::log()->info(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Updated current "
        "trajectory "
        "to hold the last configuration after the end time. New traj start/end "
        "time: {}/{}",
        current_traj.start_time(), current_traj.end_time());
  }
  const auto conf_start = current_traj.value(time_switch);
  if (waypoints.size() == 1) {
    // This case is special.
    auto cubic_path_opt = CalcOptimalCubicPath(
        conf_start, current_traj.EvalDerivative(time_switch, 1), waypoints[0],
        Eigen::VectorXd::Zero(conf_start.size()));
    if (!cubic_path_opt.has_value()) {
      logging::log()->error(
          "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Failed to find an "
          "optimal cubic path to the single waypoint.");
      return std::nullopt;
    }
    return BuildSingleWaypointTrajectory(cubic_path_opt.value(), s_switch,
                                         time_switch, time_now_after_traj,
                                         current_traj, time_now);
  }
  std::vector<double> waypoints_intervals;
  const auto velocity_limits = joint_dynamic_limits_.velocity_bound;
  if (!segment_durations.size()) {
    logging::log()->info(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: No waypoints intervals "
        "provided. Using estimates based on velocity limits.");
    for (int i = 0; i < std::ssize(waypoints) - 1; ++i) {
      auto delta_waypoint = waypoints[i + 1] - waypoints[i];
      double segment_duration = 0.01;  // TODO: Make this a parameter
      // Account for the velocity limits
      for (int j = 0; j < delta_waypoint.rows(); ++j) {
        segment_duration =
            std::max(segment_duration,
                     1.0 * std::abs(delta_waypoint(j) / velocity_limits(j)));
      }
      waypoints_intervals.push_back(segment_duration);
    }
  } else {
    waypoints_intervals = segment_durations;
  }
  auto waypoints_maybe_smoothed = waypoints;
  // If wiggle rooms are available, let's try smoothing
  if (wiggle_room.has_value()) {
    DRAKE_THROW_UNLESS(wiggle_room.value().size() == waypoints[0].size());
    // Compute sample times from waypoints_intervals
    std::vector<double> sample_times;
    sample_times.reserve(waypoints_intervals.size() + 1);
    double t = 0;
    sample_times.push_back(t);
    for (const auto& waypoints_interval : waypoints_intervals) {
      t += waypoints_interval;
      sample_times.push_back(t);
    }
    waypoints_maybe_smoothed = internal::SmoothWaypoints(
        sample_times, waypoints, std::nullopt, wiggle_room.value());
    if (!segment_durations.size()) {
      // Let's update waypoints_intervals
      logging::log()->info(
          "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: re-calculating "
          "waypoints_intervals based on smoothed waypoints.");
      waypoints_intervals.clear();
      for (int i = 0; i < std::ssize(waypoints_maybe_smoothed) - 1; ++i) {
        auto delta_waypoint =
            waypoints_maybe_smoothed[i + 1] - waypoints_maybe_smoothed[i];
        double segment_duration = 0.01;  // TODO: Make this a parameter
        // Account for the velocity limits
        for (int j = 0; j < delta_waypoint.rows(); ++j) {
          segment_duration =
              std::max(segment_duration,
                       1.0 * std::abs(delta_waypoint(j) / velocity_limits(j)));
        }
        waypoints_intervals.push_back(segment_duration);
      }
    }
  }
  logging::log()->info(
      "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Waypoints intervals: {}",
      Eigen::VectorXd::Map(waypoints_intervals.data(),
                           waypoints_intervals.size())
          .transpose());
  std::optional<Eigen::VectorXd> end_velocity;
  if (constrain_end_velocity_to_zero) {
    end_velocity = Eigen::VectorXd::Zero(conf_start.size());
  }
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
      waypoints_path_opt;
  if (IsConstant(current_traj) && segment_durations.size() == 0) {
    // We can do a simple cubic spline
    // Vector of times from 0 to N-1 as doubles
    std::vector<double> times = {};
    for (int i = 0; i < std::ssize(waypoints_maybe_smoothed); ++i) {
      times.push_back(static_cast<double>(i));
    }
    std::vector<Eigen::MatrixXd> waypoints_m = {};
    for (const auto& waypoint : waypoints_maybe_smoothed) {
      waypoints_m.push_back(waypoint);
    }
    auto start_deriv = waypoints_m[1] - waypoints_m[0];
    auto end_deriv = waypoints_m.back() - waypoints_m[waypoints_m.size() - 2];
    waypoints_path_opt = drake::trajectories::PiecewisePolynomial<
        double>::CubicWithContinuousSecondDerivatives(times, waypoints_m,
                                                      start_deriv, end_deriv);
    logging::log()->info(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Using simple cubic "
        "spline for waypoints path.");
  } else {
    waypoints_path_opt = CalcDynamicsAwareSpline(
        waypoints_maybe_smoothed, std::nullopt, end_velocity,
        waypoints_intervals, max_num_iterations, minimum_time_scale);
  }
  if (!waypoints_path_opt.has_value()) {
    logging::log()->error(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Failed to optimize a "
        "boundary-free path for the waypoints path.");
    return std::nullopt;
  }
  auto& waypoints_path = waypoints_path_opt.value();
  waypoints_path.shiftRight(s_switch);
  std::vector<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                        drake::trajectories::PiecewisePolynomial<double>>>
      q_s_t_pairs;
  // Now need to see where we can land on the waypoints_path.
  double best_cost = std::numeric_limits<double>::infinity();
  auto velocity_start = current_traj.EvalDerivative(time_switch, 1);
  std::optional<double> best_s_opt = std::nullopt;
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
      best_cubic_path = std::nullopt;
  if (!time_now_after_traj) {
    logging::log()->info(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Searching for the best "
        "landing point on the waypoints path from conf_start: {}, "
        "velocity_start: {}",
        conf_start.transpose(), velocity_start.transpose());
    for (double s = waypoints_path.start_time(); s < waypoints_path.end_time();
         s += merge_point_search_step_size) {
      auto conf_target = waypoints_path.value(s);
      auto velocity_target = waypoints_path.EvalDerivative(s, 1);
      // Now we optimize a path from the start to the target
      auto cubic_path_opt = CalcOptimalCubicPath(conf_start, velocity_start,
                                                 conf_target, velocity_target);
      if (!cubic_path_opt.has_value()) {
        logging::log()->debug(
            "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Failed to optimize "
            "a "
            "cubic_path_opt from the start to the target at s = {}",
            s);
        continue;
      }
      auto cubic_path = cubic_path_opt.value();
      // Get the duration of the path.
      double duration = cubic_path.end_time() - cubic_path.start_time();
      // Cost has two components: time spent on transition, and time lost on the
      // waypoints path.
      double cost = duration + s;
      logging::log()->debug(
          "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: at s = {}, landing "
          "duration: {}, cost = {}",
          s, duration, cost);
      if (cost < best_cost) {
        best_cost = cost;
        best_cubic_path = cubic_path;
        best_s_opt = s;
      }
    }
    if (!best_s_opt.has_value()) {
      logging::log()->error(
          "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Failed to find a "
          "path "
          "to the waypoints path.");
      return std::nullopt;
    }
  } else {
    // If the time_switch is within the trajectory, we can directly connect to
    // the waypoints path without worrying about the time lost on the
    // waypoints path, since we will be joining at time_switch.
    best_s_opt = waypoints_path.start_time();
  }
  DRAKE_DEMAND(best_s_opt.has_value());
  double best_s = best_s_opt.value();
  logging::log()->info(
      "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Best path found at s = "
      "{}. The value at the best s is {}",
      best_s, waypoints_path.value(best_s).transpose());
  // Part 1: cubic path
  if (time_now_after_traj) {
    // Making a fake best_cubic_opt that just holds the end configuration.
    logging::log()->info(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: time_switch is after "
        "the end of the trajectory. Skipping search and connecting directly "
        "to the waypoints path.");
    Eigen::MatrixXd conf_end = waypoints_path.value(best_s);
    double time_eps = 1e-3;
    std::vector<double> breaks {0, time_eps};
    std::vector<Eigen::MatrixXd> q_vec {conf_end, conf_end};
    best_cubic_path =
        drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(breaks,
                                                                        q_vec);
  }
  DRAKE_DEMAND(best_cubic_path.has_value());
  auto cubic_path = best_cubic_path.value();
  cubic_path.shiftRight(s_switch);
  auto cubic_path_timing = motion::splining::internal::MakeUniformTimingForPath(
      cubic_path, time_switch);
  q_s_t_pairs.push_back(std::make_pair(cubic_path, cubic_path_timing));
  // Log the start and end time
  // Part 2: waypoints
  auto waypoints_sliced_path =
      time_now_after_traj
          ? waypoints_path
          : waypoints_path.SliceByTime(best_s, waypoints_path.end_time());
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
      waypoints_timing = std::nullopt;
  if (run_toppra) {
    waypoints_timing =
        RunToppraOnPiecewiseTrajectory(waypoints_sliced_path, 1, 1);
  } else {
    waypoints_timing = motion::splining::internal::MakeUniformTimingForPath(
        waypoints_sliced_path);
  }
  q_s_t_pairs.push_back(
      std::make_pair(waypoints_sliced_path, waypoints_timing.value()));
  if (!constrain_end_velocity_to_zero) {
    // Part 3: trajectory to stop
    const double end_time = waypoints_sliced_path.end_time();
    const auto end_position = waypoints_sliced_path.value(end_time);
    const auto end_velocity = waypoints_sliced_path.EvalDerivative(end_time, 1);
    // Only add a stop trajectory if the final velocity is meaningfully
    // non-zero. When the start velocity is ~0, CalcTrajectoryToStop may divide
    // by zero, producing NaNs; we avoid that by skipping the stop segment.
    constexpr double kVelocityEpsilon = 1e-6;
    if (end_velocity.norm() > kVelocityEpsilon) {
      auto traj_to_stop = CalcTrajectoryToStop(end_position, end_velocity);
      auto traj_to_stop_timing =
          motion::splining::internal::MakeUniformTimingForPath(traj_to_stop);
      q_s_t_pairs.push_back(std::make_pair(traj_to_stop, traj_to_stop_timing));
    }
  }
  // Now we combine the sequential system timed trajectories.
  auto [q_s, s_t] =
      motion::splining::internal::CombineSequentialSystemTimedTrajectories(
          q_s_t_pairs);
  auto trailing_traj =
      drake::trajectories::PathParameterizedTrajectory<double>(q_s, s_t);
  // Now let's merge the to traj.
  // Let's log the start and end time of the trailing traj, and the time_switch.
  logging::log()->info(
      "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: Trailing traj start "
      "time: {}, "
      "end time: {}, time_now: {}",
      trailing_traj.start_time(), trailing_traj.end_time(), time_now);
  if (time_now_after_traj) {
    // If the time_now is after the end of the trajectory, we can just return
    // the trailing traj, since it will be joined at time_switch.
    logging::log()->info(
        "TimeOptimalSpliner:CalcTrajTowardNewWaypoints: time_switch is after "
        "the end of the trajectory. Returning trailing traj directly.");
    // Need to return trailing traj as a PathParameterizedTrajectory with the
    // correct path and time scaling.
    return trailing_traj;
  }
  return MergeTrajectory(current_traj, time_now, trailing_traj);
}

std::map<std::string,
         std::pair<drake::trajectories::PiecewisePolynomial<double>,
                   drake::trajectories::PiecewisePolynomial<double>>>
TimeOptimalSpliner::SlicePerEntities(
    drake::trajectories::PathParameterizedTrajectory<double> traj) const {
  const auto& path = traj.path();
  const auto& time_parameterization = traj.time_scaling();
  // Cast both as PiecewisePolynomial
  const auto& path_ppoly =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>&>(
          path);
  const auto& time_ppoly =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>&>(
          time_parameterization);
  auto sys_path = SlicePerEntities(path_ppoly);
  std::map<std::string,
           std::pair<drake::trajectories::PiecewisePolynomial<double>,
                     drake::trajectories::PiecewisePolynomial<double>>>
      result;
  for (const auto& [model_name, path] : sys_path) {
    result.insert({model_name, {path, time_ppoly}});
  }
  return result;
}

namespace {
bool DoBreaksMatch(drake::trajectories::PiecewisePolynomial<double> poly,
                   std::vector<double> breaks, double eps = 1e-6) {
  auto poly_breaks = poly.get_segment_times();
  if (poly_breaks.size() != breaks.size()) {
    return false;
  }
  for (int i = 0; i < std::ssize(breaks); ++i) {
    if (std::abs(poly_breaks[i] - breaks[i]) > eps) {
      return false;
    }
  }
  return true;
}
}  // namespace

drake::trajectories::PathParameterizedTrajectory<double>
TimeOptimalSpliner::ConvertToPathParameterizedTrajectory(
    const std::map<std::string,
                   std::pair<drake::trajectories::PiecewisePolynomial<double>,
                             drake::trajectories::PiecewisePolynomial<double>>>&
        sys_traj) const {
  // If not all models are present, log a warning.
  for (int i = 0; i < plant().num_model_instances(); ++i) {
    auto model_idx = drake::multibody::ModelInstanceIndex(i);
    if (plant().num_positions(model_idx) == 0) {
      continue;
    }
    if (sys_traj.find(plant().GetModelInstanceName(model_idx))
        == sys_traj.end()) {
      logging::log()->warn(
          "ConvertToPathParameterizedTrajectory: Model {} not present in the "
          "path. Correspondoing path will be set to zero values at all times "
          "and time scaling will be the same as the active model",
          plant().GetModelInstanceName(model_idx));
    }
  }
  // This is the reverse of SlicePerEntities.
  // First check if they all have the same time and path breaks
  std::vector<double> time_breaks;
  std::vector<double> path_breaks;
  for (const auto& [model_name, path_time_pair] : sys_traj) {
    // if the arm plant does not have the model, we skip it
    if (arm_index_.is_valid() && !plant().HasModelInstanceNamed(model_name)) {
      continue;
    }
    const auto& [path, time_scaling] = path_time_pair;
    if (time_breaks.empty()) {
      time_breaks = time_scaling.get_segment_times();
    } else {
      DRAKE_THROW_UNLESS(DoBreaksMatch(time_scaling, time_breaks));
    }
    if (path_breaks.empty()) {
      path_breaks = path.get_segment_times();
    } else {
      DRAKE_THROW_UNLESS(DoBreaksMatch(path, path_breaks));
    }
  }
  std::vector<
      drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix>
      poly_matrix_vec;
  for (int i = 0; i < std::ssize(path_breaks) - 1; ++i) {
    drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix
        poly_matrix(plant().num_positions(), 1);
    poly_matrix_vec.push_back(poly_matrix);
  }
  for (const auto& [model_name, path_time_pair] : sys_traj) {
    // if the arm plant does not have the model, we skip it
    if (arm_index_.is_valid() && !plant().HasModelInstanceNamed(model_name)) {
      continue;
    }
    const auto& path = path_time_pair.first;
    auto model_idx = plant().GetModelInstanceByName(model_name);
    int start_position = RobotModel::GetModelStartIndex(plant(), model_idx);
    for (int i = 0; i < std::ssize(path_breaks) - 1; ++i) {
      auto path_matrix = path.getPolynomialMatrix(i);
      poly_matrix_vec[i].block(start_position, 0,
                               plant().num_positions(model_idx), 1) =
          path.getPolynomialMatrix(i);
    }
  }
  drake::trajectories::PiecewisePolynomial<double> path_ppoly(poly_matrix_vec,
                                                              path_breaks);
  // time scaling is the same for all the models
  return drake::trajectories::PathParameterizedTrajectory<double>(
      path_ppoly, sys_traj.begin()->second.second);
}

Eigen::VectorXd TimeOptimalSpliner::CalcTorque(
    const Eigen::Ref<const Eigen::VectorXd>& q,
    const Eigen::Ref<const Eigen::VectorXd>& v,
    const Eigen::Ref<const Eigen::VectorXd>& a) const {
  plant().SetPositions(plant_context_.get(), holonomic_mapping_.Lift(q));
  plant().SetVelocities(plant_context_.get(), holonomic_mapping_.Lift(v));
  drake::multibody::MultibodyForces<double> forces(plant());
  return plant().CalcInverseDynamics(*plant_context_,
                                     holonomic_mapping_.Lift(a), forces);
}

std::optional<drake::trajectories::PiecewisePolynomial<double>>
TimeOptimalSpliner::DoRunToppra(
    const drake::trajectories::Trajectory<double>& traj,
    const Eigen::VectorXd& grid_points, double s_dot_start, double s_dot_end,
    bool acceleration_knots_interpolation) const {
  // time it and log it
  auto start_time {std::chrono::high_resolution_clock::now()};
  auto toppra = std::make_unique<drake::multibody::Toppra>(
      traj, plant(), grid_points, holonomic_mapping_);
  // short hand for the safety factors
  const double c_vel {time_optimal_spline_params_.safety_factor_velocity};
  const double c_acc {time_optimal_spline_params_.safety_factor_acceleration};
  const double c_torque {time_optimal_spline_params_.safety_factor_torque};
  toppra->AddJointVelocityLimit(-c_vel * joint_dynamic_limits_.velocity_bound,
                                c_vel * joint_dynamic_limits_.velocity_bound);
  auto toppra_discretization =
      acceleration_knots_interpolation
          ? drake::multibody::ToppraDiscretization::kInterpolation
          : drake::multibody::ToppraDiscretization::kCollocation;
  toppra->AddJointAccelerationLimit(
      -c_acc * joint_dynamic_limits_.acceleration_bound,
      c_acc * joint_dynamic_limits_.acceleration_bound, toppra_discretization);
  toppra->AddJointTorqueLimit(
      c_torque * -joint_dynamic_limits_.torque_bound,
      c_torque * joint_dynamic_limits_.torque_bound,
      drake::multibody::ToppraDiscretization::kCollocation);
  // Add cartesian dynamic limits for each frame in the map
  for (const auto& [frame_name, cartesian_dynamic_limits] :
       cartesian_dynamic_limits_map_) {
    const auto* frame = drake::multibody::parsing::GetScopedFrameByNameMaybe(
        plant(), frame_name);
    if (!frame) {
      logging::log()->warn("TimeOptimalSpliner:DoRunToppra: Frame {} not found",
                           frame_name);
      continue;
    }
    if (cartesian_dynamic_limits.speed_limit.has_value()) {
      const auto& speed_limit {cartesian_dynamic_limits.speed_limit.value()};
      toppra->AddFrameTranslationalSpeedLimit(*frame, c_vel * speed_limit);
    }
    if (cartesian_dynamic_limits.acc_lower.has_value()
        && cartesian_dynamic_limits.acc_upper.has_value()) {
      const auto& acc_lower {cartesian_dynamic_limits.acc_lower.value()};
      const auto& acc_upper {cartesian_dynamic_limits.acc_upper.value()};
      toppra->AddFrameAccelerationLimit(*frame, c_acc * acc_lower,
                                        c_acc * acc_upper);
    }
  }
  // how much time was on the construction of the toppra object
  auto construction_time {std::chrono::high_resolution_clock::now()};
  auto optimization_start_time = std::chrono::high_resolution_clock::now();
  const auto time_parameterization_opt {
      toppra->SolvePathParameterization(s_dot_start, s_dot_end)};
  logging::log()->info(
      "TimeOptimalSpliner:DoRunToppra: \n Number of grid points: {} \n"
      "Time to construct Toppra: {} ms\n "
      "Time to solve Toppra: {} ms",
      grid_points.size(),
      std::chrono::duration_cast<std::chrono::milliseconds>(construction_time
                                                            - start_time)
          .count(),
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::high_resolution_clock::now() - optimization_start_time)
          .count());
  if (!time_parameterization_opt.has_value()) {
    logging::log()->error(
        "TimeOptimalSpliner:DoRunToppra: Failed to solve path "
        "parameterization");
    return std::nullopt;
  }
  const auto& time_parameterization {time_parameterization_opt.value()};
  logging::log()->info(
      "TimeOptimalSpliner:DoRunToppra: Path parameterization found with "
      "duration: {}[s]",
      time_parameterization.end_time() - time_parameterization.start_time());
  return time_parameterization;
}

}  // namespace splining
}  // namespace motion
