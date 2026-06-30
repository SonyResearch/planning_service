#include "draco_planner.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

SolvePlanResult DracoPlanner::SolveMaxCartesianAccelerationProblem(
    const planning_service_client::planner::MaxCartesianAcceleration& problem,
    const planning_service_client::SystemConf& start_sysconf) const {
  const auto q_0 =
      conversions::ToGeneralizedPosition(robot_model(), start_sysconf);
  motion::CheckSatisfiedOptions check_options;
  check_options.verbose = true;
  if (!robot_constraints().CheckSatisfied(q_0, 0, check_options)) {
    auto msg = "Start configuration is invalid.";
    logging::log()->error(
        "DracoPlanner:SolveMaxCartesianAccelerationProblem {}", msg);
    AddToVisualizer(q_0, "Invalid start conf");
    return std::unexpected(msg);
  }
  // Let's find the references and targets for the max cartesian acceleration
  // problem
  const auto& frame_A =
      robot_model().GetScopedFrameByName(problem.twist().frame_A());
  const auto& frame_B =
      robot_model().GetScopedFrameByName(problem.twist().frame_B());
  const auto& frame_E =
      robot_model().GetScopedFrameByName(problem.twist().frame_E());
  Eigen::VectorXd direction = Eigen::VectorXd::Zero(6);
  direction.head(3) = problem.twist().delta_rpy();
  direction.tail(3) = problem.twist().delta_xyz();
  Eigen::MatrixXd J(6, robot_model().plant().num_positions());
  auto& plant_context = robot_model().plant().GetMyMutableContextFromRoot(
      robot_model().calc_pose_context_ptr());
  robot_model().plant().SetPositions(&plant_context, q_0);
  robot_model().plant().CalcJacobianSpatialVelocity(
      plant_context, drake::multibody::JacobianWrtVariable::kQDot, frame_B,
      Eigen::Vector3d::Zero(), frame_A, frame_E, &J);
  // Now we have J. Find the maximum acceleration in the direction of interest
  // given the joint acceleration limits. It is a simple linear program:
  // maximize alpha such that J*qddot = alpha*direction and |qddot| <=
  // qddot_max. Let's setup the optimization problem in Drake
  drake::solvers::MathematicalProgram prog;
  auto qddot = prog.NewContinuousVariables(
      robot_model().plant().num_positions(), "qddot");
  auto alpha = prog.NewContinuousVariables(1, "alpha");
  // Add constraint |qddot| <= qddot_max. We have them from time_optimal_spliner
  const auto acc_limit =
      time_optimal_spliner().joint_dynamic_limits().acceleration_bound;
  prog.AddBoundingBoxConstraint(-acc_limit, acc_limit, qddot);
  // Constraint J*qddot = alpha*directio or [J -direction] * [qddot; alpha] = 0
  Eigen::MatrixXd Aeq(6, robot_model().plant().num_positions() + 1);
  Aeq << J, -direction;
  Eigen::VectorXd beq = Eigen::VectorXd::Zero(6);
  prog.AddLinearEqualityConstraint(Aeq, beq, {qddot, alpha});
  // Objective: maximize alpha
  prog.AddLinearCost(-alpha(0));  // minimize negative of alpha to maximize it
  // Solve the optimization problem
  drake::solvers::MathematicalProgramResult result =
      drake::solvers::Solve(prog);
  if (!result.is_success()) {
    auto msg = "Optimization failed to find maximum acceleration";
    logging::log()->error(
        "DracoPlanner:SolveMaxCartesianAccelerationProblem {}", msg);
    return std::unexpected(msg);
  }
  double max_alpha = result.GetSolution(alpha(0));
  DRAKE_DEMAND(max_alpha > 0);
  auto max_qddotdot = result.GetSolution(qddot);
  // Let the idle models be zero.
  std::set<drake::multibody::ModelInstanceIndex> active_model_instances;
  const auto model_A = frame_A.model_instance();
  const auto model_B = frame_B.model_instance();
  active_model_instances.insert(model_A);
  active_model_instances.insert(model_B);
  max_qddotdot = robot_model().SetIdleModelsConfigToRef(
      max_qddotdot,
      Eigen::VectorXd::Zero(robot_model().plant().num_positions()),
      active_model_instances);
  // log the maximum acceleration in the direction of interest
  logging::log()->info("Maximum alpha in the direction of interest is {}",
                       max_qddotdot.transpose());
  const auto max_acceleration = max_alpha * direction;
  logging::log()->info("Maximum achievable acceleration is {} m/s²",
                       max_acceleration.transpose());
  // Let's make the spline
  double tau = 1 / std::sqrt(max_alpha);  // time to reach the max acceleration
                                          // at the end of the segment.
  logging::log()->info("Time to reach max velocity: {} s. Period = {} s. ", tau,
                       4 * tau);
  // Spline = q = q_0 + 1/2 * max_qddotdot * t^2 for t in [0, tau]. Then, v =
  // max_qddotdot * tau and q = q_0 + v_0 * t - 1/2 * max_qddotdot * t^2 for t
  // in [tau, 2*tau] to stop at the end of the segment. Check velocity limit and
  // joint limits
  const auto vel_limit =
      time_optimal_spliner().joint_dynamic_limits().velocity_bound;
  auto max_vel = max_qddotdot * tau;
  auto max_q = q_0 + 0.5 * max_qddotdot * tau * tau;
  if ((max_vel.array().abs() > vel_limit.array()).any()) {
    auto msg = fmt::format(
        "Resulting velocity exceeds velocity limit: max_vel = {}, vel_limit = "
        "{}",
        max_vel.transpose(), vel_limit);
    logging::log()->error(
        "DracoPlanner:SolveMaxCartesianAccelerationProblem {}", msg);
    return std::unexpected(msg);
  }
  if ((max_q.array() < robot_model().plant().GetPositionLowerLimits().array())
          .any()
      || (max_q.array()
          > robot_model().plant().GetPositionUpperLimits().array())
             .any()) {
    auto msg = fmt::format(
        "Resulting position exceeds joint limits: max_q = {}, lower_limits = "
        "{}, upper_limits = {}",
        max_q.transpose(),
        robot_model().plant().GetPositionLowerLimits().transpose(),
        robot_model().plant().GetPositionUpperLimits().transpose());
    logging::log()->error(
        "DracoPlanner:SolveMaxCartesianAccelerationProblem {}", msg);
    return std::unexpected(msg);
  }
  int n = robot_model().plant().num_positions();
  Eigen::MatrixXd coefficients_1 = Eigen::MatrixXd::Zero(n, 3);
  Eigen::MatrixXd coefficients_2 = Eigen::MatrixXd::Zero(n, 3);
  std::vector<double> breaks = {0.0, tau, 2 * tau};
  // Segment 1
  coefficients_1.col(0) = q_0;
  coefficients_1.col(2) = 0.5 * max_qddotdot;
  // Segment 2
  coefficients_2.col(0) = q_0 + 0.5 * max_qddotdot * tau * tau;
  coefficients_2.col(1) = max_qddotdot * tau;
  coefficients_2.col(2) = -0.5 * max_qddotdot;
  // Let's now create piecewise polynomials for each segment
  Eigen::MatrixX<drake::Polynomiald> poly_matrix_1(n, 1), poly_matrix_2(n, 1);
  for (int i = 0; i < n; i++) {
    poly_matrix_1(i, 0) = drake::Polynomiald(coefficients_1.row(i));
    poly_matrix_2(i, 0) = drake::Polynomiald(coefficients_2.row(i));
  }
  std::vector<Eigen::MatrixX<drake::Polynomiald>> polys = {poly_matrix_1,
                                                           poly_matrix_2};
  auto half_traj_1 =
      drake::trajectories::PiecewisePolynomial<double>(polys, breaks);
  auto full_traj = half_traj_1;
  // Now, reverse time and append to get the full trajectory
  auto half_traj_2 = half_traj_1;
  half_traj_2.ReverseTime();
  half_traj_2.shiftRight(4 * tau);
  full_traj.ConcatenateInTime(half_traj_2);
  // Check end_time is 4*tau
  DRAKE_DEMAND(std::abs(full_traj.end_time() - 4 * tau) < 1e-6);
  // Make this append n_cycle times
  if (problem.num_cycles() > 1) {
    logging::log()->info(
        "Appending the trajectory {} times to make a total of {} cycles.",
        problem.num_cycles() - 1, problem.num_cycles());
    auto full_traj_copy = full_traj;
    for (int i = 1; i < problem.num_cycles(); i++) {
      full_traj_copy.shiftRight(4 * tau);
      full_traj.ConcatenateInTime(full_traj_copy);
    }
  }
  // Let's get the uniform timing for this trajectory
  auto timing = motion::splining::internal::MakeUniformTimingForPath(full_traj);
  return std::make_pair(full_traj, timing);
}

}  // namespace planner
}  // namespace draco
