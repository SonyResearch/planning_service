/// @file straight_path_planner.cc

#include "planning_service/motion/planning/straight_path_planner.h"

#include "planning_service/motion/planning/internal/geodesic_math.h"

namespace motion {
namespace planning {

std::optional<drake::trajectories::PiecewisePolynomial<double>>
MaybeValidStraightLinePath(const RobotConstraints& robot_constraints,
                           Eigen::VectorXd start_point,
                           Eigen::VectorXd end_point, double step_size) {
  Eigen::VectorXd end_wrapped {end_point};
  internal::WrapConfiguration(
      start_point,
      robot_constraints.robot_model().continuous_revolute_joint_indices(),
      &end_wrapped);
  if (robot_constraints.CheckSatisfiedEdge(start_point, end_wrapped,
                                           step_size)) {
    logging::log()->info(
        "MaybeValidStraightLinePath: Straight line path is valid!");
    return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
        {0, 1}, {start_point, end_wrapped});
  }
  logging::log()->info(
      "MaybeValidStraightLinePath: Straight line path is invalid");
  return std::nullopt;
}

std::expected<drake::trajectories::PiecewisePolynomial<double>, std::string>
MaybeOutOfViolationPath(const RobotConstraints& robot_constraints,
                        const Eigen::VectorXd& start_violating_point,
                        double collision_influence_distance,
                        double initial_joint_clearance, int thread_num,
                        std::optional<Eigen::VectorXd> maybe_gradient) {
  std::optional<Eigen::VectorXd> q_valid_opt;
  if (maybe_gradient) {
    logging::log()->info(
        "MaybeOutOfViolationPath: Using provided gradient for nearest valid "
        "configuration optimization.");
    DRAKE_THROW_UNLESS(maybe_gradient.value().norm() > 1e-6);
    DRAKE_THROW_UNLESS(maybe_gradient.value().size()
                       == start_violating_point.size());
    auto gradient_normalized = maybe_gradient.value().normalized();
    double delta = initial_joint_clearance;
    auto joint_limits_constraint =
        [&robot_constraints](const Eigen::VectorXd& q) {
          const auto& lower_limit =
              robot_constraints.robot_model().plant().GetPositionLowerLimits();
          const auto& upper_limit =
              robot_constraints.robot_model().plant().GetPositionUpperLimits();
          return (q.array() >= lower_limit.array()).all()
                 && (q.array() <= upper_limit.array()).all();
        };
    Eigen::VectorXd q = start_violating_point;
    while (joint_limits_constraint(q)) {
      q += gradient_normalized * delta;
      if (robot_constraints.CheckSatisfied(q, 0)) {
        q_valid_opt = q;
        break;
      }
    }
  } else {
    logging::log()->info(
        "MaybeOutOfViolationPath: No gradient provided. The nearest valid "
        "configuration optimization will be performed without a gradient.");
    q_valid_opt = robot_constraints.CalcClosestSatisfyingConfiguration(
        start_violating_point, thread_num, {}, collision_influence_distance);
  }
  if (!q_valid_opt.has_value()) {
    return std::unexpected(
        "MaybeOutOfViolationPath: Nearest valid configuration optimization "
        "failed");
  }
  logging::log()->info(
      "MaybeOutOfViolationPath: Nearest valid configuration "
      "optimization succeeded at the distance of {} [rad]",
      (q_valid_opt.value() - start_violating_point).norm());
  CheckSatisfiedOptions options;
  options.verbose = true;
  if (!robot_constraints.CheckSatisfied(q_valid_opt.value(), 0, options)) {
    auto msg = fmt::format(
        "MaybeOutOfViolationPath: The optimized configuration is "
        "invalid. It is a bug, numerical issue, or the influence distance "
        "is too small.");
    logging::log()->error(msg);
    return std::unexpected(msg);
  }
  // let's extrapolate the trajectory to the valid configuration
  // with some configuration clearance. We start with joint_clearance and
  // try multiple iterations with decreasing clearance values.
  const int kMaxNumTries = 10;
  for (int i = 0; i < kMaxNumTries; ++i) {
    double joint_clearance_i =
        initial_joint_clearance * (1.0 - static_cast<double>(i) / kMaxNumTries);
    Eigen::VectorXd q_valid_clear =
        q_valid_opt.value()
        + (q_valid_opt.value() - start_violating_point).normalized()
              * joint_clearance_i;
    // The new edge must be valid.
    // Use the options object declared before the loop.
    if (!robot_constraints.CheckSatisfied(q_valid_clear, 0, options)) {
      logging::log()->info(
          "MaybeOutOfViolationPath: iteration {}/{}: The cleared valid "
          "configuration is invalid.",
          i + 1, kMaxNumTries);
      continue;
    }
    if (!robot_constraints.CheckSatisfiedEdge(q_valid_opt.value(),
                                              q_valid_clear)) {
      logging::log()->info(
          "MaybeOutOfViolationPath: iteration {}/{}: The edge from "
          "nearest valid to cleared valid configuration is invalid.",
          i + 1, kMaxNumTries);
      continue;
    }
    logging::log()->info(
        "MaybeOutOfViolationPath: iteration {}/{}: Trying path from "
        "start violating point to cleared valid configuration with joint "
        "clearance of {} [rad].",
        i + 1, kMaxNumTries, joint_clearance_i);
    return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
        {0, 1}, {start_violating_point, q_valid_clear});
  }
  auto msg = fmt::format(
      "MaybeOutOfViolationPath: Failed to find a valid cleared configuration "
      "after {} tries.",
      kMaxNumTries);
  logging::log()->error(msg);
  return std::unexpected(msg);
}

}  // namespace planning
}  // namespace motion
