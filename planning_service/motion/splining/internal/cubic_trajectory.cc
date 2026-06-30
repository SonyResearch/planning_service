#include "planning_service/motion/splining/internal/cubic_trajectory.h"

namespace motion {
namespace splining {
namespace internal {

namespace {

// Return max_x such that lb <= a * x <= ub. If not positive, return nullopt as
// fast as possible.
std::optional<double> max_linear_bounds(const Eigen::VectorXd& lb,
                                        const Eigen::VectorXd& ub,
                                        const Eigen::VectorXd& a) {
  double x = std::numeric_limits<double>::infinity();
  for (int i = 0; i < lb.size(); ++i) {
    if (a(i) > 0) {
      x = std::min(x, ub(i) / a(i));
    } else if (a(i) < 0) {
      x = std::min(x, lb(i) / a(i));
    }
    if (x < 0) {  // Early return
      return std::nullopt;
    }
  }
  return x;
}

// Return max_x such that lb <= a * x^2+ b * x <= ub. If not positive, return
// nullopt as fast as possible.
std::optional<double> max_quadratic_bounds(const Eigen::VectorXd& lb,
                                           const Eigen::VectorXd& ub,
                                           const Eigen::VectorXd& a,
                                           const Eigen::VectorXd& b) {
  double x = std::numeric_limits<double>::infinity();
  for (int i = 0; i < lb.size(); ++i) {
    if (a(i) > 0) {
      double delta = b(i) * b(i) + 4 * a(i) * ub(i);
      if (delta < 0) {
        return std::nullopt;
      }
      x = std::min(x, (-b(i) + std::sqrt(delta)) / (2 * a(i)));
    } else if (a(i) < 0) {
      double delta = b(i) * b(i) + 4 * a(i) * lb(i);
      if (delta < 0) {
        return std::nullopt;
      }
      x = std::min(x, (-b(i) - std::sqrt(delta)) / (2 * a(i)));
    }
    if (x < 0) {  // Early return
      return std::nullopt;
    }
  }
  return x;
}

}  // namespace

std::optional<double> CalcCubicTrajectoryMinimumTime(
    const Eigen::VectorXd& start_position,
    const Eigen::VectorXd& start_velocity, const Eigen::VectorXd& end_position,
    const Eigen::VectorXd& end_velocity,
    const Eigen::VectorXd& acceleration_bound,
    const Eigen::VectorXd& velocity_bound, double minimum_time) {
  DRAKE_THROW_UNLESS(minimum_time > 0);
  // Say the trajectory length is h.
  // Parameterize the trajectory by Bezier Curve defined by control points (q_0,
  // q_1, q_2, q_3). Where q_0 = start_position, q_3 = end_position. Therefore,
  // the velocity will be given by the Bezier Curve defined by control points
  // 3(q_1 - q_0), 3(q_2 - q_1), 3(q_3 - q_2) / h Also, the acceleration will be
  // given by the Bezier Curve defined by control points 6(q_2 - 2q_1 + q_0),
  // 6(q_3 - 2q_2 + q_1) / h^2 By inspection, we have: start_velocity = 3(q_1 -
  // start_position) / h -> q_1 = start_position + start_velocity * h / 3
  // end_velocity = 3(end_position - q_2) / h -> q_2 = end_position -
  // end_velocity * h / 3 Constraint 1: 3(q_2 - q_1)/h is within the velocity
  // limits. Constraint 2: 6(q_2 - 2q_1 + q_0) / h^2 is within the acceleration
  // limits. Constraint 3: 6(q_3 - 2q_2 + q_1) / h^2 is within the acceleration
  // limits.
  auto x_opt_1 =
      max_linear_bounds(-velocity_bound + start_velocity + end_velocity,
                        velocity_bound + start_velocity + end_velocity,
                        3 * (end_position - start_position));
  if (!x_opt_1.has_value()) {
    return std::nullopt;
  }
  auto x_opt_2 = max_quadratic_bounds(-acceleration_bound, acceleration_bound,
                                      6 * (end_position - start_position),
                                      -(2 * end_velocity + 4 * start_velocity));
  if (!x_opt_2.has_value()) {
    return std::nullopt;
  }
  auto x_opt_3 = max_quadratic_bounds(-acceleration_bound, acceleration_bound,
                                      6 * (start_position - end_position),
                                      2 * start_velocity + 4 * end_velocity);
  if (!x_opt_3.has_value()) {
    return std::nullopt;
  }
  double x_max =
      std::min(std::min(x_opt_1.value(), x_opt_2.value()), x_opt_3.value());
  double h = 1.0 / x_max;
  return std::max(h, minimum_time);  // Ensure minimum time is respected.
}

drake::trajectories::PiecewisePolynomial<double>
CubicTrajectoryFromControlPoints(const Eigen::VectorXd& q0,
                                 const Eigen::VectorXd& q1,
                                 const Eigen::VectorXd& q2,
                                 const Eigen::VectorXd& q3, double h) {
  Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(q0.size(), 4);
  coefficients.col(0) = q0;
  coefficients.col(1) = 3 * (q1 - q0) / h;
  coefficients.col(2) = 3 * (q2 - 2 * q1 + q0) / (h * h);
  coefficients.col(3) = (q3 - 3 * q2 + 3 * q1 - q0) / (h * h * h);
  Eigen::MatrixX<drake::Polynomiald> poly_matrix(q0.size(), 1);
  for (int j = 0; j < q0.size(); ++j) {
    poly_matrix(j, 0) = drake::Polynomiald(coefficients.row(j));
  }
  std::vector<Eigen::MatrixX<drake::Polynomiald>> poly_matrix_vec {poly_matrix};
  std::vector<double> breaks = {0.0, h};
  drake::trajectories::PiecewisePolynomial<double> piecewise_polynomial(
      poly_matrix_vec, breaks);
  return piecewise_polynomial;
}

}  // namespace internal
}  // namespace splining
}  // namespace motion
