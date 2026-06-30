#pragma once
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/trajectory.h>

#include "planning_service/common/logging.h"

namespace motion {
namespace splining {
namespace internal {

/**
 * @brief Calculate the minimum time for a cubic trajectory to pass through the
 * given waypoints.
 *
 * @param start_position The position at the start of the trajectory.
 * @param start_velocity The velocity at the start of the trajectory.
 * @param end_position The position at the end of the trajectory.
 * @param end_velocity The velocity at the end of the trajectory.
 * @param acceleration_bound The acceleration bound.
 * @param velocity_bound The velocity bound.
 * @param minimum_time The minimum time for the trajectory to pass through the
 * given waypoints. Default is 0.001 seconds.
 * @return std::optional<double> The minimum time for the trajectory to pass
 * through the given waypoints. If the trajectory is not feasible, return
 * nullopt.
 */
std::optional<double> CalcCubicTrajectoryMinimumTime(
    const Eigen::VectorXd& start_position,
    const Eigen::VectorXd& start_velocity, const Eigen::VectorXd& end_position,
    const Eigen::VectorXd& end_velocity,
    const Eigen::VectorXd& acceleration_bound,
    const Eigen::VectorXd& velocity_bound, double minimum_time = 0.001);

/**
 * @brief Converts the control points of a cubic trajectory to a
 * PiecewisePolynomial.
 *
 * @param q0
 * @param q1
 * @param q2
 * @param q3
 * @param h
 * @return drake::trajectories::PiecewisePolynomial<double>
 */
drake::trajectories::PiecewisePolynomial<double>
CubicTrajectoryFromControlPoints(const Eigen::VectorXd& q0,
                                 const Eigen::VectorXd& q1,
                                 const Eigen::VectorXd& q2,
                                 const Eigen::VectorXd& q3, double h);

}  // namespace internal
}  // namespace splining
}  // namespace motion
