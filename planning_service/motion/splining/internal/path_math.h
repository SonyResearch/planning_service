/*
 * Copyright © 2024 Dexai Robotics. All rights reserved.
 */

/// @file ik_planner.h

#pragma once
#include <drake/common/trajectories/bezier_curve.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/trajectories/path_parameterized_trajectory.h>
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/trajectory.h>

#include "planning_service/common/logging.h"

namespace motion {
namespace splining {
namespace internal {

/** Given a sample q and path, finds the time on the path that is closest to q.
 *
 * @param q The sample value.
 * @param path The path to search.
 * @param start_time The start time to search.
 * @param end_time The end time to search.
 * @param search_step_size The step size to search.
 *
 * @throws std::runtime_error if search_step_size <= 0.0.
 * @throws std::runtime_error if q.rows() != path.rows().
 * @throws std::runtime_error if path.start_time() > start_time.
 * @throws std::runtime_error if path.end_time() < end_time.
 * @return The time on the path that is closest to q.
 */
double FindBestMatchTime(const Eigen::VectorXd& q,
                         const drake::trajectories::Trajectory<double>& path,
                         double start_time, double end_time,
                         double search_step_size);

/** Given a set of waypoints and a path, finds the times on the path that are
 * closest to the waypoints.
 * @param waypts The waypoints to search.
 * @param path The path to search.
 * @param start_time The start time to search.
 * @param end_time The end time to search.
 * @param minimum_time_difference The minimum time difference between waypoints.
 * @param search_step_size The step size to search.
 *
 * @throws std::runtime_error if search_step_size <= 0.0.
 * @throws std::runtime_error if path.start_time() > start_time.
 * @throws std::runtime_error if path.end_time() < end_time.
 * @return The times on the path that are closest to the waypoints.
 */
std::vector<double> FindBestMatchTimes(
    const std::vector<Eigen::VectorXd>& waypts,
    const drake::trajectories::Trajectory<double>& path, double start_time,
    double end_time, double minimum_time_difference, double search_step_size);

/** Calculates a path that passes through the given waypoints and has the given
 * start tangent.
 *
 * @param sample_times The times at which the waypoints are sampled.
 * @param sample_values The values of the waypoints.
 * @param start_tangent The tangent at the start of the path.
 *
 * @return A cubic PiecewisePolynomial trajectory passing through the waypoints.
 */
drake::trajectories::PiecewisePolynomial<double> CalcCubicPath(
    const std::vector<double>& sample_times,
    const std::vector<Eigen::MatrixXd>& sample_values,
    const Eigen::MatrixXd& start_tangent);

/** Calculates a path that passes through the given waypoints and has the given
 * start tangent.
 *
 * @param sample_times The times at which the waypoints are sampled.
 * @param sample_values The values of the waypoints.
 * @param start_tangent The tangent at the start of the path.
 * @param epsilon The epsilon value for the smoothing.
 *
 * @return A cubic PiecewisePolynomial trajectory passing through the waypoints.
 */
std::vector<Eigen::VectorXd> SmoothWaypoints(
    const std::vector<double>& sample_times,
    const std::vector<Eigen::VectorXd>& sample_values,
    const std::optional<Eigen::VectorXd>& start_tangent,
    const Eigen::VectorXd& epsilon);

/** Given a trajectory and a set of timed waypoints, calculates the vector of
 * waypoints that can be visited at times after time_switch.
 *
 * @param waypts The waypoints to visit.
 * @param path The original path.
 * @param time_now The current time.
 * @param delta_switch The time at which to switch to the new path.
 *
 * @return A trajectory that starts at time_now + delta_switch and visits the
 * waypoints that can are best visited after time_now + delta_switch - those
 * waypoints that are best visited before time_now + delta_switch are ignored.
 */
drake::trajectories::PiecewisePolynomial<double> CalcTrailingPathTowardWaypts(
    std::vector<Eigen::VectorXd> waypts,
    const drake::trajectories::Trajectory<double>& path, double time_now,
    double delta_switch, double minimum_spacing = 0.1,
    double search_step_size = 0.01, bool smoothing = false,
    const Eigen::VectorXd& smoothing_epsilon = Eigen::VectorXd::Zero(0));

/** Given a set of waypoints and a path, finds the times on the path that are
 * closest to the waypoints and calculates the tangent at the switch time.
 *
 * @param waypts The waypoints to search.
 * @param path The path to search.
 * @param time_now The current time.
 * @param delta_switch The time at which to switch to the new path.
 * @param minimum_spacing The minimum spacing between waypoints.
 * @param search_step_size The step size to search.
 * @param smoothing Whether to smooth the waypoints.
 * @param smoothing_epsilon The epsilon value for the smoothing.
 *
 * @throws std::runtime_error if delta_switch <= 0.0.
 * @return A tuple of the times and values of the waypoints that are best
 * visited after time_now + delta_switch, and the tangent at time_now +
 * delta_switch.
 */
std::tuple<std::vector<double>, std::vector<Eigen::VectorXd>, Eigen::VectorXd>
CalcSamplesAndTangentFromNewWaypts(
    std::vector<Eigen::VectorXd> waypts,
    const drake::trajectories::Trajectory<double>& path, double time_now,
    double delta_switch, double minimum_spacing = 0.1,
    double search_step_size = 0.01, bool smoothing = false,
    const Eigen::VectorXd& smoothing_epsilon = Eigen::VectorXd::Zero(0));

/** Merges a trajectory into another one.
 *
 * @param original_traj The original trajectory.
 * @param time_now The current time.
 * @param other_traj The other trajectory.
 *
 * @throws std::runtime_error if other_traj.start_time() < time_now.
 * @return The merged trajectory traj_merged such that
 * traj_merged.start_time() = time_now and traj_merged.end_time() =
 * other_traj.end_time() and traj_merged.value(t) = original_traj.value(t) for t
 * < other_traj.start_time(), and traj_merged.value(t) = other_traj.value(t) for
 * t >= other_traj.start_time().
 */
drake::trajectories::PiecewisePolynomial<double> MergeTrajectory(
    const drake::trajectories::PiecewisePolynomial<double>& original_traj,
    double time_now,
    const drake::trajectories::PiecewisePolynomial<double>& other_traj);

/** Merges a path parameterized trajectory into another one.
 * @param original_traj The original trajectory.
 * @param time_now The current time.
 * @param other_traj The other trajectory.
 * See MergeTrajectory for more details.
 *
 * @warning This function currently only works if the path and time
 * parameterization are both PiecewisePolynomial.
 */
drake::trajectories::PathParameterizedTrajectory<double> MergeTrajectory(
    const drake::trajectories::PathParameterizedTrajectory<double>&
        original_traj,
    double time_now,
    const drake::trajectories::PathParameterizedTrajectory<double>& other_traj);

drake::trajectories::PiecewisePolynomial<double> BezierCurveToPolynomial(
    const drake::trajectories::BezierCurve<double>& bezier_curve);

/** Converts a CompositeTrajectory to a PiecewisePolynomial.
 * @param composite_bezier_curve The CompositeTrajectory to convert.
 * @param start_time The start time of the trajectory.
 * @param end_time The end time of the trajectory.
 *
 * @throws std::runtime_error if all the elements of the composite_bezier_curve
 * are not PiecewisePolynomial.
 * @return A PiecewisePolynomial representing the same trajectory as the
 * CompositeTrajectory.
 */
drake::trajectories::PiecewisePolynomial<double>
CompositeBezierCurveToPiecewisePolynomial(
    const drake::trajectories::CompositeTrajectory<double>&
        composite_bezier_curve);

/**
 * Combine the system timed trajectories into a single trajectory, making sure
 * that continuity is not violated.
 * @param system_timed_trajectories the system timed trajectories to combine.
 * @param continuity_tolerance the tolerance for continuity check.
 * @return the combined system timed trajectory.
 */
std::pair<drake::trajectories::PiecewisePolynomial<double>,
          drake::trajectories::PiecewisePolynomial<double>>
CombineSequentialSystemTimedTrajectories(
    const std::vector<
        std::pair<drake::trajectories::PiecewisePolynomial<double>,
                  drake::trajectories::PiecewisePolynomial<double>>>&
        system_timed_trajectories,
    double continuity_tolerance = 1e-3);

/**
 * @brief Given path q(s), make s(t) such that s_dot = 1, i.e. the path
 * parameterization is uniform in time.
 *
 * @param path
 * @param t_start
 * @return drake::trajectories::PiecewisePolynomial<double>
 */
drake::trajectories::PiecewisePolynomial<double> MakeUniformTimingForPath(
    const drake::trajectories::PiecewisePolynomial<double>& path,
    double t_start = 0.0);

/** Removes any constant polynomial segments prepended to a piecewise
 * polynomial.
 *
 * Visits each segment and checks whether all of its polynomials are constant
 * (i.e. only the degree-0 coefficient is nonzero). The first segment that
 * contains a non-constant polynomial marks the start of the returned
 * trajectory. If no constant prepend is found the original trajectory is
 * returned unchanged.
 *
 * @param pp The piecewise polynomial to process.
 * @return The piecewise polynomial with constant prepend segments removed.
 */
drake::trajectories::PiecewisePolynomial<double> RemoveConstantPrepend(
    const drake::trajectories::PiecewisePolynomial<double>& pp);

}  // namespace internal
}  // namespace splining
}  // namespace motion
