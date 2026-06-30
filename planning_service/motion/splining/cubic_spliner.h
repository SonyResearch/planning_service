/*
 * Copyright © 2024 Dexai Robotics. All rights reserved.
 */

/// @file cubic_spliner.h
#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>

#include "planning_service/motion/planning/sample_based_planner.h"

namespace motion {
namespace splining {

struct CubicSpliningParameters {
  // number of attempts to construct a valid spline before giving up
  int max_resplines {10};
  // TODO: what are the "boundary conditions"?
  std::optional<Eigen::VectorXd> forward_tangent_splines {std::nullopt};
};

/**
 * @brief Class which constructs a valid (satisfying all constraints)
 * parameterized spline q(s) for a set of waypoints.
 */
class CubicSpliner {
 public:
  /** constructor from RobotModel and ConstraintsAdapter */
  CubicSpliner(const RobotConstraints& robot_constraints);

  /** Given a matrix of waypoints, attempt to compute a smooth cubic spline
   * which also satisfies all constraints.
   * @param waypts matrix of waypoints to spline
   * @param splining_parameters boundary conditions, number of iterations
   *
   * @return The computed spline and updated waypoint vector as a pair, or
   * std::nullopt on failure
   */
  std::optional<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                          std::vector<Eigen::VectorXd>>>
  WayptsToValidPath(const Eigen::MatrixXd& waypts,
                    const CubicSpliningParameters splining_parameters) const;

  /** Vector overload.
   * @param waypts vector of waypoints to spline
   * @param splining_parameters boundary conditions, number of iterations
   *
   * @return The computed spline and updated waypoint vector as a pair, or
   * std::nullopt on failure
   */
  std::optional<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                          std::vector<Eigen::VectorXd>>>
  WayptsToValidPath(const std::vector<Eigen::VectorXd>& waypts,
                    const CubicSpliningParameters splining_parameters) const;

 protected:
  /** Given a matrix of waypoints, compute a smooth (i.e., continuous in its
   * second derivative) cubic spline which passes through all waypoints.
   * NOTE: There are no guarantees on the safety of the spline returned by this
   * method.
   * @param waypts matrix of waypoints to
   * @param forward_tangent_splines if specified, it will set the boundary
   * conditions at the start of the spline
   *
   * @return The computed spline
   */
  drake::trajectories::PiecewisePolynomial<double> ConstructCubicPath(
      const Eigen::MatrixXd& waypts,
      const std::optional<Eigen::VectorXd> forward_tangent_splines =
          std::nullopt) const;

  /** Add nodes to fix the spline resulting from the path:
   *  For each knot point, check if the spline violates constraints
   * between this knot point and the previous one.
   * If it does, add a knot point that is the average of these two knots
   * and insert it in between them.
   * @param waypts matrix of waypoints that generated the spline
   * @param q_sample matrix of sampled states from the spline
   * @param s_sample vector of s values for each sampled state from the
   * spline
   * @param are_states_valid_vec Vector containing the validity of each entry
   * q_i in q_sample
   *
   * @return The adjusted waypoints matrix with new points added
   */
  Eigen::MatrixXd AddNodesToFixPath(
      const Eigen::MatrixXd& spline_waypts, const Eigen::MatrixXd& q_sample,
      const Eigen::VectorXd& s_sample,
      std::vector<uint8_t> are_states_valid_vec) const;

 public:
  const std::unique_ptr<planning::ompl::SampleBasedPlanningContext>&
  planning_context() const {
    return planning_context_;
  }

 private:
  std::unique_ptr<planning::ompl::SampleBasedPlanningContext> planning_context_;
};
}  // namespace splining
}  // namespace motion
