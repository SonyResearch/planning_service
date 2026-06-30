/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file time_optimal_spliner.h

#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/common/trajectories/piecewise_trajectory.h>
#include <drake/common/trajectories/trajectory.h>
// #include <drake/multibody/optimization/toppra.h>
#include <drake/systems/framework/context.h>

#include "planning_service/common/override.h"
#include "planning_service/motion/robot_model.h"
#include "planning_service/motion/splining/drake/toppra.h"
#include "planning_service/motion/splining/internal/path_math.h"
#include "planning_service/motion/splining/trajopt_adapters.h"
namespace motion {
namespace splining {

using joint_dynamic_limits_map_t = std::map<std::string, JointDynamicLimits>;
using cartesian_dynamic_limits_map_t =
    std::map<std::string, CartesianDynamicLimits>;

struct TimeOptimalSplineParams {
  drake::multibody::CalcGridPointsOptions calc_grid_points_options;
  double safety_factor_velocity {0.99};
  double safety_factor_acceleration {0.99};
  double safety_factor_torque {0.8};

  // serialization
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(calc_grid_points_options));
    a->Visit(DRAKE_NVP(safety_factor_velocity));
    a->Visit(DRAKE_NVP(safety_factor_acceleration));
    a->Visit(DRAKE_NVP(safety_factor_torque));
  }
};

/** TimeOptimalSpliner*/
class TimeOptimalSpliner {
 public:
  /** Constructor
   * @param robot_model the robot model
   * @param joint_dynamic_limits_map the joint dynamic limits map
   * @param cartesian_dynamic_limits_map the cartesian dynamic limits map
   * @param time_optimal_spline_params the time optimal spline parameters
   * @param arm: if provided, the spliner will use the plant for the specified
   * arm. This is useful for multiple arms, when one is interested in splining
   * only one arm.
   */
  TimeOptimalSpliner(
      const RobotModel& robot_model,
      const joint_dynamic_limits_map_t& joint_dynamic_limits_map,
      const cartesian_dynamic_limits_map_t& cartesian_dynamic_limits_map,
      const TimeOptimalSplineParams& time_optimal_spline_params,
      ArmIndex arm_index = ArmIndex());
  /**
   * @brief Adopt the cartesian dynamic limits. Performs a correctness check
   * to ensure that all frames in the limits map are valid.
   *
   * @param cartesian_dynamic_limits_map
   */
  cartesian_dynamic_limits_map_t ValidateCartesianDynamicLimits(
      const cartesian_dynamic_limits_map_t& cartesian_dynamic_limits_map) const;

  /** Calculate a dynamics-aware spline that passes through the given waypoints.
   * The method is based on an solving an optimization problem in which the
   * decision variables are two control points between each pair of consecutive
   * waypoints. The optimization ints for each segment is:
   * \f[
   * \begin{align*}
   * q_1, q_2 \in \text{Joint Limits} \\
   * 3(q_1 - q_s), 3(q_2 - q_1), 3(q_e - q_2) \in \tau * \text{Velocity Limits}
   * \\
   * 6(q_2 - 2q_1 + q_s), 6(q_e - 2q_2 + q_1) \in \tau^2 * \text{Acceleration
   * Limits} \\
   * \end{align*}
   * \f]
   * where q_s, q_e are 𝜏 is the segment duration. However, since we do not want
   * the quadratic term in 𝜏, we approximate it conservatively by
   * minimum_total_duration * 𝜏 * (segment_time_fraction).
   *
   * For derivation, see, e.g.,
   * \url{https://en.wikipedia.org/wiki/B%C3%A9zier_curve#Cubic_B%C3%A9zier_curves}
   *
   * The overall optimization problem is to minimize the total duration plus a
   * notion of Energy, subject to C1 and velocity/acceleration ints. The
   * solution in each segment becomes a 4th order Bezier curve, or a cubic
   * spline.The composite Bezier curve result is converted into a
   * PiecewisePolynomial.
   *
   * Note that the segment durations are only used relatively, as they are
   * scaled by a decision variable to satisfy the velocity and acceleration
   * limits.
   *
   * @param waypoints the waypoints
   * @param start_velocity the start velocity, if nullopt, the start velocity is
   * optimzied.
   * @param end_velocity the end velocity
   * @param segment_durations the segment durations.
   * @param max_num_iterations the maximum number of iterations.
   * @param minimum_time_scaling the minimum time scaling.
   * @return the dynamics-aware spline if successful, nullopt otherwise
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  CalcDynamicsAwareSpline(const std::vector<Eigen::VectorXd>& waypoints,
                          const std::optional<Eigen::VectorXd>& start_velocity,
                          const std::optional<Eigen::VectorXd>& end_velocity,
                          const std::vector<double>& segment_durations,
                          int max_num_iterations = 10,
                          double minimum_time_scaling = 1.1) const;

  /** Calculate a trajectory to stop the robot from the given start position and
   * velocity with the minimum time. It is a quadratic polynomial:
   * \begin{align*}
   * q(t) = q_0 + v_0 * t + 0.5 * a * t^2
   * \end{align*}
   * where q_0 is the start position, v_0 is the start velocity, and a is the
   * breaking acceleration is determined such that the slowest joint uses its
   * maximum acceleration.
   * @param start_position the start position
   * @param start_velocity the start velocity
   * @return the trajectory to stop the robot
   */
  drake::trajectories::PiecewisePolynomial<double> CalcTrajectoryToStop(
      const Eigen::VectorXd& start_position,
      const Eigen::VectorXd& start_velocity) const;

  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  CalcOptimalCubicPath(const Eigen::VectorXd& start_position,
                       const Eigen::VectorXd& start_velocity,
                       const Eigen::VectorXd& end_position,
                       const Eigen::VectorXd& end_velocity) const;

  std::optional<drake::trajectories::PiecewisePolynomial<double>> RunToppra(
      const drake::trajectories::Trajectory<double>& traj) const;

  /** Run Toppra on a piecewise trajectory
   @param piecewise_traj the piecewise trajectory to run Toppra on
   @return the time parameterization s(t) if successful, nullopt otherwise
   @note the advantage of using a piecewise trajectory is that the time
    parameterization uses the breakpoints of the piecewise trajectory
    as the grid points for Toppra.
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  RunToppraOnPiecewiseTrajectory(
      const drake::trajectories::PiecewiseTrajectory<double>& piecewise_traj,
      double s_dot_start = 0.0, double s_dot_end = 0.0) const;

  drake::trajectories::PiecewisePolynomial<double> ConvertToPoly(
      const drake::trajectories::Trajectory<double>& traj,
      const drake::trajectories::PiecewisePolynomial<double> path_time) const;

  system_poly_t ConvertToSystemPolynomial(
      const drake::trajectories::Trajectory<double>& traj,
      const drake::trajectories::PiecewisePolynomial<double>
          time_parameterization) const;

  system_poly_t SlicePerEntities(
      const drake::trajectories::PiecewisePolynomial<double>& poly) const;

  /** Check if the time parameterization satisfies the dynamic limits
   * @param traj the trajectory to check, treated as q(s)
   * @param time_parameterization the time parameterization s(t)
   * @return true if q(t) satisfies the dynamic limits
   */
  bool CheckSatisfied(const drake::trajectories::Trajectory<double>& traj,
                      const drake::trajectories::PiecewisePolynomial<double>
                          time_parameterization) const;

  /** Check if the time parameterization satisfies the dynamic limits
   * @param traj the trajectory to check, treated as q(t)
   * @return true if the time parameterization satisfies the dynamic limits
   */
  bool CheckSatisfied(
      const drake::trajectories::Trajectory<double>& traj) const;

  const JointDynamicLimits& joint_dynamic_limits() const {
    return joint_dynamic_limits_;
  }

  enum SpliningApproach {
    /** Use the dynamics-aware splining approach */
    kDynamicsAwareWithToppra,
    /*** Use the dynamics-aware splining approach with concatenating with
       trajectory to full stop */
    kDynamicsAwareWithStop,
    /** Use the cubic spline with continuous second derivatives */
    kCubicSpline
  };

  struct TrajTowardNewWaypointsParams {
    double delta_switch {0.1};
    double minimum_spacing {0.01};
    double search_step_size {0.001};
    bool smoothing {false};
    SpliningApproach splining_approach {kDynamicsAwareWithStop};
    Eigen::VectorXd smoothing_epsilon {Eigen::VectorXd::Zero(0)};
  };

  /** Calculate a trajectory toward new waypoints.
   * @param traj the current path-parameterized trajectory to update
   * @param time_now the current time (in the same time frame as @p traj)
   * @param delta_switch the time offset from @p time_now at which to switch
   *        from the original trajectory to the new trajectory
   * @param waypoints the new waypoints to track, in joint space
   * @param segment_durations the desired durations between consecutive
   *        waypoints; may be empty, in which case durations are estimated
   *        automatically
   * @param merge_point_search_step_size the step size used when searching
   *        along the original trajectory for a feasible merge/switch point
   * @param max_num_iterations the maximum number of scaling/feasibility
   *        iterations when solving for the time-parameterization
   * @param scale_duration_per_iteration factor by which segment durations
   *        are scaled on each iteration when feasibility is not yet met
   * @param run_toppra if true, run TOPPRA-based, dynamics-aware time
   *        parameterization; if false, use a kinematic splining approach
   * @param wiggle_room optional per-joint tolerance allowing small
   *        deviations from the exact waypoints, to improve feasibility
   * @param constrain_end_velocity_to_zero if true (default), the dynamics-
   *        aware spline is solved with a zero end-velocity boundary condition
   *        so the trajectory ends at rest without an explicit stop segment;
   *        if false, no end-velocity constraint is applied and an explicit
   *        stop trajectory is appended instead (the original behavior)
   * @return the updated trajectory if successful, nullopt otherwise. When
   *         @p constrain_end_velocity_to_zero is true the optimizer enforces
   *         a zero end-velocity; when false an explicit stop segment is
   *         appended, bringing the robot to rest after the last waypoint.
   */
  std::optional<drake::trajectories::PathParameterizedTrajectory<double>>
  CalcTrajTowardNewWaypoints(
      drake::trajectories::PathParameterizedTrajectory<double> traj,
      double time_now, double delta_switch,
      const std::vector<Eigen::VectorXd>& waypoints,
      const std::vector<double>& segment_durations,
      double merge_point_search_step_size, int max_num_iterations = 10,
      double scale_duration_per_iteration = 1.1, bool run_toppra = false,
      const std::optional<Eigen::VectorXd>& wiggle_room = std::nullopt,
      bool constrain_end_velocity_to_zero = true) const;

  /** Slices a path parameterized trajectory into a map of
   * model_name -> (path, time_parameterization) pair
   * @param traj the path parameterized trajectory to slice
   * @return the sliced path parameterized trajectory
   */
  std::map<std::string,
           std::pair<drake::trajectories::PiecewisePolynomial<double>,
                     drake::trajectories::PiecewisePolynomial<double>>>
  SlicePerEntities(
      drake::trajectories::PathParameterizedTrajectory<double> traj) const;

  /** Aggregates a map of model_name -> (path, time_parameterization) pair
   * into a path parameterized trajectory
   * @param sys_traj the map of model_name -> (path, time_parameterization) pair
   * @return the aggregated path parameterized trajectory
   */
  drake::trajectories::PathParameterizedTrajectory<double>
  ConvertToPathParameterizedTrajectory(
      const std::map<
          std::string,
          std::pair<drake::trajectories::PiecewisePolynomial<double>,
                    drake::trajectories::PiecewisePolynomial<double>>>& traj)
      const;

  const RobotModel& robot_model() const {
    return robot_model_;
  }

  const drake::multibody::MultibodyPlant<double>& plant() const {
    return plant_;
  }

  ArmIndex arm_index() const {
    return arm_index_;
  }

  void SetCartesianDynamicLimits(const cartesian_dynamic_limits_map_t&
                                     cartesian_dynamic_limits_map) const {
    cartesian_dynamic_limits_map_ =
        ValidateCartesianDynamicLimits(cartesian_dynamic_limits_map);
  }
  void SetTimeOptimalSplineParams(const TimeOptimalSplineParams& params) const {
    time_optimal_spline_params_ = params;
  }

  /** Returns the holonomic mapping associated with the spliner */
  const auto& holonomic_mapping() const {
    return holonomic_mapping_;
  }

 private:
  Eigen::VectorXd CalcTorque(const Eigen::Ref<const Eigen::VectorXd>& q,
                             const Eigen::Ref<const Eigen::VectorXd>& v,
                             const Eigen::Ref<const Eigen::VectorXd>& a) const;

  // Runs Toppra on a trajectory with given grid points
  // If acceleration_knots_interpolation is true, use acceleration knots
  // interpolation discretization method, which adds more constraints but
  // produces better results. Otherwise, use collocation method, which is faster
  // and numerically more stable.
  std::optional<drake::trajectories::PiecewisePolynomial<double>> DoRunToppra(
      const drake::trajectories::Trajectory<double>& traj,
      const Eigen::VectorXd& grid_points, double s_dot_start = 0.0,
      double s_dot_end = 0.0,
      bool acceleration_knots_interpolation = false) const;

  const RobotModel& robot_model_;
  const drake::multibody::MultibodyPlant<double>& plant_;
  const HolonomicMapping& holonomic_mapping_;
  const JointDynamicLimits joint_dynamic_limits_;
  mutable cartesian_dynamic_limits_map_t cartesian_dynamic_limits_map_;
  mutable TimeOptimalSplineParams time_optimal_spline_params_;
  const std::unique_ptr<drake::systems::Context<double>> plant_context_;
  const ArmIndex arm_index_;
};

}  // namespace splining
}  // namespace motion
