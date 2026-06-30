/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file trajopt_adaptors.h

#pragma once

#include <drake/common/trajectories/bezier_curve.h>
#include <drake/common/trajectories/composite_trajectory.h>
#include <drake/common/yaml/yaml_io.h>

#include <optional>

namespace motion {
namespace splining {

using traj_ptr_t =
    drake::copyable_unique_ptr<drake::trajectories::Trajectory<double>>;

using system_poly_t =
    std::map<std::string, drake::trajectories::PiecewisePolynomial<double>>;

struct CompositeBezierTrajectoryAdapter {
  std::vector<double> start_times_vec {};
  std::vector<double> end_times_vec {};
  std::vector<Eigen::MatrixXd> control_points_vec {};

  /** serialization function for PiecewisePolynomialTrajectoryAdapter */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(start_times_vec));
    a->Visit(DRAKE_NVP(end_times_vec));
    a->Visit(DRAKE_NVP(control_points_vec));
  }
  static drake::trajectories::CompositeTrajectory<double>
  AdapterToCompositeTrajectory(
      const CompositeBezierTrajectoryAdapter& adapter) {
    std::vector<traj_ptr_t> subtrajs;
    for (size_t i = 0; i < adapter.start_times_vec.size(); ++i) {
      const auto& start_time = adapter.start_times_vec[i];
      const auto& end_time = adapter.end_times_vec[i];
      const auto& control_points = adapter.control_points_vec[i];
      traj_ptr_t traj_ptr =
          drake::copyable_unique_ptr<drake::trajectories::Trajectory<double>>(
              new drake::trajectories::BezierCurve<double>(start_time, end_time,
                                                           control_points));
      subtrajs.push_back(traj_ptr);
    }
    return drake::trajectories::CompositeTrajectory<double>(subtrajs);
  }
  static CompositeBezierTrajectoryAdapter CompositeTrajectoryToAdapter(
      const drake::trajectories::CompositeTrajectory<double>& trajectory) {
    CompositeBezierTrajectoryAdapter adapter;
    for (int i = 0; i < trajectory.get_number_of_segments(); ++i) {
      const auto& subtraj = trajectory.segment(i);
      adapter.start_times_vec.push_back(subtraj.start_time());
      adapter.end_times_vec.push_back(subtraj.end_time());
      const auto& bezier_curve =
          dynamic_cast<const drake::trajectories::BezierCurve<double>&>(
              subtraj);
      adapter.control_points_vec.push_back(bezier_curve.control_points());
    }
    return adapter;
  }
  static void SaveYamlFile(
      const std::string& filename,
      const drake::trajectories::CompositeTrajectory<double>& trajectory) {
    drake::yaml::SaveYamlFile(filename,
                              CompositeTrajectoryToAdapter(trajectory));
  }

  static drake::trajectories::CompositeTrajectory<double> LoadYamlFile(
      const std::string& filename) {
    const auto adapter =
        drake::yaml::LoadYamlFile<CompositeBezierTrajectoryAdapter>(filename);
    return AdapterToCompositeTrajectory(adapter);
  }
};

struct JointDynamicLimits {
  Eigen::VectorXd velocity_bound {Eigen::VectorXd::Zero(0)};
  Eigen::VectorXd acceleration_bound {Eigen::VectorXd::Zero(0)};
  Eigen::VectorXd torque_bound {Eigen::VectorXd::Zero(0)};

  /** serialization function for JointDynamicLimits */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(velocity_bound));
    a->Visit(DRAKE_NVP(acceleration_bound));
    a->Visit(DRAKE_NVP(torque_bound));
  }
};

using joint_dynamic_limits_map_t = std::map<std::string, JointDynamicLimits>;

struct CartesianDynamicLimits {
  std::optional<double> speed_limit {std::nullopt};
  std::optional<drake::Vector6d> acc_lower {std::nullopt};
  std::optional<drake::Vector6d> acc_upper {std::nullopt};

  /** serialization function for CartesianDynamicLimits */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(speed_limit));
    a->Visit(DRAKE_NVP(acc_lower));
    a->Visit(DRAKE_NVP(acc_upper));
  }
};

using cartesian_dynamic_limits_map_t =
    std::map<std::string, CartesianDynamicLimits>;

}  // namespace splining
}  // namespace motion
