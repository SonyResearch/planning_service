
/*
 * Copyright © 2025 Sony Research. All rights reserved.
 */

/// @file holonomic_mapping.h

#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/tree/multibody_element.h>
#include <drake/planning/dof_mask.h>
#include <drake/planning/iris/iris_common.h>

#include "drake/multibody/plant/multibody_plant.h"
#include "planning_service/common/logging.h"

namespace motion {

/** A HolonomicMapping represents a mapping between the full generalized
 * positions of a robot and a reduced set of generalized positions that
 * eliminates mimicking joints. For example, if a robot has two revolute joints
 * where the second joint mimics the first joint with a gear ratio of 2, then
 * the holonomic mapping will map the two joint angles (q1, q2) to a single
 * joint angle q1, where q2 = 2*q1. If no mimicking joints are present in the
 * robot model, then the holonomic mapping is the identity mapping.
 *
 * The two primary operations are Reduce() and Lift():
 * - Reduce() takes a full configuration vector and returns a reduced
 * configuration vector. Each mimicking joint is removed from the configuration
 * vector.
 * - Lift() takes a reduced configuration vector and returns a full
 * configuration vector. Each mimicking joint is added back to the configuration
 * vector using the mimicking relationship. The order of the joints in the full
 * configuration vector is preserved in the reduced configuration vector.
 */
class HolonomicMapping {
 public:
  /** Returns a reduced vector such that the dimension is reduced according to
   * the holonomic mapping.
   */
  Eigen::VectorXd Reduce(const Eigen::VectorXd& full_vector) const;

  /** Returns a lifted vector such that the dimension is lifted according to
   * the holonomic mapping.
   */
  Eigen::VectorXd Lift(const Eigen::VectorXd& reduced_vector) const;

  /** Returns a reduced PiecewisePolynomial such that the dimension is reduced
   * according to the holonomic mapping.
   */
  drake::trajectories::PiecewisePolynomial<double> Reduce(
      const drake::trajectories::PiecewisePolynomial<double>& full_pp) const;

  /** Returns a lifted PiecewisePolynomial such that the dimension is lifted
   * according to the holonomic mapping.
   */
  drake::trajectories::PiecewisePolynomial<double> Lift(
      const drake::trajectories::PiecewisePolynomial<double>& reduced_pp) const;

  /** Returns a reduced vector for AutoDiffXd such that the dimension is reduced
   * according to the holonomic mapping.
   */
  Eigen::VectorX<drake::AutoDiffXd> LiftAutoDiff(
      const Eigen::VectorX<drake::AutoDiffXd>& reduced_vector) const;

  /** Returns the full dimension of the configuration space. */
  int full_dim() const {
    return n_;
  }

  /** Returns the reduced dimension of the configuration space. */
  int minimal_dim() const {
    return m_;
  }

  /** Returns true if the mapping is the identity mapping. */
  bool is_identity() const {
    return is_identity_;
  }

  /** Returns the index of the generalized position that corresponds to the
   * reduced index.
   * @param reduced_index The index in the reduced configuration space.
   * @throws std::runtime_error if the reduced index is out of bounds.
   */
  int LiftedIndex(int reduced_index) const;

  /** Returns the index of the generalized position that corresponds to the
   * full index.
   * @param full_index The index in the full configuration space.
   * @throws std::runtime_error if the full index is out of bounds or if the
   * full index corresponds to a mimicking joint.
   */
  int ReducedIndex(int full_index) const;

  /** Returns true if the full index corresponds to a mimicking joint.
   * @param full_index The index in the full configuration space.
   * @throws std::runtime_error if the full index is out of bounds.
   */
  bool IsMimickingJoint(int full_index) const;

  /** Returns the indices of the mimicking joints in the full configuration
   * space.
   */
  std::vector<int> MimickingJointIndices() const;

  /** Returns the map from mimicking joint index to a tuple of (mimicked joint
   * index, gear ratio, offset).
   * @param full_index The index in the full configuration space.
   * @throws std::runtime_error if the full index is out of bounds or if the
   * full index does not correspond to a mimicking joint.
   */
  std::tuple<int, double, double> MimickingJointInfo(int full_index) const;

  /** Returns a reduced vector for a specific model instance such that the
   * dimension is reduced according to the holonomic mapping.
   * @param instance The model instance index.
   * @param q_instance_full The full configuration vector for the model
   * instance.
   * @throws std::runtime_error if the model instance does not exist in the
   * holonomic mapping or if the size of q_instance_full does not match the
   * number of positions in the model instance.
   */
  Eigen::VectorXd ReduceInstance(
      const drake::multibody::ModelInstanceIndex& instance,
      const Eigen::VectorXd& q_instance_full) const;

  /** Returns a lifted vector for a specific model instance such that the
   * dimension is lifted according to the holonomic mapping.
   * @param instance The model instance index.
   * @param q_instance_minimal The reduced configuration vector for the model
   * instance.
   * @throws std::runtime_error if the model instance does not exist in the
   * holonomic mapping or if the size of q_instance_minimal does not match the
   * number of positions in the reduced model instance.
   */
  Eigen::VectorXd LiftInstance(
      const drake::multibody::ModelInstanceIndex& instance,
      const Eigen::VectorXd& q_instance_minimal) const;

  const drake::planning::IrisParameterizationFunction&
  iris_parameterization_function() const {
    return iris_parameterization_function_;
  }

 private:
  // Constructs a HolonomicMapping from a MultibodyPlant and a set of DofMasks.
  // Each DofMask specifies which joints belong to a model instance.
  // @param plant The MultibodyPlant containing the model instances.
  // @param instance_dof_masks A map from model instance index to DofMask.
  HolonomicMapping(const drake::multibody::MultibodyPlant<double>& plant);

  const int n_ {0};
  const int m_ {0};
  const bool is_identity_ {true};
  const std::map<drake::multibody::ModelInstanceIndex, drake::planning::DofMask>
      instance_dof_masks_;
  std::map<int, int> full_to_minimal_;
  std::map<int, int> minimal_to_full_;
  // map from mimicking joint index to a tuple of (mimicked joint index, gear
  // ratio, offset)
  std::map<int, std::tuple<int, double, double>> mimicking_joints_;
  std::map<drake::multibody::ModelInstanceIndex, bool> instance_is_identity_;
  std::map<drake::multibody::ModelInstanceIndex, std::map<int, int>>
      instance_full_to_minimal_;
  std::map<drake::multibody::ModelInstanceIndex, std::map<int, int>>
      instance_minimal_to_full_;
  std::map<drake::multibody::ModelInstanceIndex, int> instance_start_index_;
  drake::planning::IrisParameterizationFunction iris_parameterization_function_;

  friend class RobotModel;
};

}  // namespace motion
