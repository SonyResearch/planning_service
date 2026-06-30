/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */
#include "holonomic_mapping.h"

#include "planning_service/common/misc_utils.h"

namespace motion {

namespace {
std::map<drake::multibody::ModelInstanceIndex, drake::planning::DofMask>
FindInstanceDofMasks(const drake::multibody::MultibodyPlant<double>& plant) {
  std::map<drake::multibody::ModelInstanceIndex, drake::planning::DofMask>
      instance_dof_masks;
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    auto model_instance = drake::multibody::ModelInstanceIndex(i);
    int num_positions = plant.num_positions(model_instance);
    if (num_positions > 0) {
      instance_dof_masks.emplace(
          model_instance,
          drake::planning::DofMask::MakeFromModel(plant, model_instance));
    }
  }
  logging::log()->debug(
      "HolonomicMapping: Created dof masks for {} model instances.",
      instance_dof_masks.size());
  return instance_dof_masks;
}

drake::planning::IrisParameterizationFunction
ConstructIrisParameterizationFunction(
    const HolonomicMapping& holonomic_mapping) {
  int dim = holonomic_mapping.minimal_dim();
  bool is_threadsafe = true;
  auto parameterization_double =
      [&holonomic_mapping](const Eigen::VectorXd& x) {
        return holonomic_mapping.Lift(x);
      };
  auto parameterization_autodiff =
      [&holonomic_mapping](const Eigen::VectorX<drake::AutoDiffXd>& x) {
        return holonomic_mapping.LiftAutoDiff(x);
      };
  return drake::planning::IrisParameterizationFunction(
      parameterization_double, parameterization_autodiff, is_threadsafe, dim);
}
}  // namespace

HolonomicMapping::HolonomicMapping(
    const drake::multibody::MultibodyPlant<double>& plant)
    : n_(plant.num_positions()),
      m_(n_ - std::ssize(plant.get_coupler_constraint_specs())),
      is_identity_(n_ == m_),
      instance_dof_masks_(FindInstanceDofMasks(plant)) {
  if (is_identity_) {
    logging::log()->debug("HolonomicMapping: Identity mapping in {}D", n_);
    return;
  }
  logging::log()->info("HolonomicMapping: Creating mapping from {}D to {}D", n_,
                       m_);
  std::vector<int> mimicking_indices;
  auto coupler_constraints = plant.get_coupler_constraint_specs();
  for (const auto& [_, spec] : coupler_constraints) {
    auto joint_index_0 = spec.joint0_index;
    auto joint_index_1 = spec.joint1_index;
    double gear_ratio = spec.gear_ratio;
    double offset = spec.offset;
    int position_0 = plant.get_joint(joint_index_0).position_start();
    int position_1 = plant.get_joint(joint_index_1).position_start();
    logging::log()->info(
        "HolonomicMapping: Coupler Constraint between joints {} and {} with "
        "gear ratio {} "
        "and offset {}, position {} is mimicking position {}",
        plant.get_joint(joint_index_0).name(),
        plant.get_joint(joint_index_1).name(), gear_ratio, offset, position_0,
        position_1);
    DRAKE_THROW_UNLESS(!common::utils::contains(mimicking_indices, position_0));
    mimicking_indices.push_back(position_0);
    mimicking_joints_[position_0] =
        std::make_tuple(position_1, gear_ratio, offset);
  }
  int j = 0;
  for (int i = 0; i < n_; ++i) {
    if (!common::utils::contains(mimicking_indices, i)) {
      full_to_minimal_[i] = j;
      minimal_to_full_[j] = i;
      ++j;
    }
  }
  DRAKE_DEMAND(j == m_);
  // Now, let's generate per model instance mappings
  for (const auto& [model_instance, dof_mask] : instance_dof_masks_) {
    std::map<int, int> full_to_minimal_instance;
    std::map<int, int> minimal_to_full_instance;
    int j = 0;
    int n_instance = dof_mask.count();
    for (int i = 0; i < n_; ++i) {
      if (dof_mask[i]) {
        instance_start_index_[model_instance] = i;
        // We have the mask for this model instance started.
        DRAKE_DEMAND(n_instance == plant.num_positions(model_instance));
        // It has to be contiguous
        for (int k = 0; k < n_instance; ++k) {
          if (!dof_mask[i + k]) {
            throw std::runtime_error(
                fmt::format("HolonomicMapping: i = {}, k = {}, Dof mask for "
                            "model instance {} is not "
                            "contiguous: {}",
                            i, k, plant.GetModelInstanceName(model_instance),
                            dof_mask.to_string()));
          }
          // Let's see if this index is mimicking another index
          if (!common::utils::contains(mimicking_indices, i + k)) {
            full_to_minimal_instance[k] = j;
            minimal_to_full_instance[j] = k;
            ++j;
          }
        }
        break;
      }
    }
    DRAKE_DEMAND(j
                 > 0);  // Otherwise, nothing is masked for this model instance
    instance_full_to_minimal_[model_instance] = full_to_minimal_instance;
    instance_minimal_to_full_[model_instance] = minimal_to_full_instance;
    instance_is_identity_[model_instance] = (n_instance == j);
    if (instance_is_identity_[model_instance]) {
      logging::log()->debug(
          "HolonomicMapping: For model instance {} ({}D), identity mapping",
          plant.GetModelInstanceName(model_instance),
          plant.num_positions(model_instance));
    } else {
      logging::log()->info(
          "HolonomicMapping: For model instance {} ({}D), created mapping from "
          "{}D to {}D",
          plant.GetModelInstanceName(model_instance),
          plant.num_positions(model_instance), n_instance, j);
    }
  }
  logging::log()->info("HolonomicMapping: Created mapping from {}D to {}D", n_,
                       m_);
  iris_parameterization_function_ =
      ConstructIrisParameterizationFunction(*this);
}

Eigen::VectorXd HolonomicMapping::Reduce(
    const Eigen::VectorXd& full_vector) const {
  if (is_identity_) {
    return full_vector;
  }
  DRAKE_THROW_UNLESS(full_vector.size() == n_);
  Eigen::VectorXd reduced_vector = Eigen::VectorXd::Zero(m_);
  for (const auto& [full_index, minimal_index] : full_to_minimal_) {
    reduced_vector(minimal_index) = full_vector(full_index);
  }
  return reduced_vector;
}

Eigen::VectorX<drake::AutoDiffXd> HolonomicMapping::LiftAutoDiff(
    const Eigen::VectorX<drake::AutoDiffXd>& reduced_vector) const {
  if (is_identity_) {
    return reduced_vector;
  }
  DRAKE_THROW_UNLESS(reduced_vector.size() == m_);
  Eigen::VectorX<drake::AutoDiffXd> full_vector =
      Eigen::VectorX<drake::AutoDiffXd>::Zero(n_);
  // First, let's fill in the non-mimicking joints
  for (const auto& [full_index, minimal_index] : full_to_minimal_) {
    full_vector(full_index) = reduced_vector(minimal_index);
  }
  for (const auto& [full_index, tuple] : mimicking_joints_) {
    const auto [mimicked_index, gear_ratio, offset] = tuple;
    full_vector(full_index) = gear_ratio * full_vector(mimicked_index) + offset;
  }
  return full_vector;
}

Eigen::VectorXd HolonomicMapping::Lift(
    const Eigen::VectorXd& reduced_vector) const {
  if (is_identity_) {
    return reduced_vector;
  }
  DRAKE_THROW_UNLESS(reduced_vector.size() == m_);
  Eigen::VectorXd full_vector = Eigen::VectorXd::Zero(n_);
  // First, let's fill in the non-mimicking joints
  for (const auto& [full_index, minimal_index] : full_to_minimal_) {
    full_vector(full_index) = reduced_vector(minimal_index);
  }
  for (const auto& [full_index, tuple] : mimicking_joints_) {
    const auto [mimicked_index, gear_ratio, offset] = tuple;
    full_vector(full_index) = gear_ratio * full_vector(mimicked_index) + offset;
  }
  return full_vector;
}

drake::trajectories::PiecewisePolynomial<double> HolonomicMapping::Lift(
    const drake::trajectories::PiecewisePolynomial<double>& reduced_pp) const {
  if (is_identity_) {
    return reduced_pp;
  }
  throw std::runtime_error(
      "HolonomicMapping::Lift for PiecewisePolynomial is not implemented yet.");
}

drake::trajectories::PiecewisePolynomial<double> HolonomicMapping::Reduce(
    const drake::trajectories::PiecewisePolynomial<double>& full_pp) const {
  if (is_identity_) {
    return full_pp;
  }
  throw std::runtime_error(
      "HolonomicMapping::Reduce for PiecewisePolynomial is not implemented "
      "yet.");
}

int HolonomicMapping::LiftedIndex(int reduced_index) const {
  if (is_identity_) {
    return reduced_index;
  }
  DRAKE_THROW_UNLESS(reduced_index >= 0 && reduced_index < m_);
  return minimal_to_full_.at(reduced_index);
}

int HolonomicMapping::ReducedIndex(int full_index) const {
  if (is_identity_) {
    return full_index;
  }
  DRAKE_THROW_UNLESS(full_index >= 0 && full_index < n_);
  if (mimicking_joints_.count(full_index) > 0) {
    throw std::runtime_error(fmt::format(
        "HolonomicMapping: Full index {} is a mimicking joint and does not "
        "have a corresponding reduced index.",
        full_index));
  }
  return full_to_minimal_.at(full_index);
}

bool HolonomicMapping::IsMimickingJoint(int full_index) const {
  DRAKE_THROW_UNLESS(full_index >= 0 && full_index < n_);
  return mimicking_joints_.count(full_index) > 0;
}

std::vector<int> HolonomicMapping::MimickingJointIndices() const {
  std::vector<int> result;
  for (const auto& [full_index, _] : mimicking_joints_) {
    result.push_back(full_index);
  }
  return result;
}

std::tuple<int, double, double> HolonomicMapping::MimickingJointInfo(
    int full_index) const {
  return mimicking_joints_.at(full_index);
}

Eigen::VectorXd HolonomicMapping::ReduceInstance(
    const drake::multibody::ModelInstanceIndex& instance,
    const Eigen::VectorXd& q_instance_full) const {
  if (is_identity_) {
    return q_instance_full;
  }
  DRAKE_THROW_UNLESS(instance_dof_masks_.count(instance) == 1);
  DRAKE_THROW_UNLESS(q_instance_full.size()
                     == instance_dof_masks_.at(instance).count());
  if (instance_is_identity_.at(instance)) {
    return q_instance_full;
  }
  Eigen::VectorXd q_instance_minimal =
      Eigen::VectorXd::Zero(instance_full_to_minimal_.at(instance).size());
  for (const auto& [full_index, minimal_index] :
       instance_full_to_minimal_.at(instance)) {
    q_instance_minimal(minimal_index) = q_instance_full(full_index);
  }
  return q_instance_minimal;
}

Eigen::VectorXd HolonomicMapping::LiftInstance(
    const drake::multibody::ModelInstanceIndex& instance,
    const Eigen::VectorXd& q_instance_reduced) const {
  if (is_identity_) {
    return q_instance_reduced;
  }
  DRAKE_THROW_UNLESS(instance_dof_masks_.count(instance) == 1);
  DRAKE_THROW_UNLESS(q_instance_reduced.size()
                     == static_cast<Eigen::Index>(
                         instance_minimal_to_full_.at(instance).size()));
  if (instance_is_identity_.at(instance)) {
    return q_instance_reduced;
  }
  Eigen::VectorXd q_instance_lifted =
      Eigen::VectorXd::Zero(instance_dof_masks_.at(instance).count());
  for (const auto& [minimal_index, full_index] :
       instance_minimal_to_full_.at(instance)) {
    q_instance_lifted(full_index) = q_instance_reduced(minimal_index);
  }
  for (const auto& [full_index, tuple] : mimicking_joints_) {
    const auto [mimicked_index, gear_ratio, offset] = tuple;
    // is the mimicked_index part of this instance?
    if (instance_dof_masks_.at(instance)[mimicked_index]) {
      int instance_full_index = full_index - instance_start_index_.at(instance);
      int instance_mimicked_index =
          mimicked_index - instance_start_index_.at(instance);
      q_instance_lifted(instance_full_index) =
          gear_ratio * q_instance_lifted(instance_mimicked_index) + offset;
    }
  }
  return q_instance_lifted;
}

}  // namespace motion
