/*
  Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file constraints_adapter.h

#pragma once

#include <drake/geometry/rgba.h>

#include "robot_model.h"

namespace motion {

/** A pair of collision groups. */
using collision_group_pair_t = std::array<std::string, 2>;

/** Parameters used for minimum value penalty function.
The minimum value penalty function is defined as:
psi(x) = m * (exp(-gamma*x) - exp(-gamma*x0))/(1 - exp(-gamma*x0)), if x < x_0
psi(x) = 0, otherwise
where
- m > 0 is the value of the penalty function at x = 0
- gamma is the decay rate of the penalty function
- x_0 > 0 is the threshold value of the penalty function
The function is convex for gamma > 0 and concave for gamma < 0. At gamma = 0,
the function is not defined, but the limit as gamma -> 0 is psi(x) = M (1 -
x/x_0), which is a linear function of x.
*/
struct MinimumValuePenaltyParams {
  double m {1.0};
  double gamma {1.0};
  double x0 {0.01};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(m));
    a->Visit(DRAKE_NVP(gamma));
    a->Visit(DRAKE_NVP(x0));
  }
};

/**
 * Serializable struct for padding info. Padding is defined between a pair of
 * bodies. Padding of D means the collision checker will report collision if
 * separation is less than D.
 */
struct PaddingAdapter {
  // The pair of groups to be padded
  collision_group_pair_t pair {"", ""};
  // Padding distance in meters
  double distance {0.0};

  PaddingAdapter(const collision_group_pair_t& group_pair_in,
                 double distance_in)
      : pair(group_pair_in), distance(distance_in) {}

  PaddingAdapter() = default;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(pair));
    a->Visit(DRAKE_NVP(distance));
  }
};

/**
 * Serializable struct for a single collision checker.
 */
struct CollisionCheckerAdapter {
  /** The names of the groups to be filtered. All the collision caused by the
   * bodies in theses groups will be ignored by the collision checker. */
  std::optional<std::vector<std::string>> filtered_groups {std::nullopt};

  /** Filtered pairs of groups. All the collision between the bodies in the pair
  will be ignored by the collision checker.
  @warn: two bodies within a group are not filtered by this.
  */
  std::optional<std::vector<collision_group_pair_t>> filtered_pairs {
      std::nullopt};

  /** Padding information as a vector of PaddingAdapter. */
  std::optional<std::vector<PaddingAdapter>> paddings {std::nullopt};

  /** Parameters used for minimum value penalty function. */
  std::optional<MinimumValuePenaltyParams> minimum_value_penalty_params {
      std::nullopt};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(filtered_groups));
    a->Visit(DRAKE_NVP(filtered_pairs));
    a->Visit(DRAKE_NVP(paddings));
    a->Visit(DRAKE_NVP(minimum_value_penalty_params));
  }
};

/**
 * Serializable struct for a single position constraint. Check the
 * documentation for the PositionConstraint class for more info:
 * https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_position_constraint.html
 */
struct PositionConstraintAdapter {
  /** The names of the frames in which the constraint is specified. */
  std::string frame_A {""};
  std::string frame_B {""};
  Eigen::Vector3d position_BQ {Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_AQ_lower {Eigen::Vector3d::Zero()};
  Eigen::Vector3d position_AQ_upper {Eigen::Vector3d::Zero()};

  /** the color of the box representing the box constraint in meshcat */
  std::optional<drake::geometry::Rgba> color {std::nullopt};

  /** Parameters used for minimum value penalty function. */
  std::optional<MinimumValuePenaltyParams> minimum_value_penalty_params {
      std::nullopt};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(frame_A));
    a->Visit(DRAKE_NVP(frame_B));
    a->Visit(DRAKE_NVP(position_BQ));
    a->Visit(DRAKE_NVP(position_AQ_lower));
    a->Visit(DRAKE_NVP(position_AQ_upper));
    a->Visit(DRAKE_NVP(color));
    a->Visit(DRAKE_NVP(minimum_value_penalty_params));
  }
};

/**
 * Serializable struct for a single angle between vectors constraint. Check the
 * documentation for the AngleBetweenVectorsConstraint class for more info:
 * https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_angle_between_vectors_constraint.html
 */
struct AngleBetweenVectorsConstraintAdapter {
  std::string frame_A {""}, frame_B {""};
  Eigen::Vector3d a_A {Eigen::Vector3d::Zero()};
  Eigen::Vector3d b_B {Eigen::Vector3d::Zero()};
  double angle_lower {0.0};
  double angle_upper {0.0};

  /** Parameters used for minimum value penalty function. */
  std::optional<MinimumValuePenaltyParams> minimum_value_penalty_params {
      std::nullopt};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(frame_A));
    a->Visit(DRAKE_NVP(frame_B));
    a->Visit(DRAKE_NVP(a_A));
    a->Visit(DRAKE_NVP(b_B));
    a->Visit(DRAKE_NVP(angle_lower));
    a->Visit(DRAKE_NVP(angle_upper));
    a->Visit(DRAKE_NVP(minimum_value_penalty_params));
  }
};

/**
 Serializable struct for a joint position constraint.
 */
struct JointPositionsBoxConstraintAdapter {
  /**
  The name of the multibody entity as in the drake model directives.
  */
  std::string multibody_entity_name {""};

  /**
   vector of lower bound values for the joint positions.
   */
  Eigen::VectorXd lower_bounds {Eigen::VectorXd::Zero(0)};

  /**
  vector of upper bound values for the joint positions.
   */
  Eigen::VectorXd upper_bounds {Eigen::VectorXd::Zero(0)};

  /** Parameters used for minimum value penalty function. */
  std::optional<MinimumValuePenaltyParams> minimum_value_penalty_params {
      std::nullopt};

  /**
  Serialization function.
  */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(multibody_entity_name));
    a->Visit(DRAKE_NVP(lower_bounds));
    a->Visit(DRAKE_NVP(upper_bounds));
  }
};

/**
 * Container for adapters for all constraints to be enforced for a given plan.
 * Currently supported:
 *  - PositionConstraint
 *  - AngleBetweenVectorsConstraint
 *  - CollisionChecker
 */
struct ConstraintsAdapter {
  std::string plan_name {""};
  std::optional<std::vector<PositionConstraintAdapter>> position_constraints {
      std::nullopt};
  std::optional<std::vector<AngleBetweenVectorsConstraintAdapter>>
      angle_constraints {std::nullopt};
  std::optional<CollisionCheckerAdapter> collision_checker {std::nullopt};
  /** Joint Position Constraints*/
  std::optional<std::vector<JointPositionsBoxConstraintAdapter>>
      joint_position_box_constraints {std::nullopt};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(plan_name));
    a->Visit(DRAKE_NVP(position_constraints));
    a->Visit(DRAKE_NVP(angle_constraints));
    a->Visit(DRAKE_NVP(collision_checker));
    a->Visit(DRAKE_NVP(joint_position_box_constraints));
  }
};

/**
 * Container of data structure for constructing composite robot constraints
 */
struct CompositeConstraintsAdapter {
  std::string composite_plan_name {""};
  std::vector<ConstraintsAdapter> constraints_vec;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(composite_plan_name));
    a->Visit(DRAKE_NVP(constraints_vec));
  }
};

/** Adds joint limit constraints to a constraints Adapter. */
ConstraintsAdapter AddJointLimitToConstraintsAdapter(
    const RobotModel& robot_model,
    const ConstraintsAdapter& constraints_adapter);

}  // namespace motion
