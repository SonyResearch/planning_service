/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file state_space.cc

#include "state_space.h"

#include <ompl/tools/config/MagicConstants.h>

#include <cmath>
#include <cstring>

#include <boost/math/constants/constants.hpp>

namespace motion {
namespace planning {
namespace ompl {

RobotStateSpace::RobotStateSpace(const motion::RobotModel& robot_model)
    : ob::CompoundStateSpace() {
  setName("RobotStateSpace" + getName());
  type_ = 98;  // must be above 14
  const auto& upper_limits {robot_model.plant().GetPositionUpperLimits()};
  // get lower limit vector from robot_model_
  const auto& lower_limits {robot_model.plant().GetPositionLowerLimits()};
  // Get the holonomic mapping
  const auto& hm = robot_model.holonomic_mapping();
  const auto lower_limit_reduced = hm.Reduce(lower_limits);
  const auto upper_limit_reduced = hm.Reduce(upper_limits);
  for (int i = 0; i < hm.minimal_dim(); ++i) {
    // if i is in fully_revolute_joint_indices, add SO2 subspace
    // else add R^n subspace and joint_limits[i] as bounds
    logging::log()->debug(
        "RobotStateSpace: Adding subspace for joint {} (full index {}) with "
        "limits [{}, {}]",
        i, hm.LiftedIndex(i), lower_limit_reduced[i], upper_limit_reduced[i]);
    int i_lifted = hm.LiftedIndex(i);
    if (std::find(robot_model.continuous_revolute_joint_indices().begin(),
                  robot_model.continuous_revolute_joint_indices().end(),
                  i_lifted)
        != robot_model.continuous_revolute_joint_indices().end()) {
      auto subspace {std::make_shared<ob::SO2StateSpace>()};
      addSubspace(subspace, 1.0);
    } else {
      auto subspace {std::make_shared<ob::RealVectorStateSpace>(1)};
      subspace->setBounds(lower_limit_reduced[i], upper_limit_reduced[i]);
      addSubspace(subspace, 1.0);
    }
  }
  lock();
}

ob::State* RobotStateSpace::allocState() const {
  auto* state = new StateType();
  allocStateComponents(state);
  return state;
}

void RobotStateSpace::freeState(ob::State* state) const {
  ob::CompoundStateSpace::freeState(state);
}
}  // namespace ompl
}  // namespace planning
}  // namespace motion
