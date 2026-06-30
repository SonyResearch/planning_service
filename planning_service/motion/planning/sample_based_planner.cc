

/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file sample_based_planner.cc

#include "sample_based_planner.h"
namespace motion {
namespace planning {
namespace ompl {
SampleBasedPlanningContext::SampleBasedPlanningContext(
    const RobotConstraints& robot_constraints)
    : si_ {std::make_shared<ob::SpaceInformation>(
          std::make_shared<RobotStateSpace>(robot_constraints.robot_model()))},
      validity_checker_ {
          std::make_shared<ValidityChecker>(si_, robot_constraints)} {
  si_->getStateSpace()->setup();
  // set state validity checker
  si_->setStateValidityChecker(validity_checker_);
  // set state validity checking resolution
  si_->setStateValidityCheckingResolution(0.001);
  si_->setup();
}

MaxConstraintsClearanceObjective::MaxConstraintsClearanceObjective(
    const SampleBasedPlanningContext& context)
    : ob::MinimaxObjective {context.space_information()},
      context_ {std::make_unique<SampleBasedPlanningContext>(context)} {}

ob::Cost MaxConstraintsClearanceObjective::stateCost(const ob::State* s) const {
  const auto& state_eigen {context_->validity_checker()->OmplStateToEigen(s)};
  const auto& [distance,
               valid] {context_->robot_constraints().CalcPenalty(state_eigen)};
  if (!valid) {
    // return infinite cost if the state is invalid
    return ob::Cost(std::numeric_limits<double>::infinity());
  }
  return ob::Cost(distance);
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
