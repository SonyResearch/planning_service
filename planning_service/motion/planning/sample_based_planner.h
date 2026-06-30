/*
 * Copyright © 2024 Dexai Robotics. All rights reserved.
 */

/// @file sample_based_planner.h
#pragma once

#include "validity_checker.h"

namespace og = ::ompl::geometric;

namespace motion {
namespace planning {
namespace ompl {

using conf_edge_t = std::pair<Eigen::VectorXd, Eigen::VectorXd>;
using conf_edge_vec_t = std::vector<conf_edge_t>;

class SampleBasedPlanningContext {
 public:
  SampleBasedPlanningContext(const RobotConstraints& robot_constraints);

  const RobotConstraints& robot_constraints() const {
    return validity_checker_->robot_constraints();
  }

  const ob::StateSpacePtr& state_space() const {
    return si_->getStateSpace();
  }

  const std::shared_ptr<ob::SpaceInformation>& space_information() const {
    return si_;
  }

  const std::shared_ptr<ValidityChecker>& validity_checker() const {
    return validity_checker_;
  }

 private:
  std::shared_ptr<ob::SpaceInformation> si_;
  std::shared_ptr<ValidityChecker> validity_checker_;
};

class MaxConstraintsClearanceObjective : public ob::MinimaxObjective {
 public:
  explicit MaxConstraintsClearanceObjective(
      const SampleBasedPlanningContext& context);

  ob::Cost stateCost(const ob::State* s) const;

 private:
  std::unique_ptr<SampleBasedPlanningContext> context_;
};

}  // namespace ompl
}  // namespace planning
}  // namespace motion

/** Formatter for ompl::base::Cost */
template <>
struct fmt::formatter<ompl::base::Cost> : fmt::formatter<double> {
  template <typename FormatContext>
  auto format(const ompl::base::Cost& cost, FormatContext& ctx) const {
    return fmt::formatter<double>::format(cost.value(), ctx);
  }
};
