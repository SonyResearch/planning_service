/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file validity_checker.h
#pragma once

#include <ompl/base/DiscreteMotionValidator.h>
#include <ompl/base/SpaceInformation.h>
#include <ompl/base/objectives/MinimaxObjective.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/config.h>
#include <ompl/geometric/SimpleSetup.h>

#include <algorithm>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "state_space.h"
namespace motion {
namespace planning {
namespace ompl {
class ValidityChecker : public ob::StateValidityChecker {
 public:
  ValidityChecker(const ob::SpaceInformationPtr& sip,
                  const RobotConstraints& robot_constraints)
      : ob::StateValidityChecker {sip},
        robot_constraints_ {robot_constraints} {}

  bool isValid(const Eigen::VectorXd& state_eigen,
               const bool verbose = false) const;

  bool isValidEdge(const Eigen::VectorXd& start,
                   const Eigen::VectorXd& goal) const;

  void ValidatePlanningProblemOrThrow(const Eigen::VectorXd& start,
                                      const Eigen::VectorXd& goal) const;

  /** Checks if the planning problem is valid. It only checks the size of the
  start and goal states and their validity. */
  bool IsPlanningProblemValid(const Eigen::VectorXd& start,
                              const Eigen::VectorXd& goal) const;

  const ob::StateSpacePtr& state_space() const {
    return si_->getStateSpace();
  }

  const RobotConstraints& robot_constraints() const {
    return robot_constraints_;
  }

  /** Checks the validity of a vector of configurations and returns a vector of
   * integers, where 0 is valid and 1 is invalid. The order of the output vector
   * is the same as the input vector. */
  std::vector<uint8_t> AreStatesValid(
      const std::vector<Eigen::VectorXd>& states,
      const bool verbose = false) const;

  /** Checks the validity of a the configurations given as the columns of a
   * matrix of configurations and returns a vector of integers, where 0 is valid
   * and 1 is invalid. The order of the output vector is the same as the columns
   * of the input matrix. */
  std::vector<uint8_t> AreStatesValid(const Eigen::MatrixXd& states,
                                      const bool verbose = false) const;

  Eigen::VectorXd OmplStateToEigen(const ob::State* state) const;

 private:
  bool isValid(const ob::State* state) const override;
  bool isValidEdge(const ob::State* start, const ob::State* goal) const;
  // robot constraints
  const RobotConstraints& robot_constraints_;
  std::unique_ptr<int> thread_counter_ {std::make_unique<int>(0)};
};

}  // namespace ompl
}  // namespace planning
}  // namespace motion
