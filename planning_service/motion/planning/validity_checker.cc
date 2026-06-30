/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file validity_checker.cc
#include "validity_checker.h"

namespace motion {
namespace planning {
namespace ompl {

bool ValidityChecker::isValid(const ob::State* state) const {
  return isValid(OmplStateToEigen(state));
}

bool ValidityChecker::isValid(const Eigen::VectorXd& state_eigen,
                              const bool verbose) const {
  // throw if the state is not the right size
  if (state_eigen.size() != si_->getStateDimension()) {
    logging::log()->error("State is of incorrect size (got: {}, expected: {})",
                          state_eigen.size(), si_->getStateDimension());
    return false;
  }
  CheckSatisfiedOptions options;
  options.verbose = verbose;
  *thread_counter_ += 1;
  // thread_counter_ is not thread-safe, so store it locally to pass to the
  // constraints check
  const auto thread_num {*thread_counter_ % robot_constraints_.num_threads()};
  const auto& is_valid {
      robot_constraints_.CheckSatisfied(state_eigen, thread_num, options)};
  // now it is safe to increment the thread counter
  *thread_counter_ = thread_num;
  return is_valid;
}

std::vector<uint8_t> ValidityChecker::AreStatesValid(
    const Eigen::MatrixXd& states_eigen, const bool verbose) const {
  std::vector<uint8_t> is_valid(states_eigen.cols(), 0);
  CheckSatisfiedOptions options;
  options.verbose = verbose;
  omp_set_num_threads(robot_constraints_.num_threads());
#pragma omp parallel for shared(is_valid)
  for (int i = 0; i < states_eigen.cols(); ++i) {
    int thread_num = omp_get_thread_num();
    is_valid[i] = robot_constraints_.CheckSatisfied(states_eigen.col(i),
                                                    thread_num, options);
  }
  return is_valid;
}

std::vector<uint8_t> ValidityChecker::AreStatesValid(
    const std::vector<Eigen::VectorXd>& states_eigen,
    const bool verbose) const {
  // wrap around the Eigen::MatrixXd version
  Eigen::MatrixXd states_eigen_matrix(states_eigen[0].rows(),
                                      states_eigen.size());
  for (size_t i = 0; i < states_eigen.size(); ++i) {
    states_eigen_matrix.col(i) = states_eigen[i];
  }
  return AreStatesValid(states_eigen_matrix, verbose);
}

bool ValidityChecker::isValidEdge(const Eigen::VectorXd& start,
                                  const Eigen::VectorXd& goal) const {
  // throw if the state is not the right sizew
  const auto is_valid {robot_constraints_.CheckSatisfiedEdge(start, goal)};
  return is_valid;
}

bool ValidityChecker::isValidEdge(const ob::State* start,
                                  const ob::State* goal) const {
  return isValidEdge(OmplStateToEigen(start), OmplStateToEigen(goal));
}

Eigen::VectorXd ValidityChecker::OmplStateToEigen(
    const ob::State* state) const {
  std::vector<double> state_vec;
  si_->getStateSpace()->copyToReals(state_vec, state);
  return Eigen::Map<Eigen::VectorXd>(state_vec.data(), state_vec.size());
}

void ValidityChecker::ValidatePlanningProblemOrThrow(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal) const {
  DRAKE_THROW_UNLESS(IsPlanningProblemValid(start, goal));
}

bool ValidityChecker::IsPlanningProblemValid(
    const Eigen::VectorXd& start, const Eigen::VectorXd& goal) const {
  // check that both the start and goal are valid
  const bool verbose {true};
  if (!isValid(start, verbose)) {
    logging::log()->error(
        "ValidityChecker:IsPlanningProblemValid: Start state is not valid!");
    return false;
  }
  if (!isValid(goal, verbose)) {
    logging::log()->error(
        "ValidityChecker:IsPlanningProblemValid: Goal state is not valid!");
    return false;
  }

  return true;
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
