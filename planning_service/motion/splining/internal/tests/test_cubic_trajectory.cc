#include <drake/solvers/constraint.h>
#include <gtest/gtest.h>

#include "planning_service/common/logging.h"
#include "planning_service/motion/splining/internal/cubic_trajectory.h"

namespace motion {
namespace splining {
namespace internal {

TEST(CubicTrajectory, OneDimensional) {
  auto start_position = Eigen::VectorXd::Zero(1);
  auto start_velocity = Eigen::VectorXd::Ones(1);
  auto end_position = Eigen::VectorXd::Ones(1);
  auto end_velocity = Eigen::VectorXd::Ones(1);
  auto acceleration_bound = Eigen::VectorXd::Ones(1);
  auto velocity_bound = Eigen::VectorXd::Ones(1);
  auto result = CalcCubicTrajectoryMinimumTime(
      start_position, start_velocity, end_position, end_velocity,
      acceleration_bound, velocity_bound);
  EXPECT_TRUE(result.has_value());
  EXPECT_NEAR(result.value(), 1.0, 1e-6);
}

TEST(CubicTrajectory, MinimumTime) {
  auto start_position = Eigen::VectorXd::Zero(1);
  auto start_velocity = Eigen::VectorXd::Zero(1);
  auto end_position = Eigen::VectorXd::Zero(1);
  auto end_velocity = Eigen::VectorXd::Zero(1);
  auto acceleration_bound = Eigen::VectorXd::Ones(1);
  auto velocity_bound = Eigen::VectorXd::Ones(1);
  double minimum_time = 0.002;
  auto result = CalcCubicTrajectoryMinimumTime(
      start_position, start_velocity, end_position, end_velocity,
      acceleration_bound, velocity_bound, minimum_time);
  EXPECT_TRUE(result.has_value());
  EXPECT_DOUBLE_EQ(result.value(), minimum_time);
}

TEST(CubicTrajectory, TestVsCheckSatisfied) {
  auto start_position = Eigen::Vector3d {0.0, 0.0, 0.0};
  auto start_velocity = Eigen::Vector3d {-1.0, 1.0, -2.0};
  auto end_position = Eigen::Vector3d {0.0, 1.0, 1.0};
  auto end_velocity = Eigen::Vector3d {0.5, 0.5, 1.0};
  auto acceleration_bound = Eigen::Vector3d {1.0, 1.0, 1.0};
  auto velocity_bound = Eigen::Vector3d {2.0, 2.0, 2.0};
  auto result = CalcCubicTrajectoryMinimumTime(
      start_position, start_velocity, end_position, end_velocity,
      acceleration_bound, velocity_bound);
  EXPECT_TRUE(result.has_value());
  logging::log()->info(
      "TimeOptimalSpliner::CalcOptimalCubicPath: minimum time: {}",
      result.value());
  // Let's do a linear search
  double h_min = 0.01;
  double h_search = 0.3;
  double h_max = 10.0;
  drake::solvers::BoundingBoxConstraint velocity_bounds(-velocity_bound,
                                                        velocity_bound);
  drake::solvers::BoundingBoxConstraint acceleration_bounds(-acceleration_bound,
                                                            acceleration_bound);
  double h = h_min;
  while (h < h_max) {
    auto q_1 = start_position + start_velocity * h / 3;
    auto q_2 = end_position - end_velocity * h / 3;
    // Check if the velocity bounds are satisfied
    if (!velocity_bounds.CheckSatisfied(3 / h * (q_2 - q_1))) {
      h += h_search;
      continue;
    }
    // Check if the acceleration bounds are satisfied
    if (!acceleration_bounds.CheckSatisfied(
            6 / h / h * (q_2 - 2 * q_1 + start_position))) {
      h += h_search;
      continue;
    }
    if (!acceleration_bounds.CheckSatisfied(6 / h / h
                                            * (end_position - 2 * q_2 + q_1))) {
      h += h_search;
      continue;
    }
    logging::log()->info(
        "TimeOptimalSpliner::CalcOptimalCubicPath at {} All bounds satisfied",
        h);
    break;
  }
  EXPECT_NEAR(h, result.value(), h_search);
  // log the control points
  auto q_1 = start_position + start_velocity * h / 3;
  auto q_2 = end_position - end_velocity * h / 3;
  logging::log()->info("q_0: {}, \nq_1: {}, \nq_2: {}, \nq_3: {}",
                       start_position.transpose(), q_1.transpose(),
                       q_2.transpose(), end_position.transpose());
}

TEST(CubicTrajectory, CubicTrajectoryFromControlPoints) {
  Eigen::Vector3d q0 {0.0, 0.0, 0.0};
  Eigen::Vector3d q1 {1.0, -1.0, 1.0};
  Eigen::Vector3d q2 {2.0, 2.0, -2.0};
  Eigen::Vector3d q3 {3.0, 3.0, 3.0};
  double h = 3.0;
  auto piecewise_polynomial =
      CubicTrajectoryFromControlPoints(q0, q1, q2, q3, h);
  EXPECT_TRUE(piecewise_polynomial.value(0.0).isApprox(q0, 1e-6));
  EXPECT_TRUE(piecewise_polynomial.value(h).isApprox(q3, 1e-6));
  EXPECT_NEAR(piecewise_polynomial.end_time(), h, 1e-6);
}

}  // namespace internal
}  // namespace splining
}  // namespace motion
