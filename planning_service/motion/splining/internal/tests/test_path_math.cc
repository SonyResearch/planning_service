// File: test_path_math.cc
#include <drake/common/yaml/yaml_io.h>
#include <gtest/gtest.h>

#include "planning_service/common/logging.h"
#include "planning_service/motion/splining/internal/path_math.h"

namespace motion {
namespace splining {
namespace internal {

TEST(PathMath, FindBestMatchTime) {
  Eigen::MatrixXd samples(1, 3);
  Eigen::VectorXd times(3);
  samples << 0, 1, 0;
  times << 0, 1, 2;
  auto path = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
  EXPECT_NEAR(FindBestMatchTime(Eigen::VectorXd::Ones(1), path, 0.0, 2.0, 0.01),
              1.0, 1e-3);
  EXPECT_NEAR(
      FindBestMatchTime(Eigen::VectorXd::Ones(1) * 0.5, path, 0.0, 2.0, 0.01),
      0.5, 1e-3);
  EXPECT_NEAR(
      FindBestMatchTime(Eigen::VectorXd::Ones(1) * 0.5, path, 1.0, 2.0, 0.01),
      1.5, 1e-3);
  // Test throw because of negative search_step_size
  EXPECT_THROW(
      FindBestMatchTime(Eigen::VectorXd::Ones(1), path, 0.0, 2.0, -0.01),
      std::runtime_error);
  // Test throw because of different dimensions
  EXPECT_THROW(
      FindBestMatchTime(Eigen::VectorXd::Ones(2), path, 0.0, 2.0, 0.01),
      std::runtime_error);
  // Test throw because of start_time < path.start_time()
  EXPECT_THROW(
      FindBestMatchTime(Eigen::VectorXd::Ones(1), path, -1.0, 2.0, 0.01),
      std::runtime_error);
  // Test throw because of end_time > path.end_time()
  EXPECT_THROW(
      FindBestMatchTime(Eigen::VectorXd::Ones(1), path, 1.0, 3.0, 0.01),
      std::runtime_error);
}

TEST(PathMath, FindBestMatchTimes1) {
  Eigen::MatrixXd samples(1, 3);
  Eigen::VectorXd times(3);
  samples << 0, 1, 0;
  times << 0, 1, 2;
  auto path = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
  std::vector<Eigen::VectorXd> waypts;
  waypts.push_back(Eigen::VectorXd::Ones(1) * 0.25);
  waypts.push_back(Eigen::VectorXd::Ones(1) * 1.1);
  waypts.push_back(Eigen::VectorXd::Ones(1) * 0.75);
  auto best_times = FindBestMatchTimes(waypts, path, 0.0, 2.0, 0.01, 2e-3);
  EXPECT_NEAR(best_times[0], 0.25, 1e-3);
  EXPECT_NEAR(best_times[1], 1.0, 1e-3);
  EXPECT_NEAR(best_times[2], 1.25, 1e-3);
}

TEST(PathMath, FindBestMatchTimes2) {
  Eigen::MatrixXd samples(1, 3);
  Eigen::VectorXd times(3);
  samples << 0, 1, 0;
  times << 0, 1, 2;
  auto path = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
  std::vector<Eigen::VectorXd> waypts;
  waypts.push_back(Eigen::VectorXd::Ones(1) * 1.25);
  waypts.push_back(Eigen::VectorXd::Ones(1) * 1.35);
  waypts.push_back(Eigen::VectorXd::Ones(1) * -0.5);
  waypts.push_back(Eigen::VectorXd::Ones(1) * -0.5);
  auto best_times = FindBestMatchTimes(waypts, path, 0.0, 2.0, 0.01, 2e-3);
  EXPECT_NEAR(best_times[0], 1.0, 5e-3);
  EXPECT_NEAR(best_times[1], 1.01, 5e-3);
  EXPECT_NEAR(best_times[2], 1.99, 5e-3);
  EXPECT_NEAR(best_times[3], 2.00, 5e-3);
}

TEST(PathMath, CalcCubicPath) {
  std::vector<double> times = {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> samples = {Eigen::VectorXd::Zero(1),
                                          Eigen::VectorXd::Ones(1),
                                          Eigen::VectorXd::Zero(1)};
  auto path = CalcCubicPath(times, samples, Eigen::VectorXd::Ones(1) * 2.0);
  EXPECT_NEAR(path.value(0.0)(0, 0), 0.0, 1e-3);
  EXPECT_NEAR(path.value(1.0)(0, 0), 1.0, 1e-3);
  EXPECT_NEAR(path.value(2.0)(0, 0), 0.0, 1e-3);
  EXPECT_NEAR(path.EvalDerivative(0.0, 1)(0), 2.0, 1e-3);
}

TEST(PathMath, SmoothWaypoints) {
  std::vector<double> times = {0.0, 1.0, 2.0, 2.1, 3.0, 5.0};
  auto one = Eigen::VectorXd::Ones(1);
  std::vector<Eigen::VectorXd> samples = {one * 0.1, one * 1.0, one * 2.0,
                                          one * 1.9, one * 3.0, one * 5.5};
  auto tangent = Eigen::VectorXd::Ones(1) * 2.0;
  auto smooth_waypts =
      SmoothWaypoints(times, samples, tangent, Eigen::VectorXd::Ones(1) * 0.05);
  // The first sample should be the same
  EXPECT_NEAR(smooth_waypts[0](0, 0), samples[0](0, 0), 1e-3);
  // Log the result
  for (int i = 0; i < std::ssize(smooth_waypts); ++i) {
    logging::log()->info("original: {}, \t smoothed: {}",
                         samples[i].transpose(), smooth_waypts[i].transpose());
  }
}

TEST(TestPath2D, CalcTrailingPathTowardWaypts) {
  std::vector<double> times = {0.0, 1.0, 2.0};
  auto sample_1 = Eigen::Vector2d(0.0, 0.0);
  auto sample_2 = Eigen::Vector2d(1.0, 1.0);
  auto sample_3 = Eigen::Vector2d(2.0, 0.0);
  std::vector<Eigen::MatrixXd> samples = {sample_1, sample_2, sample_3};
  auto original_path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, samples);
  double time_now = 0.5;
  double delta_switch = 0.1;  // all new waypoints are matched after switch time
  std::vector<Eigen::VectorXd> new_waypts;
  new_waypts.push_back(Eigen::Vector2d {0.7, 0.7});
  new_waypts.push_back(Eigen::Vector2d {0.8, 0.8});
  new_waypts.push_back(Eigen::Vector2d {0.9, 0.9});
  auto trailing_path = CalcTrailingPathTowardWaypts(new_waypts, original_path,
                                                    time_now, delta_switch);
  EXPECT_NEAR(trailing_path.start_time(), time_now + delta_switch, 1e-3);
  // The last waypoint will be reached at the end_time of the trailing path
  EXPECT_NEAR(trailing_path.value(trailing_path.end_time())(0, 0), 0.9, 1e-3);
  EXPECT_NEAR(trailing_path.value(trailing_path.end_time())(1, 0), 0.9, 1e-3);
  // Continuity at the start
  EXPECT_NEAR(trailing_path.value(time_now + delta_switch)(0, 0),
              original_path.value(time_now + delta_switch)(0, 0), 1e-3);
  EXPECT_NEAR(trailing_path.EvalDerivative(time_now + delta_switch, 1)(0, 0),
              original_path.EvalDerivative(time_now + delta_switch, 1)(0, 0),
              1e-3);
}

TEST(TestPath2D, MergeTrajectory) {
  std::vector<double> times = {0.0, 1.0, 2.0};
  auto sample_1 = Eigen::Vector2d(0.0, 0.0);
  auto sample_2 = Eigen::Vector2d(1.0, 1.0);
  auto sample_3 = Eigen::Vector2d(2.0, 0.0);
  std::vector<Eigen::MatrixXd> samples = {sample_1, sample_2, sample_3};
  auto original_path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, samples);
  double time_now = 0.5;
  double delta_switch = 0.2;  // all new waypoints are matched after switch time
  std::vector<Eigen::VectorXd> new_waypts;
  new_waypts.push_back(Eigen::Vector2d {0.7, 0.7});
  new_waypts.push_back(Eigen::Vector2d {0.8, 0.8});
  new_waypts.push_back(Eigen::Vector2d {0.9, 0.9});
  auto trailing_path = CalcTrailingPathTowardWaypts(new_waypts, original_path,
                                                    time_now, delta_switch);
  auto merged_traj = MergeTrajectory(original_path, time_now, trailing_path);
  // merged trajectory should start from time_now and end at
  double switch_time = time_now + delta_switch;
  EXPECT_NEAR(merged_traj.start_time(), time_now, 1e-3);
  // Check 0 and 1st degree continuity
  double zero_order_continuity =
      (merged_traj.value(switch_time) - trailing_path.value(switch_time))
          .norm();
  double first_order_continuity =
      (merged_traj.EvalDerivative(switch_time, 1)
       - trailing_path.EvalDerivative(switch_time, 1))
          .norm();
  double eps = 1e-6;
  DRAKE_DEMAND(zero_order_continuity < eps);
  DRAKE_DEMAND(first_order_continuity < eps);
}

TEST(TestPathMath, CompositeBezierCurveToPiecewisePolynomial) {
  // Create a CompositeBezierCurve
  Eigen::MatrixXd control_points_1(2, 3), control_points_2(2, 4),
      control_points_3(2, 3);
  // clang-format off
  control_points_1 <<
   0.0, 1.0, 2.0,
   0.0, 1.0, 0.0;
  control_points_2 <<
    2.0, 3.0, 4.0, 5.0,
    0.0, 1.0, 0.0, 1.0;
  control_points_3 <<
    5.0, 3.0, 1.0,
    1.0, 0.0, -1.0;
  // clang-format on
  drake::trajectories::BezierCurve<double> bezier_curve_1(1.0, 2.0,
                                                          control_points_1);
  drake::trajectories::BezierCurve<double> bezier_curve_2(2.0, 4.0,
                                                          control_points_2);
  drake::trajectories::BezierCurve<double> bezier_curve_3(4.0, 5.0,
                                                          control_points_3);
  std::vector<
      drake::copyable_unique_ptr<drake::trajectories::Trajectory<double>>>
      curves;
  curves.push_back(
      drake::copyable_unique_ptr<drake::trajectories::Trajectory<double>>(
          bezier_curve_1));
  curves.push_back(
      drake::copyable_unique_ptr<drake::trajectories::Trajectory<double>>(
          bezier_curve_2));
  curves.push_back(
      drake::copyable_unique_ptr<drake::trajectories::Trajectory<double>>(
          bezier_curve_3));
  drake::trajectories::CompositeTrajectory<double> composite_bezier_curve(
      curves);
  // Convert to PiecewisePolynomial
  auto ppoly =
      CompositeBezierCurveToPiecewisePolynomial(composite_bezier_curve);
  // Verify that ppoly and composite trajectories have the same start, goal, and
  // sizes
  EXPECT_NEAR(composite_bezier_curve.start_time(), ppoly.start_time(), 1e-6);
  EXPECT_NEAR(composite_bezier_curve.end_time(), ppoly.end_time(), 1e-6);
  EXPECT_EQ(ppoly.rows(), 2);
  EXPECT_EQ(ppoly.cols(), 1);
  EXPECT_EQ(ppoly.get_number_of_segments(),
            composite_bezier_curve.get_number_of_segments());
  // Verify that ppoly and composite trajectories are the same across 100 points
  int num_points = 100;
  for (int i = 0; i < num_points; ++i) {
    double t = composite_bezier_curve.start_time()
               + (composite_bezier_curve.end_time()
                  - composite_bezier_curve.start_time())
                     * i / (num_points - 1);
    auto ppoly_value = ppoly.value(t);
    auto bezier_value = composite_bezier_curve.value(t);
    EXPECT_NEAR(ppoly_value(0, 0), bezier_value(0, 0), 1e-6);
    EXPECT_NEAR(ppoly_value(1, 0), bezier_value(1, 0), 1e-6);
  }
}

TEST(TestPathMath, CombineSequentialSystemTimedTrajectories) {
  std::vector<double> times = {5.0, 6.0, 7.0};
  std::vector<Eigen::MatrixXd> s_matrix_vec;
  s_matrix_vec.push_back(Eigen::VectorXd::Zero(1));
  s_matrix_vec.push_back(Eigen::VectorXd::Ones(1));
  s_matrix_vec.push_back(Eigen::VectorXd::Ones(1) * 2.0);
  // Make a version for s_samples
  std::vector<double> s_samples = {0.0, 1.0, 2.0};
  auto sample_1 = Eigen::Vector2d(0.0, 0.0);
  auto sample_2 = Eigen::Vector2d(1.0, 1.0);
  auto sample_3 = Eigen::Vector2d(2.0, 0.0);
  std::vector<Eigen::MatrixXd> samples = {sample_1, sample_2, sample_3};
  auto original_path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_samples, samples);

  std::vector<Eigen::MatrixXd> additional_samples {sample_3, sample_2,
                                                   sample_1};
  auto additional_path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_samples,
                                                    additional_samples);
  // make q(s)
  const auto& uniform_time = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, s_matrix_vec);

  // Create a vector of pairs of PiecewisePolynomial and time
  std::vector<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                        drake::trajectories::PiecewisePolynomial<double>>>
      trajectories {std::make_pair(original_path, uniform_time),
                    std::make_pair(additional_path, uniform_time)};
  // Combine the trajectories
  auto combined_trajectory =
      motion::splining::internal::CombineSequentialSystemTimedTrajectories(
          trajectories);
  // Check that the combined trajectory is not empty
  EXPECT_FALSE(combined_trajectory.first.empty())
      << "Combined trajectory path is empty";
  EXPECT_FALSE(combined_trajectory.second.empty())
      << "Combined trajectory time is empty";
  // Check that the start matches the first configuration of the first
  // trajectory
  EXPECT_TRUE(combined_trajectory.first.value(0).isApprox(sample_1))
      << "Start does not match the first configuration of the first trajectory";
  // Check that the end matches the last configuration of the second
  // trajectory
  EXPECT_TRUE(combined_trajectory.first
                  .value(original_path.end_time() + additional_path.end_time())
                  .isApprox(sample_1))
      << "End does not match the last configuration of the second trajectory";
  // Check that the value of the combined trajectory at the time the first
  // trajectory ends is the same as the final configuration of the first
  // trajectory
  EXPECT_TRUE(combined_trajectory.first.value(original_path.end_time())
                  .isApprox(sample_3))
      << "Value at the end of the first trajectory does not match";
}

TEST(TestPathMath, MakeUniformTimingForPath) {
  std::vector<double> s_samples = {0.5, 1.0, 2.5};
  auto sample_1 = Eigen::Vector2d(0.0, 0.0);
  auto sample_2 = Eigen::Vector2d(1.0, 1.0);
  auto sample_3 = Eigen::Vector2d(2.0, 0.0);
  std::vector<Eigen::MatrixXd> samples = {sample_1, sample_2, sample_3};
  auto original_path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_samples, samples);
  double start_time = 0.7;
  auto uniform_timed_path_zero =
      motion::splining::internal::MakeUniformTimingForPath(original_path,
                                                           start_time);
  // Let's construct a path parameterized in time
  auto ppt = drake::trajectories::PathParameterizedTrajectory<double>(
      original_path, uniform_timed_path_zero);
  // Check that the start time is 0
  EXPECT_NEAR(ppt.start_time(), start_time, 1e-6);
  // Check that the end time is the same as the original path
  EXPECT_NEAR(
      ppt.end_time(),
      start_time + original_path.end_time() - original_path.start_time(), 1e-6);
  // The value q(t = start time) must be sample_1
  EXPECT_TRUE(ppt.value(start_time).isApprox(sample_1));
  // The value q(t = end time) must be sample_3
  EXPECT_TRUE(ppt.value(ppt.end_time()).isApprox(sample_3));
}

TEST(TestPathMath, RemoveConstantPrepend_WithPrepend) {
  // Segments [0,1] and [1,2] are constant (slope 0), segment [2,3] is linear.
  std::vector<double> times = {0.0, 1.0, 2.0, 3.0};
  std::vector<Eigen::MatrixXd> samples = {
      Eigen::VectorXd::Constant(1, 5.0), Eigen::VectorXd::Constant(1, 5.0),
      Eigen::VectorXd::Constant(1, 5.0), Eigen::VectorXd::Constant(1, 7.0)};
  auto pp = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
  auto result = RemoveConstantPrepend(pp);
  EXPECT_NEAR(result.start_time(), 2.0, 1e-9);
  EXPECT_NEAR(result.end_time(), 3.0, 1e-9);
  EXPECT_NEAR(result.value(2.0)(0, 0), 5.0, 1e-9);
  EXPECT_NEAR(result.value(3.0)(0, 0), 7.0, 1e-9);
}

TEST(TestPathMath, RemoveConstantPrepend_NoPrepend) {
  // No constant prepend: all segments are non-constant.
  std::vector<double> times = {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> samples = {Eigen::VectorXd::Constant(1, 0.0),
                                          Eigen::VectorXd::Constant(1, 1.0),
                                          Eigen::VectorXd::Constant(1, 3.0)};
  auto pp = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
  auto result = RemoveConstantPrepend(pp);
  EXPECT_NEAR(result.start_time(), pp.start_time(), 1e-9);
  EXPECT_NEAR(result.end_time(), pp.end_time(), 1e-9);
}

TEST(TestPathMath, RemoveConstantPrepend_AllConstant) {
  // All segments are constant: return the original unchanged.
  std::vector<double> times = {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> samples = {Eigen::VectorXd::Constant(1, 3.0),
                                          Eigen::VectorXd::Constant(1, 3.0),
                                          Eigen::VectorXd::Constant(1, 3.0)};
  auto pp = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
  auto result = RemoveConstantPrepend(pp);
  EXPECT_NEAR(result.start_time(), pp.start_time(), 1e-9);
  EXPECT_NEAR(result.end_time(), pp.end_time(), 1e-9);
}

}  // namespace internal
}  // namespace splining
}  // namespace motion
