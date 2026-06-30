#include <gtest/gtest.h>

#include "planning_service_client/trajectories.h"

namespace planning_service_client {

TEST(PiecewisePolynomial, basics) {
  std::vector<double> breaks {0.0, 1.0, 2.0, 3.0, 4.5};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(3.0 * Eigen::MatrixXd::Ones(3, 2));
  coefficients_vec.push_back(2.0 * Eigen::MatrixXd::Ones(3, 3));
  coefficients_vec.push_back(-1.0 * Eigen::MatrixXd::Ones(3, 2));
  PiecewisePolynomial dut(coefficients_vec, breaks);
  EXPECT_EQ(dut.dim(), 3);
  EXPECT_EQ(dut.degree(), 2);
  EXPECT_EQ(dut.breaks().size(), 5);
  // Test FindSegment
  EXPECT_EQ(dut.FindSegment(-1.0), 0);
  EXPECT_EQ(dut.FindSegment(0.0), 0);
  EXPECT_EQ(dut.FindSegment(1.0), 1);
  EXPECT_EQ(dut.FindSegment(0.5), 0);
  EXPECT_EQ(dut.FindSegment(1.9), 1);
  EXPECT_EQ(dut.FindSegment(3.0), 3);
  EXPECT_EQ(dut.FindSegment(3.1), 3);
  EXPECT_EQ(dut.FindSegment(4.6), 3);
  // Test values
  EXPECT_TRUE(dut.Value(0.3).isApprox(Eigen::VectorXd::Zero(3)));
  // At time 1.6, we are 0.6 into the second segment: 1 + 0.6
  EXPECT_TRUE(dut.Value(1.6).isApprox(3.0 * 1.6 * Eigen::VectorXd::Ones(3)));
  // At time 2.3, we are 0.3 into the third segment: 1 + 0.3 + 0.3^2
  EXPECT_TRUE(dut.Value(2.3).isApprox(2.0 * 1.39 * Eigen::VectorXd::Ones(3)));
}

TEST(PiecewisePolynomial, invalids) {
  std::vector<double> breaks {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(2, 1));
  coefficients_vec.push_back(3.0 * Eigen::MatrixXd::Ones(3, 2));
  // Dimensions don't match
  EXPECT_THROW(PiecewisePolynomial dut(coefficients_vec, breaks),
               std::runtime_error);
  coefficients_vec.clear();
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(3.0 * Eigen::MatrixXd::Ones(3, 2));
  EXPECT_NO_THROW(PiecewisePolynomial dut(coefficients_vec, breaks));
  breaks.push_back(3.0);
  // Number of breaks don't match the number of coefficients + 1
  EXPECT_THROW(PiecewisePolynomial dut(coefficients_vec, breaks),
               std::runtime_error);
  // Breaks are not increasing
  breaks = {0.0, 1.2, 1.0};
  EXPECT_THROW(PiecewisePolynomial dut(coefficients_vec, breaks),
               std::runtime_error);
}

TEST(PiecewisePolynomial, Derivatives) {
  std::vector<double> breaks {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(2, 1));
  Eigen::MatrixXd A(2, 4);
  A.row(0) << 1.0, 2.0, 3.0, 4.0;
  A.row(1) << 0.0, 1.0, 0.0, 5.0;
  coefficients_vec.push_back(A);
  PiecewisePolynomial dut(coefficients_vec, breaks);
  // At t = 0.6, the trajectory is a line. The derivative must be zero
  EXPECT_TRUE(dut.EvalDerivative(0.6).isApprox(Eigen::VectorXd::Zero(2)));
  // At time 1.6, trajectory is
  // 1 + 2t + 3t^2 + 4t^3: derivative: 2 + 6t + 12t^2, evaluated at 0.6: 9.92
  // 0 + 1t + 0t^2 + 5t^3: derivative: 1 + 15t^2, evaluated at 0.6: 6.4
  EXPECT_TRUE(dut.EvalDerivative(1.6).isApprox(Eigen::Vector2d(9.92, 6.4)));
  // Alternative way to compute the derivative
  auto dut_derivative = dut.Derivative();
  EXPECT_TRUE(dut_derivative.Value(0.6).isApprox(Eigen::VectorXd::Zero(2)));
  EXPECT_TRUE(dut_derivative.Value(1.6).isApprox(Eigen::Vector2d(9.92, 6.4)));
}

TEST(PiecewisePolynomial, ToProtoFromProto) {
  std::vector<double> breaks {0.0, 1.0, 2.0, 3.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(3, 7));
  coefficients_vec.push_back(3.0 * Eigen::MatrixXd::Ones(3, 1));
  PiecewisePolynomial dut(coefficients_vec, breaks);
  auto msg = ToProto(dut);
  EXPECT_EQ(msg.dim(), 3);
  EXPECT_EQ(msg.breaks().size(), 4);
  EXPECT_EQ(msg.coeffs().size(), 9);
  // Now let's go back to object
  auto dut_deserialized = FromProto<PiecewisePolynomial>(msg);
  EXPECT_EQ(3, dut_deserialized.dim());
  EXPECT_EQ(6, dut_deserialized.degree());
  EXPECT_EQ(4, dut_deserialized.breaks().size());
  EXPECT_TRUE(dut.IsCloseTo(dut_deserialized, 1e-6));
}

TEST(PiecewisePolynomial, constant) {
  Eigen::VectorXd conf(3);
  conf << 1.0, 2.0, 3.0;
  auto dut = PiecewisePolynomial::Constant(conf);
  EXPECT_EQ(dut.dim(), 3);
  EXPECT_EQ(dut.degree(), 0);
  EXPECT_EQ(dut.breaks().size(), 2);
  EXPECT_TRUE(dut.Value(0.0).isApprox(conf));
  EXPECT_TRUE(dut.Value(1.0).isApprox(conf));
  // Derivative is zero at any time
  EXPECT_TRUE(dut.EvalDerivative(0.5).isApprox(Eigen::VectorXd::Zero(3)));
}

TEST(SystemPiecewisePolynomial, ToProtoFromProto) {
  std::vector<double> breaks {0.0, 1.0, 2.0, 3.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(3, 7));
  coefficients_vec.push_back(3.0 * Eigen::MatrixXd::Ones(3, 1));
  SystemPiecewisePolynomial dut;
  dut["left"] = PiecewisePolynomial(coefficients_vec, breaks);
  // reverse the coefficients
  std::reverse(coefficients_vec.begin(), coefficients_vec.end());
  dut["right"] = PiecewisePolynomial(coefficients_vec, breaks);
  // Test serialization and deserialization
  auto msg = ToProto(dut);
  auto dut_deserialized = FromProto<SystemPiecewisePolynomial>(msg);
  EXPECT_TRUE(dut["left"].IsCloseTo(dut_deserialized["left"], 1e-6));
  EXPECT_TRUE(dut["right"].IsCloseTo(dut_deserialized["right"], 1e-6));
}

TEST(TimedTrajectory, Basics) {
  std::vector<double> breaks {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(3, 3));
  auto path = PiecewisePolynomial(coefficients_vec, breaks);
  // A time scaling trajectory
  std::vector<double> time_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> s_coefficients_vec;
  s_coefficients_vec.push_back(Eigen::Vector3d(0.0, 1.0, 1.0).transpose());
  // s = t + t^2 for t in [0, 1]. Start s: 0, end s: 2.
  auto time_scaling = PiecewisePolynomial(s_coefficients_vec, time_breaks);
  const double global_time_offset {123.45};
  auto dut = TimedTrajectory(path, time_scaling, global_time_offset);
  EXPECT_EQ(dut.start_time(), time_breaks[0]);
  EXPECT_EQ(dut.end_time(), time_breaks[1]);
  EXPECT_DOUBLE_EQ(dut.global_start_time(),
                   time_breaks[0] + global_time_offset);
  EXPECT_DOUBLE_EQ(dut.global_end_time(), time_breaks[1] + global_time_offset);
  EXPECT_EQ(dut.global_time_offset(), global_time_offset);
  // Test values and derivatives
  EXPECT_TRUE(dut.GlobalValue(0.0 + global_time_offset)
                  .isApprox(Eigen::VectorXd::Zero(3)));
  EXPECT_TRUE(dut.Value(0.0).isApprox(Eigen::VectorXd::Zero(3)));
  // At time t = 0.7, s(t) = 0.7 + 0.7^2 = 1.19;
  // The path would be 1 + 0.19 + 0.19^2 = 1.2261
  EXPECT_TRUE(dut.GlobalValue(0.7 + global_time_offset)
                  .isApprox(1.2261 * Eigen::VectorXd::Ones(3)));
  EXPECT_TRUE(dut.Value(0.7).isApprox(1.2261 * Eigen::VectorXd::Ones(3)));
  // s(t) = t + t^2. Therefore sdot = 1 + 2t. At t = 1, sdot = 3.
  // path_prime = 1 + 2s. At s = 2, path_prime = 3.
  EXPECT_TRUE(dut.EvalGlobalDerivative(1.0 + global_time_offset)
                  .isApprox(9.0 * Eigen::VectorXd::Ones(3)));
  EXPECT_TRUE(dut.EvalDerivative(1.0).isApprox(9.0 * Eigen::VectorXd::Ones(3)));
  EXPECT_EQ(dut.global_time_offset(), global_time_offset);

  // Build same trajectory with default global time offset of 0.0
  auto dut_default = TimedTrajectory(path, time_scaling);
  EXPECT_TRUE(dut_default.Value(0.0).isApprox(Eigen::VectorXd::Zero(3)));
  EXPECT_TRUE(
      dut_default.Value(0.7).isApprox(1.2261 * Eigen::VectorXd::Ones(3)));
  EXPECT_TRUE(
      dut_default.EvalDerivative(1.0).isApprox(9.0 * Eigen::VectorXd::Ones(3)));
  EXPECT_TRUE(dut_default.GlobalValue(0.0).isApprox(Eigen::VectorXd::Zero(3)));
  EXPECT_TRUE(
      dut_default.GlobalValue(0.7).isApprox(1.2261 * Eigen::VectorXd::Ones(3)));
  EXPECT_TRUE(dut_default.EvalGlobalDerivative(1.0).isApprox(
      9.0 * Eigen::VectorXd::Ones(3)));
}

TEST(TimedTrajectory, ToProtoFromProto) {
  std::vector<double> breaks {0.0, 1.0, 3.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(3, 7));
  auto path = PiecewisePolynomial(coefficients_vec, breaks);
  // A time scaling trajectory
  std::vector<double> time_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> s_coefficients_vec;
  s_coefficients_vec.push_back(Eigen::Vector3d(0.0, 1.0, 2.0).transpose());
  auto time_scaling = PiecewisePolynomial(s_coefficients_vec, time_breaks);
  const double global_time_offset {123.45};
  auto dut = TimedTrajectory(path, time_scaling, global_time_offset);
  // Test serialization and deserialization
  auto msg = ToProto(dut);
  auto dut_deserialized = FromProto<TimedTrajectory>(msg);
  EXPECT_TRUE(path.IsCloseTo(dut_deserialized.path(), 1e-6));
  EXPECT_TRUE(time_scaling.IsCloseTo(dut_deserialized.time_scaling(), 1e-6));
  EXPECT_EQ(dut_deserialized.global_time_offset(), global_time_offset);
}

TEST(TimedTrajectory, constant) {
  Eigen::VectorXd conf(3);
  conf << 1.0, 2.0, 3.0;
  auto dut = TimedTrajectory::Constant(conf);
  EXPECT_EQ(dut.path().dim(), 3);
  // Time scaling is s(t) = t + 0.0
  EXPECT_EQ(dut.time_scaling().dim(), 1);
  EXPECT_EQ(dut.time_scaling().degree(), 1);
  double test_time = 3.5;
  EXPECT_EQ(dut.time_scaling().Value(test_time)(0), test_time);
  // Choose an epoch time far in the future
  EXPECT_EQ(dut.Value(3e9)(0), conf(0));
  EXPECT_EQ(dut.Value(3e9)(1), conf(1));
  EXPECT_EQ(dut.Value(3e9)(2), conf(2));
}

TEST(SystemTimedTrajectory, ToProtoFromProto) {
  std::vector<double> breaks {0.0, 1.0, 6.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(3, 7));
  auto path = PiecewisePolynomial(coefficients_vec, breaks);
  // A time scaling trajectory
  std::vector<double> time_breaks {0.0, 2.0};
  std::vector<Eigen::MatrixXd> s_coefficients_vec;
  s_coefficients_vec.push_back(Eigen::Vector3d(0.0, 1.0, 1.0).transpose());
  auto time_scaling = PiecewisePolynomial(s_coefficients_vec, time_breaks);
  const double global_time_offset {123.45};
  auto dut_left = TimedTrajectory(path, time_scaling, global_time_offset);
  // reverse the coefficients
  std::reverse(coefficients_vec.begin(), coefficients_vec.end());
  auto dut_right =
      TimedTrajectory(PiecewisePolynomial(coefficients_vec, breaks),
                      time_scaling, global_time_offset);
  SystemTimedTrajectory dut;
  dut["left"] = dut_left;
  dut["right"] = dut_right;
  // Test serialization and deserialization
  auto msg = ToProto(dut);
  auto dut_deserialized = FromProto<SystemTimedTrajectory>(msg);
  EXPECT_TRUE(
      dut["left"].path().IsCloseTo(dut_deserialized["left"].path(), 1e-6));
  EXPECT_TRUE(
      dut["right"].path().IsCloseTo(dut_deserialized["right"].path(), 1e-6));
  EXPECT_TRUE(dut["left"].time_scaling().IsCloseTo(
      dut_deserialized["left"].time_scaling(), 1e-6));
  EXPECT_TRUE(dut["right"].time_scaling().IsCloseTo(
      dut_deserialized["right"].time_scaling(), 1e-6));
  EXPECT_EQ(dut["left"].global_time_offset(),
            dut_deserialized["left"].global_time_offset());
  EXPECT_EQ(dut["right"].global_time_offset(),
            dut_deserialized["right"].global_time_offset());
}

TEST(SystemTimedTrajectory, ConstantSystemTimedTrajectory) {
  Eigen::VectorXd conf_right(3);
  conf_right << 1.0, 2.0, 3.0;
  Eigen::VectorXd conf_left(2);
  conf_left << 4.0, 5.0;
  SystemConf system_conf;
  system_conf["left"] = conf_left;
  system_conf["right"] = conf_right;
  auto dut = ConstantSystemTimedTrajectory(system_conf);
  const double time_test = 5.8;
  EXPECT_TRUE(dut["left"].Value(time_test).isApprox(conf_left, 1e-6));
  EXPECT_TRUE(dut["right"].Value(time_test).isApprox(conf_right, 1e-6));
  // The case we have start time and end_timn
  auto dut2 = ConstantSystemTimedTrajectory(system_conf, 1.0, 3.0);
  EXPECT_TRUE(dut2["left"].Value(1.0).isApprox(conf_left, 1e-6));
  EXPECT_TRUE(dut2["right"].Value(1.0).isApprox(conf_right, 1e-6));
  // And the timing is correct
  EXPECT_DOUBLE_EQ(dut2["left"].start_time(), 1.0);
  EXPECT_DOUBLE_EQ(dut2["left"].end_time(), 3.0);
  EXPECT_DOUBLE_EQ(dut2["right"].start_time(), 1.0);
  EXPECT_DOUBLE_EQ(dut2["right"].end_time(), 3.0);
}

TEST(TimedTrajectory, SliceFromTime_MultipleSegments) {
  // Create a trajectory with multiple segments in path
  std::vector<double> path_breaks {3.0, 4.0, 5.0, 6.0};
  std::vector<Eigen::MatrixXd> path_coeffs;
  // First segment: constant
  Eigen::MatrixXd coeff1(2, 1);
  coeff1 << 1.0, 2.0;
  path_coeffs.push_back(coeff1);
  // Second segment: linear
  Eigen::MatrixXd coeff2(2, 2);
  coeff2 << 1.0, 1.0, 2.0, 0.5;
  path_coeffs.push_back(coeff2);
  // Third segment: quadratic
  Eigen::MatrixXd coeff3(2, 3);
  coeff3 << 2.0, 1.0, 0.5, 2.5, 0.5, 0.25;
  path_coeffs.push_back(coeff3);
  auto path = PiecewisePolynomial(path_coeffs, path_breaks);
  // Time scaling with two segments: linear then quadratic
  // First segment [3.0, 4.5]: s(t) = 3.0 + 1.0*(t - 3.0), at t=4.5: s=4.5
  // Second segment [4.5, 6.0]: s(t) = 4.5 + 0.5*(t - 4.5) + (1/3)*(t - 4.5)^2
  // at t=6.0: s = 4.5 + 0.75 + 0.75 = 6.0
  std::vector<double> time_breaks {3.0, 4.5, 6.0};
  std::vector<Eigen::MatrixXd> time_coeffs;
  Eigen::MatrixXd time_coeff1(1, 2);
  time_coeff1 << 3.0, 1.0;
  time_coeffs.push_back(time_coeff1);
  Eigen::MatrixXd time_coeff2(1, 3);
  time_coeff2 << 4.5, 0.5, 1.0 / 3.0;
  time_coeffs.push_back(time_coeff2);
  auto time_scaling = PiecewisePolynomial(time_coeffs, time_breaks);
  const double global_time_offset {123.45};
  auto traj = TimedTrajectory(path, time_scaling, global_time_offset);
  // Slice from t = 4.7 (in the quadratic segment)
  auto sliced = traj.SliceFromTime(4.7);
  EXPECT_DOUBLE_EQ(sliced.start_time(), 4.7);
  EXPECT_DOUBLE_EQ(sliced.end_time(), 6.0);
  EXPECT_EQ(sliced.global_time_offset(), global_time_offset);
  // Test values at several points, including in the quadratic time scaling
  // segment
  std::vector<double> test_times {4.7, 4.9, 5.2, 5.5, 5.8, 6.0};
  for (double t : test_times) {
    EXPECT_TRUE(sliced.Value(t).isApprox(traj.Value(t), 1e-6));
    EXPECT_TRUE(
        sliced.EvalDerivative(t).isApprox(traj.EvalDerivative(t), 1e-6));
    EXPECT_TRUE(sliced.GlobalValue(t + global_time_offset)
                    .isApprox(traj.GlobalValue(t + global_time_offset), 1e-6));
    EXPECT_TRUE(
        sliced.EvalGlobalDerivative(t + global_time_offset)
            .isApprox(traj.EvalGlobalDerivative(t + global_time_offset), 1e-6));
  }
}

TEST(TimedTrajectory, SliceFromTime_Exceptions) {
  Eigen::VectorXd conf(3);
  conf << 1.0, 2.0, 3.0;
  auto traj = TimedTrajectory::Constant(conf, 1.0, 3.0);
  EXPECT_THROW(traj.SliceFromTime(0.5), std::runtime_error);
  EXPECT_THROW(traj.SliceFromTime(3.0), std::runtime_error);
  EXPECT_THROW(traj.SliceFromTime(4.0), std::runtime_error);
  EXPECT_NO_THROW(traj.SliceFromTime(3.0, true /* allow clamping end */));
  EXPECT_NO_THROW(traj.SliceFromTime(4.0, true /* allow clamping end */));
  EXPECT_NO_THROW(traj.SliceFromTime(1.0));
  EXPECT_NO_THROW(traj.SliceFromTime(2.0));
  EXPECT_NO_THROW(traj.SliceFromTime(2.9));
}

TEST(TimedTrajectory, Slice_ClampsAndPreservesValues) {
  std::vector<double> path_breaks {0.0, 2.0};
  std::vector<Eigen::MatrixXd> path_coeffs;
  Eigen::MatrixXd path_coeff(2, 2);
  path_coeff << 1.0, 2.0, 3.0, -1.0;
  path_coeffs.push_back(path_coeff);
  auto path = PiecewisePolynomial(path_coeffs, path_breaks);

  std::vector<double> time_breaks {1.0, 3.0};
  std::vector<Eigen::MatrixXd> time_coeffs;
  Eigen::MatrixXd time_coeff(1, 2);
  time_coeff << 0.0, 1.0;  // s(t) = t - 1
  time_coeffs.push_back(time_coeff);
  auto time_scaling = PiecewisePolynomial(time_coeffs, time_breaks);
  auto traj = TimedTrajectory(path, time_scaling, 10.0);

  auto sliced = traj.Slice(-5.0, 10.0);  // clamp to [1, 3]
  EXPECT_DOUBLE_EQ(sliced.start_time(), 1.0);
  EXPECT_DOUBLE_EQ(sliced.end_time(), 3.0);
  EXPECT_DOUBLE_EQ(sliced.global_time_offset(), 10.0);
  EXPECT_TRUE(sliced.Value(1.0).isApprox(traj.Value(1.0), 1e-9));
  EXPECT_TRUE(sliced.Value(2.0).isApprox(traj.Value(2.0), 1e-9));
  EXPECT_TRUE(sliced.Value(3.0).isApprox(traj.Value(3.0), 1e-9));
}

TEST(TimedTrajectory, Slice_ReversedRangeThrows) {
  Eigen::VectorXd conf(2);
  conf << 1.0, 2.0;
  auto traj = TimedTrajectory::Constant(conf, 1.0, 3.0);
  EXPECT_THROW(traj.Slice(2.5, 2.0), std::runtime_error);
}

TEST(TimedTrajectory, Slice_AfterEndClampsToConstant) {
  Eigen::VectorXd conf(2);
  conf << 1.0, 2.0;
  auto traj = TimedTrajectory::Constant(conf, 1.0, 3.0);
  auto sliced = traj.Slice(10.0, 20.0);
  EXPECT_DOUBLE_EQ(sliced.start_time(), 10.0);
  EXPECT_DOUBLE_EQ(sliced.end_time(), 20.0);
  EXPECT_TRUE(sliced.Value(10.0).isApprox(traj.Value(3.0), 1e-9));
  EXPECT_TRUE(sliced.Value(20.0).isApprox(traj.Value(3.0), 1e-9));
}

TEST(TimedTrajectory, Merge_Basics) {
  // First trajectory: q = s, s = t, t in [0, 1]
  std::vector<double> path1_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> path1_coeffs;
  Eigen::MatrixXd path1_coeff(1, 2);
  path1_coeff << 0.0, 1.0;
  path1_coeffs.push_back(path1_coeff);
  auto path1 = PiecewisePolynomial(path1_coeffs, path1_breaks);

  std::vector<double> time1_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> time1_coeffs;
  Eigen::MatrixXd time1_coeff(1, 2);
  time1_coeff << 0.0, 1.0;
  time1_coeffs.push_back(time1_coeff);
  auto time1 = PiecewisePolynomial(time1_coeffs, time1_breaks);
  auto traj1 = TimedTrajectory(path1, time1, 50.0);

  // Second trajectory: q = s, s = t - 4, t in [5, 6] (same motion, shifted in
  // time)
  std::vector<double> path2_breaks {1.0, 2.0};
  std::vector<Eigen::MatrixXd> path2_coeffs;
  Eigen::MatrixXd path2_coeff(1, 2);
  path2_coeff << 1.0, 1.0;
  path2_coeffs.push_back(path2_coeff);
  auto path2 = PiecewisePolynomial(path2_coeffs, path2_breaks);

  std::vector<double> time2_breaks {5.0, 6.0};
  std::vector<Eigen::MatrixXd> time2_coeffs;
  Eigen::MatrixXd time2_coeff(1, 2);
  time2_coeff << 1.0, 1.0;
  time2_coeffs.push_back(time2_coeff);
  auto time2 = PiecewisePolynomial(time2_coeffs, time2_breaks);
  auto traj2 = TimedTrajectory(path2, time2, 500.0);

  auto merged = TimedTrajectory::Merge(traj1, traj2);
  EXPECT_DOUBLE_EQ(merged.start_time(), 0.0);
  EXPECT_DOUBLE_EQ(merged.end_time(), 2.0);
  EXPECT_DOUBLE_EQ(merged.global_time_offset(), 50.0);
  EXPECT_TRUE(merged.Value(0.5).isApprox(traj1.Value(0.5), 1e-9));
  EXPECT_TRUE(merged.Value(1.5).isApprox(traj2.Value(5.5), 1e-9));
  EXPECT_TRUE(
      merged.EvalDerivative(1.5).isApprox(traj2.EvalDerivative(5.5), 1e-9));
}

TEST(TimedTrajectory, Merge_ContinuityChecks) {
  // Shared first trajectory
  std::vector<double> path1_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> path1_coeffs;
  Eigen::MatrixXd path1_coeff(1, 2);
  path1_coeff << 0.0, 1.0;
  path1_coeffs.push_back(path1_coeff);
  auto path1 = PiecewisePolynomial(path1_coeffs, path1_breaks);
  std::vector<double> time1_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> time1_coeffs;
  Eigen::MatrixXd time1_coeff(1, 2);
  time1_coeff << 0.0, 1.0;
  time1_coeffs.push_back(time1_coeff);
  auto time1 = PiecewisePolynomial(time1_coeffs, time1_breaks);
  auto traj1 = TimedTrajectory(path1, time1);

  // Position discontinuity at start (starts at q=2 instead of q=1)
  std::vector<double> pos_bad_breaks {2.0, 3.0};
  std::vector<Eigen::MatrixXd> pos_bad_coeffs;
  Eigen::MatrixXd pos_bad_coeff(1, 2);
  pos_bad_coeff << 2.0, 1.0;
  pos_bad_coeffs.push_back(pos_bad_coeff);
  auto pos_bad_path = PiecewisePolynomial(pos_bad_coeffs, pos_bad_breaks);
  std::vector<double> pos_bad_time_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> pos_bad_time_coeffs;
  Eigen::MatrixXd pos_bad_time_coeff(1, 2);
  pos_bad_time_coeff << 2.0, 1.0;
  pos_bad_time_coeffs.push_back(pos_bad_time_coeff);
  auto pos_bad_time =
      PiecewisePolynomial(pos_bad_time_coeffs, pos_bad_time_breaks);
  auto traj2_pos_bad = TimedTrajectory(pos_bad_path, pos_bad_time);
  EXPECT_THROW(TimedTrajectory::Merge(traj1, traj2_pos_bad, true, false),
               std::runtime_error);

  // Velocity discontinuity at start (same position, but ds/dt=2 at t=0)
  std::vector<double> vel_bad_breaks {1.0, 2.0};
  std::vector<Eigen::MatrixXd> vel_bad_coeffs;
  Eigen::MatrixXd vel_bad_coeff(1, 2);
  vel_bad_coeff << 1.0, 1.0;
  vel_bad_coeffs.push_back(vel_bad_coeff);
  auto vel_bad_path = PiecewisePolynomial(vel_bad_coeffs, vel_bad_breaks);
  std::vector<double> vel_bad_time_breaks {0.0, 1.0};
  std::vector<Eigen::MatrixXd> vel_bad_time_coeffs;
  Eigen::MatrixXd vel_bad_time_coeff(1, 3);
  vel_bad_time_coeff << 1.0, 2.0, -1.0;  // s(0)=1, s(1)=2, ds/dt(0)=2
  vel_bad_time_coeffs.push_back(vel_bad_time_coeff);
  auto vel_bad_time =
      PiecewisePolynomial(vel_bad_time_coeffs, vel_bad_time_breaks);
  auto traj2_vel_bad = TimedTrajectory(vel_bad_path, vel_bad_time);
  EXPECT_THROW(TimedTrajectory::Merge(traj1, traj2_vel_bad, false, true),
               std::runtime_error);
}

TEST(PiecewisePolynomial, ShiftTime) {
  std::vector<double> breaks {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(2, 1));
  Eigen::MatrixXd A(2, 2);
  A << 1.0, 2.0, 0.0, 1.0;
  coefficients_vec.push_back(A);
  PiecewisePolynomial dut(coefficients_vec, breaks);

  // Store original values at some test points
  double t1 = 0.5;
  double t2 = 1.3;
  Eigen::VectorXd val1 = dut.Value(t1);
  Eigen::VectorXd val2 = dut.Value(t2);

  // Shift by 10.0
  double shift = 10.0;
  dut.ShiftTime(shift);

  // Check that breaks are shifted
  EXPECT_DOUBLE_EQ(dut.start_time(), 10.0);
  EXPECT_DOUBLE_EQ(dut.end_time(), 12.0);

  // Check that values at shifted times match original values
  EXPECT_TRUE(dut.Value(t1 + shift).isApprox(val1));
  EXPECT_TRUE(dut.Value(t2 + shift).isApprox(val2));
}

TEST(TimedTrajectory, ShiftTime) {
  // Create a trajectory with path and time_scaling
  std::vector<double> path_breaks {0.0, 1.0, 4.0};
  std::vector<Eigen::MatrixXd> path_coeffs;
  path_coeffs.push_back(Eigen::MatrixXd::Ones(2, 1));
  Eigen::MatrixXd A(2, 2);
  A << 1.0, 2.0, 0.0, 1.0;
  path_coeffs.push_back(A);
  auto path = PiecewisePolynomial(path_coeffs, path_breaks);

  // Time scaling: s(t) = t + 0.5*t^2 for t in [0, 2]
  std::vector<double> time_breaks {0.0, 2.0};
  std::vector<Eigen::MatrixXd> time_coeffs;
  Eigen::MatrixXd time_coeff(1, 3);
  time_coeff << 0.0, 1.0, 0.5;
  time_coeffs.push_back(time_coeff);
  auto time_scaling = PiecewisePolynomial(time_coeffs, time_breaks);

  auto traj = TimedTrajectory(path, time_scaling);

  // Store original values at some test points
  double t1 = 0.5;
  double t2 = 1.5;
  Eigen::VectorXd val1 = traj.Value(t1);
  Eigen::VectorXd val2 = traj.Value(t2);
  Eigen::VectorXd deriv1 = traj.EvalDerivative(t1);
  Eigen::VectorXd deriv2 = traj.EvalDerivative(t2);

  // Shift by 5.0
  double shift = 5.0;
  traj.ShiftTime(shift);

  // Check that start_time and end_time are shifted
  EXPECT_DOUBLE_EQ(traj.start_time(), 5.0);
  EXPECT_DOUBLE_EQ(traj.end_time(), 7.0);

  // Check that values at shifted times match original values
  EXPECT_TRUE(traj.Value(t1 + shift).isApprox(val1));
  EXPECT_TRUE(traj.Value(t2 + shift).isApprox(val2));

  // Check that derivatives are also preserved
  EXPECT_TRUE(traj.EvalDerivative(t1 + shift).isApprox(deriv1));
  EXPECT_TRUE(traj.EvalDerivative(t2 + shift).isApprox(deriv2));
  EXPECT_GT(deriv2.norm(), 0.0);  // Should have non-zero derivative
}

TEST(TimedTrajectory, SetGlobalTimeOffset) {
  // Create a simple constant trajectory
  Eigen::VectorXd conf(2);
  conf << 1.0, 2.0;
  auto traj = TimedTrajectory::Constant(conf, 0.0, 2.0);

  // Check initial global_time_offset (should be 0)
  EXPECT_DOUBLE_EQ(traj.global_time_offset(), 0.0);

  // Set a global time offset
  double offset = 10.0;
  traj.SetGlobalTimeOffset(offset);

  // Verify the offset was set
  EXPECT_DOUBLE_EQ(traj.global_time_offset(), offset);

  // Check that local times are unchanged
  EXPECT_DOUBLE_EQ(traj.start_time(), 0.0);
  EXPECT_DOUBLE_EQ(traj.end_time(), 2.0);

  // Check that global times are correctly offset
  EXPECT_DOUBLE_EQ(traj.global_start_time(), offset);
  EXPECT_DOUBLE_EQ(traj.global_end_time(), offset + 2.0);

  // Check that Value() uses local time (not affected by offset)
  EXPECT_TRUE(traj.Value(1.0).isApprox(conf));

  // Check that GlobalValue() uses global time
  EXPECT_TRUE(traj.GlobalValue(offset + 1.0).isApprox(conf));
}

TEST(TimedTrajectory, SetGlobalTimeOffset_WithComplexTrajectory) {
  // Create a more complex trajectory
  std::vector<double> path_breaks {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> path_coeffs;
  Eigen::MatrixXd coeff1(2, 2);
  coeff1 << 0.0, 1.0,  // q1: t
      0.0, 2.0;        // q2: 2*t
  path_coeffs.push_back(coeff1);

  Eigen::MatrixXd coeff2(2, 2);
  coeff2 << 1.0, 0.5,  // q1: 1 + 0.5*t
      2.0, 1.0;        // q2: 2 + t
  path_coeffs.push_back(coeff2);

  auto path = PiecewisePolynomial(path_coeffs, path_breaks);

  // Linear time scaling: s(t) = t
  std::vector<double> time_breaks {0.0, 2.0};
  std::vector<Eigen::MatrixXd> time_coeffs;
  Eigen::MatrixXd time_coeff(1, 2);
  time_coeff << 0.0, 1.0;
  time_coeffs.push_back(time_coeff);
  auto time_scaling = PiecewisePolynomial(time_coeffs, time_breaks);

  // Create trajectory with an initial offset
  double initial_offset = 100.0;
  auto traj = TimedTrajectory(path, time_scaling, initial_offset);

  // Check initial offset
  EXPECT_DOUBLE_EQ(traj.global_time_offset(), initial_offset);
  EXPECT_DOUBLE_EQ(traj.global_start_time(), initial_offset);

  // Change the offset
  double new_offset = 200.0;
  traj.SetGlobalTimeOffset(new_offset);

  // Verify new offset
  EXPECT_DOUBLE_EQ(traj.global_time_offset(), new_offset);
  EXPECT_DOUBLE_EQ(traj.global_start_time(), new_offset);
  EXPECT_DOUBLE_EQ(traj.global_end_time(), new_offset + 2.0);

  // Test that values are correct with both local and global time
  double local_t = 0.5;
  Eigen::VectorXd val_local = traj.Value(local_t);
  Eigen::VectorXd val_global = traj.GlobalValue(new_offset + local_t);
  EXPECT_TRUE(val_local.isApprox(val_global));

  // Test derivatives as well
  Eigen::VectorXd deriv_local = traj.EvalDerivative(local_t);
  Eigen::VectorXd deriv_global =
      traj.EvalGlobalDerivative(new_offset + local_t);
  EXPECT_TRUE(deriv_local.isApprox(deriv_global));
}

}  // namespace planning_service_client
