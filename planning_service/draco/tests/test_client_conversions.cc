#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace conversions {

using RobotAndTimeOptimalSpliner =
    std::pair<std::unique_ptr<motion::RobotModel>,
              std::unique_ptr<motion::splining::TimeOptimalSpliner>>;

namespace {
RobotAndTimeOptimalSpliner MakeRobotModelAndSpliner(
    const std::string& xml_file, const std::string& dmd_file,
    const std::string& dynamic_limits_file,
    const std::string& time_optimal_spline_params_file) {
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<motion::RobotModel>(xml_file, dmd);
  const auto joint_dynamic_limits_map =
      drake::yaml::LoadYamlFile<motion::splining::joint_dynamic_limits_map_t>(
          dynamic_limits_file, "joint_limits");
  const auto cartesian_dynamic_limits_map = drake::yaml::LoadYamlFile<
      motion::splining::cartesian_dynamic_limits_map_t>(dynamic_limits_file,
                                                        "cartesian_limits");
  const auto time_optimal_spline_params =
      drake::yaml::LoadYamlFile<motion::splining::TimeOptimalSplineParams>(
          time_optimal_spline_params_file);
  return std::make_pair(
      std::move(robot_model),
      std::make_unique<motion::splining::TimeOptimalSpliner>(
          *robot_model, joint_dynamic_limits_map, cartesian_dynamic_limits_map,
          time_optimal_spline_params));
}

RobotAndTimeOptimalSpliner MakeDualWallflowerRobotModelAndSpliner() {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_wallflowers/dmd.yaml"};
  const std::string dynamic_limits_file {
      "planning_service/test_data/dual_wallflowers/dynamic_limits.yaml"};
  const std::string time_optimal_spline_params_file {
      "planning_service/test_data/time_optimal_spline_params.yaml"};
  return MakeRobotModelAndSpliner(xml_file, dmd_file, dynamic_limits_file,
                                  time_optimal_spline_params_file);
}
}  // namespace

TEST(TestDracoPlanner, ToGeneralizedPosition) {
  planning_service_client::SystemConf system_conf;
  auto [robot_model, _] = MakeDualWallflowerRobotModelAndSpliner();
  system_conf["flower2"] = Eigen::Vector2d(1.0, 2.0);
  // with throw on missing
  EXPECT_THROW(ToGeneralizedPosition(*robot_model, system_conf),
               std::runtime_error);
  // with assume zero on missing
  Eigen::VectorXd q = ToGeneralizedPosition(
      *robot_model, system_conf, ToGeneralizedBehavior::kAssumeZeroOnMissing);
  EXPECT_EQ(q.size(), 4);
  EXPECT_TRUE(q.isApprox(Eigen::Vector4d(0.0, 0.0, 1.0, 2.0)));
  // Add flower 1
  system_conf["flower1"] = Eigen::Vector2d(3.0, 4.0);
  q = ToGeneralizedPosition(*robot_model, system_conf);
  EXPECT_EQ(q.size(), 4);
  EXPECT_TRUE(q.isApprox(Eigen::Vector4d(3.0, 4.0, 1.0, 2.0)));
}

TEST(TestDracoPlanner, ToPathParameterizedTrajectory) {
  auto [_, time_optimal_spliner] = MakeDualWallflowerRobotModelAndSpliner();
  // Let's make a path parameterized trajectory
  auto s_breaks = Eigen::VectorXd::LinSpaced(4, 0.0, 1.0);
  Eigen::MatrixXd samples(2, 4);
  samples.row(0) << 0.1, 0.5, 1.0, 1.5;
  samples.row(1) << 0.3, 0.7, 1.2, 1.9;
  auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(s_breaks, samples);
  auto t_breaks =
      Eigen::VectorXd::LinSpaced(3, 0.0, 1.5);  // Time between 0 and 1.5
  Eigen::MatrixXd s_samples(1, 3);
  s_samples.row(0) << 0.0, 0.3, 1.0;  // samples between 0 and 1.0
  auto time_scaling = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(t_breaks, s_samples);
  planning_service_client::SystemTimedTrajectory sys_timed_trajectory;
  // Let's add flower2
  sys_timed_trajectory["flower2"] = planning_service_client::TimedTrajectory(
      DrakePiecewisePolynomialToClient(path),
      DrakePiecewisePolynomialToClient(time_scaling));
  // with throw on missing
  EXPECT_THROW(ToPathParameterizedTrajectory(*time_optimal_spliner,
                                             sys_timed_trajectory),
               std::runtime_error);
  // with assume zero on missing
  auto traj_2_only = ToPathParameterizedTrajectory(
      *time_optimal_spliner, sys_timed_trajectory,
      ToGeneralizedBehavior::kAssumeZeroOnMissing);
  EXPECT_EQ(traj_2_only.rows(), 4);
  EXPECT_EQ(traj_2_only.cols(), 1);
  // The start value will be the first sample column, the rest will be zero
  EXPECT_TRUE(traj_2_only.value(0.0).isApprox(
      Eigen::Vector4d(0, 0, samples(0, 0), samples(1, 0))));
  EXPECT_TRUE(traj_2_only.value(1.5).isApprox(
      Eigen::Vector4d(0, 0, samples(0, 3), samples(1, 3))));
  // Add flower with the same path + offset
  auto path_1 = path + Eigen::Vector2d(5.0, 6.0);
  sys_timed_trajectory["flower1"] = planning_service_client::TimedTrajectory(
      DrakePiecewisePolynomialToClient(path_1),
      DrakePiecewisePolynomialToClient(time_scaling));
  auto traj = ToPathParameterizedTrajectory(*time_optimal_spliner,
                                            sys_timed_trajectory);
  EXPECT_EQ(traj.rows(), 4);
  EXPECT_EQ(traj.cols(), 1);
  // The start value will be the first sample column
  EXPECT_TRUE(traj.value(0.0).isApprox(Eigen::Vector4d(
      5.0 + samples(0, 0), 6.0 + samples(1, 0), samples(0, 0), samples(1, 0))));
  EXPECT_TRUE(traj.value(1.5).isApprox(Eigen::Vector4d(
      5.0 + samples(0, 3), 6.0 + samples(1, 3), samples(0, 3), samples(1, 3))));
}

TEST(TestClientConversions, ClientPiecewisePolynomialToDrake) {
  std::vector<double> breaks {0.3, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> coefficients;
  coefficients.push_back(Eigen::MatrixXd::Ones(3, 2));
  coefficients.push_back(2 * Eigen::MatrixXd::Ones(3, 4));
  auto client_traj =
      planning_service_client::PiecewisePolynomial(coefficients, breaks);
  auto drake_traj = ClientPiecewisePolynomialToDrake(client_traj);
  // Check the properties
  EXPECT_EQ(client_traj.start_time(), drake_traj.start_time());
  EXPECT_EQ(client_traj.end_time(), drake_traj.end_time());
  // Check the values at all times
  for (double t = drake_traj.start_time(); t < drake_traj.end_time();
       t += 0.1) {
    EXPECT_TRUE(client_traj.Value(t).isApprox(drake_traj.value(t), 1e-4));
  }
  auto client_traj_back = DrakePiecewisePolynomialToClient(drake_traj);
  EXPECT_TRUE(client_traj_back.IsCloseTo(client_traj, 1e-4));
}

TEST(TestClientConversions, DrakePiecewisePolynomialToClient) {
  std::vector<double> breaks {0.3, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> samples;
  samples.push_back(Eigen::Vector2d(0.0, 3.0));
  samples.push_back(Eigen::Vector2d(1.0, 4.0));
  samples.push_back(Eigen::Vector2d(3.0, 2.0));
  auto drake_traj = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(breaks, samples);
  auto client_traj = DrakePiecewisePolynomialToClient(drake_traj);
  // Check the properties
  EXPECT_EQ(client_traj.start_time(), drake_traj.start_time());
  EXPECT_EQ(client_traj.end_time(), drake_traj.end_time());
  // Check the values at all times
  for (double t = drake_traj.start_time(); t < drake_traj.end_time();
       t += 0.1) {
    EXPECT_TRUE(client_traj.Value(t).isApprox(drake_traj.value(t), 1e-4));
  }
  auto drake_traj_back = ClientPiecewisePolynomialToDrake(client_traj);
  EXPECT_TRUE(drake_traj_back.isApprox(drake_traj, 1e-4));
}

TEST(TestClientConversions, ToDrakeRigidTransform) {}

TEST(TestClientConversions, ToDrakeShapeSphere) {
  const auto result = ToDrakeShape(planning_service_client::Sphere(0.05));
  ASSERT_NE(result, nullptr);
  const auto* sphere =
      dynamic_cast<const drake::geometry::Sphere*>(result.get());
  ASSERT_NE(sphere, nullptr);
  EXPECT_EQ(sphere->type_name(), "Sphere");
  EXPECT_DOUBLE_EQ(sphere->radius(), 0.05);
}

TEST(TestClientConversions, ToDrakeShapeCylinder) {
  const auto result =
      ToDrakeShape(planning_service_client::Cylinder(0.03, 0.2));
  ASSERT_NE(result, nullptr);
  const auto* cylinder =
      dynamic_cast<const drake::geometry::Cylinder*>(result.get());
  ASSERT_NE(cylinder, nullptr);
  EXPECT_EQ(cylinder->type_name(), "Cylinder");
  EXPECT_DOUBLE_EQ(cylinder->radius(), 0.03);
  EXPECT_DOUBLE_EQ(cylinder->length(), 0.2);
}

TEST(TestClientConversions, ToDrakeShapeCapsule) {
  const auto result =
      ToDrakeShape(planning_service_client::Capsule(0.04, 0.15));
  ASSERT_NE(result, nullptr);
  const auto* capsule =
      dynamic_cast<const drake::geometry::Capsule*>(result.get());
  ASSERT_NE(capsule, nullptr);
  EXPECT_EQ(capsule->type_name(), "Capsule");
  EXPECT_DOUBLE_EQ(capsule->radius(), 0.04);
  EXPECT_DOUBLE_EQ(capsule->length(), 0.15);
}

TEST(TestClientConversions, ToDrakeShapeBox) {
  const auto result = ToDrakeShape(planning_service_client::Box(0.1, 0.2, 0.3));
  ASSERT_NE(result, nullptr);
  const auto* box = dynamic_cast<const drake::geometry::Box*>(result.get());
  ASSERT_NE(box, nullptr);
  EXPECT_EQ(box->type_name(), "Box");
  EXPECT_DOUBLE_EQ(box->width(), 0.1);
  EXPECT_DOUBLE_EQ(box->depth(), 0.2);
  EXPECT_DOUBLE_EQ(box->height(), 0.3);
}

}  // namespace conversions
}  // namespace draco
