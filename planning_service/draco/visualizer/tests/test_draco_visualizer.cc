#include <gtest/gtest.h>

#include "planning_service/draco/visualizer/draco_visualizer.h"

namespace draco {
namespace visualizer {

namespace {
std::tuple<std::string, drake::multibody::parsing::ModelDirectives,
           motion::RobotMeshcatParams, motion::ConstraintsAdapter>
MakeWallflower() {
  std::string xml_file = "planning_service/test_data/package.xml";
  std::string dmd_file = "planning_service/test_data/wallflower/dmd.yaml";
  std::string meshcat_params_file = "planning_service/test_data/meshcat.yaml";
  std::string plan_adapter_file =
      "planning_service/test_data/default_constraints.yaml";
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_meshcat_params =
      drake::yaml::LoadYamlFile<motion::RobotMeshcatParams>(
          meshcat_params_file);
  auto constraints_adapter =
      drake::yaml::LoadYamlFile<motion::ConstraintsAdapter>(plan_adapter_file);
  return std::make_tuple(xml_file, dmd, robot_meshcat_params,
                         constraints_adapter);
}

planning_service_client::SystemTimedTrajectory WallflowerTraj() {
  std::vector<double> s_breaks {3.0, 9.0};
  std::vector<Eigen::MatrixXd> coeffs;
  Eigen::MatrixXd q_coeffs(2, 3);
  q_coeffs << 1.0, 1.5, 0.02, 0.3, 0.01, 0.0;
  coeffs.push_back(q_coeffs);
  auto path = planning_service_client::PiecewisePolynomial(coeffs, s_breaks);
  std::vector<double> t_breaks {2.0, 5.0};
  std::vector<Eigen::MatrixXd> s_coeffs;
  Eigen::MatrixXd s_element(1, 2);
  s_element << 3.0, 2.0;
  s_coeffs.push_back(s_element);
  auto timing =
      planning_service_client::PiecewisePolynomial(s_coeffs, t_breaks);
  auto traj = planning_service_client::TimedTrajectory(path, timing);
  planning_service_client::SystemTimedTrajectory sys_traj;
  sys_traj["robot"] = traj;
  return sys_traj;
}

}  // namespace

TEST(TestDracoVisualizer, Basics) {
  auto [xml_file, dmd, robot_meshcat_params, constraints_adapter] =
      MakeWallflower();
  auto visualizer = visualizer::DracoVisualizer(
      xml_file, dmd, robot_meshcat_params, constraints_adapter);
  visualizer.Kill();
}

TEST(TestDracoVisualizer, Add) {
  auto [xml_file, dmd, robot_meshcat_params, constraints_adapter] =
      MakeWallflower();
  auto visualizer = visualizer::DracoVisualizer(
      xml_file, dmd, robot_meshcat_params, constraints_adapter);
  auto sys_traj = WallflowerTraj();
  visualizer.Add(sys_traj, "test traj");
  visualizer.Kill();
}

TEST(TestDracoVisualizer, AddMultiple) {
  auto [xml_file, dmd, robot_meshcat_params, constraints_adapter] =
      MakeWallflower();
  auto visualizer = visualizer::DracoVisualizer(
      xml_file, dmd, robot_meshcat_params, constraints_adapter);
  auto sys_traj = WallflowerTraj();
  for (int i = 0; i < 10; i++) {
    Eigen::VectorXd q_random = Eigen::VectorXd::Random(2);
    visualizer.Add(q_random);
    visualizer.Add(sys_traj);
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  visualizer.Kill();
}

}  // namespace visualizer
}  // namespace draco
