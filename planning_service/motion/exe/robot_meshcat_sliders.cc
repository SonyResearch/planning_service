/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <drake/multibody/parsing/model_directives.h>

#include <string>
#include <thread>

#include "planning_service/motion/robot_constraints.h"

using motion::CheckSatisfiedOptions;
using motion::ConstraintsAdapter;
using motion::RobotConstraints;
using motion::RobotMeshcatParams;
using motion::RobotModel;

struct MeshcatSlidersParams {
  std::string xml_file = "planning_service/test_data/package.xml";
  std::string dmd_file = "planning_service/test_data/wallflower/dual.dmd.yaml";
  std::string meshcat_params_file =
      "planning_service/test_data/wallflower/meshcat.yaml";
  std::string constraints_adapter_file =
      "planning_service/test_data/wallflower/default_constraints.yaml";

  // serialize
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(xml_file));
    a->Visit(DRAKE_NVP(dmd_file));
    a->Visit(DRAKE_NVP(meshcat_params_file));
    a->Visit(DRAKE_NVP(constraints_adapter_file));
  }
};

int main() {
  logging::create_log("RobotMeshcatSliders");
  const auto params = drake::yaml::LoadYamlFile<MeshcatSlidersParams>(
      "planning_service/test_data/meshcat_sliders_params.yaml");
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          params.dmd_file)};
  const auto meshcat_params {drake::yaml::LoadYamlFile<RobotMeshcatParams>(
      params.meshcat_params_file)};
  const auto robot {
      std::make_unique<RobotModel>(params.xml_file, dmd, meshcat_params)};
  logging::log()->info("MeshcatViz: Running Joint Sliders");
  robot->meshcat()->AddButton("Stop Sliders");
  const auto plan_adapter {drake::yaml::LoadYamlFile<ConstraintsAdapter>(
      params.constraints_adapter_file)};
  const auto robot_constraints {RobotConstraints(*robot, plan_adapter)};
  robot_constraints.PrintAllCollisionPairs();
  RobotConstraints::RunMeshcatSlidersWithConstraints(robot_constraints);
  return 0;
}
