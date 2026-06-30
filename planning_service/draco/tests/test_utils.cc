#include "planning_service/draco/tests/test_utils.h"

#include "planning_service/draco/draco.h"

namespace draco {
namespace test {

DracoAdapter DracoFilesToAdapter(const DracoAdapterFiles& files) {
  DracoAdapter draco_adapter;
  draco_adapter.xml_file = files.xml_file,
  draco_adapter.dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          files.dmd_file);
  if (files.meshcat_params_file.has_value()) {
    draco_adapter.robot_meshcat_params =
        drake::yaml::LoadYamlFile<motion::RobotMeshcatParams>(
            files.meshcat_params_file.value());
  } else {
    logging::log()->info(
        "Draco: DracoFilesToAdapter: No meshcat params file provided.");
  }
  draco_adapter.constraints_adapter =
      drake::yaml::LoadYamlFile<motion::ConstraintsAdapter>(
          files.plan_adapter_file);
  draco_adapter.iris_regions_adapter =
      drake::yaml::LoadYamlFile<motion::iris::IrisRegionsAdapter>(
          files.iris_regions_adapter_file);
  draco_adapter.joint_dynamic_limits_map =
      drake::yaml::LoadYamlFile<motion::splining::joint_dynamic_limits_map_t>(
          files.dynamic_limits_file, "joint_limits");
  draco_adapter.cartesian_dynamic_limits_map = drake::yaml::LoadYamlFile<
      motion::splining::cartesian_dynamic_limits_map_t>(
      files.dynamic_limits_file, "cartesian_limits");
  draco_adapter.time_optimal_spline_params =
      drake::yaml::LoadYamlFile<motion::splining::TimeOptimalSplineParams>(
          files.time_optimal_spline_params_file);
  draco_adapter.thunder_parameters =
      drake::yaml::LoadYamlFile<motion::planning::ompl::ThunderParameters>(
          files.thunder_params_file);
  draco_adapter.context_dir = fs::path(files.context_dir_path);
  draco_adapter.thunder_dat_file = files.thunder_dat_file;
  draco_adapter.gcs_planner_options =
      drake::yaml::LoadYamlFile<motion::planning::GcsPlannerOptions>(
          files.gcs_planner_options_file);
  draco_adapter.options =
      drake::yaml::LoadYamlFile<DracoOptions>(files.draco_options_file);
  draco_adapter.problems_dir = draco_adapter.context_dir / "problems";
  return draco_adapter;
}

DracoAdapterFiles wallflower_files = {
    .xml_file = "planning_service/test_data/package.xml",
    .dmd_file = "planning_service/test_data/wallflower/dmd.yaml",
    .meshcat_params_file = std::nullopt,
    .plan_adapter_file = "planning_service/test_data/default_constraints.yaml",
    .thunder_dat_file = "planning_service/test_data/wallflower/thunder_prm.dat",
    .iris_regions_adapter_file =
        "planning_service/test_data/wallflower/regions.yaml",
    .iris_builder_options_file =
        "planning_service/test_data/iris_builder_options.yaml",
    .dynamic_limits_file =
        "planning_service/test_data/wallflower/dynamic_limits.yaml",
    .time_optimal_spline_params_file =
        "planning_service/test_data/time_optimal_spline_params.yaml",
    .thunder_params_file = "planning_service/test_data/thunder_params.yaml",
    .gcs_planner_options_file = "planning_service/test_data/gcs_options.yaml",
    .context_dir_path = "planning_service/test_data/wallflower",
    .draco_options_file = "planning_service/test_data/draco_options.yaml",
};

DracoAdapterFiles dual_wallflower_files = {
    .xml_file = "planning_service/test_data/package.xml",
    .dmd_file = "planning_service/test_data/dual_wallflowers/dmd.yaml",
    .meshcat_params_file = std::nullopt,
    .plan_adapter_file =
        "planning_service/test_data/wallflower/default_constraints.yaml",
    .thunder_dat_file =
        "planning_service/test_data/dual_wallflowers/"
        "thunder_prm.dat",
    .iris_regions_adapter_file =
        "planning_service/test_data/dual_wallflowers/regions.yaml",
    .iris_builder_options_file =
        "planning_service/test_data/iris_builder_options.yaml",
    .dynamic_limits_file =
        "planning_service/test_data/dual_wallflowers/dynamic_limits.yaml",
    .time_optimal_spline_params_file =
        "planning_service/test_data/time_optimal_spline_params.yaml",
    .thunder_params_file = "planning_service/test_data/thunder_params.yaml",
    .gcs_planner_options_file = "planning_service/test_data/gcs_options.yaml",
    .context_dir_path = "planning_service/test_data/dual_wallflowers",
    .draco_options_file = "planning_service/test_data/draco_options.yaml",
};

DracoAdapterFiles dual_pandas_files = {
    .xml_file = "planning_service/test_data/package.xml",
    .dmd_file = "planning_service/test_data/dual_pandas/dmd.yaml",
    .meshcat_params_file = "planning_service/test_data/meshcat.yaml",
    .plan_adapter_file = "planning_service/test_data/default_constraints.yaml",
    .thunder_dat_file =
        "planning_service/test_data/dual_pandas/thunder_prm.dat",
    .iris_regions_adapter_file =
        "planning_service/test_data/dual_pandas/iris_regions.yaml",
    .iris_builder_options_file =
        "planning_service/test_data/iris_builder_options.yaml",
    .dynamic_limits_file =
        "planning_service/test_data/dual_pandas/dynamic_limits.yaml",
    .time_optimal_spline_params_file =
        "planning_service/test_data/time_optimal_spline_params.yaml",
    .thunder_params_file = "planning_service/test_data/thunder_params.yaml",
    .gcs_planner_options_file = "planning_service/test_data/gcs_options.yaml",
    .context_dir_path = "planning_service/test_data/dual_pandas",
    .draco_options_file = "planning_service/test_data/draco_options.yaml",
};

DracoAdapterFiles franka_files = {
    .xml_file = "planning_service/test_data/package.xml",
    .dmd_file = "planning_service/test_data/franka/dmd.yaml",
    .meshcat_params_file = "planning_service/test_data/meshcat.yaml",
    .plan_adapter_file = "planning_service/test_data/franka/constraints.yaml",
    .thunder_dat_file = "",
    .iris_regions_adapter_file =
        "planning_service/test_data/iiwa/contexts/8878108380082535913/"
        "iris_regions.yaml",
    .iris_builder_options_file =
        "planning_service/test_data/iris_builder_options.yaml",
    .dynamic_limits_file =
        "planning_service/test_data/franka/dynamic_limits.yaml",
    .time_optimal_spline_params_file =
        "planning_service/test_data/time_optimal_spline_params.yaml",
    .thunder_params_file = "planning_service/test_data/thunder_params.yaml",
    .gcs_planner_options_file = "planning_service/test_data/gcs_options.yaml",
    .context_dir_path = "planning_service/test_data/franka",
    .draco_options_file = "planning_service/test_data/draco_options.yaml",
};

DracoAdapter Wallflower() {
  return DracoFilesToAdapter(wallflower_files);
}

DracoAdapter DualWallflowers() {
  return DracoFilesToAdapter(dual_wallflower_files);
}

DracoAdapter DualPandas() {
  return DracoFilesToAdapter(dual_pandas_files);
}

DracoAdapter Franka() {
  return DracoFilesToAdapter(franka_files);
}

}  // namespace test
}  // namespace draco
