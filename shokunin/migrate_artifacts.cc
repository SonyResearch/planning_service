#include "shokunin/shokunin_common.h"

int main(int argc, char** argv) {
  CLI::App app {"Migrate Artifacts"};
  logging::create_log("Migrate Artifacts");
  app.require_subcommand(0);
  // Load the context from the CLI
  uint64_t context_id = 0;
  std::string regions_file = "";
  std::string prm_file = "";
  int num_samples_repair_region = 5000;
  app.add_option("-c,--context", context_id, "Context ID")->required();
  app.add_option("-r,--regions_file", regions_file,
                 "IRIS regions file to migrate");
  app.add_option("-p,--prm_file", prm_file,
                 "PRM file to migrate to the current draco");
  app.add_option("-n,--num_samples_repair_region", num_samples_repair_region,
                 "Number of samples to repair the region")
      ->default_val("1000");
  CLI11_PARSE(app, argc, argv);
  auto planner = shokunin::MakeDracoPlannerFromContext(
      context_id, draco::VisualizerMode::kNone, true);
  // Now, let's inspect the artifacts
  auto& artifact_builder = planner.mutable_artifact_builder();
  // Let's first migrate the Regions.
  if (!regions_file.empty()) {
    auto iris_regions_adapter =
        drake::yaml::LoadYamlFile<motion::iris::IrisRegionsAdapter>(
            regions_file);
    artifact_builder.SetIrisRegionsAdapter(iris_regions_adapter);
    artifact_builder.RepairRegions({}, num_samples_repair_region);
  }
  // Now migrate the PRM
  if (!prm_file.empty()) {
    logging::log()->info("Loading from PRM file: {}", prm_file);
    std::string prm_file_this_draco =
        artifact_builder.thunder_planner().GetDatabaseFilePath();
    logging::log()->info(
        "Saving migrated PRM to this draco's database file: {}",
        prm_file_this_draco);
    artifact_builder.LoadValidateAndSaveFiles(prm_file, prm_file_this_draco,
                                              true);
  }
  return 0;
}
