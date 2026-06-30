/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */
#include "CLI/CLI.hpp"
#include "planning_service/common/misc_utils.h"
#include "planning_service/common/string_utils.h"
#include "planning_service/service/utils/resource_registry.h"

int main(int argc, char* argv[]) {
  const std::string program {"resource_validator"};
  logging::create_log(fmt::format("{}-{}", program, common::utils::GetTag()));
  CLI::App app {program};
  std::string system;
  app.add_option("system", system,
                 "System name to validate resources for. If no system is "
                 "provided, all systems will be validated.");
  bool update_ids {false};
  app.add_flag(
      "-u,--update-ids", update_ids,
      "Overwrite existing resource IDs if they do not match the assigned CID");
  bool no_parallel {false};
  app.add_flag("--no-parallel", no_parallel,
               "Disable parallel loading of resources (useful for debugging)");
  std::string include_names_str = "";
  app.add_option("-i,--include-names", include_names_str,
                 "List of names that systems must contain in their DMDs to be "
                 "included, separated by commas");
  CLI11_PARSE(app, argc, argv);

  // Parse include names to ResourceOptions
  std::optional<std::vector<std::string>> dmd_contains_names {};
  if (!include_names_str.empty()) {
    dmd_contains_names = common::utils::split_string(include_names_str, ',');
  }
  const fs::path data_root {"/data"};
  std::map<std::string, std::string> invalid_systems;
  std::vector<std::string> systems;
  if (system.empty()) {
    const auto dit {fs::directory_iterator(data_root)};
    std::transform(fs::begin(dit), fs::end(dit), std::back_inserter(systems),
                   [](const auto& entry) {
                     return entry.path().stem().string();
                   });
  } else {
    systems.push_back(system);
  }
  logging::log()->warn("Validating resources for systems: {}",
                       common::utils::join_strings(systems, ", "));
  for (const auto& system : systems) {
    const auto path {data_root / system};
    if (!fs::is_directory(path)) {
      invalid_systems.emplace(path.string(), "Not a directory");
      continue;
    }
    try {
      service::utils::ResourceOptions options;
      options.require_parameters = true;
      options.make_draco_visualizers = false;
      options.require_all_resources = true;
      options.load_resources_on_init = true;
      options.update_resource_ids = update_ids;
      options.dry_run = true;
      options.load_in_parallel = !no_parallel;
      options.dmd_contains_names = dmd_contains_names;
      service::utils::ResourceRegistry(path, options);
    } catch (const std::exception& e) {
      invalid_systems.emplace(path.stem().string(), e.what());
      continue;
    }
  }
  if (!invalid_systems.empty()) {
    std::cout << "The following resources are invalid:" << std::endl;
    for (const auto& [path, reason] : invalid_systems) {
      std::cout << fmt::format("FAILED: \"{}\"\n{}", path, reason) << std::endl;
    }
    return 1;
  }
  std::cout << "All resources loaded successfully!" << std::endl;
  return 0;
}
