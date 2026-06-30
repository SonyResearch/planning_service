#include "shokunin/shokunin_common.h"

// need CLI11
#include <CLI/CLI.hpp>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  CLI::App app {"RunSavedPlan"};
  logging::create_log("RunSavedPlan");
  app.require_subcommand(0);
  // Load the context from the CLI
  uint64_t context_id = 0;
  std::string result_path = "";
  app.add_option("-c,--context", context_id, "Context ID")->required();
  app.add_option("-r,--result", result_path, "Result path")->required();
  CLI11_PARSE(app, argc, argv);
  const auto path = fs::path(result_path);
  // Let's load the saved plan
  logging::log()->info("Loading result from {} and context ID: {}", result_path,
                       context_id);
  // Log if the file exists
  if (!fs::exists(path)) {
    logging::log()->error("File {} does not exist.", result_path);
    return 0;
  } else {
    logging::log()->info("File {} exists.", result_path);
  }
  auto result = planning_service_client::common::LoadFromJsonFile<
      planning_service_client::planner::MotionPlanResult>(path);
  auto planner = shokunin::MakeDracoPlannerFromContext(context_id);
  // Send the trajectory to the visualizer
  auto sys_timed_traj = result.system_timed_trajectory();
  if (planner.has_draco_visualizer()) {
    planner.mutable_draco_visualizer().Add(sys_timed_traj, "Loaded Trajectory");
  } else {
    logging::log()->error("No visualizer found in the planner.");
  }
  // Wait until user kills the visualizer
  while (planner.has_draco_visualizer()
         && planner.draco_visualizer().IsRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
