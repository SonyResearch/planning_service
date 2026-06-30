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
  bool check_encapsulation = false;
  bool neutral_position = false;
  double tolerance = 0.005;
  app.add_option("-c,--context", context_id, "Context ID")->required();
  app.add_flag(
      "-e,--encapsulation", check_encapsulation,
      "Check if all visual meshes are encapsulated by collision shapes");
  app.add_flag(
      "-n,--neutral", neutral_position,
      "Set the robot to its neutral position before checking encapsulation");
  app.add_option(
      "-t,--tolerance", tolerance,
      "Tolerance for encapsulation check (in meters), default is 0.005m");
  CLI11_PARSE(app, argc, argv);
  auto planner = shokunin::MakeDracoPlannerFromContext(
      context_id, draco::VisualizerMode::kNative);
  // Let's run the meshcat sliders
  auto meshcat = planner.robot_model().meshcat();
  logging::log()->info(
      "RunSavedPlan: Loaded planner with context ID {}. Launching Meshcat "
      "sliders... and checking encapsulation: {}",
      context_id, check_encapsulation ? "yes" : "no");
  std::vector<std::string> uncovered_meshes_paths;
  meshcat->AddButton("Context ID: " + std::to_string(context_id));
  std::optional<Eigen::VectorXd> default_positions;
  if (neutral_position) {
    logging::log()->info(
        "RunSavedPlan: Setting robot to its neutral position before checking "
        "encapsulation...");
    auto default_sysconf = planner.options().default_configuration;
    default_positions =
        planner.robot_model().ToGeneralizedPosition(default_sysconf.value());
    planner.robot_model().SetMeshcatPositions(default_positions.value());
  }
  planner.robot_model().PublishMeshcatContext();
  if (check_encapsulation) {
    logging::log()->info(
        "RunSavedPlan: Checking if all visual meshes are encapsulated by "
        "collision shapes...");
    bool uncovered_meshes_exist =
        planner.robot_model().AreAllVisualShapesEncapsulatedByCollisionShapes(
            tolerance, default_positions);
    if (!uncovered_meshes_exist) {
      logging::log()->critical(
          "❌ Some visual shapes are not encapsulated by collision "
          "shapes. This may lead to unexpected collisions. ❌");
    } else {
      logging::log()->critical(
          "✅ All visual shapes are encapsulated by collision "
          "shapes. ✅");
    }
  }
  if (neutral_position) {
    std::string button_name = "End Neutral Position";
    meshcat->AddButton(button_name);
    while (!meshcat->GetButtonClicks(button_name)) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    meshcat->DeleteButton(button_name);
  }
  motion::RobotConstraints::RunMeshcatSlidersWithConstraints(
      planner.robot_constraints());
}
