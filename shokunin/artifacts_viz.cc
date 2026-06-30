#include "shokunin/shokunin_common.h"

int main(int argc, char** argv) {
  CLI::App app {"Artifact Inspector"};
  logging::create_log("Artifact Inspector");
  app.require_subcommand(0);
  // Load the context from the CLI
  uint64_t context_id = 0;
  app.add_option("-c,--context", context_id, "Context ID")->required();
  CLI11_PARSE(app, argc, argv);
  if (context_id == 0) {
    logging::log()->error("Context ID must be provided and non-zero");
    return 0;
  }
  logging::log()->info("Loading context ID: {}", context_id);
  auto planner = shokunin::MakeDracoPlannerFromContext(
      context_id, draco::VisualizerMode::kNative, true);
  logging::log()->info("Loaded draco planner for context ID: {}", context_id);
  // Now, let's inspect the artifacts
  auto& artifact_builder = planner.mutable_artifact_builder();
  auto meshcat = planner.robot_model().meshcat();
  const auto& prm_vertices =
      artifact_builder.mutable_thunder_planner().vertices_confs();
  logging::log()->info("Number of PRM vertices: {}", prm_vertices.size());
  std::string slider_name = "PRM Explorer";
  DRAKE_THROW_UNLESS(meshcat != nullptr);
  meshcat->AddSlider(slider_name, 0.0, static_cast<double>(prm_vertices.size()),
                     1.0, 1.0);
  logging::log()->info("Use the slider in meshcat to explore the PRM vertices");
  std::string artifact_graph = artifact_builder.GetGraphvizString();
  // save the string to a file
  std::string graphviz_file = "/logs/shokunin/context_"
                              + std::to_string(context_id)
                              + "_artifact_graph.dot";
  std::ofstream ofs(graphviz_file);
  ofs << artifact_graph;
  ofs.close();
  logging::log()->info("Saved artifact graph to {}", graphviz_file);
  const auto prm_2_regions =
      artifact_builder.roadmap_vertex_to_iris_regions_set();
  meshcat->AddButton(fmt::format("Context ID: {}", context_id));
  while (true) {
    int vertex_index = static_cast<int>(meshcat->GetSliderValue(slider_name));
    // clamp vertex_index to be in range
    vertex_index = std::max(
        0, std::min(vertex_index, static_cast<int>(prm_vertices.size()) - 1));
    if (prm_2_regions.count(vertex_index) > 0) {
      // Covered by region, turn background green
      meshcat->SetProperty("/Background", "top_color", {0.6, 1.0, 0.6});
    } else {
      // Not covered by region. Turn background red.
      meshcat->SetProperty("/Background", "top_color", {1.0, 0.6, 0.6});
    }
    const auto& q = prm_vertices[vertex_index];
    planner.robot_model().SetMeshcatPositions(q);
    planner.robot_model().PublishMeshcatContext();
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
  return 0;
}
