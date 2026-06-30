#include "planning_service/comms/register_client_planning_problems.h"
#include "planning_service/draco/client_conversions.h"
#include "shokunin/shokunin_common.h"

// need CLI11
#include <CLI/CLI.hpp>

namespace fs = std::filesystem;
using planning_service_client::planner::MotionProblemDefinition;

std::vector<motion::planning::ConfigSpacePlanningProblem>
LoadPlanningProblemsFromDirectory(const fs::path& problems_dir) {
  std::vector<motion::planning::ConfigSpacePlanningProblem> problems;
  for (const auto& entry : fs::directory_iterator(problems_dir)) {
    if (entry.path().extension() == ".yaml") {
      motion::planning::ConfigSpacePlanningProblem problem;
      problem = drake::yaml::LoadYamlFile<
          motion::planning::ConfigSpacePlanningProblem>(entry.path());
      problems.push_back(problem);
    }
  }
  return problems;
}

int main(int argc, char** argv) {
  CLI::App app {"UpdateRoadmapFromProblems"};
  logging::create_log("UpdateRoadmapFromProblems");
  app.require_subcommand(0);

  uint64_t context_id = 0;
  std::string problems_dir_path;
  app.add_option("-c,--context", context_id, "Context ID")->required();
  app.add_option("-p,--problems_dir", problems_dir_path,
                 "Path to problems directory")
      ->required();

  CLI11_PARSE(app, argc, argv);

  const auto path = fs::path(problems_dir_path);
  if (!fs::exists(path)) {
    logging::log()->error("Directory {} does not exist.", problems_dir_path);
    return 1;
  }

  // Ensure client problems are registered so we can parse json -> proto
  planning_service_client::planner::RegisterAllPlanningProblems();

  // Create planner from context
  auto planner = shokunin::MakeDracoPlannerFromContext(
      context_id, draco::VisualizerMode::kNone, true);

  // Load planning problems
  auto problems = LoadPlanningProblemsFromDirectory(path);

  // Build artifacts using loaded problems
  for (const auto& problem : problems) {
    logging::log()->info(
        "UpdateRoadmapFromProblem: Building artifacts using problem with "
        "start {} and goal {}",
        problem.q_start.transpose(), problem.q_goal.transpose());
    if (!planner.mutable_artifact_builder().IsProblemValid(problem.q_start,
                                                           problem.q_goal)) {
      logging::log()->warn(
          "Problem with start {} and goal {} is invalid, skipping.",
          problem.q_start.transpose(), problem.q_goal.transpose());
      continue;
    }
    planner.mutable_artifact_builder().GetSampleBasedSolution(
        problem.q_start, problem.q_goal, false, true);
  }
}
