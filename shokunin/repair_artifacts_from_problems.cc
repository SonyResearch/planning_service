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
  CLI::App app {"RepairArtifactsFromProblem"};
  logging::create_log("RepairArtifactsFromProblem");
  app.require_subcommand(0);

  uint64_t context_id = 0;
  std::string problems_dir_path;
  double sampling_dt = 0.01;
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

  // Repair artifacts using loaded problems
  for (const auto& problem : problems) {
    logging::log()->info(
        "RepairArtifactsFromProblem: Repairing artifacts using problem with "
        "start {} and goal {}",
        problem.q_start.transpose(), problem.q_goal.transpose());
    if (!planner.mutable_artifact_builder().IsProblemValid(problem.q_start,
                                                           problem.q_goal)) {
      logging::log()->warn(
          "Problem with start {} and goal {} is invalid, skipping.",
          problem.q_start.transpose(), problem.q_goal.transpose());
      continue;
    }
    // Solve problem using GCS planner
    std::optional<drake::trajectories::CompositeTrajectory<double>>
        sol_trajectory_opt;
    try {
      sol_trajectory_opt =
          planner.mutable_single_mode_gcs_planner().CalcOptimalPath(
              problem.q_start, problem.q_goal);
    } catch (const std::exception& e) {
      logging::log()->error(
          "RepairArtifactsFromProblem: Exception while solving problem with "
          "start {} and goal {}: {}",
          problem.q_start.transpose(), problem.q_goal.transpose(), e.what());
      continue;
    }
    if (!sol_trajectory_opt.has_value()) {
      logging::log()->error(
          "RepairArtifactsFromProblem: Failed to solve problem with start "
          "{} and goal {}",
          problem.q_start.transpose(), problem.q_goal.transpose());
      continue;
    }
    logging::log()->debug(
        "RepairArtifactsFromProblem: Successfully solved problem with "
        "start {} and goal {}",
        problem.q_start.transpose(), problem.q_goal.transpose());
    // Repair regions using any invalid configurations in the solution path
    auto invalid_conf =
        planner.MaybeInvalidConf(sol_trajectory_opt.value(), sampling_dt);
    if (invalid_conf.has_value()) {
      logging::log()->info(
          "RepairArtifactsFromProblem: Found invalid configuration {} in "
          "solution path. Repairing regions.",
          invalid_conf.value().transpose());
      planner.mutable_artifact_builder().RepairRegions({invalid_conf.value()},
                                                       0);
    } else {
      logging::log()->info(
          "RepairArtifactsFromProblem: All configurations in solution "
          "path are valid. No repair needed.");
    }
  }
}
