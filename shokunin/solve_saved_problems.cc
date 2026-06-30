#include <filesystem>
#include <fstream>

#include <CLI/CLI.hpp>

#include "planning_service/comms/register_client_planning_problems.h"
#include "planning_service/draco/client_conversions.h"
#include "shokunin/shokunin_common.h"

namespace fs = std::filesystem;
namespace psc = planning_service_client;

struct SavedProblemResult {
  std::string filename;
  bool success = false;
  std::string message;
  double duration = 0.0;
  double configuration_twirl = 0.0;
  double solve_time_ms = 0.0;

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(filename));
    a->Visit(DRAKE_NVP(success));
    a->Visit(DRAKE_NVP(message));
    a->Visit(DRAKE_NVP(duration));
    a->Visit(DRAKE_NVP(configuration_twirl));
    a->Visit(DRAKE_NVP(solve_time_ms));
  }
};

struct SavedProblemsResults {
  std::vector<SavedProblemResult> results;
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(results));
  }
};

int main(int argc, char** argv) {
  CLI::App app {"SolveSavedProblems"};
  logging::create_log("SolveSavedProblems");
  logging::disable_drake_logging();
  app.require_subcommand(0);

  uint64_t context_id = 0;
  std::string problems_dir;
  std::string results_out = "/logs/shokunin/saved_problems_results.yaml";
  double sample_dt = 0.05;
  int log_level = spdlog::level::err;
  app.add_option("-l,--log_level", log_level, "Logging level (spdlog levels)")
      ->default_val(log_level);
  app.add_option("-d,--dir", problems_dir, "Directory containing problem jsons")
      ->required();
  app.add_option("-c,--context", context_id, "Context ID")
      ->default_val(context_id);
  app.add_option("-o,--out", results_out, "Output results yaml file")
      ->default_val(results_out);

  CLI11_PARSE(app, argc, argv);

  logging::log()->set_level(static_cast<spdlog::level::level_enum>(log_level));

  if (!fs::exists(problems_dir) || !fs::is_directory(problems_dir)) {
    logging::log()->error("Provided path is not a directory: {}", problems_dir);
    return 2;
  }

  // Register problems so json -> proto deserialization works
  psc::planner::RegisterAllPlanningProblems();

  SavedProblemsResults all_results;
  int num_problems = 0;
  int num_success = 0;

  bool override_context_id = (context_id != 0);

  // First, load problems
  std::vector<std::pair<fs::path, psc::planner::MotionProblemDefinition>>
      loaded_file_problem_pairs;
  std::set<uint64_t> context_ids;
  std::map<fs::path, std::string> problem_status;
  std::map<fs::path, std::string> problem_load_errors;
  for (const auto& entry : fs::directory_iterator(problems_dir)) {
    if (!entry.is_regular_file()) continue;
    const auto path = entry.path();
    if (path.extension() != ".json") continue;

    ++num_problems;

    psc::planner::MotionProblemDefinition motion_def;
    try {
      motion_def =
          psc::common::LoadFromJsonFile<psc::planner::MotionProblemDefinition>(
              path);
    } catch (const std::exception& e) {
      problem_load_errors[path] = e.what();

      continue;
    }
    loaded_file_problem_pairs.emplace_back(path, std::move(motion_def));
    if (!override_context_id) {
      context_ids.insert(motion_def.context_id().value());
    }
  }
  if (!problem_load_errors.empty()) {
    for (const auto& [path, msg] : problem_load_errors) {
      logging::log()->error("Failed to load problem {}: {}", path.string(),
                            msg);
    }
    logging::log()->error("Aborting SolveSavedProblems due to load errors");
    return 2;
  }
  logging::log()->info("Loaded {} problems from disk",
                       loaded_file_problem_pairs.size());

  if (override_context_id) {
    context_ids.insert(context_id);
    // Assert only one context id is available for use
    if (context_ids.size() > 1) {
      logging::log()->error(
          "Multiple context IDs found in problems but override context ID was "
          "provided!");
      return 2;
    }
  }

  // Load all planners to be used
  std::map<uint64_t, std::unique_ptr<draco::planner::DracoPlanner>>
      planner_cache;
  std::map<uint64_t, std::string> context_load_errors;
  logging::log()->info("Loading DracoPlanners for {} context IDs",
                       context_ids.size());
  const std::vector<uint64_t> context_ids_vec(context_ids.begin(),
                                              context_ids.end());
#pragma omp parallel for shared(planner_cache, context_load_errors)
  for (const auto& cid : context_ids_vec) {
    try {
      service::PlanContext context(cid);
      const fs::path context_base_dir = "/data/shokunin/contexts/";
      if (!service::utils::LoadContext(context, context_base_dir)) {
#pragma omp critical
        {
          context_load_errors[cid] = "Failed to load context from disk!";
        }
        continue;
      }
      bool require_parameters = true;
      bool is_builder = false;
      auto adapter = service::utils::MakeDracoAdapterFromContext(
          "shokunin", context, require_parameters,
          draco::VisualizerMode::kDraco);
      auto pdraco =
          std::make_unique<draco::planner::DracoPlanner>(adapter, is_builder);
#pragma omp critical
      {
        planner_cache[cid] = std::move(pdraco);
      }
      logging::log()->info("Created DracoPlanner for context {}", cid);
    } catch (const std::exception& e) {
#pragma omp critical
      {
        context_load_errors[cid] = e.what();
      }
      continue;
    }
  }

  if (!context_load_errors.empty()) {
    logging::log()->error(
        "Failed to create {} DracoPlanners due to context load errors:",
        context_load_errors.size());
    for (const auto& [cid, msg] : context_load_errors) {
      logging::log()->error("\t{}: {}", cid, msg);
    }
    return 2;
  }

  for (const auto& [path, motion_def] : loaded_file_problem_pairs) {
    const auto problem_ptr = motion_def.problem_clone();
    const auto start_sys_conf = motion_def.start_system_conf();

    const auto prob_ctx = motion_def.context_id().value();
    // Shouldn't be possible due to prior loading step
    if (!planner_cache.count(prob_ctx)) {
      logging::log()->error("No cached DracoPlanner found for context ID {}!",
                            prob_ctx);
      return 2;
    }

    // Solve using cached planner instance. Measure only SolvePlan time (not
    // creation).
    planning_service_client::planner::MotionPlanResult motion_plan_result;
    const auto start_time_clock = std::chrono::high_resolution_clock::now();
    try {
      auto& planner = *planner_cache[prob_ctx];
      motion_plan_result = planner.SolvePlan(
          *problem_ptr, path.string(), motion_def.maybe_plan_options(),
          start_sys_conf, motion_def.maybe_active_trajectory());
    } catch (const std::exception& e) {
      logging::log()->critical("SolvePlan threw exception for {}: {}",
                               path.string(), e.what());
      SavedProblemResult r;
      r.filename = path.filename().string();
      r.success = false;
      r.message = fmt::format("SolvePlan exception: {}", e.what());
      all_results.results.push_back(r);
      continue;
    }
    const auto end_time_clock = std::chrono::high_resolution_clock::now();
    const double solve_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            end_time_clock - start_time_clock)
            .count();

    SavedProblemResult res;
    res.filename = path.filename().string();
    res.success = motion_plan_result.is_success();
    res.message = motion_plan_result.message();
    res.solve_time_ms = solve_ms;

    if (!motion_plan_result.is_success()) {
      logging::log()->critical("SolvePlan failed for {}: {}", path.string(),
                               motion_plan_result.message());
      all_results.results.push_back(res);
      continue;
    }

    const auto& sys_traj = motion_plan_result.system_timed_trajectory();
    if (sys_traj.empty()) {
      logging::log()->warn("Empty system trajectory for {}", path.string());
      all_results.results.push_back(res);
      continue;
    }

    // Compute duration (max end time - min start time across system
    // trajectories)
    double start_time = std::numeric_limits<double>::infinity();
    double end_time = -std::numeric_limits<double>::infinity();
    for (const auto& [k, traj] : sys_traj) {
      start_time = std::min(start_time, traj.start_time());
      end_time = std::max(end_time, traj.end_time());
    }
    res.duration = (end_time > start_time) ? (end_time - start_time) : 0.0;

    // Compute configuration twirl (sum of generalized position step norms)
    res.configuration_twirl =
        shokunin::CalcTrajectoryVelocityTwirl(sys_traj, sample_dt);

    logging::log()->info(
        "Solved {} success={} solve_ms={} duration={} twirl={}", res.filename,
        res.success, solve_ms, res.duration, res.configuration_twirl);

    if (res.success) ++num_success;
    all_results.results.push_back(res);
  }

  logging::log()->critical("SolveSavedProblems: {}/{} problems solved",
                           num_success, num_problems);

  try {
    // log the absolute path of results_out
    logging::log()->info("Saving results to absolute path: {}",
                         std::filesystem::absolute(results_out));
    drake::yaml::SaveYamlFile<SavedProblemsResults>(results_out, all_results);
    auto results_str = drake::yaml::SaveYamlString(all_results);
    logging::log()->critical("Saved results to {}\n: {}", results_out,
                             results_str);
  } catch (const std::exception& e) {
    logging::log()->error("Failed to save results to {}: {}", results_out,
                          e.what());
    return 3;
  }

  return (num_problems == num_success) ? 0 : 1;
}
