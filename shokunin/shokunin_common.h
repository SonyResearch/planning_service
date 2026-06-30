#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/service/types/error.h"
#include "planning_service/service/utils/utils.h"
#include "planning_service_client/common/io_utils.h"
#include "planning_service_client/planner/motion_problem_definition.h"
#include "planning_service_client/trajectories.h"

// need CLI11
#include <CLI/CLI.hpp>

namespace fs = std::filesystem;

namespace shokunin {

draco::planner::DracoPlanner MakeDracoPlannerFromContext(
    uint64_t context_id,
    draco::VisualizerMode viz_mode = draco::VisualizerMode::kDraco,
    bool is_builder = false) {
  service::PlanContext context(context_id);
  fs::path context_base_dir = "/data/shokunin/contexts/";
  bool load_context = service::utils::LoadContext(context, context_base_dir);
  logging::log()->info("Load context: {}", load_context);
  // Make draco adapter
  bool require_parameters = true;
  auto adapter = service::utils::MakeDracoAdapterFromContext(
      "shokunin", context, require_parameters, viz_mode);
  // Make Draco planner
  return draco::planner::DracoPlanner(adapter, is_builder);
}

double CalcTrajectoryConfigurationTwirl(
    const planning_service_client::TimedTrajectory& sys_traj,
    double t_step = 0.05) {
  // Get start, end
  auto start = sys_traj.Value(sys_traj.start_time());
  auto end = sys_traj.Value(sys_traj.end_time());
  double duration = sys_traj.duration();
  // Calculate twirl: maximum deviation from outside the box of start and end
  double max_twirl = 0.0;
  for (double t = 0; t < duration; t += t_step) {
    auto conf = sys_traj.Value(t);
    for (int i = 0; i < conf.size(); i++) {
      double v = conf(i);
      double min_val = std::min(start(i), end(i));
      double max_val = std::max(start(i), end(i));
      double twirl = 0.0;
      if (v < min_val) {
        twirl = std::max(twirl, std::abs(v - min_val));
      } else if (v > max_val) {
        twirl = std::max(twirl, std::abs(v - max_val));
      }
      max_twirl = std::max(max_twirl, twirl);
    }
  }
  return max_twirl;
}

double CalcTrajectoryVelocityTwirl(
    const planning_service_client::SystemTimedTrajectory& sys_traj,
    double t_step = 0.05) {
  double max_twirl = 0.0;
  for (const auto& [name, traj] : sys_traj) {
    max_twirl =
        std::max(max_twirl, CalcTrajectoryConfigurationTwirl(traj, t_step));
  }
  return max_twirl;
}
}  // namespace shokunin
