#include "draco_planner.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

SolvePlanResult DracoPlanner::SolveGlobalIKPlan(
    const planning_service_client::planner::GlobalIKProblem& def) const {
  // Retrieve problem parameters
  const std::vector<planning_service_client::FrameRelativePose>& poses =
      def.poses();
  const auto& fixed_sysconf_opt = def.fixed_system_conf_opt();
  const auto& ik_seed_sysconf_opt = def.ik_seed_system_conf_opt();
  // Sanity check
  if (poses.empty()) {
    return std::unexpected(
        "DracoPlanner:SolveGlobalIKPlan: No poses provided.");
  }
  // Build an anchor
  planning_service_client::planner::Anchor anchor {
      fixed_sysconf_opt.value_or(planning_service_client::SystemConf()),
      poses,
  };
  // Resolve one anchor point
  std::optional<Eigen::VectorXd> q_ik_seed;
  if (ik_seed_sysconf_opt.has_value()) {
    q_ik_seed = conversions::ToGeneralizedPosition(robot_model(),
                                                   ik_seed_sysconf_opt.value());
  }
  auto start_conf_exp = ResolveAnchorConf(anchor, q_ik_seed);
  // If failure
  if (!start_conf_exp.has_value()) {
    auto msg = start_conf_exp.error();
    logging::log()->error(
        "DracoPlanner:SolveGlobalIKPlan: Failed to resolve the "
        "anchor configuration: {}",
        msg);
    return std::unexpected(
        "DracoPlanner:SolveGlobalIKPlan: Failed to resolve the "
        "anchor configuration: "
        + msg);
  }
  // Make a trajectory with epsilon time from the solution
  const auto& start_conf = start_conf_exp.value();
  std::vector<double> times {0.0, 1e-6};
  std::vector<Eigen::MatrixXd> samples {start_conf, start_conf};
  auto path = drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
  auto time_parametrization =
      motion::splining::internal::MakeUniformTimingForPath(path);
  return std::make_pair(path, time_parametrization);
}

}  // namespace planner
}  // namespace draco
