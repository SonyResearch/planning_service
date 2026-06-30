#include "draco_planner.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

std::expected<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                        drake::trajectories::PiecewisePolynomial<double>>,
              std::string>
DracoPlanner::SolveOutOfViolationPlan(
    const planning_service_client::planner::OutOfViolation& def,
    const planning_service_client::SystemConf& start_sysconf) const {
  const auto q =
      conversions::ToGeneralizedPosition(robot_model(), start_sysconf);
  // Check if the configuration is valid. No motion planning is needed if the
  // configuration is valid.
  motion::CheckSatisfiedOptions options;
  options.verbose = true;
  options.collect_offending_model_names = true;
  auto check_satisfied_result =
      robot_constraints().CheckSatisfied(q, 0, options);
  if (check_satisfied_result.satisfied()) {
    const auto msg =
        "DracoPlanner:OutOfViolationProblem: The provided system "
        "configuration is already valid. No motion planning is needed.";
    return std::unexpected(msg);
  }
  std::vector<std::string> offending_model_names =
      check_satisfied_result.offending_model_names().value();
  DRAKE_DEMAND(!offending_model_names.empty());
  std::expected<drake::trajectories::PiecewisePolynomial<double>, std::string>
      path_opt;
  if (!def.use_gradient() && !def.twist().has_value()) {
    logging::log()->info(
        "OutOfViolationProblem: Using PRM to find the nearest valid "
        "configuration.");
    const auto& plant = robot_constraints().robot_model().plant();
    std::vector<drake::multibody::ModelInstanceIndex> fixed_models;
    for (int i = 0; i < plant.num_model_instances(); ++i) {
      auto model_idx = drake::multibody::ModelInstanceIndex(i);
      if (plant.num_positions(model_idx) == 0) {
        continue;  // Skip model instances that do not have any positions.
      }
      const auto& model_instance_name = plant.GetModelInstanceName(model_idx);
      // Insert to fixed_models if the model instance is not in the
      // offending_model_names.
      if (std::find(offending_model_names.begin(), offending_model_names.end(),
                    model_instance_name)
          == offending_model_names.end()) {
        fixed_models.push_back(model_idx);
        logging::log()->info(
            "OutOfViolationProblem: Non-Gradient approoach: Adding model "
            "instance {} to fixed models.",
            model_instance_name);
      }
    }
    auto maybe_prm_nearest_valid =
        thunder_planner_->CalcNearestValidConf(q, fixed_models);
    if (maybe_prm_nearest_valid.has_value()) {
      std::vector<Eigen::MatrixXd> prm_path = {q,
                                               maybe_prm_nearest_valid.value()};
      std::vector<double> breaks {0, 1};
      path_opt =
          drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
              breaks, prm_path);
    } else {
      std::string msg =
          "DracoPlanner:OutOfViolationProblem: PRM failed to find a valid "
          "configuration.";
      path_opt = std::unexpected(msg);
    }
  } else {
    logging::log()->info(
        "OutOfViolationProblem: Using gradient-based optimization to find the "
        "nearest valid configuration.");
    path_opt = motion::planning::MaybeOutOfViolationPath(
        robot_constraints(), q, def.influence_distance(),
        def.configuration_clearance_norm(), 0);
  }
  if (!path_opt.has_value()) {
    const auto msg =
        "DracoPlanner:OutOfViolationProblem: Failed to compute a path "
        "out of violation.";
    AddToVisualizer(q, "OutOfViolation failure");
    return std::unexpected(msg);
  }
  const auto& path = path_opt.value();
  // Toppra it
  const auto time_parametrization_opt {
      time_optimal_spliner().RunToppraOnPiecewiseTrajectory(path)};
  if (!time_parametrization_opt.has_value()) {
    const auto msg =
        "DracoPlanner:OutOfViolationProblem: Failed to time "
        "parameterize the out-of-violation path.";
    return std::unexpected(msg);
  }
  const auto& time_parametrization = time_parametrization_opt.value();
  // Convert to system timed trajectory
  return std::make_pair(path, time_parametrization);
}

}  // namespace planner
}  // namespace draco
