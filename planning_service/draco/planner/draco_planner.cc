#include "draco_planner.h"

#include <magic_enum/magic_enum.hpp>

#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

DracoPlanner::DracoPlanner(const DracoAdapter& draco_adapter, bool is_builder)
    : Draco(draco_adapter),
      iris_regions_adapter_ {draco_adapter.iris_regions_adapter},
      cartesian_dynamic_limits_map_ {
          draco_adapter.cartesian_dynamic_limits_map},
      time_optimal_spline_params_ {draco_adapter.time_optimal_spline_params},
      time_optimal_spliner_ {
          robot_model(), draco_adapter.joint_dynamic_limits_map,
          *cartesian_dynamic_limits_map_, *time_optimal_spline_params_},
      cubic_spliner_ {robot_constraints()},
      iris_inspector_ {std::make_unique<motion::iris::IrisInspector>(
          robot_constraints(), iris_regions_adapter_)},
      thunder_planner_ {
          std::make_unique<motion::planning::ompl::ThunderPlanner>(
              robot_constraints(), draco_adapter.thunder_parameters,
              draco_adapter.thunder_dat_file)},
      ik_planner_ {std::make_unique<motion::planning::IkPlanner>(
          robot_constraints(), thunder_planner_->vertices_confs())},
      problems_dir_ {
          draco_adapter.problems_dir.empty()
              ? fs::path("/logs") / draco_adapter.system
                    / "artifact_builder_problems"
                    / std::to_string(robot_constraints().constraints_hash())
              : draco_adapter.problems_dir},
      artifact_builder_ {
          is_builder
              ? std::make_unique<motion::planning::ArtifactBuilder>(
                    std::make_unique<motion::planning::ompl::ThunderPlanner>(
                        robot_constraints(), draco_adapter.thunder_parameters,
                        draco_adapter.thunder_dat_file),
                    draco_adapter.iris_builder_options,
                    draco_adapter.iris_regions_adapter_file,
                    draco_adapter.context_dir, problems_dir_)
              : nullptr} {
  // ToDo(@ramy): add back non-builder conditional once we get rid of migration
  // after camera calibration
  if (/* !is_builder && */ !iris_regions_adapter_.regions_vec().empty()) {
    try {
      single_mode_gcs_planner_ =
          std::make_unique<motion::planning::SingleModeGcsPlanner>(
              robot_constraints(), iris_regions_adapter_,
              time_optimal_spliner_.joint_dynamic_limits().velocity_bound,
              draco_adapter.gcs_planner_options);
    } catch (const std::exception& e) {
      logging::log()->error(
          "Draco:Ctor: Failed to initialize GCS planner due to exception: {}",
          e.what());
      logging::log()->warn(
          "Draco:Ctor: GCS-based planning WILL NOT be possible!");
    }
  }
  // Make time optimal spliners for each arm
  if (robot_model().num_arms() > 1) {
    logging::log()->info(
        "DracoPlanner:Ctor: Multi-arm robots exist. Constructing time "
        "optimal spliners for each arm.");
    for (int i = 0; i < robot_model().num_arms(); ++i) {
      auto arm_index = motion::ArmIndex(i);
      arms_time_optimal_spliners_[arm_index] =
          std::make_unique<motion::splining::TimeOptimalSpliner>(
              robot_model(), draco_adapter.joint_dynamic_limits_map,
              *cartesian_dynamic_limits_map_, *time_optimal_spline_params_,
              arm_index);
      logging::log()->info(
          "DracoPlanner:Ctor: Created time optimal spliner for arm {} with {} "
          "DOFs.",
          i, robot_model().GetArm(arm_index).plant().num_positions());
    }
  } else {
    logging::log()->info(
        "DracoPlanner:Ctor: Single-arm robot. No additional time optimal "
        "spliners will be created.");
  }
  if (robot_model().meshcat()) {
    auto msg = fmt::format(fg(FMT_YELLOW),
                           "DracoPlanner: {} has native meshcat at port {}",
                           robot_constraints().constraints_hash(),
                           robot_model().meshcat()->port());
    logging::log()->info(msg);
  }
  // Let's make meshcat visualizer
  if (options().visualizer_options.mode == draco::VisualizerMode::kDraco
      && draco_adapter.robot_meshcat_params.has_value()) {
    DRAKE_DEMAND(robot_model().meshcat() == nullptr);
    draco_visualizer_ = std::make_unique<visualizer::DracoVisualizer>(
        draco_adapter.xml_file, draco_adapter.dmd,
        draco_adapter.robot_meshcat_params.value(),
        draco_adapter.constraints_adapter, options().default_configuration,
        options().visualizer_options.plans_buffer_size,
        options().visualizer_options.display_trajectory_frequency,
        options().visualizer_options.meshcat_beat_interval_ms);
  } else {
    logging::log()->info(
        "DracoPlanner:Ctor: Visualizer not created for Draco instance {} in "
        "visualizer mode {}",
        robot_constraints().constraints_hash(),
        magic_enum::enum_name(options().visualizer_options.mode));
  }
}

std::tuple<
    common::ScopedOverride<motion::splining::cartesian_dynamic_limits_map_t>,
    common::ScopedOverride<motion::splining::TimeOptimalSplineParams>>
DracoPlanner::ApplyScopedDynamicLimits(
    const std::optional<psc::planner::DynamicLimits>& dynamic_limits) const {
  // Override splining parameters
  common::ScopedOverride cartesian_limits_scope(cartesian_dynamic_limits_map_);
  common::ScopedOverride spline_params_scope(time_optimal_spline_params_);
  if (!dynamic_limits.has_value()) {
    return {std::move(cartesian_limits_scope), std::move(spline_params_scope)};
  }
  // Set splining parameters on set/restore.
  cartesian_limits_scope.on_event([this]() {
    SetActiveCartesianDynamicLimits();
  });
  spline_params_scope.on_event([this]() {
    SetActiveTimeOptimalSplineParams();
  });
  // Override splining parameters
  auto params = spline_params_scope.clone();
  if (dynamic_limits->safety_factor_velocity > 0.0) {
    params.safety_factor_velocity = dynamic_limits->safety_factor_velocity;
  }
  if (dynamic_limits->safety_factor_acceleration > 0.0) {
    params.safety_factor_acceleration =
        dynamic_limits->safety_factor_acceleration;
  }
  if (dynamic_limits->safety_factor_torque > 0.0) {
    params.safety_factor_torque = dynamic_limits->safety_factor_torque;
  }
  spline_params_scope.set(params);
  logging::log()->info(
      "DracoPlanner:ApplyScopedDynamicLimits: Overrode safety factors (vel: "
      "{:.3f}, acc: {:.3f}, torque: {:.3f})",
      params.safety_factor_velocity, params.safety_factor_acceleration,
      params.safety_factor_torque);
  // Override cartesian dynamic limits
  if (!dynamic_limits->cartesian_velocity_limits.empty()) {
    motion::splining::cartesian_dynamic_limits_map_t limits_map;
    for (const auto& [name, vel] : dynamic_limits->cartesian_velocity_limits) {
      motion::splining::CartesianDynamicLimits limits;
      limits.speed_limit = vel;
      limits_map[name] = limits;
    }
    // Validate then assign
    cartesian_limits_scope.set(limits_map);
    logging::log()->info(
        "DracoPlanner:ApplyScopedDynamicLimits: Overrode cartesian dynamic "
        "limits ({})",
        dynamic_limits->cartesian_velocity_limits);
  }
  return {std::move(cartesian_limits_scope), std::move(spline_params_scope)};
}

planning_service_client::CheckSatisfiedResponse DracoPlanner::CheckSatisfied(
    const std::vector<psc::SystemConf>& system_conf_vec,
    const std::optional<psc::planner::CollisionOptions>& collision_options)
    const {
  auto scopes {ApplyScopedCollisionOptions(collision_options)};
  std::optional<motion::CheckSatisfiedResult> result;
  motion::CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  check_satisfied_options.collect_offending_model_names = true;
  check_satisfied_options.collect_failed_constraint_strings = true;
  std::vector<Eigen::VectorXd> vq;
  vq.reserve(system_conf_vec.size());
  for (const auto& system_conf : system_conf_vec) {
    vq.push_back(draco::conversions::ToGeneralizedPosition(
        robot_model(), system_conf,
        draco::conversions::ToGeneralizedBehavior::kThrowOnMissing));
  }
  if (vq.size() == 1) {
    // If only one configuration, call the single-configuration version of
    // CheckSatisfied to get more detailed info (e.g. which constraints failed).
    result = robot_constraints_->CheckSatisfied(vq.front(), 0,
                                                check_satisfied_options);
  } else {
    // Otherwise, call the vectorized version for efficiency, which only returns
    // whether the constraints are satisfied or not.
    result = robot_constraints_->CheckSatisfied(vq, check_satisfied_options);
  }
  // Now we need to convert CheckSatisfiedResult to CheckSatisfiedResponse,
  // which is what the MPS gRPC service returns.
  DRAKE_DEMAND(result.has_value());
  DRAKE_DEMAND(result->failed_constraint_strings().has_value());
  DRAKE_DEMAND(result->offending_model_names().has_value());
  return planning_service_client::CheckSatisfiedResponse(
      result->satisfied(), result->failed_constraint_strings().value(),
      result->offending_model_names().value());
}

planning_service_client::planner::MotionPlanResult DracoPlanner::SolvePlan(
    const planning_service_client::planner::PlanningProblemBase& problem,
    const std::string& label,
    const std::optional<planning_service_client::planner::PlanOptions>&
        maybe_plan_options,
    const std::optional<planning_service_client::SystemConf>&
        maybe_start_sysconf,
    const std::optional<planning_service_client::SystemTimedTrajectory>&
        maybe_active_trajectory) const {
  // Default-constructed plan options have nullopt members
  const auto plan_options {maybe_plan_options.value_or(
      planning_service_client::planner::PlanOptions())};
  const auto maybe_traj_update_options {
      plan_options.maybe_trajectory_update_options()};
  // Apply and collect scoped overrides
  auto collision_scope {
      ApplyScopedCollisionOptions(plan_options.maybe_collision_options())};
  auto dynamic_limits_scope {
      ApplyScopedDynamicLimits(plan_options.maybe_dynamic_limits())};
  std::optional<
      planning_service_client::planner::GeneralizedMultimodalPlanningProblem>
      gen_mm_problem_opt;
  // If problem is StartToGoalProblem, convert it to
  // GeneralizedMultimodalPlanningProblem
  if (const auto* p_start_to_goal = dynamic_cast<
          const planning_service_client::planner::StartToGoalProblem*>(
          &problem)) {
    bool allow_partial_start_anchor_solutions = false;
    bool allow_partial_goal_anchor_solutions = false;
    gen_mm_problem_opt =
        planning_service_client::planner::GeneralizedMultimodalPlanningProblem(
            {}, {p_start_to_goal->goal()},
            p_start_to_goal->fast_estimate_solution(),
            allow_partial_start_anchor_solutions,
            allow_partial_goal_anchor_solutions,
            p_start_to_goal->allow_async_partial_solutions());
  } else if (const auto* p_cartesian_linear_move =
                 dynamic_cast<const planning_service_client::planner::
                                  CartesianLinearMoveProblem*>(&problem)) {
    auto gen_problem_result =
        ConstructGeneralizedProblemFromCartesianLinearProblem(
            *p_cartesian_linear_move, 1e-2, 2e-2);
    if (gen_problem_result.has_value()) {
      gen_mm_problem_opt = gen_problem_result.value();
    } else {
      // Failed to construct generalized problem, return error result
      planning_service_client::planner::MotionPlanResult error_result;
      error_result.SetFailureStatus();
      error_result.SetMessage(gen_problem_result.error());
      return error_result;
    }
  } else if (const auto* p_multimodal =
                 dynamic_cast<const planning_service_client::planner::
                                  MultimodalPlanningProblem*>(&problem)) {
    gen_mm_problem_opt =
        ConstructGeneralizedProblemFromMultimodal(*p_multimodal, 1e-2, 2e-2);
  }
  logging::log()->info(
      "DracoPlanner:SolvePlan: Planning Problem cast as generalized "
      "multimodal: {}",
      gen_mm_problem_opt.has_value() ? "Yes" : "No");
  planning_service_client::planner::MotionPlanResult result;
  if (gen_mm_problem_opt.has_value()) {
    // Solve as generalized multimodal
    result = SolvePlanImpl(gen_mm_problem_opt.value(), maybe_start_sysconf,
                           maybe_active_trajectory, maybe_traj_update_options);
  } else {
    // A supported problem type was not cast to generalized multimodal.
    result = SolvePlanImpl(problem, maybe_start_sysconf,
                           maybe_active_trajectory, maybe_traj_update_options);
  }
  // If the result is successful, we can visualize it.
  if (result.is_success()) {
    AddToVisualizer(result.system_timed_trajectory(), label);
  }
  // If a resource's result is not kNew or kUpdate, remove it
  std::vector<std::string> resources;
  auto systraj = result.system_timed_trajectory();
  resources.reserve(systraj.size());
  std::transform(systraj.begin(), systraj.end(), std::back_inserter(resources),
                 [](const auto& kv) {
                   return kv.first;
                 });
  for (const auto& resource : resources) {
    const auto res_type = result.GetResourceResultType(resource);
    if (res_type
            != planning_service_client::planner::MotionPlanResult::
                MotionResultType::kNewAndTargeted
        && res_type
               != planning_service_client::planner::MotionPlanResult::
                   MotionResultType::kNewNotTargeted
        && res_type
               != planning_service_client::planner::MotionPlanResult::
                   MotionResultType::kUpdate) {
      logging::log()->info(
          "DracoPlanner:SolvePlan: Removing resource {} from result as its "
          "result type is {}",
          resource, magic_enum::enum_name(res_type));
      systraj.erase(resource);
    }
  }
  result.SetSystemTimedTrajectory(systraj);
  return result;
}

motion::planning::PlanningArtifactStatus
DracoPlanner::GetPlanningArtifactsSizes() const {
  motion::planning::PlanningArtifactStatus status;

  // Get roadmap statistics from thunder planner
  if (thunder_planner_) {
    const auto& planner_data = thunder_planner_->planner_data();
    status.num_roadmap_vertices = static_cast<int>(planner_data.numVertices());
    status.num_roadmap_edges = static_cast<int>(planner_data.numEdges());

    logging::log()->debug(
        "DracoPlanner::GetPlanningArtifactsSizes: Thunder planner has {} "
        "vertices and {} edges",
        status.num_roadmap_vertices, status.num_roadmap_edges);
  } else {
    logging::log()->warn(
        "DracoPlanner::GetPlanningArtifactsSizes: Thunder planner is not "
        "available");
    status.num_roadmap_vertices = 0;
    status.num_roadmap_edges = 0;
  }

  // Get IRIS regions count from iris inspector
  if (iris_inspector_) {
    status.num_iris_regions = static_cast<int>(
        iris_inspector_->iris_regions_adapter().regions_vec().size());

    logging::log()->debug(
        "DracoPlanner::GetPlanningArtifactsSizes: IRIS inspector has {} "
        "regions",
        status.num_iris_regions);
  } else {
    logging::log()->warn(
        "DracoPlanner::GetPlanningArtifactsSizes: IRIS inspector is not "
        "available");
    status.num_iris_regions = 0;
  }

  logging::log()->info(
      "DracoPlanner::GetPlanningArtifactsSizes: Roadmap: {} vertices, {} "
      "edges; "
      "IRIS: {} regions",
      status.num_roadmap_vertices, status.num_roadmap_edges,
      status.num_iris_regions);

  return status;
}

std::expected<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                        drake::trajectories::PiecewisePolynomial<double>>,
              std::string>
DracoPlanner::SplineAndTimePath(std::vector<Eigen::VectorXd> path,
                                bool fast_estimate_solution) const {
  // Make matrix representing path
  std::vector<double> times;
  Eigen::MatrixXd path_matrix(path[0].size(), path.size());
  for (int i = 0; i < static_cast<int>(path.size()); ++i) {
    path_matrix.col(i) = path[i].transpose();
    times.push_back(static_cast<double>(i) / (path.size() - 1));
  }
  logging::log()->debug(
      "DracoPlanner:SplineAndTimePath: The path matrix has {} rows and {} "
      "columns.",
      path_matrix.rows(), path_matrix.cols());
  motion::splining::CubicSpliningParameters splining_parameters;
  auto pp_path_opt =
      cubic_spliner_.WayptsToValidPath(path_matrix, splining_parameters);
  if (!pp_path_opt.has_value()) {
    auto msg =
        "Draco:SplineAndTimePath: Failed to create a valid piecewise "
        "polynomial path from the given waypoints.";
    logging::log()->error("{}", msg);
    return std::unexpected(msg);
  }

  const auto& pp_path = pp_path_opt.value().first;

  Eigen::VectorXd times_vec;
  times_vec.resize(times.size());
  logging::log()->debug(
      "DracoPlanner:SplineAndTimePath: The times vec has {} points.",
      times.size());
  for (size_t i = 0; i < times.size(); ++i) {
    times_vec(i) = times[i];
  }
  // Create a time parameterization
  if (fast_estimate_solution) {
    auto path =
        drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            times_vec, path_matrix);
    auto time_parametrization =
        motion::splining::internal::MakeUniformTimingForPath(pp_path);
    return std::make_pair(pp_path, time_parametrization);
  }
  const auto time_parametrization_opt =
      time_optimal_spliner().RunToppraOnPiecewiseTrajectory(pp_path);
  if (!time_parametrization_opt.has_value()) {
    auto msg = "Draco:SplineAndTimePath: Failed to time parameterize the path.";
    logging::log()->error("{}", msg);
    return std::unexpected(msg);
  }
  const auto& time_parametrization = time_parametrization_opt.value();
  return std::make_pair(pp_path, time_parametrization);
}

/**
 * @private
 * @brief Plan using fastest/best available planner. This method will return a
 * trajectory that is not time parameterized.
 * @param q1 Start state as an array of joint positions
 * @param q2 Goal state as an array of joint positions
 */
std::expected<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                        drake::trajectories::PiecewisePolynomial<double>>,
              std::string>
DracoPlanner::CalcTrajectoryBestAvailablePlanner(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
    bool fast_estimate_plan) const {
  if (fast_estimate_plan) {
    auto time_now = std::chrono::high_resolution_clock::now();
    auto result_opt = FastEstimatePlan(q1, q2);
    if (!result_opt.has_value()) {
      return std::unexpected(
          "Draco:CalcPathBestAvailablePlanner: Fast estimate plan failed");
    }
    logging::log()->info(
        "Draco:CalcPathBestAvailablePlanner: Fast estimate plan took {} ms",
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::high_resolution_clock::now() - time_now)
            .count());
    auto time_parameterization =
        motion::splining::internal::MakeUniformTimingForPath(
            result_opt.value());
    return std::make_pair(result_opt.value(), time_parameterization);
  }
  // Save
  SaveConfigSpaceProblem(q1, q2);
  bool straight_line_tried, gcs_tried, sample_based_tried = false;
  // try straight line first
  if (options().planner_options.use_straight_path_planner) {
    auto path_opt = CalcStraightLinePath(q1, q2);
    if (path_opt.has_value()) {
      logging::log()->info(
          "Draco:CalcPathBestAvailablePlanner: Straight line planner "
          "succeeded.");
      const auto path = path_opt.value();
      const auto time_parametrization_opt =
          time_optimal_spliner().RunToppraOnPiecewiseTrajectory(path);
      if (!time_parametrization_opt.has_value()) {
        logging::log()->error(
            "Draco:CalcPathBestAvailablePlanner: Failed to time "
            "parameterize the straight line path.");
        return std::unexpected(
            "Draco:CalcPathBestAvailablePlanner: Failed to time "
            "parameterize the straight line path.");
      }
      const auto& time_parametrization = time_parametrization_opt.value();
      return std::make_pair(path, time_parametrization);
    } else {
      logging::log()->info(
          "Draco:CalcPathBestAvailablePlanner: Straight line planner "
          "failed.");
      straight_line_tried = true;
    }
  }
  // try GCS
  if (options().planner_options.use_gcs_planner) {
    try {
      auto path_opt = CalcGcsPath(q1, q2);
      if (path_opt.has_value()) {
        logging::log()->info(
            "Draco:CalcPathBestAvailablePlanner: GCS planner succeeded.");
        const auto path = path_opt.value();
        const auto time_parametrization_opt =
            time_optimal_spliner().RunToppraOnPiecewiseTrajectory(path);
        if (!time_parametrization_opt.has_value()) {
          logging::log()->error(
              "Draco:CalcPathBestAvailablePlanner: Failed to time "
              "parameterize the GCS path.");
          return std::unexpected(
              "Draco:CalcPathBestAvailablePlanner: Failed to time "
              "parameterize the GCS path.");
        }
        const auto& time_parametrization = time_parametrization_opt.value();
        return std::make_pair(path, time_parametrization);
      } else {
        logging::log()->info(
            "Draco:CalcPathBestAvailablePlanner: GCS planner failed.");
        gcs_tried = true;
      }
    } catch (const std::exception& e) {
      logging::log()->error(
          "Draco:CalcPathBestAvailablePlanner: Failed to compute GCS path "
          "between q1 and q2: {}",
          e.what());
    }
  }
  if (options().planner_options.use_sample_based_planner) {
    // try sample-based
    try {
      auto path_opt = CalcSampleBasedPath(q1, q2);
      if (path_opt.has_value()) {
        logging::log()->info(
            "Draco:CalcPathBestAvailablePlanner: Sample-based planner "
            "succeeded.");
        const auto path = path_opt.value();
        const auto time_parametrization_opt =
            time_optimal_spliner().RunToppraOnPiecewiseTrajectory(path);
        if (!time_parametrization_opt.has_value()) {
          logging::log()->error(
              "Draco:CalcPathBestAvailablePlanner: Failed to time "
              "parameterize the sample-based path.");
          return std::unexpected(
              "Draco:CalcPathBestAvailablePlanner: Failed to time "
              "parameterize the sample-based path.");
        }
        const auto& time_parametrization = time_parametrization_opt.value();
        return std::make_pair(path, time_parametrization);
      } else {
        logging::log()->info(
            "Draco:CalcPathBestAvailablePlanner: Sample-based planner "
            "failed.");
        sample_based_tried = true;
      }
    } catch (const std::exception& e) {
      logging::log()->error(
          "Draco:CalcPathBestAvailablePlanner: Failed to compute sample-based "
          "path between q1 and q2: {}",
          e.what());
    }
  }
  auto msg = fmt::format(
      "Draco:CalcPathBestAvailablePlanner: All planners failed. Tried: "
      "Straight line: "
      "{}, GCS: {}, Sample-based: {}.",
      straight_line_tried ? "yes" : "no", gcs_tried ? "yes" : "no",
      sample_based_tried ? "yes" : "no");
  logging::log()->error(msg);
  return std::unexpected(msg);
}

void DracoPlanner::SaveConfigSpaceProblem(const Eigen::VectorXd& q_start,
                                          const Eigen::VectorXd& q_goal) const {
  motion::planning::ConfigSpacePlanningProblem problem {q_start, q_goal};
  const auto problem_filename {fmt::format(
      "problem-{}.yaml",
      std::hash<motion::planning::ConfigSpacePlanningProblem>()(problem))};
  // Create directory if it does not exist
  if (!fs::exists(problems_dir_)) {
    fs::create_directories(problems_dir_);
    logging::log()->info(
        "DracoPlanner:SaveConfigSpaceProblem: Created directory {} for "
        "storing planning problems.",
        problems_dir_.string());
  }
  drake::yaml::SaveYamlFile(problems_dir_ / problem_filename, problem);
}

std::optional<drake::trajectories::PiecewisePolynomial<double>>
DracoPlanner::CalcGcsPath(const Eigen::VectorXd& q1,
                          const Eigen::VectorXd& q2) const {
  DRAKE_THROW_UNLESS(single_mode_gcs_planner_ != nullptr);
  const auto path_opt = single_mode_gcs_planner_->CalcOptimalPath(q1, q2);
  if (!path_opt.has_value()) {
    logging::log()->error(
        "Draco:CalcGcsPath: GCS failed to compute a path "
        "(CompositeTrajectory)");
    return std::nullopt;
  }
  logging::log()->info(
      "Draco:CalcGcsPath: GCS computed a trajectory with {} segments",
      path_opt.value().get_number_of_segments());
  const auto path = path_opt.value();
  // Let's collision check the path
  const bool is_path_valid =
      robot_constraints_->CheckSatisfiedTrajectory(path, 0.02);
  if (!is_path_valid) {
    logging::log()->error(
        "Draco:CalcGcsPath: The GCS path is not valid! There might be "
        "collisions or constraint violations along the path.");
    return std::nullopt;
  }
  double end_time = path.end_time();
  DRAKE_DEMAND(std::abs(path.start_time()) < 1e-8);
  const auto path_normalized = drake::planning::trajectory_optimization::
      GcsTrajectoryOptimization::NormalizeSegmentTimes(path);
  double end_time_normalized = path_normalized.end_time();
  auto pp_path =
      motion::splining::internal::CompositeBezierCurveToPiecewisePolynomial(
          path_normalized);
  // Ensure end time normalization is reasonable (magic number)
  DRAKE_DEMAND(end_time_normalized < 1e8);
  // Rescale pp_path to reflect the original timing
  pp_path.ScaleTime(end_time / end_time_normalized);
  bool fix_arms_in_path = true;  // TODO: make this a parameter in Options
  if (fix_arms_in_path) {
    auto fixed_idle_arm_path_opt = MaybeFixArmsInPath(pp_path);
    if (fixed_idle_arm_path_opt.has_value()) {
      pp_path = fixed_idle_arm_path_opt.value();
      logging::log()->info(
          "Draco:CalcGcsPath: Fixed idle arms in the GCS path.");
    }
  }
  return pp_path;
}

/**
 * @private
 * @brief Compute a time-optimal trajectory using the RRT planner. The
 * computed trajectory will not be time parameterized.
 *
 * @param q1 Start state as an array of joint positions
 * @param q2 Goal state as an array of joint positions
 */
std::optional<drake::trajectories::PiecewisePolynomial<double>>
DracoPlanner::CalcSampleBasedPath(const Eigen::VectorXd& q1,
                                  const Eigen::VectorXd& q2) const {
  const bool try_recall {true};
  const bool save_solution {false};
  std::optional<std::vector<Eigen::VectorXd>> waypts_vec_opt;
  waypts_vec_opt = thunder_planner_->SolveRecallPlan(q1, q2);
  if (waypts_vec_opt.has_value() && try_recall) {
    logging::log()->info(
        "Draco:CalcSampleBasedPath: Successfully recalled a solution from "
        "the cache with {} waypoints.",
        waypts_vec_opt->size());
  } else {
    logging::log()->info(
        "Draco:CalcSampleBasedPath: No cached solution found. "
        "Computing a new solution using the sample-based planner.");
    waypts_vec_opt = thunder_planner_->SolveParallelPlan(q1, q2, save_solution);
  }

  if (!waypts_vec_opt) {
    logging::log()->error(
        "Draco:CalcSampleBasedPath: Failed to compute waypoints!");
    return std::nullopt;
  }

  const auto path_opt {cubic_spliner_.WayptsToValidPath(
      *waypts_vec_opt, motion::splining::CubicSpliningParameters())};
  if (!path_opt.has_value()) {
    logging::log()->error(
        "Draco:CalcSampleBasedPath: Failed to compute cubic spline "
        "q(s)!");

    return std::nullopt;
  }
  auto pp_path = path_opt->first;
  bool fix_arms_in_path = true;  // TODO: make this a parameter in Options
  if (fix_arms_in_path) {
    auto fixed_idle_arm_path_opt = MaybeFixArmsInPath(pp_path);
    if (fixed_idle_arm_path_opt.has_value()) {
      pp_path = fixed_idle_arm_path_opt.value();
      logging::log()->info(
          "Draco:CalcSampleBasedPath: Fixed idle arms in the sampling-based "
          "path.");
    }
  }
  return pp_path;
}

/**
 * @private
 * @brief Compute a time-optimal trajectory by evaluating the straight-line
 * path in joint space. The computed trajectory will not be time
 * parameterized.
 *
 * @param q1 Start state as an array of joint positions
 * @param q2 Goal state as an array of joint positions
 * @param result_ptr The motion plan result (as a pointer)
 */
std::optional<drake::trajectories::PiecewisePolynomial<double>>
DracoPlanner::CalcStraightLinePath(const Eigen::VectorXd& q1,
                                   const Eigen::VectorXd& q2) const {
  const auto straight_line_path {motion::planning::MaybeValidStraightLinePath(
      robot_constraints(), q1, q2)};

  if (!straight_line_path) {
    logging::log()->error(
        "Draco:CalcStraightLinePath: Failed to compute a straight-line path!");
    return std::nullopt;
  }
  return straight_line_path.value();
}

/**
 * @brief Calculates a fast estimate of the plan length.
 *
 * @return If plan found, the estimated plan length, otherwise nullopt.
 */
std::optional<drake::trajectories::PiecewisePolynomial<double>>
DracoPlanner::FastEstimatePlan(const Eigen::VectorXd& q1,
                               const Eigen::VectorXd& q2) const {
  // If q1 and q2 are the same, return a zero cost
  double kConfigEpsilon {1e-6};  // TODO:(@sadraddini) move to options
  double kTimeEpsilon {1e-6};    // TODO:(@sadraddini) move to options
  if ((q1 - q2).norm() < kConfigEpsilon) {
    auto pp_path =
        drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
            {0.0, kTimeEpsilon}, {q1, q2});
    return pp_path;
  }
  bool success = false;
  double path_cost_estimate = 0;
  // First try the straight line planner
  if (motion::planning::MaybeValidStraightLinePath(robot_constraints(), q1, q2)
          .has_value()) {
    const auto& metric = single_mode_gcs_planner_->metric();
    path_cost_estimate = metric(q1, q2);
    success = true;
  }
  // If no success, try A* on the GCS planner
  if (!success) {
    if (auto cost_opt =
            single_mode_gcs_planner_->FastEstimatePathLength(q1, q2);
        cost_opt.has_value()) {
      path_cost_estimate = cost_opt.value();
      success = true;
    }
  }
  if (!success) {
    logging::log()->error(
        "Draco:FastEstimatePlan: GCS and straight line planners "
        "failed to compute a fast estimate of the plan!"
        "FastEstimatePlan has not been implemented yet for the "
        "other planners.");
    return std::nullopt;
  }
  // HACK: We are returning the path cost estimate as time.
  std::vector<Eigen::MatrixXd> samples;
  std::vector<double> times;
  samples.push_back(q1);
  samples.push_back(q2);
  times.push_back(0.0);
  times.push_back(path_cost_estimate);
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, samples);
}

/** Completes a partial system configuration and returns the complete
 * configuration that is closest to the seed configuration.
 *
 * @param sys_conf The partial system configuration
 * @param q_seed The seed configuration that has to be made close to.
 * @return std::optional<Eigen::VectorXd> The closest valid complete
 * configuration to the seed configuration that completes the partial
 * configuration.
 */
std::optional<Eigen::VectorXd> DracoPlanner::CalcClosestValidCompleteConf(
    const motion::system_conf_t& partial_sysconf,
    const Eigen::VectorXd q_seed) const {
  motion::CheckSatisfiedOptions options;
  options.verbose = true;
  Eigen::VectorXd q = q_seed;
  std::vector<drake::multibody::ModelInstanceIndex> fixed_model_intances;
  for (const auto& [model_name, conf] : partial_sysconf) {
    auto model_idx = robot_model().plant().GetModelInstanceByName(model_name);
    robot_model().plant().SetPositionsInArray(model_idx, conf, &q);
    fixed_model_intances.push_back(model_idx);
  }
  // Option 1: we just complete with seed and see if it is valid
  {
    if (robot_constraints().CheckSatisfied(q, 0, options)) {
      logging::log()->info(
          "Draco:CalcClosestValidCompleteConf: The completed configuration "
          "[{}] is valid. No need to find closest.",
          q.transpose());
      return q;
    }
    logging::log()->error(
        "Draco:CalcClosestValidCompleteConf: The completed configuration [{}]"
        "is not valid",
        q.transpose());
  }
  // Option 2: Find nearest valid configuration to the Iris regions.
  const auto closest_conf_opt =
      iris_inspector().CalcClosestValidConfToRegions(q, fixed_model_intances);
  if (closest_conf_opt.has_value()) {
    logging::log()->info(
        "Draco:CalcClosestValidCompleteConf: Found closest valid configuration "
        "to the Iris regions.");
    return closest_conf_opt.value();
  }
  // Option 3 by solving a nonlinear non-convex program.
  auto q_opt = robot_constraints().CalcClosestSatisfyingConfiguration(
      q, 0, fixed_model_intances);
  if (q_opt.has_value()) {
    logging::log()->info(
        "Draco:CalcClosestValidCompleteConf: Found closest valid configuration "
        "by solving a nonlinear non-convex program.");
    return q_opt.value();
  }
  logging::log()->error(
      "Draco:CalcClosestValidCompleteConf: Failed to find closest valid "
      "configuration to the Iris regions.");
  return std::nullopt;
}

std::optional<drake::trajectories::PiecewisePolynomial<double>>
DracoPlanner::MaybeFixArmsInPath(
    const drake::trajectories::PiecewisePolynomial<double>& path) const {
  // See if there are arms that move from start to the same place.
  const auto q1 = path.value(path.start_time());
  const auto q2 = path.value(path.end_time());
  std::vector<motion::ArmIndex> same_start_goal_arms;
  for (int i = 0; i < robot_model().num_arms(); ++i) {
    auto arm_index = motion::ArmIndex(i);
    const auto& arm = robot_model().GetArm(arm_index);
    auto q1_arm = arm.GetPositionFromOriginalPlant(q1);
    auto q2_arm = arm.GetPositionFromOriginalPlant(q2);
    // Check if the arm is moving to the same place
    if (q1_arm.isApprox(q2_arm, 1e-6)) {
      // If the arm is moving to the same place, add it to the list
      same_start_goal_arms.push_back(arm_index);
      logging::log()->info(
          "DracoPlanner:FindArmsMovingToSamePlace: Arm {} is moving to the "
          "same "
          "place",
          arm.name());
    }
  }
  if (same_start_goal_arms.empty()) {
    logging::log()->info(
        "DracoPlanner:MaybeFixArmsInPath: No arms are moving to the same "
        "place. "
        "No need to fix the path. Returning nullopt - use the original path.");
    return std::nullopt;
  }
  // If we found arms, we need to fix their positions in the path.
  auto fixed_path = path;
  for (int i = 0; i < robot_model().plant().num_positions(); ++i) {
    // Check index i belongs to any arm
    for (const auto& arm_index : same_start_goal_arms) {
      const auto& arm_model = robot_model().GetArm(arm_index);
      if (arm_model.IsGeneralIndexInArm(i)) {
        int original_degree =
            path.getPolynomial(0, i, 0).GetNumberOfCoefficients();
        Eigen::VectorXd coeffs = Eigen::VectorXd::Zero(original_degree);
        coeffs(0) = q1(i);  // Fix the first coefficient to q1(i)
        drake::Polynomial<double> fixed_poly =
            drake::Polynomial<double>(coeffs);
        auto poly_matrix =
            drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix(
                1, 1);
        poly_matrix << fixed_poly;
        // Set it in all segments of the path
        for (int j = 0; j < fixed_path.get_number_of_segments(); ++j) {
          fixed_path.setPolynomialMatrixBlock(poly_matrix, j, i, 0);
        }
        break;
      }
    }
  }
  // Now let's collision check the fixed path
  double step =
      0.01;  // ToDo: make this a parameter, and pass it from an options struct
  if (!robot_constraints().CheckSatisfiedTrajectory(fixed_path, step)) {
    logging::log()->info(
        "DracoPlanner:MaybeFixArmsInPath: The fixed path is not valid. "
        "Returning nullopt - use the original path.");
    return std::nullopt;
  }
  logging::log()->info(
      "DracoPlanner:MaybeFixArmsInPath: The fixed path is valid. "
      "Returning the fixed path.");
  // Return the fixed path
  return fixed_path;
}

void DracoPlanner::SetPoseInParentFrame(
    const planning_service_client::FrameRelativePose& frp) const {
  // Log the names of the frames
  logging::log()->debug(
      "MPS:SetPoseInParentFrame: Asked to set pose in parent frame for frames: "
      "{} and {} by offset: t: {}, q: {}",
      frp.frame_A(), frp.frame_B(), frp.X_AB_translation().transpose(),
      frp.X_AB_quaternion().coeffs().transpose());
  const auto& frame_A = robot_model().GetScopedFrameByName(frp.frame_A());
  const auto& frame_B = robot_model().GetScopedFrameByName(frp.frame_B());
  // Get their body frame names
  logging::log()->debug(
      "MPS:SetPoseInParentFrame: Frame A body: {}, Frame B body: {}",
      frame_A.body().name(), frame_B.body().name());
  // Check that frame B is a FixedOffsetFrame
  const auto* fixed_offset_frame =
      dynamic_cast<const drake::multibody::FixedOffsetFrame<double>*>(&frame_B);
  if (!fixed_offset_frame) {
    auto msg = fmt::format(
        "MPS:SetPoseInParentFrame: Frame {} is not a FixedOffsetFrame. "
        "Cannot set pose in parent frame.",
        frame_B.name());
    throw std::runtime_error(msg);
  }
  // Need to check that parent frame of frame B is frame A
  if (&fixed_offset_frame->parent_frame() != &frame_A) {
    auto msg = fmt::format(
        "MPS:SetPoseInParentFrame: Frame {} is not the parent frame of {}. "
        "Cannot set pose in parent frame.",
        frame_A.name(), frame_B.name());
    throw std::runtime_error(msg);
  }
  auto offset = drake::math::RigidTransformd(frp.X_AB_quaternion(),
                                             frp.X_AB_translation());
  robot_model().SetFixedOffsetFramePoseInParentFrame(frame_B, offset);
  // Do the same for all the robot constraints contexts
  for (int i = 0; i < robot_constraints_->num_threads(); ++i) {
    fixed_offset_frame->SetPoseInParentFrame(
        &(robot_constraints_->mutable_plant_context(i)), offset);
  }
  // Add to draco_visualizer if it exists
  if (has_draco_visualizer()) {
    logging::log()->info(
        "MPS:SetPoseInParentFrame: Adding frames to DracoVisualizer");
    const auto frame_scoped_name = frame_B.scoped_name().to_string();
    draco_visualizer_->AddSetFixedOffsetFramePoseInParentFrame(
        frame_scoped_name, offset);
  }
}

std::optional<Eigen::VectorXd> DracoPlanner::MaybeInvalidConf(
    const drake::trajectories::Trajectory<double>& trajectory,
    const double step,
    const std::optional<motion::CheckSatisfiedOptions>& check_satisfied_options)
    const {
  DRAKE_DEMAND(step > 0);
  // Sample up to but not including end_time, matching CheckSatisfiedTrajectory
  for (double t {0.0}; t < trajectory.end_time(); t += step) {
    const Eigen::VectorXd q = trajectory.value(t);
    // Use thread 0 for single-threaded check
    if (!robot_constraints().CheckSatisfied(q, 0, check_satisfied_options)) {
      return q;
    }
  }
  // Also check the final state at end_time
  const Eigen::VectorXd q_end = trajectory.value(trajectory.end_time());
  if (!robot_constraints().CheckSatisfied(q_end, 0, check_satisfied_options)) {
    return q_end;
  }
  return std::nullopt;
}

}  // namespace planner
}  // namespace draco
