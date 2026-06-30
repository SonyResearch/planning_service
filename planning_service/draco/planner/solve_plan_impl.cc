#include "draco_planner.h"
#include "planning_service/common/string_utils.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

namespace {
// Given a path and time parametrization that starts at time 0, slice and
// shift it to start at t_switch, with a constant segment between t_now and
// t_switch.
void PrependTrajectoryWithConstant(
    drake::trajectories::PiecewisePolynomial<double>* path,
    drake::trajectories::PiecewisePolynomial<double>* time_parametrization,
    double t_now, double delta_switch) {
  DRAKE_DEMAND(std::abs(path->start_time()) < 1e-6);
  const Eigen::MatrixXd q_start = path->value(path->start_time());
  double t_switch = t_now + delta_switch;
  logging::log()->info(
      "DracoPlanner:GeneralizedMultimodal: Shifting trajectory "
      "to start at t_switch = {}s, t_now = {}s",
      t_switch, t_now);
  // Need to make a constant trajectory between t_now and t_switch
  auto const_path_until_switch =
      drake::trajectories::PiecewisePolynomial<double>::ZeroOrderHold(
          std::vector<double>({-delta_switch, 0.0}),
          std::vector<Eigen::MatrixXd>({q_start, q_start}));
  // Do the same for time parametrization
  auto time_until_switch = motion::splining::internal::MakeUniformTimingForPath(
      const_path_until_switch, t_now);
  DRAKE_DEMAND(std::abs(time_until_switch.start_time() - t_now) < 1e-8);
  DRAKE_DEMAND(std::abs(time_until_switch.end_time() - t_switch) < 1e-8);
  // Shift the original time parametrization
  time_parametrization->shiftRight(t_switch);
  // Now concatenate
  const_path_until_switch.ConcatenateInTime(*path);
  time_until_switch.ConcatenateInTime(*time_parametrization);
  // Replace
  *path = const_path_until_switch;
  *time_parametrization = time_until_switch;
}

// Given an anchor, find all arms involved in it
std::pair<std::set<std::string>, std::set<std::string>>
get_models_and_frames_in_anchor(const psc::planner::Anchor& anchor) {
  std::set<std::string> model_names;
  std::set<std::string> frame_names;
  // Go through system_conf
  for (const auto& [robot_name, _] : anchor.system_conf()) {
    model_names.insert(robot_name);
  }
  for (const auto& frp : anchor.poses()) {
    frame_names.insert(frp.frame_A());
    frame_names.insert(frp.frame_B());
  }
  return {model_names, frame_names};
}

std::set<motion::ArmIndex> GetArmsInvolvedInProblem(
    const planning_service_client::planner::
        GeneralizedMultimodalPlanningProblem& problem,
    const motion::RobotModel& robot_model) {
  // Let's go through the start anchors and collect the arms involved
  std::set<std::string> model_names, frame_names;
  for (const auto& anchor : problem.start_anchors()) {
    const auto [models, frames] = get_models_and_frames_in_anchor(anchor);
    model_names.insert(models.begin(), models.end());
    frame_names.insert(frames.begin(), frames.end());
  }
  // Do the same for goal anchors
  for (const auto& anchor : problem.goal_anchors()) {
    const auto [models, frames] = get_models_and_frames_in_anchor(anchor);
    model_names.insert(models.begin(), models.end());
    frame_names.insert(frames.begin(), frames.end());
  }
  // Now, find arms
  std::set<motion::ArmIndex> arms;
  for (const auto& model_name : model_names) {
    auto model_idx = robot_model.plant().GetModelInstanceByName(model_name);
    auto arm_index = robot_model.get_arm_index(model_idx);
    DRAKE_DEMAND(arm_index.is_valid());
    arms.insert(arm_index);
  }
  for (const auto& frame_name : frame_names) {
    const auto& frame = robot_model.GetScopedFrameByName(frame_name);
    auto model_idx = frame.model_instance();
    auto arm_index = robot_model.get_arm_index(model_idx);
    if (arm_index.is_valid()) {
      arms.insert(arm_index);
    }
  }
  {
    // Temporary logging block
    std::string active_arm_names = "";
    for (const auto& arm_index : arms) {
      active_arm_names += robot_model.GetArm(arm_index).name() + " ";
    }
    logging::log()->info(
        "DracoPlanner:GetArmsInvolvedInProblem: Active arms involved in the "
        "problem: {}",
        active_arm_names);
  }
  return arms;
}

std::set<motion::ArmIndex> GetArmsInvolvedInProblem(
    const planning_service_client::planner::MaxCartesianAcceleration& problem,
    const motion::RobotModel& robot_model) {
  // Look at frame_A and frame_B.
  std::set<motion::ArmIndex> arms;
  const auto& frame_A =
      robot_model.GetScopedFrameByName(problem.twist().frame_A());
  const auto& frame_B =
      robot_model.GetScopedFrameByName(problem.twist().frame_B());
  auto arm_A = robot_model.get_arm_index(frame_A.model_instance());
  auto arm_B = robot_model.get_arm_index(frame_B.model_instance());
  if (arm_A.is_valid()) {
    arms.insert(arm_A);
  }
  if (arm_B.is_valid()) {
    arms.insert(arm_B);
  }
  {
    // Temporary logging block
    std::string active_arm_names = "";
    for (const auto& arm_index : arms) {
      active_arm_names += robot_model.GetArm(arm_index).name() + " ";
    }
    logging::log()->info(
        "DracoPlanner:GetArmsInvolvedInProblem: Active arms involved in the "
        "MaxCartesianAcceleration problem: {}",
        active_arm_names);
  }
  return arms;
}

// UpdateTrajTowardWaypointsProblem is currently supported only for
// single-arm robots. It's only because of performance reasons.
// Otherwise, it is totally possible to support multi-arm robots.
std::optional<motion::ArmIndex> FindActiveArmIndex(
    const motion::RobotModel& robot_model,
    const planning_service_client::planner::UpdateTrajTowardWaypointsProblem&
        def) {
  const auto& full_plant = robot_model.plant();
  // We can look at def to see what the active arm is.
  if (robot_model.num_arms() == 1) {
    return motion::ArmIndex(0);
  }
  // Let's look at the wayposes and see which arm is active.
  motion::ArmIndex active_arm;
  logging::log()->info(
      "DracoPlanner:FindActiveArmIndex: # wayposes: {}; # waypoints: {}",
      def.wayposes().size(), def.waypoints().size());
  for (const auto& waypose : def.wayposes()) {
    const auto& frame_A = drake::multibody::parsing::GetScopedFrameByName(
        full_plant, waypose.frame_A());
    const auto& frame_B = drake::multibody::parsing::GetScopedFrameByName(
        full_plant, waypose.frame_B());
    auto arm_A = robot_model.get_arm_index(frame_A.model_instance());
    auto arm_B = robot_model.get_arm_index(frame_B.model_instance());
    if (arm_A.is_valid()) {
      if (active_arm.is_valid() && active_arm != arm_A) {
        logging::log()->info(
            "DracoPlanner:FindActiveArmIndex: More than one active arm found. "
            "Returning nullopt.");
        return std::nullopt;  // More than one active arm found.
      } else {
        active_arm = arm_A;
      }
    }
    // Do the same for frame_B.
    if (arm_B.is_valid()) {
      if (active_arm.is_valid() && active_arm != arm_B) {
        logging::log()->info(
            "DracoPlanner:FindActiveArmIndex: More than one active arm found. "
            "Returning nullopt.");
        return std::nullopt;  // More than one active arm found.
      } else {
        active_arm = arm_B;
      }
    }
  }
  // Do the same for waypoints if they are provided.
  for (const auto& waypoint : def.waypoints()) {
    for (const auto& [model_name, conf] : waypoint) {
      DRAKE_THROW_UNLESS(full_plant.HasModelInstanceNamed(model_name));
      const auto model_index = full_plant.GetModelInstanceByName(model_name);
      auto arm_index = robot_model.get_arm_index(model_index);
      DRAKE_DEMAND(arm_index.is_valid());  // Does not make sense for sysconf to
      // give conf for a model that is not part of any arm.
      if (arm_index.is_valid()) {
        if (active_arm.is_valid() && active_arm != arm_index) {
          logging::log()->info(
              "DracoPlanner:FindActiveArmIndex: More than one active arm "
              "found. Returning nullopt.");
          return std::nullopt;  // More than one active arm found.
        } else {
          active_arm = arm_index;
        }
      }
    }
  }
  logging::log()->info("DracoPlanner:FindActiveArmIndex: Active arm found: {}",
                       active_arm.is_valid() ? active_arm : -1);
  return active_arm;
}

bool IsConstant(const planning_service_client::PiecewisePolynomial& pp,
                double tolerance = 1e-5) {
  // Get the value at the start
  const Eigen::VectorXd start_value = pp.Value(pp.start_time());
  // Get the values at the breaks after the start, and compare
  for (int i = 0; i < std::ssize(pp.breaks()) - 1; ++i) {
    double t_break = pp.breaks()[i + 1];
    Eigen::VectorXd value_at_break = pp.Value(t_break);
    if (!value_at_break.isApprox(start_value, tolerance)) {
      return false;
    }
  }
  return true;
}

// Sets the resource result types for a new planning result. Depending on
// whether the arm was active in the problem and whether its trajectory is
// constant, the resource result type is set accordingly.
void SetResourcesFromNewResult(
    planning_service_client::planner::MotionPlanResult* result,
    const std::set<motion::ArmIndex>& active_arm_indices,
    const motion::RobotModel& robot_model) {
  // Find the arms that were not involved in the problem AND are constant and
  // label their result type as kEmpty
  const auto& sys_timed_trajectory = result->system_timed_trajectory();
  for (const auto& [resource, traj] : sys_timed_trajectory) {
    auto model_idx = robot_model.plant().GetModelInstanceByName(resource);
    auto arm_index = robot_model.get_arm_index(model_idx);
    bool does_plan_target_resource_arm = active_arm_indices.contains(arm_index);
    DRAKE_DEMAND(arm_index.is_valid());
    // If the arm is not involved and its trajectory is constant, label it
    const auto is_traj_constant = IsConstant(traj.path());
    // Temporary log
    logging::log()->info(
        "DracoPlanner: Trajectory "
        "for resource {} is constant: {}",
        resource, is_traj_constant ? "Yes" : "No");
    if (does_plan_target_resource_arm) {
      logging::log()->info(
          "DracoPlanner:GeneralizedMultimodal: Resource {} was "
          "targeted",
          resource);
      result->SetResourceResultType(
          resource, planning_service_client::planner::MotionPlanResult::
                        MotionResultType::kNewAndTargeted);
    } else if (!is_traj_constant) {
      logging::log()->info(
          "DracoPlanner: Resource "
          "{} was not targeted and has non-constant trajectory",
          resource);
      result->SetResourceResultType(
          resource, planning_service_client::planner::MotionPlanResult::
                        MotionResultType::kNewNotTargeted);
    } else {
      logging::log()->info(
          "DracoPlanner: Resource "
          "{} was not targeted and has constant trajectory.",
          resource);
      result->SetResourceResultType(
          resource, planning_service_client::planner::MotionPlanResult::
                        MotionResultType::kEmpty);
    }
    DRAKE_DEMAND(result->GetResourceResultType(resource)
                 != planning_service_client::planner::MotionPlanResult::
                     MotionResultType::kUndefined);
  }
}

// Sets the resource result types for an updated planning result. This
// is much simpler than for new.
void SetResourcesFromUpdatedResult(
    planning_service_client::planner::MotionPlanResult* result,
    const std::set<motion::ArmIndex>& active_arm_indices,
    const motion::RobotModel& robot_model) {
  (void)active_arm_indices;  // Unused currently
  (void)robot_model;         // Unused currently
  // For updated results, all active arms are labeled as kUpdate. Others are
  // not changed because update traj works on one arm at a time.
  const auto& sys_timed_trajectory = result->system_timed_trajectory();
  for (const auto& [resource, traj] : sys_timed_trajectory) {
    logging::log()->info(
        "DracoPlanner: Updated result: Labeling resource "
        "{} as kUpdate",
        resource);
    result->SetResourceResultType(
        resource, planning_service_client::planner::MotionPlanResult::
                      MotionResultType::kUpdate);
  }
}

double CalcLatencyTime(
    double new_traj_duration,
    const planning_service_client::SystemTimedTrajectory& active_traj,
    double global_time_now, double collision_checking_time_step,
    double collision_checking_latency_per_call, double communication_latency) {
  double max_end_time = new_traj_duration;
  // Now check the time from now until the global time of active trajectories
  for (const auto& [_, traj] : active_traj) {
    double remaining_time =
        traj.end_time() + traj.global_time_offset() - global_time_now;
    max_end_time = std::max(max_end_time, remaining_time);
  }
  // Now calculate how much latency time is needed
  int num_collision_checks =
      static_cast<int>(std::ceil(max_end_time / collision_checking_time_step));
  double total_latency_time =
      num_collision_checks * collision_checking_latency_per_call
      + communication_latency;
  // Keep this logging for debugging
  logging::log()->info(
      "DracoPlanner: CalcLatencyTime: max_end_time = {}s, num_collision_checks "
      "= {}, total_latency_time = {}s",
      max_end_time, num_collision_checks, total_latency_time);
  return total_latency_time;
}
}  // namespace

planning_service_client::planner::MotionPlanResult DracoPlanner::SolvePlanImpl(
    const planning_service_client::planner::PlanningProblemBase& def,
    const std::optional<planning_service_client::SystemConf>& start_sysconf,
    const std::optional<planning_service_client::SystemTimedTrajectory>&
        maybe_active_trajectory,
    const std::optional<
        planning_service_client::planner::TrajectoryUpdateOptions>&
        maybe_traj_update_options) const {
  (void)maybe_traj_update_options;  // Unused currently. May be used later.
  std::set<motion::ArmIndex> active_arm_indices = {};
  SolvePlanResult path_and_timimg_opt =
      std::unexpected("DracoPlanner: Not solved yet.");
  const motion::splining::TimeOptimalSpliner* tos = &time_optimal_spliner();
  bool updated_trajectory = false;
  planning_service_client::planner::MotionPlanResult result;
  planning_service_client::SystemTimedTrajectory sys_timed_trajectory;
  std::optional<motion::ArmIndex> maybe_update_active_arm;
  std::optional<double> time_now = std::nullopt;
  double calculated_plan_duration = 0.0;
  std::optional<double> min_partial_solution_time = std::nullopt;
  std::optional<double> max_partial_solution_time = std::nullopt;
  if (const auto* def_gen_mmodal =
          dynamic_cast<const planning_service_client::planner::
                           GeneralizedMultimodalPlanningProblem*>(&def)) {
    DRAKE_THROW_UNLESS(start_sysconf.has_value());
    active_arm_indices =
        GetArmsInvolvedInProblem(*def_gen_mmodal, robot_model());
    const auto gen_mmodal_result_opt =
        SolveGeneralizedMultiModalPlan(*def_gen_mmodal, start_sysconf.value());
    // if the result has value, extract the path and time parametrization and
    // use to populate the path_and_timimg_opt
    if (gen_mmodal_result_opt.has_value()) {
      const auto& gen_mmodal_result = gen_mmodal_result_opt.value();
      path_and_timimg_opt = gen_mmodal_result.trajectory;
      min_partial_solution_time = gen_mmodal_result.transit_start_time;
      max_partial_solution_time = gen_mmodal_result.transit_end_time;
    } else {
      path_and_timimg_opt = std::unexpected(gen_mmodal_result_opt.error());
    }
  } else if (const auto* def_out_of_violation = dynamic_cast<
                 const planning_service_client::planner::OutOfViolation*>(
                 &def)) {
    DRAKE_THROW_UNLESS(start_sysconf.has_value());
    path_and_timimg_opt =
        SolveOutOfViolationPlan(*def_out_of_violation, start_sysconf.value());
  } else if (const auto* def_fixed_frames_motion = dynamic_cast<
                 const planning_service_client::planner::FixedFramesMotion*>(
                 &def)) {
    DRAKE_THROW_UNLESS(start_sysconf.has_value());
    path_and_timimg_opt = SolveFixedFramesMotionPlan(*def_fixed_frames_motion,
                                                     start_sysconf.value());
  } else if (const auto* update_traj_problem =
                 dynamic_cast<const planning_service_client::planner::
                                  UpdateTrajTowardWaypointsProblem*>(&def)) {
    // Find active arm index
    updated_trajectory = true;
    maybe_update_active_arm =
        FindActiveArmIndex(robot_model(), *update_traj_problem);
    const auto& current_sys_timed_trajectory =
        update_traj_problem->current_trajectory();
    // Get the time_global_traj from the current trajectory. All of them should
    // be the same.
    std::optional<double> time_global_traj_opt;
    for (const auto& [_, traj] : current_sys_timed_trajectory) {
      if (!time_global_traj_opt.has_value()) {
        time_global_traj_opt = traj.global_time_offset();
      } else {
        DRAKE_THROW_UNLESS(
            std::abs(time_global_traj_opt.value() - traj.global_time_offset())
            < 1e-6);
      }
    }
    DRAKE_THROW_UNLESS(time_global_traj_opt.has_value());
    const double time_global_traj = time_global_traj_opt.value();
    auto timed_path_opt = SolveUpdateTrajTowardWaypointsPlan(
        *update_traj_problem, maybe_update_active_arm);
    if (!timed_path_opt.has_value()) {
      result.SetFailureStatus();
      result.SetMessage(timed_path_opt.error());
      return result;
    }
    calculated_plan_duration = timed_path_opt.value().end_time();
    if (maybe_update_active_arm.has_value() && robot_model().num_arms() > 1) {
      active_arm_indices.insert(maybe_update_active_arm.value());
      // Also set the time optimal spliner pointer
      tos =
          arms_time_optimal_spliners_.at(maybe_update_active_arm.value()).get();
    } else {
      logging::log()->info(
          "DracoPlanner:UpdateTrajTowardWaypointsProblem "
          "with no single active arm index. Treating as full-plant problem.");
      tos = &time_optimal_spliner();
    }
    sys_timed_trajectory =
        conversions::ToSystemTimedTrajectory(*tos, timed_path_opt.value());
    for (auto& [_, traj] : sys_timed_trajectory) {
      // We don't change the global time offset here because we are updating an
      // existing trajectory.
      traj.SetGlobalTimeOffset(time_global_traj);
    }
    result.SetSystemTimedTrajectory(sys_timed_trajectory);
    SetResourcesFromUpdatedResult(&result, active_arm_indices, robot_model());
    time_now = update_traj_problem->time_now();
  } else if (const auto* global_ik_problem = dynamic_cast<
                 const planning_service_client::planner::GlobalIKProblem*>(
                 &def)) {
    path_and_timimg_opt = SolveGlobalIKPlan(*global_ik_problem);
    // For this problem, let all arms be active.
    // ToDo(@anyone) Make global IK its own problem type that returns a sysconf
    // instead of trajectory.
    for (motion::ArmIndex arm_index(0); arm_index < robot_model().num_arms();
         ++arm_index) {
      active_arm_indices.insert(arm_index);
    }
  } else if (const auto* max_cartesian_acceleration_problem =
                 dynamic_cast<const planning_service_client::planner::
                                  MaxCartesianAcceleration*>(&def)) {
    DRAKE_THROW_UNLESS(start_sysconf.has_value());
    path_and_timimg_opt = SolveMaxCartesianAccelerationProblem(
        *max_cartesian_acceleration_problem, start_sysconf.value());
    active_arm_indices = GetArmsInvolvedInProblem(
        *max_cartesian_acceleration_problem, robot_model());
  } else {
    logging::log()->error(
        "DracoPlanner: The problem type {} is not "
        "supported in this method.",
        typeid(def).name());
    // path_and_timimg_opt is already an unexpected
  }
  if (!path_and_timimg_opt.has_value() && !updated_trajectory) {
    result.SetFailureStatus();
    result.SetMessage(path_and_timimg_opt.error());
    logging::log()->error("DracoPlanner: Failed to solve the problem: {}",
                          path_and_timimg_opt.error());
    return result;
  }
  // Log if we have active trajectory and trajectory update options
  bool involves_async = maybe_active_trajectory.has_value()
                        && maybe_active_trajectory->size() > 0;
  logging::log()->info(
      "DracoPlanner:SolvePlanImpl: Involves async "
      "trajectory: {}.",
      involves_async ? "Yes" : "No");
  auto computations_start_time = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> epoch_duration_now =
      std::chrono::system_clock::now().time_since_epoch();
  const double time_global_now = epoch_duration_now.count();
  DRAKE_DEMAND(path_and_timimg_opt.has_value()
               || updated_trajectory);  // Should have been caught earlier.
  if (path_and_timimg_opt.has_value()) {
    calculated_plan_duration = path_and_timimg_opt->second.end_time();
  }
  const double latency = CalcLatencyTime(
      calculated_plan_duration,
      involves_async ? maybe_active_trajectory.value()
                     : planning_service_client::SystemTimedTrajectory {},
      time_global_now,
      options().planner_options.async_collison_checking_time_step,
      options().planner_options.async_latency_check_per_call,
      options().planner_options.latency_comms);
  // If New trajectory is new (not update)
  if (!updated_trajectory) {
    auto& [path, time_parametrization] = path_and_timimg_opt.value();
    // Get time since epoch
    if (involves_async && !updated_trajectory) {
      PrependTrajectoryWithConstant(&path, &time_parametrization, 0.0, latency);
      min_partial_solution_time =
          min_partial_solution_time.value_or(0.0) + latency;
      max_partial_solution_time = max_partial_solution_time.value_or(
                                      std::numeric_limits<double>::infinity())
                                  + latency;
      logging::log()->info(
          "DracoPlanner:BestApproachTrajectory: Prepending trajectory with "
          "constant segment of duration {}s to account for latency. Updated "
          "min_partial_solution_time = {}s, max_partial_solution_time = {}s",
          latency, min_partial_solution_time.value(),
          max_partial_solution_time.value());
    }
    auto timed_path = drake::trajectories::PathParameterizedTrajectory<double>(
        path, time_parametrization);
    sys_timed_trajectory =
        conversions::ToSystemTimedTrajectory(*tos, timed_path);
    for (auto& [resource, traj] : sys_timed_trajectory) {
      logging::log()->info(
          "DracoPlanner: Setting global time offset for resource {} to "
          "{}s (latency {}s)",
          resource, time_global_now + latency, latency);
      traj.SetGlobalTimeOffset(time_global_now + latency);
    }
    result.SetSystemTimedTrajectory(sys_timed_trajectory);
    SetResourcesFromNewResult(&result, active_arm_indices, robot_model());
  }
  if (involves_async) {
    logging::log()->info(
        "DracoPlanner: active trajectories involved. Replacing ...");
    std::set<motion::ArmIndex> movable_arm_indices;
    std::set<motion::ArmIndex> arm_indices_in_active_traj;
    const auto& active_traj = maybe_active_trajectory.value();
    // Replace the trajectories for the active arms
    for (const auto& [resource, traj] : active_traj) {
      if (robot_model().plant().HasModelInstanceNamed(resource) == false) {
        // Not part of the robot model. Skip.
        logging::log()->warn(
            "DracoPlanner: Active trajectory resource {} is not part of the "
            "robot model. This warning will change to throw in future "
            "versions. Make sure the active traj is compatible with the robot "
            "model.",
            resource);
        continue;
      }
      // Find the arm index and add it to the set of arms in active traj
      const auto model_idx =
          robot_model().plant().GetModelInstanceByName(resource);
      const auto arm_index = robot_model().get_arm_index(model_idx);
      DRAKE_DEMAND(arm_index.is_valid());
      arm_indices_in_active_traj.insert(arm_index);
      // Check if we don't have a new trajectory for this resource. Because then
      // this is a bug!
      if (!sys_timed_trajectory.contains(resource)) {
        // Just add it for collision checking purposes
        if (robot_model().plant().HasModelInstanceNamed(resource)) {
          logging::log()->info(
              "DracoPlanner: Adding active trajectory for resource {} for "
              "collision checking.",
              resource);
        } else {
          logging::log()->warn(
              "DracoPlanner: Active trajectory resource {} is not part of the "
              "robot model. This warning will change to throw in future "
              "versions. Make sure the active traj is compatible with the "
              "robot model.",
              resource);
          continue;
        }
        sys_timed_trajectory[resource] = traj;
        continue;
      }
      switch (result.GetResourceResultType(
          resource)) {  // Does this need to be a switch case?
        case planning_service_client::planner::MotionPlanResult::
            MotionResultType::kNewAndTargeted: {
          auto msg = fmt::format(
              "Active trajectory resource {} is running but asked a different "
              "plan for it. Should have been an update plan instead.",
              resource);
          logging::log()->error("DracoPlanner: {}", msg);
          result.SetMessage(msg);
          result.SetFailureStatus(
              planning_service_client::planner::MotionPlanResult::
                  FailureStatus::kAsyncActiveResource);
          result.SetSystemTimedTrajectory(
              planning_service_client::SystemTimedTrajectory());
          return result;
        }
        case planning_service_client::planner::MotionPlanResult::
            MotionResultType::kUpdate: {
          // Let it go as update
          continue;
        }
        case planning_service_client::planner::MotionPlanResult::
            MotionResultType::kNewNotTargeted:
        case planning_service_client::planner::MotionPlanResult::
            MotionResultType::kRunningNoUpdate:
        case planning_service_client::planner::MotionPlanResult::
            MotionResultType::kEmpty:
        default: {
          result.SetResourceResultType(
              resource, planning_service_client::planner::MotionPlanResult::
                            MotionResultType::kRunningNoUpdate);
          // Slice the active trajectory from time now
          double traj_time_now = time_global_now - traj.global_time_offset();
          // ToDO(@Sadra) Remove the following log after debugging
          logging::log()->info(
              "DracoPlanner: Slicing active trajectory for resource {} from "
              "time {}s (global time now {}s, traj offset {}s)",
              resource, traj_time_now, time_global_now,
              traj.global_time_offset());
          if (traj_time_now > traj.end_time()) {
            logging::log()->info(
                "DracoPlanner: Active trajectory for resource {} has already "
                "ended at time {}>{}s. Skipping.",
                resource, traj_time_now, traj.end_time());
            continue;
          }
          auto sliced_traj =
              traj.SliceFromTime(traj_time_now, true /* allow clamping end */);
          sliced_traj.ShiftTime(-traj_time_now);  // Bring time to zero
          logging::log()->info(
              "DracoPlanner: Sliced active trajectory for resource {} has "
              "start time {}s, end time {}s",
              resource, sliced_traj.start_time(), sliced_traj.end_time());
          DRAKE_DEMAND(std::abs(sliced_traj.start_time()) <= 1e-6);
          sliced_traj.SetGlobalTimeOffset(time_global_now);
          // Add them to the system trajectory
          sys_timed_trajectory[resource] = sliced_traj;
          break;
        }
      }
    }
    // Add all arms NOT in the active traj to movable arms
    for (motion::ArmIndex arm_index(0); arm_index < robot_model().num_arms();
         ++arm_index) {
      if (!arm_indices_in_active_traj.contains(arm_index)) {
        movable_arm_indices.insert(arm_index);
      }
    }
    // Let's check if the update trajectory needs to be collision checked. If
    // The active trajectory is constant, no need to check.
    bool is_updated_and_active_traj_constant = updated_trajectory;
    for (const auto& [resource, traj] : active_traj) {
      if (!is_updated_and_active_traj_constant) {
        // Exit before checking further
        break;
      }
      if (robot_model().plant().HasModelInstanceNamed(resource) == false) {
        // Not part of the robot model. Skip.
        logging::log()->warn(
            "DracoPlanner: Active trajectory resource {} is not part of the "
            "robot model. This warning will change to throw in future "
            "versions. Make sure the active traj is compatible with the robot "
            "model.",
            resource);
        continue;
      }
      // Update means maybe_active_arm_index is set
      DRAKE_DEMAND(maybe_update_active_arm.has_value());
      const auto& targeted_resources =
          robot_model()
              .GetArm(maybe_update_active_arm.value())
              .model_instances();
      auto resource_idx =
          robot_model().plant().GetModelInstanceByName(resource);
      if (std::find(targeted_resources.begin(), targeted_resources.end(),
                    resource_idx)
          != targeted_resources.end()) {
        logging::log()->debug(
            "DracoPlanner: Resource {} is targeted arm. Skipping constant "
            "check.",
            resource);
        // This is the targeted arm. Skip.
        continue;
      }
      is_updated_and_active_traj_constant =
          is_updated_and_active_traj_constant && IsConstant(traj.path());
    }
    if (is_updated_and_active_traj_constant) {
      // No need to collision check!
      logging::log()->info(
          "DracoPlanner: Updated trajectory. Skipping collision checking "
          "because the active trajectory for not-targeted arms is constant.");
      result.SetSuccessStatus();
      return result;
    }
    const auto& collision_time_opt = TimeOfArmsCollision(
        sys_timed_trajectory,
        options().planner_options.async_collison_checking_time_step);
    if (collision_time_opt.has_value()) {
      // Try partial async solution if pdef is generalized multimodal and the
      // flag allow_async_partial_solutions is true.
      bool try_partial_async_solution = false;
      if (const auto* def_gen_mmodal =
              dynamic_cast<const planning_service_client::planner::
                               GeneralizedMultimodalPlanningProblem*>(&def)) {
        try_partial_async_solution =
            def_gen_mmodal->allow_async_partial_solutions();
      }
      if (try_partial_async_solution) {
        // Collision detected, try to find best approach trajectory
        logging::log()->info(
            "DracoPlanner: Collision detected in the calculated trajectory at "
            "{}s. "
            "Trying to find best approach trajectory.",
            collision_time_opt.value());
        auto best_approach_trajectory_opt = BestApproachTrajectory(
            sys_timed_trajectory, movable_arm_indices, 10 /* num_slices */,
            options().planner_options.async_collison_checking_time_step,
            min_partial_solution_time, max_partial_solution_time,
            options().planner_options.partial_solution_time_buffer);
        if (best_approach_trajectory_opt.has_value()) {
          logging::log()->info(
              "DracoPlanner: Found best approach trajectory to avoid "
              "collision.");
          sys_timed_trajectory = best_approach_trajectory_opt.value();
          result.SetSystemTimedTrajectory(sys_timed_trajectory);
          result.SetSuccessStatus(
              planning_service_client::planner::MotionPlanResult::
                  SuccessStatus::kStoppedShort);
          return result;
        }
      }
      auto msg =
          "The calculated trajectory collides with active trajectories of "
          "other arms.";
      logging::log()->error("DracoPlanner: {}", msg);
      result.SetFailureStatus(
          planning_service_client::planner::MotionPlanResult::FailureStatus::
              kAsyncCollision);
      result.SetMessage(msg);
      result.SetSystemTimedTrajectory(
          planning_service_client::SystemTimedTrajectory());
      return result;
    }
    // Check how much time has passed since we started processing. If longer
    // than total_latency_sec, we have a problem.
    auto computation_time =
        std::chrono::duration<double>(std::chrono::high_resolution_clock::now()
                                      - computations_start_time)
            .count();
    if (computation_time > latency) {
      auto msg = fmt::format(
          "Processing time {}s>latency {}s. The planner is too slow for "
          "the given latency requirements.",
          computation_time, latency);
      logging::log()->error("DracoPlanner: {}", msg);
      result.SetFailureStatus();
      result.SetMessage(msg);
      result.SetSystemTimedTrajectory(
          planning_service_client::SystemTimedTrajectory());
      return result;
    }
  }
  result.SetSystemTimedTrajectory(sys_timed_trajectory);
  result.SetSuccessStatus();
  return result;
}

}  // namespace planner
}  // namespace draco
