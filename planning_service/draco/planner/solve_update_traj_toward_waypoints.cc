
#include "draco_planner.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

std::expected<drake::trajectories::PathParameterizedTrajectory<double>,
              std::string>
DracoPlanner::SolveUpdateTrajTowardWaypointsPlan(
    const planning_service_client::planner::UpdateTrajTowardWaypointsProblem&
        def,
    const std::optional<motion::ArmIndex> maybe_active_arm_index) const {
  const auto clock_now = std::chrono::steady_clock::now();
  auto sys_timed_trajectory = def.current_trajectory();
  motion::ArmIndex active_arm_index;
  if (!maybe_active_arm_index.has_value()) {
    logging::log()->warn(
        "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: "
        "No single active arm index. Using full plant");
  } else {
    active_arm_index = maybe_active_arm_index.value();
    logging::log()->info(
        "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: "
        "Active arm index: {}",
        robot_model().GetArm(active_arm_index).name());
  }
  // We need to construct the time optimal spliner if it is not already
  // constructed.
  const motion::splining::TimeOptimalSpliner* arm_tos = nullptr;
  if (robot_model().num_arms() == 1) {
    arm_tos = &time_optimal_spliner();
  } else {
    arm_tos = arms_time_optimal_spliners_.at(active_arm_index).get();
  }
  DRAKE_DEMAND(arm_tos != nullptr);
  auto ppt = conversions::ToPathParameterizedTrajectory(
      *arm_tos, sys_timed_trajectory,
      conversions::ToGeneralizedBehavior::kThrowOnMissing);
  // Get the waypoints in q form
  std::vector<Eigen::VectorXd> waypoints_eigen;
  std::optional<Eigen::VectorXd> waypoints_wiggle_room;
  DRAKE_THROW_UNLESS(def.waypoints().empty() != def.wayposes().empty());
  logging::log()->info(
      "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: "
      "Wayposes given: {}, waypoints given: {}",
      def.wayposes().size(), def.waypoints().size());
  if (!def.waypoints().empty()) {
    logging::log()->info(
        "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: "
        "Waypoints given, will solve Ik for the waypoints");
    for (const auto& waypoint : def.waypoints()) {
      waypoints_eigen.push_back(conversions::ToGeneralizedPosition(
          robot_model(), waypoint,
          conversions::ToGeneralizedBehavior::kAssumeZeroOnMissing));
    }
    logging::log()->info(
        "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: Number "
        "of waypoints: {}",
        waypoints_eigen.size());
  } else {
    // Need to solve Ik for the waypoints.
    // The natual seed is the current configuration.
    motion::system_conf_t seed_syscconf;
    for (const auto& [model_name, traj] : sys_timed_trajectory) {
      seed_syscconf[model_name] = traj.Value(def.time_now());
      logging::log()->info(
          "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: "
          "Seed sysconf for model {}: {}",
          model_name, seed_syscconf[model_name].transpose());
    }
    auto seed = robot_model().ToGeneralizedPosition(seed_syscconf);
    logging::log()->info(
        "DracoPlanner:UpdateTrajTowardWaypointsProblem: "
        "ppt start time: {}, end time: {}, current time: {}",
        ppt.start_time(), ppt.end_time(), def.time_now());
    motion::planning::IkPlannerOptions ik_options;
    ik_options.fix_idle_joints = true;
    ik_options.resolve_with_collision_avoidance = false;
    ik_options.ignore_multi_arm_collision = true;
    for (int i = 0; i < std::ssize(def.wayposes()); ++i) {
      const auto& waypose = def.wayposes().at(i);
      const auto& frame_A = drake::multibody::parsing::GetScopedFrameByName(
          robot_model().plant(), waypose.frame_A());
      const auto& frame_B = drake::multibody::parsing::GetScopedFrameByName(
          robot_model().plant(), waypose.frame_B());
      drake::math::RigidTransformd X_AB(waypose.X_AB_quaternion(),
                                        waypose.X_AB_translation());
      // Set the joint limits safety margin to 0.0 for all wayposes except the
      // last one.
      ik_options.joint_limits_safety_margin =
          (i == std::ssize(def.wayposes()) - 1)
              ? ik_options.joint_limits_safety_margin
              : 0.0;
      auto ik_result =
          ik_planner().SolveIk(frame_A, frame_B, X_AB, seed, 0, ik_options);
      if (!ik_result.is_valid()) {
        auto msg = fmt::format("Failed to solve IK for waypose: {}/{}: {}",
                               i + 1, std::ssize(def.wayposes()),
                               ik_result.failure_status_message());
        if (!ik_result.optimization_success()) {
          // This is typically a limit issue.
          motion::planning::FrameRelativePoses frps = {
              std::make_tuple(&frame_A, &frame_B, X_AB)};
          if (has_draco_visualizer()) {
            auto failed_ik_frps_index =
                std::vector<std::tuple<drake::multibody::FrameIndex,
                                       drake::multibody::FrameIndex,
                                       drake::math::RigidTransformd>>();
            auto frp = std::make_tuple(frame_A.index(), frame_B.index(), X_AB);
            failed_ik_frps_index.push_back(frp);
            const auto q_best_effort = ik_result.value();
            auto failed_ik =
                std::make_pair(q_best_effort, failed_ik_frps_index);
            AddToVisualizer(failed_ik, "Failed IK optimization");
          }
        } else if (has_draco_visualizer()) {
          // Let the visualizer show the solved configuration.
          AddToVisualizer(ik_result.value(), "Invalid IK pose");
        }
        return std::unexpected(
            "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: " + msg);
      }
      seed = ik_result.value();
      waypoints_eigen.push_back(ik_result.value());
    }
  }
  // Need to convert all waypoints_eigen to arm's position.
  if (active_arm_index.is_valid()) {
    // If arm_index is valid, we need to convert the waypoints to the arm's
    // position.
    for (auto& waypoint : waypoints_eigen) {
      waypoint = robot_model()
                     .GetArm(active_arm_index)
                     .GetPositionFromOriginalPlant(waypoint);
    }
    if (def.waypoint_wiggle_room().has_value()) {
      waypoints_wiggle_room = conversions::ToGeneralizedPosition(
          robot_model(), def.waypoint_wiggle_room().value(),
          conversions::ToGeneralizedBehavior::kAssumeZeroOnMissing);
      if (active_arm_index.is_valid()) {
        // Convert the wiggle room to the arm's position.
        waypoints_wiggle_room =
            robot_model()
                .GetArm(active_arm_index)
                .GetPositionFromOriginalPlant(waypoints_wiggle_room.value());
      }
    } else {
      logging::log()->info(
          "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: "
          "No wiggle room provided for waypoints. Not using smoothing");
    }
  } else {
    logging::log()->info(
        "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: "
        "No arm index provided. Going forward with single/original arm.");
  }
  // If arm_index is valid, we need to get the arm's discover.
  // Now let's solve the problem with 10 iterations and 1.1 time scale
  // TODO(@Sadra): Move these either to problem def or to a YAML
  auto clock_time_passed_ms =
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::steady_clock::now() - clock_now)
          .count();
  double delta_switch =
      options().planner_options.update_spline_per_conf * waypoints_eigen.size()
      + options().planner_options.update_spline_extra_time
      + clock_time_passed_ms / 1000.0;
  logging::log()->info(
      "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: Time passed for "
      "pre-processing (IK) {} waypoints: {} ms, delta_switch set to: {} s",
      waypoints_eigen.size(), clock_time_passed_ms, delta_switch);
  auto new_ppt_opt = arm_tos->CalcTrajTowardNewWaypoints(
      ppt, def.time_now(), delta_switch, waypoints_eigen,
      def.suggested_segment_durations(), def.merge_point_search_step_size(), 10,
      1.1, def.time_optimal(), waypoints_wiggle_room,
      /*constrain_end_velocity_to_zero=*/true);
  if (!new_ppt_opt.has_value()) {
    // Warning: this is an ugly hack.
    // TODO: we should expose the reason for failure in the time optimal spliner
    // and only retry if the failure is due to the end_velocity = 0 constraint.
    logging::log()->warn(
        "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: Failed to compute "
        "updated trajectory with end-velocity constrained to zero; retrying "
        "without the end-velocity = 0 constraint.");

    new_ppt_opt = arm_tos->CalcTrajTowardNewWaypoints(
        ppt, def.time_now(), delta_switch, waypoints_eigen,
        def.suggested_segment_durations(), def.merge_point_search_step_size(),
        10, 1.1, def.time_optimal(), waypoints_wiggle_room,
        /*constrain_end_velocity_to_zero=*/false);
  }
  if (!new_ppt_opt.has_value()) {
    return std::unexpected(
        "DracoPlanner:SolveUpdateTrajTowardWaypointsPlan: Failed "
        "to compute updated trajectory toward waypoints.");
  }
  return new_ppt_opt.value();
}

}  // namespace planner
}  // namespace draco
