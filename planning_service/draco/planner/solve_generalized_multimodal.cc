#include "draco_planner.h"
#include "planning_service/common/string_utils.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

// Check if the trajectory has a collision between arms, and return the time of
// the first collision if there is one.
std::optional<double> DracoPlanner::TimeOfArmsCollision(
    const planning_service_client::SystemTimedTrajectory& sys_timed_trajectory,
    double step) const {
  DRAKE_DEMAND(step > 0);
  // Get the values at time now - all models should be there.
  planning_service_client::SystemConf sys_conf_now;
  for (const auto& [robot_name, traj] : sys_timed_trajectory) {
    double global_time = traj.global_time_offset();
    sys_conf_now[robot_name] = traj.GlobalValue(global_time);
  }
  double t = 0;
  double final_time = 0;
  for (const auto& [robot_name, traj] : sys_timed_trajectory) {
    final_time = std::max(final_time, traj.end_time());
  }
  while (t <= final_time) {
    planning_service_client::SystemConf sysconf;
    for (const auto& [robot_name, traj] : sys_timed_trajectory) {
      const double global_time = t + traj.global_time_offset();
      sysconf[robot_name] = traj.GlobalValue(global_time);
    }
    const auto q = conversions::ToGeneralizedPosition(
        robot_constraints().robot_model(), sysconf,
        conversions::ToGeneralizedBehavior::kCompleteFromReferenceOnMissing,
        sys_conf_now);
    if (robot_constraints().DoArmsCollide(q, 0)) {
      logging::log()->info(
          "DracoPlanner:TimeOfArmsCollision: Trajectory becomes invalid at "
          "t = {}s",
          t);
      return t;
    }
    t += step;
  }
  return std::nullopt;
}

/*
@brief Get a best approach trajectory that is the same as the input trajectory
until a certain time, and then becomes constant. The time is chosen to be the
time when the arms start to collide, with some buffer. If there is no collision,
then the original trajectory is returned. If the best approach time is less than
min_collision_time, then nullopt is returned. If the best approach time is
greater than max_collision_time, then max_collision_time is used as the best
approach time. The trajectory after the best approach time is replaced with a
constant trajectory, and then retimed using Toppra to get a valid trajectory.
* @param sys_timed_trajectory The input trajectory to check for collision and
modify.
* @param movable_arms_indices The set of indices of the movable arms. The
trajectory for these arms will be modified after the best approach time. The
trajectory for the non-movable arms will be kept the same as the input
trajectory.
* @param num_slices The number of candidate best approach times to try between
min_collision_time and the time of collision. The candidates will be uniformly
sampled between min_collision_time and the time of collision.
* @param step The time step to use when checking for collision along the
trajectory.
* @param min_collision_time The minimum time to consider for the best approach
time. If the time of collision is less than this value, then nullopt is
returned.
* @param max_collision_time The maximum time to consider for the best approach
time. If the time of collision is greater than this value, then
max_collision_time is used as the best approach time.
* @return The best approach trajectory that is the same as the input trajectory
until the best approach time, and then becomes constant for the movable arms. If
there is no collision, then the original trajectory is returned. If the best
approach time is less than min_collision_time, then nullopt is returned.
*/
std::optional<planning_service_client::SystemTimedTrajectory>
DracoPlanner::BestApproachTrajectory(
    const planning_service_client::SystemTimedTrajectory& sys_timed_trajectory,
    const std::set<motion::ArmIndex>& movable_arms_indices, int num_slices,
    double step, std::optional<double> min_collision_time,
    std::optional<double> max_collision_time,
    double partial_solution_time_buffer) const {
  DRAKE_DEMAND(step > 0);
  logging::log()->debug(
      "DracoPlanner:BestApproachTrajectory: Checking trajectory for arms "
      "collision "
      "with step {}s, num_slices {}, min_collision_time {}s, "
      "max_collision_time {}s",
      step, num_slices,
      min_collision_time.has_value()
          ? std::to_string(min_collision_time.value())
          : "nullopt",
      max_collision_time.has_value()
          ? std::to_string(max_collision_time.value())
          : "nullopt");
  // Get the values at time now - all models should be there.
  planning_service_client::SystemConf sys_conf_now;
  for (const auto& [robot_name, traj] : sys_timed_trajectory) {
    double global_time = traj.global_time_offset();
    sys_conf_now[robot_name] = traj.GlobalValue(global_time);
    // log the number of breaks and time scaling breaks for debugging
    logging::log()->debug(
        "DracoPlanner:BestApproachTrajectory: Robot {} trajectory breaks "
        "size: {}",
        robot_name, traj.path().breaks().size());
    logging::log()->debug(
        "DracoPlanner:BestApproachTrajectory: Robot {} time scaling breaks "
        "size: {}",
        robot_name, traj.time_scaling().breaks().size());
  }
  const auto q_now = conversions::ToGeneralizedPosition(
      robot_constraints().robot_model(), sys_conf_now);
  double t = 0;
  double final_time = 0;
  for (const auto& [robot_name, traj] : sys_timed_trajectory) {
    final_time = std::max(final_time, traj.end_time());
  }
  double best_approach_time = final_time;
  bool collion_found = false;
  while (t <= final_time) {
    planning_service_client::SystemConf sysconf;
    for (const auto& [robot_name, traj] : sys_timed_trajectory) {
      double global_time = t + traj.global_time_offset();
      DRAKE_THROW_UNLESS(traj.global_start_time() <= global_time + 1e-6);
      sysconf[robot_name] = traj.GlobalValue(global_time);
    }
    const auto q = conversions::ToGeneralizedPosition(
        robot_constraints().robot_model(), sysconf,
        conversions::ToGeneralizedBehavior::kCompleteFromReferenceOnMissing,
        sys_conf_now);
    if (robot_constraints().DoArmsCollide(q, 0)) {
      logging::log()->debug(
          "DracoPlanner:TimeOfArmsCollision: Trajectory becomes invalid at "
          "t = {}s",
          t);
      best_approach_time =
          t - partial_solution_time_buffer;  // Give some buffer
      collion_found = true;
      break;
    }
    t += step;
  }
  if (!collion_found) {
    logging::log()->info(
        "DracoPlanner:BestApproachTrajectory: No collision found in the "
        "trajectory. Returning original trajectory.");
    return sys_timed_trajectory;
  }
  if (min_collision_time.has_value()
      && best_approach_time < min_collision_time.value()) {
    // don't want to partially execute less than min_collision_time
    logging::log()->info(
        "DracoPlanner:BestApproachTrajectory: Best approach time candidate "
        "found at t = {:.2f}s, which is less than min_collision_time = "
        "{:.2f}s. Returning nullopt.",
        best_approach_time, min_collision_time.value());
    return std::nullopt;
  }
  if (max_collision_time.has_value()
      && best_approach_time > max_collision_time.value()) {
    // don't want to partially execute more than max_collision_time
    logging::log()->info(
        "DracoPlanner:BestApproachTrajectory: Best approach time candidate "
        "found at t = {:.2f}s, which is greater than max_collision_time = "
        "{:.2f}s. Setting best_approach_time to max_collision_time.",
        best_approach_time, max_collision_time.value());
    best_approach_time = max_collision_time.value();
  }
  logging::log()->info(
      "DracoPlanner:BestApproachTrajectory: Best approach time candidate found "
      "at t = {:.2f}s. Trying to find a valid trajectory by fixing active arms "
      "from t = {:.2f}s to t = {:.2f}s.",
      best_approach_time, best_approach_time, final_time);
  // Prepare a set of candidate t_constant values uniformly sampled between
  // min_collision_time and best_approach_time (inclusive) using num_slices.
  double t_min = min_collision_time.value_or(0);
  std::vector<double> t_candidates;
  if (num_slices <= 1) {
    t_candidates.push_back(best_approach_time);
  } else {
    for (int i = 0; i < num_slices; ++i) {
      double alpha = static_cast<double>(i) / (num_slices - 1);
      double t_candidate = t_min * alpha + best_approach_time * (1 - alpha);
      t_candidates.push_back(t_candidate);
    }
  }

  for (unsigned int i = 0; i < t_candidates.size(); ++i) {
    double t_constant = t_candidates[i];
    logging::log()->info(
        "DracoPlanner:BestApproachTrajectory: Trying candidate best approach "
        "time t_constant = {}s",
        t_constant);
    // Check if we can get a valid trajectory by fixing the movable resources
    // from t_constant to final_time
    auto sliced_sys_timed_trajectory = sys_timed_trajectory;

    for (const auto& movable_arm_idx : movable_arms_indices) {
      const auto& movable_arm = robot_model().GetArm(movable_arm_idx);
      const auto& movable_model_instances = movable_arm.model_instances();
      logging::log()->debug(
          "DracoPlanner:BestApproachTrajectory: Processing movable arm {} with "
          "{} model instances",
          movable_arm.name(), movable_model_instances.size());
      planning_service_client::SystemTimedTrajectory
          arm_sliced_sys_timed_trajectory;
      for (const auto& movable_model_instance : movable_model_instances) {
        logging::log()->debug(
            "DracoPlanner:BestApproachTrajectory: Processing movable model "
            "instance {} of arm {}",
            robot_model().plant().GetModelInstanceName(movable_model_instance),
            movable_arm.name());
        // If the dimension of the model instance is 0, then we can skip it
        // since it doesn't contribute to the collision and we don't need to fix
        // it.
        if (robot_model().plant().num_positions(movable_model_instance) == 0) {
          logging::log()->debug(
              "DracoPlanner:BestApproachTrajectory: Skipping model instance {} "
              "of arm {} since it has 0 positions",
              robot_model().plant().GetModelInstanceName(
                  movable_model_instance),
              movable_arm.name());
          continue;
        }
        const auto& robot_name =
            robot_model().plant().GetModelInstanceName(movable_model_instance);
        double max_slice_time = std::min(
            t_constant, sliced_sys_timed_trajectory.at(robot_name).end_time());
        logging::log()->info(
            "DracoPlanner:BestApproachTrajectory: Replacing trajectory tail of "
            "robot {} from t = {}s to t = {}s with a constant trajectory",
            robot_name, max_slice_time,
            sliced_sys_timed_trajectory.at(robot_name).end_time());
        auto& traj = sliced_sys_timed_trajectory[robot_name];
        // log the time length of the original trajectory for debugging
        logging::log()->debug(
            "DracoPlanner:BestApproachTrajectory: Original trajectory for "
            "robot {} has end time {}s",
            robot_name, traj.end_time());
        auto& traj_time_parametrization = traj.time_scaling();
        auto& traj_path = traj.path();
        auto traj_time_parametrized_truncated {
            conversions::ClientPiecewisePolynomialToDrake(
                traj_time_parametrization)
                .SliceByTime(traj_time_parametrization.start_time(),
                             max_slice_time)};
        auto s_at_max_slice_time =
            traj_time_parametrization.Value(max_slice_time)[0];
        auto traj_path_truncated =
            conversions::ClientPiecewisePolynomialToDrake(traj_path)
                .SliceByTime(traj_path.start_time(), s_at_max_slice_time);
        traj = planning_service_client::TimedTrajectory(
            conversions::DrakePiecewisePolynomialToClient(traj_path_truncated),
            conversions::DrakePiecewisePolynomialToClient(
                traj_time_parametrized_truncated),
            traj.global_time_offset());
        // log the time length  of the new trajectory for debugging
        logging::log()->debug(
            "DracoPlanner:BestApproachTrajectory: New trajectory for robot {} "
            "has end time {}s",
            robot_name, traj.end_time());
        arm_sliced_sys_timed_trajectory[robot_name] = traj;
      }
      // Use the arm-specific time_optimal_spliner to convert back to timed
      // polynomial
      auto arm_path_parametrized_trajectory =
          conversions::ToPathParameterizedTrajectory(
              *arms_time_optimal_spliners_.at(movable_arm_idx),
              arm_sliced_sys_timed_trajectory);
      // Re-toppra the path to get new time parametrization
      const auto* path_pp =
          dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
              &arm_path_parametrized_trajectory.path());
      DRAKE_DEMAND(path_pp != nullptr);
      const auto pp_path_removed_prepend =
          motion::splining::internal::RemoveConstantPrepend(*path_pp);
      const auto time_parameterization_opt =
          arms_time_optimal_spliners_.at(movable_arm_idx)
              ->RunToppraOnPiecewiseTrajectory(pp_path_removed_prepend);
      if (!time_parameterization_opt.has_value()) {
        logging::log()->info(
            "DracoPlanner:BestApproachTrajectory: Toppra failed on the sliced "
            "trajectory for arm {}. Skipping this approach time {}s.",
            movable_arm.name(), best_approach_time);
        continue;
      }
      const auto& retimed_trajectory =
          drake::trajectories::PathParameterizedTrajectory<double>(
              pp_path_removed_prepend, time_parameterization_opt.value());
      const auto arm_new_sys_timed_trajectory =
          conversions::ToSystemTimedTrajectory(
              *arms_time_optimal_spliners_.at(movable_arm_idx),
              retimed_trajectory);
      // Update the sliced_sys_timed_trajectory with the new arm trajectory
      for (const auto& [robot_name, traj] : arm_new_sys_timed_trajectory) {
        logging::log()->info(
            "DracoPlanner:BestApproachTrajectory: Updating trajectory of robot "
            "{} with new retimed trajectory for arm {}",
            robot_name, movable_arm.name());
        sliced_sys_timed_trajectory[robot_name] = traj;
      }
    }

    // Log the time duration of the trajs for each ressource, as well as the
    // break sizes, time_scaling break sizes, and global time offsets for
    // debugging
    for (const auto& [robot_name, traj] : sliced_sys_timed_trajectory) {
      logging::log()->debug(
          "DracoPlanner:BestApproachTrajectory: After slicing, trajectory for "
          "robot {} has end time {}s",
          robot_name, traj.end_time());
      logging::log()->debug(
          "DracoPlanner:BestApproachTrajectory: After slicing, trajectory for "
          "robot {} has breaks size {} and time scaling breaks size {}",
          robot_name, traj.path().breaks().size(),
          traj.time_scaling().breaks().size());
      logging::log()->debug(
          "DracoPlanner:BestApproachTrajectory: After slicing, trajectory for "
          "robot {} has global time offset {}s",
          robot_name, traj.global_time_offset());
    }

    // Check if the new trajectory is valid
    auto collision_time_opt =
        TimeOfArmsCollision(sliced_sys_timed_trajectory, step);
    if (!collision_time_opt.has_value()) {
      logging::log()->debug(
          "DracoPlanner:BestApproachTrajectory: Found valid trajectory by "
          "fixing active arms from t = {:.2f}s to t = {:.2f}s",
          t_constant, final_time);
      return sliced_sys_timed_trajectory;
    } else {
      logging::log()->debug(
          "DracoPlanner:BestApproachTrajectory: Collision still found at t = "
          "{}s after fixing active arms from t = {:.2f}s to t = {:.2f}s",
          collision_time_opt.value(), t_constant, final_time);
    }
  }
  logging::log()->info(
      "DracoPlanner:BestApproachTrajectory: Could not find a valid trajectory "
      "by fixing active arms. Returning nullopt.");
  return std::nullopt;
}

std::expected<SolveGeneralizedMultiModalPlanResult, std::string>
DracoPlanner::SolveGeneralizedMultiModalPlan(
    const planning_service_client::planner::
        GeneralizedMultimodalPlanningProblem& problem,
    const planning_service_client::SystemConf& start_sysconf,
    const std::optional<planning_service_client::SystemTimedTrajectory>&
        maybe_active_trajectory) const {
  // ToDo(@ramy, @sadra) only use start_sysconf for this.
  const auto q_ref =
      conversions::ToGeneralizedPosition(robot_model(), start_sysconf);
  motion::CheckSatisfiedOptions check_options;
  check_options.verbose = true;
  if (!robot_constraints().CheckSatisfied(q_ref, 0, check_options)) {
    auto msg = "Start configuration is invalid.";
    logging::log()->error("DracoPlanner:SolveGeneralizedMultiModalPlan {}",
                          msg);
    AddToVisualizer(q_ref, "Invalid start conf");
    return std::unexpected(msg);
  }
  std::vector<Eigen::VectorXd> start_seq {q_ref};
  if (!problem.start_anchors().empty()) {
    // --- 1. Solve start anchor sequence ---
    const auto& start_anchors = problem.start_anchors();
    logging::log()->info(
        "DracoPlanner:SolveGeneralizedMultiModalPlan Resolving start "
        "sequential wayposes...");
    const auto start_seq_result =
        SolveSequentialAnchors(start_anchors, q_ref, true,
                               problem.allow_partial_start_anchor_solutions());
    if (!start_seq_result.has_value()) {
      auto msg = fmt::format("Failed to resolve start sequential wayposes: {}",
                             start_seq_result.error());
      logging::log()->error("DracoPlanner:SolveGeneralizedMultiModalPlan {}",
                            msg);
      return std::unexpected(msg);
    }
    // push start_seq_result to start_seq
    const auto& start_seq_result_vec = start_seq_result.value();
    start_seq.insert(start_seq.end(), start_seq_result_vec.begin(),
                     start_seq_result_vec.end());
    DRAKE_DEMAND(problem.allow_partial_start_anchor_solutions()
                 || (start_seq.size() == start_anchors.size() + 1));
  }
  // --- 2. Solve goal anchor sequence ---
  const auto& goal_anchors = problem.goal_anchors();
  logging::log()->info(
      "DracoPlanner:SolveGeneralizedMultiModalPlan Solving goal "
      "sequential wayposes.");
  const auto goal_seq_result =
      SolveSequentialAnchors(goal_anchors, start_seq.back(), false,
                             problem.allow_partial_goal_anchor_solutions());
  if (!goal_seq_result.has_value()) {
    auto msg = fmt::format("Failed to resolve goal sequential wayposes: {}",
                           goal_seq_result.error());
    logging::log()->error("DracoPlanner:SolveGeneralizedMultiModalPlan {}",
                          msg);
    return std::unexpected(msg);
  }
  const auto& goal_seq = goal_seq_result.value();
  DRAKE_DEMAND(problem.allow_partial_goal_anchor_solutions()
               || goal_seq.size() == goal_anchors.size());

  // --- 3. Solve transit plan ---
  std::optional<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                          drake::trajectories::PiecewisePolynomial<double>>>
      transit_result;
  if (!start_seq.empty() && !goal_seq.empty()) {
    const Eigen::VectorXd& transit_start = start_seq.back();
    const Eigen::VectorXd& transit_goal = goal_seq.front();
    auto transit_result_opt = CalcTrajectoryBestAvailablePlanner(
        transit_start, transit_goal, problem.fast_estimate_solution());
    if (!transit_result_opt.has_value()) {
      auto msg =
          "Failed to solve the transit plan between start and goal anchor "
          "sequences.";
      logging::log()->error("DracoPlanner:SolveGeneralizedMultiModalPlan {}",
                            msg);
      return std::unexpected(msg);
    }
    transit_result = transit_result_opt.value();
  }
  // --- 4. Concatenate all segments ---
  draco::planner::SolveGeneralizedMultiModalPlanResult result;
  std::vector<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                        drake::trajectories::PiecewisePolynomial<double>>>
      trajs_to_combine;
  if (start_seq.size() > 1) {
    auto start_path_time_opt = SplineAndTimePath(start_seq);
    if (!start_path_time_opt.has_value()) {
      auto msg = fmt::format(
          "DracoPlanner:SolveGeneralizedMultiModalPlan: Could not spline the "
          "start sequence: {}",
          start_path_time_opt.error());
      logging::log()->error("{}", msg);
      return std::unexpected(msg);
    }
    trajs_to_combine.push_back(start_path_time_opt.value());
    result.transit_start_time = start_path_time_opt.value().second.end_time();
  } else {
    result.transit_start_time = 0.0;
  }
  DRAKE_DEMAND(transit_result.has_value() || start_seq.empty()
               || goal_seq.empty());
  if (transit_result.has_value()) {
    trajs_to_combine.push_back(transit_result.value());
    result.transit_end_time =
        result.transit_start_time + transit_result.value().second.end_time();
  } else {
    result.transit_end_time = result.transit_start_time;
  }
  if (goal_seq.size() > 1) {
    auto goal_path_time_opt = SplineAndTimePath(goal_seq);
    if (!goal_path_time_opt.has_value()) {
      auto msg = fmt::format(
          "DracoPlanner:SolveGeneralizedMultiModalPlan: Could not spline the "
          "goal sequence: {}",
          goal_path_time_opt.error());
      logging::log()->error("{}", msg);
      return std::unexpected(msg);
    }
    trajs_to_combine.push_back(goal_path_time_opt.value());
  }

  result.trajectory =
      motion::splining::internal::CombineSequentialSystemTimedTrajectories(
          trajs_to_combine);
  if (maybe_active_trajectory.has_value()) {
    logging::log()->info(
        "DracoPlanner:SolveGeneralizedMultiModalPlan: Merging with active "
        "trajectory.");
  }
  return result;
}

std::vector<planning_service_client::planner::Anchor>
DracoPlanner::ConstructWaypointAnchorsFromRelativeTransforms(
    const planning_service_client::planner::Anchor& start_anchor,
    const std::vector<planning_service_client::FrameRelativePose>&
        goal_offset_frps,
    const std::optional<Eigen::VectorXd>& q_ref_opt,
    double max_translation_spacing, double max_rotation_spacing) const {
  std::vector<planning_service_client::planner::Anchor> waypoint_anchors {};
  // Resolve the start anchor
  auto start_conf_opt = ResolveAnchorConf(start_anchor, q_ref_opt, false);
  if (!start_conf_opt.has_value() && !start_conf_opt.is_conflicted()) {
    logging::log()->error(
        "DracoPlanner:ConstructWaypointAnchorsFromRelativeTransforms: Could "
        "not resolve start anchor because {}:",
        start_conf_opt.error());
    return waypoint_anchors;
  }
  const auto& world_frame = robot_model().plant().world_frame();
  DRAKE_DEMAND(start_conf_opt.has_value() || start_conf_opt.is_conflicted());
  const auto& start_conf = start_conf_opt.has_value()
                               ? start_conf_opt.value()
                               : start_conf_opt.conflicted_value();
  // Compute the start frame relative poses
  int needed_num_steps = 0;
  // std::unordered_set<motion::ArmIndex> moving_arms;
  std::vector<planning_service_client::FrameRelativePose> start_frps;
  std::vector<planning_service_client::FrameRelativePose> goal_frps;
  for (const auto& goal_offset_pose : goal_offset_frps) {
    // Check if the goal pose has the same frame pairs as the start anchor
    const auto& frame_A_str = goal_offset_pose.frame_A();
    const auto& frame_B_str = goal_offset_pose.frame_B();
    // Calculate the frame relative pose for the start anchor
    const auto& frame_A = robot_model().GetScopedFrameByName(frame_A_str);
    const auto& frame_B = robot_model().GetScopedFrameByName(frame_B_str);
    drake::math::RigidTransformd offset(goal_offset_pose.X_AB_quaternion(),
                                        goal_offset_pose.X_AB_translation());
    const auto X_AB_start =
        robot_model().CalcRelativeTransform(start_conf, frame_A, frame_B);
    const auto X_world_A_start =
        robot_model().CalcRelativeTransform(start_conf, world_frame, frame_A);
    const auto X_world_B_start = X_world_A_start * X_AB_start;
    const auto X_world_B_goal = X_world_A_start * offset * X_AB_start;
    planning_service_client::FrameRelativePose start_pose {
        world_frame.name(), frame_B_str, X_world_B_start.translation(),
        X_world_B_start.rotation().ToQuaternion()};
    start_frps.push_back(start_pose);
    planning_service_client::FrameRelativePose goal_pose {
        world_frame.name(), frame_B_str, X_world_B_goal.translation(),
        X_world_B_goal.rotation().ToQuaternion()};
    goal_frps.push_back(goal_pose);

    const auto delta_pose = X_world_B_start.inverse() * X_world_B_goal;
    // Calculate the translation and rotation norms
    const auto translation_norm = delta_pose.translation().norm();
    const auto rotation_norm = delta_pose.rotation().ToAngleAxis().angle();
    // Calculate the number of steps for this pose pair
    const int num_steps_fp =
        std::max({static_cast<int>(translation_norm / max_translation_spacing),
                  static_cast<int>(rotation_norm / max_rotation_spacing), 3});
    needed_num_steps = std::max(needed_num_steps, num_steps_fp);
  }
  if (needed_num_steps == 0) {
    logging::log()->error(
        "DracoPlanner:ConstructWaypointAnchorsFromRelativeTransforms: Could "
        "not "
        "interpolate waypoint anchors because the needed number of steps is "
        "zero. This can happen "
        "if the start and goal poses are the same or if the max translation "
        "and rotation spacing are too large.");
    return waypoint_anchors;
  }

  for (int i = 0; i < needed_num_steps; ++i) {
    double t = static_cast<double>(i) / (needed_num_steps - 1);
    logging::log()->debug(
        "DracoPlanner:ConstructWaypointAnchorsFromRelativeTransforms: Creating "
        "waypoint "
        "anchor {}/{} (t={:.3f})",
        i + 1, needed_num_steps, t);
    std::vector<planning_service_client::FrameRelativePose> interpolated_frps;
    for (size_t j = 0; j < start_frps.size(); ++j) {
      const auto& start_pose = start_frps[j];
      const auto& goal_pose = goal_frps[j];
      drake::math::RigidTransformd start_X_AB(start_pose.X_AB_quaternion(),
                                              start_pose.X_AB_translation());
      drake::math::RigidTransformd goal_X_AB(goal_pose.X_AB_quaternion(),
                                             goal_pose.X_AB_translation());
      drake::math::RigidTransformd interpolated_X_AB =
          motion::planning::IkPlanner::CalcInterpolatedPose(start_X_AB,
                                                            goal_X_AB, t);
      planning_service_client::FrameRelativePose interpolated_pose {
          start_pose.frame_A(), start_pose.frame_B(),
          interpolated_X_AB.translation(),
          interpolated_X_AB.rotation().ToQuaternion()};
      interpolated_frps.push_back(interpolated_pose);
    }
    planning_service_client::planner::Anchor waypoint_anchor {
        planning_service_client::SystemConf(), interpolated_frps};
    waypoint_anchors.push_back(waypoint_anchor);
  }
  logging::log()->info(
      "DracoPlanner:ConstructWaypointAnchorsFromRelativeTransforms: Generated "
      "{} "
      "waypoint anchors.",
      waypoint_anchors.size());
  return waypoint_anchors;
}

std::expected<std::vector<planning_service_client::planner::Anchor>,
              std::string>
DracoPlanner::ConstructWaypointAnchorsFromAbsoluteTransforms(
    const planning_service_client::planner::Anchor& start_anchor,
    const std::vector<planning_service_client::FrameRelativePose>& goal_frps,
    const std::optional<Eigen::VectorXd>& q_ref_opt,
    double max_translation_spacing, double max_rotation_spacing) const {
  std::vector<planning_service_client::planner::Anchor> waypoint_anchors {};
  // Resolve the start anchor
  auto start_conf_opt = ResolveAnchorConf(start_anchor, q_ref_opt);
  if (!start_conf_opt.has_value()) {
    auto error_msg = fmt::format(
        "DracoPlanner:ConstructWaypointAnchorsFromAbsoluteTransforms: Could "
        "not resolve start anchor because {}",
        start_conf_opt.error());
    logging::log()->error(error_msg);
    return std::unexpected(error_msg);
  }
  const auto& start_conf = start_conf_opt.value();
  // Compute the start frame relative poses
  int needed_num_steps = 0;
  // std::unordered_set<motion::ArmIndex> moving_arms;
  std::vector<planning_service_client::FrameRelativePose> start_frps;
  for (const auto& goal_frp : goal_frps) {
    // Check if the goal pose has the same frame pairs as the start anchor
    const auto& frame_A_str = goal_frp.frame_A();
    const auto& frame_B_str = goal_frp.frame_B();
    // Calculate the frame relative pose for the start anchor
    const auto& frame_A = robot_model().GetScopedFrameByName(frame_A_str);
    const auto& frame_B = robot_model().GetScopedFrameByName(frame_B_str);
    drake::math::RigidTransformd X_AB_goal(goal_frp.X_AB_quaternion(),
                                           goal_frp.X_AB_translation());
    const auto X_AB_start =
        robot_model().CalcRelativeTransform(start_conf, frame_A, frame_B);
    planning_service_client::FrameRelativePose start_pose {
        frame_A_str, frame_B_str, X_AB_start.translation(),
        X_AB_start.rotation().ToQuaternion()};
    start_frps.push_back(start_pose);
    const auto delta_pose = X_AB_start.inverse() * X_AB_goal;
    // Calculate the translation and rotation norms
    const auto translation_norm = delta_pose.translation().norm();
    const auto rotation_norm = delta_pose.rotation().ToAngleAxis().angle();
    // Calculate the number of steps for this pose pair
    const int num_steps_fp =
        std::max({static_cast<int>(translation_norm / max_translation_spacing),
                  static_cast<int>(rotation_norm / max_rotation_spacing), 3});
    needed_num_steps = std::max(needed_num_steps, num_steps_fp);
  }
  if (needed_num_steps == 0) {
    auto error_msg =
        "DracoPlanner:ConstructWaypointAnchorsFromAbsoluteTransforms: Could "
        "not "
        "interpolate waypoint anchors because the needed number of steps is "
        "zero. This can happen if the start and goal poses are the same or if "
        "the max translation and rotation spacing are too large.";
    logging::log()->error(error_msg);
    return std::unexpected(error_msg);
  }

  for (int i = 0; i < needed_num_steps; ++i) {
    double t = static_cast<double>(i) / (needed_num_steps - 1);
    std::vector<planning_service_client::FrameRelativePose> interpolated_frps;
    for (size_t j = 0; j < start_frps.size(); ++j) {
      const auto& start_pose = start_frps[j];
      const auto& goal_pose = goal_frps[j];
      drake::math::RigidTransformd start_X_AB(start_pose.X_AB_quaternion(),
                                              start_pose.X_AB_translation());
      drake::math::RigidTransformd goal_X_AB(goal_pose.X_AB_quaternion(),
                                             goal_pose.X_AB_translation());
      drake::math::RigidTransformd interpolated_X_AB =
          motion::planning::IkPlanner::CalcInterpolatedPose(start_X_AB,
                                                            goal_X_AB, t);
      planning_service_client::FrameRelativePose interpolated_pose {
          start_pose.frame_A(), start_pose.frame_B(),
          interpolated_X_AB.translation(),
          interpolated_X_AB.rotation().ToQuaternion()};
      interpolated_frps.push_back(interpolated_pose);
    }
    planning_service_client::planner::Anchor waypoint_anchor {
        planning_service_client::SystemConf(), interpolated_frps};
    waypoint_anchors.push_back(waypoint_anchor);
  }
  logging::log()->info(
      "DracoPlanner:ConstructWaypointAnchorsFromAbsoluteTransforms: Generated "
      "{} "
      "waypoint anchors.",
      waypoint_anchors.size());
  return waypoint_anchors;
}

std::vector<planning_service_client::planner::Anchor>
DracoPlanner::ConstructWaypointAnchorsFromWayposesVec(
    const std::vector<planning_service_client::FrameRelativePosesVec>& wayposes,
    std::optional<planning_service_client::SystemConf> fixed_sysconf_opt)
    const {
  std::vector<planning_service_client::planner::Anchor> waypoint_anchors;
  for (const auto& waypose_frps : wayposes) {
    planning_service_client::planner::Anchor waypoint_anchor {
        fixed_sysconf_opt.value_or(planning_service_client::SystemConf()),
        waypose_frps.FrameRelativePoses()};
    waypoint_anchors.push_back(waypoint_anchor);
  }
  return waypoint_anchors;
}

// Write a function that creates a generazlized multimodal planning problem
// from a regular
planning_service_client::planner::GeneralizedMultimodalPlanningProblem
DracoPlanner::ConstructGeneralizedProblemFromMultimodal(
    const planning_service_client::planner::MultimodalPlanningProblem&
        multimodal_problem,
    double max_translation_spacing, double max_rotation_spacing) const {
  // Resolve the waypoint anchors from the relative transforms
  std::vector<planning_service_client::planner::Anchor> start_anchors =
      ConstructWaypointAnchorsFromRelativeTransforms(
          multimodal_problem.start(),
          multimodal_problem.start_transform_poses(), std::nullopt,
          max_translation_spacing, max_rotation_spacing);

  // Convert start system conf to Eigen::VectorXd
  auto q_ref = conversions::ToGeneralizedPosition(
      robot_model(), multimodal_problem.start().system_conf());
  std::vector<planning_service_client::planner::Anchor> goal_anchors_reversed =
      ConstructWaypointAnchorsFromRelativeTransforms(
          multimodal_problem.goal(), multimodal_problem.goal_transform_poses(),
          q_ref, max_translation_spacing, max_rotation_spacing);
  // Reverse the goal anchors to have them from transit to goal
  std::vector<planning_service_client::planner::Anchor> goal_anchors {
      goal_anchors_reversed.rbegin(), goal_anchors_reversed.rend()};
  goal_anchors.push_back(multimodal_problem.goal());
  // Create the generalized multimodal planning problem
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      generalized_problem(start_anchors, goal_anchors, false, false, false,
                          multimodal_problem.allow_async_partial_solutions());
  return generalized_problem;
}

std::expected<
    planning_service_client::planner::GeneralizedMultimodalPlanningProblem,
    std::string>
DracoPlanner::ConstructGeneralizedProblemFromCartesianLinearProblem(
    const planning_service_client::planner::CartesianLinearMoveProblem&
        cartesian_problem,
    double max_translation_spacing, double max_rotation_spacing) const {
  auto start_anchors_result = ConstructWaypointAnchorsFromAbsoluteTransforms(
      cartesian_problem.start(), cartesian_problem.start_transform_poses(),
      std::nullopt, max_translation_spacing, max_rotation_spacing);
  if (!start_anchors_result.has_value()) {
    return std::unexpected(start_anchors_result.error());
  }
  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
      generalized_problem(start_anchors_result.value(), {}, false, false, false,
                          cartesian_problem.allow_async_partial_solutions());
  return generalized_problem;
}

std::expected<std::vector<Eigen::VectorXd>, std::string>
DracoPlanner::SolveSequentialAnchors(
    const std::vector<planning_service_client::planner::Anchor>& anchors,
    const Eigen::VectorXd& q_ref, bool only_use_ref_seed,
    bool return_longest_incomplete_solution) const {
  if (anchors.empty()) {
    return std::vector<Eigen::VectorXd> {};
  }
  std::vector<Eigen::VectorXd> seed_vec {q_ref};
  if (!only_use_ref_seed) {
    // Extract the start anchor frps from all anchors
    motion::planning::FrameRelativePoses relevant_frps;
    for (const auto& anchor : anchors) {
      const auto& frps = anchor.poses();
      // Convert to draco frame relative poses
      auto draco_frps {
          conversions::ToDracoFrameRelativePoses(robot_model(), frps)};
      // Add to relevant frps -- duplicate frps are okay
      for (const auto& frp : draco_frps) {
        relevant_frps.push_back(frp);
      }
    }
    if (!relevant_frps.empty()) {
      // Get seeds from the IK cache
      seed_vec =
          ik_planner().ik_cache().CalcClosestSeed(relevant_frps, q_ref, 1.0, 5);
      logging::log()->debug(
          "DracoPlanner:SolveSequentialAnchors: Found {} seeds in the IK "
          "cache.",
          seed_vec.size());
    }
  }
  logging::log()->info(
      "DracoPlanner:SolveSequentialAnchors: Attempting to resolve {} anchors "
      "sequentially ({} seeds available).",
      anchors.size(), seed_vec.size());
  std::optional<std::string> best_seed_failure_msg;
  std::vector<Eigen::VectorXd> best_partial_solution;
  size_t best_partial_size = 0;
  for (int seed_idx = 0; seed_idx < std::ssize(seed_vec); ++seed_idx) {
    const auto& seed = seed_vec[seed_idx];
    std::vector<Eigen::VectorXd> configs;
    logging::log()->debug(
        "DracoPlanner:SolveSequentialAnchors: Attempting to resolve "
        "anchors with seed {}/{}.",
        seed_idx + 1, seed_vec.size());
    bool success = true;
    std::vector<Eigen::VectorXd> conflicted_configs;
    std::set<drake::multibody::ModelInstanceIndex>
        conflicted_fixed_model_instances;
    bool needs_deconfliction = false;
    for (int i = 0; i < std::ssize(anchors); ++i) {
      const auto& anchor = anchors[i];
      auto conf_result = ResolveAnchorConf(
          anchor, configs.empty() ? seed : configs.back(), true, false);
      std::optional<Eigen::VectorXd> conflicted_conf_opt;
      if (!conf_result.has_value()) {
        if (conf_result.is_conflicted()) {
          logging::log()->debug(
              "DracoPlanner:SolveSequentialAnchors: Multi-arm confliction "
              "detected for anchor {}/{}. Will try to deconflict later.",
              i + 1, anchors.size());
          // Ok, pretend as if nothing has happened. Just gather them.
          const auto& [conflicted_conf, fixed_models] =
              conf_result.conflicted_conf_and_fixed_models();
          conflicted_conf_opt = conflicted_conf;
          conflicted_fixed_model_instances.insert(fixed_models.begin(),
                                                  fixed_models.end());
          needs_deconfliction = true;
        } else {
          logging::log()->debug(
              "DracoPlanner:SolveSequentialAnchors: Anchor resolution failed "
              "for anchor {}/{}: {}. Will try other seeds if available.",
              i + 1, anchors.size(), conf_result.error());
          // Failed.
          success = false;
          auto msg = fmt::format("Failed to resolve anchor {}/{}: {}", i + 1,
                                 anchors.size(), conf_result.error());
          if (seed_idx == 0 && !best_seed_failure_msg.has_value()) {
            best_seed_failure_msg = msg;
          }
          logging::log()->error("DracoPlanner:SolveSequentialAnchors: {}", msg);
          // If msg contains "unavoidable collision", normally we'd stop.
          if (common::utils::string_includes(conf_result.error(),
                                             "unavoidable collision")) {
            logging::log()->error(
                "DracoPlanner:SolveSequentialAnchors: Detected unavoidable "
                "collision for seed {}/{}.",
                seed_idx + 1, seed_vec.size());
            // Treat as failed seed; capture partial solution if requested.
            if (return_longest_incomplete_solution) {
              if (configs.size() > best_partial_size) {
                best_partial_size = configs.size();
                best_partial_solution = configs;
              }
              // Keep success=false so we continue trying other seeds.
              break;
            }
            return std::unexpected(msg);
          }
          // Capture partial progress for this seed if requested.
          if (return_longest_incomplete_solution) {
            if (configs.size() > best_partial_size) {
              best_partial_size = configs.size();
              best_partial_solution = configs;
            }
            // Keep success=false so we continue trying other seeds.
            break;
          }
          // Otherwise, break the for loop and try the next seed.
          success = false;
          break;
        }
      }
      DRAKE_DEMAND(conflicted_conf_opt.has_value() || conf_result.has_value());
      configs.push_back(conf_result.has_value() ? conf_result.value()
                                                : conflicted_conf_opt.value());
    }
    if (success
        || (best_partial_size > 0 && return_longest_incomplete_solution)) {
      logging::log()->info(
          "DracoPlanner::SolveSequentialAnchors: Successfully resolved {}/{} "
          "anchors with seed {}/{}. Deconfliction needed: {}",
          configs.size(), anchors.size(), seed_idx + 1, seed_vec.size(),
          needs_deconfliction);
      if (!needs_deconfliction) {
        logging::log()->info(
            "DracoPlanner:SolveSequentialAnchors: Successfully resolved {} "
            "anchors.",
            configs.size());
        return configs;
      }
      bool is_start_sequence = only_use_ref_seed;
      std::optional<Eigen::VectorXd> maybe_q_start;
      if (is_start_sequence) {
        maybe_q_start = q_ref;
      }
      auto q_opt = thunder_planner().CalcNearestValidConf(
          configs, conflicted_fixed_model_instances,
          options().planner_options.deconfliction_offset,
          options().planner_options.deconfliction_step, maybe_q_start);
      if (!q_opt.has_value()) {
        auto msg = fmt::format(
            "Failed to deconflict the multiple conflicting "
            "configurations.");
        logging::log()->error("DracoPlanner:SolveSequentialAnchors: {}", msg);
        if (return_longest_incomplete_solution) {
          // Use the configs collected so far as a candidate partial solution.
          if (configs.size() > best_partial_size) {
            best_partial_size = configs.size();
            best_partial_solution = configs;
          }
          // try next seed instead of failing outright
          continue;
        }
        return std::unexpected(msg);
      }
      std::vector<Eigen::VectorXd> result;
      const auto& delta = q_opt.value() - configs.front();
      // We can shift all configs by the same delta.
      for (const auto& conf : configs) {
        result.push_back(conf + delta);
      }
      return result;
    }
  }
  auto msg = fmt::format(
      "All seeds failed to resolve the sequential anchors. Best seed failed: "
      "{}",
      best_seed_failure_msg.value_or("No seeds were available."));
  logging::log()->error("DracoPlanner:SolveSequentialAnchors: {}", msg);
  if (return_longest_incomplete_solution && best_partial_size > 0) {
    logging::log()->info(
        "DracoPlanner:SolveSequentialAnchors: Returning longest partial "
        "solution of size {}.",
        best_partial_size);
    return best_partial_solution;
  }
  return std::unexpected(msg);
}

}  // namespace planner
}  // namespace draco
