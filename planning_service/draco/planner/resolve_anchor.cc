#include <magic_enum/magic_enum.hpp>

#include "draco_planner.h"
#include "planning_service/draco/client_conversions.h"

namespace draco {
namespace planner {

DracoPlanner::ResolveAnchorConfResult DracoPlanner::ResolveAnchorConf(
    const planning_service_client::planner::Anchor& anchor,
    const std::optional<Eigen::VectorXd>& q_ref_opt, bool only_use_passed_seed,
    bool verbose) const {
  if (verbose) {
    logging::log()->info(
        "DracoPlanner:ResolveAnchorConf: Resolving anchor configuration with "
        "anchor sysconf size: {}, poses size: {}, "
        "q_ref_opt provided: {}",
        anchor.system_conf().size(), anchor.poses().size(),
        q_ref_opt.has_value());
  }
  // If fixed_system_conf is fully populated, we don't need to solve the IK
  // problem. We can just return the fixed system configuration.
  motion::system_conf_t fixed_sysconf;
  std::unordered_set<drake::multibody::ModelInstanceIndex>
      fixed_model_instances;
  for (const auto& [key, value] : anchor.system_conf()) {
    auto model_instance = robot_model().plant().GetModelInstanceByName(key);
    fixed_model_instances.insert(model_instance);
    fixed_sysconf[key] = value.q();
  }
  motion::CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = verbose;
  if (robot_model().IsSysconfComplete(fixed_sysconf)) {
    // if frps are populated, throw as we don't need to solve the IK
    if (!anchor.poses().empty()) {
      auto msg = fmt::format(
          "The anchor has a complete fixed "
          "sysconf, as well as frame relative poses to solve for. This is not "
          "supported. Please provide either a complete fixed sysconf or frame "
          "relative poses, but not both");
      logging::log()->error("DracoPlanner:ResolveAnchorConf: {}", msg);
      return msg;
    }
    // Run CheckSatisfied
    auto q_result = robot_model().ToGeneralizedPosition(fixed_sysconf);
    bool is_valid = robot_constraints().CheckSatisfied(q_result, 0,
                                                       check_satisfied_options);
    if (is_valid) {
      logging::log()->info(
          "DracoPlanner:ResolveAnchorConf: Returning valid complete conf");
      return q_result;  // Return the fixed sysconf as a valid solution
    } else {
      // We have an invalid complete conf
      // Let the visualizer show the conf
      AddToVisualizer(q_result, "Invalid complete conf");
      auto msg = fmt::format(
          "DracoPlanner:ResolveAnchorConf: The anchor is complete conf and "
          "not valid.");
      return msg;
    }
  }
  DRAKE_THROW_UNLESS(q_ref_opt.has_value());
  Eigen::VectorXd q_ref = q_ref_opt.value();
  for (const auto& [key, value] : anchor.system_conf()) {
    auto model_instance = robot_model().plant().GetModelInstanceByName(key);
    robot_model().plant().SetPositionsInArray(model_instance, value.q(),
                                              &q_ref);
  }
  // Let's solve IK Problems
  motion::planning::FrameRelativePoses frame_relative_poses;
  // Populate frame_relative_poses
  for (const auto& pose : anchor.poses()) {
    const auto& frame_A = drake::multibody::parsing::GetScopedFrameByName(
        robot_model().plant(), pose.frame_A());
    const auto& frame_B = drake::multibody::parsing::GetScopedFrameByName(
        robot_model().plant(), pose.frame_B());
    auto instance_A = frame_A.model_instance();
    auto instance_B = frame_B.model_instance();
    if (!fixed_model_instances.contains(instance_A)) {
      fixed_model_instances.insert(instance_A);
    }
    if (!fixed_model_instances.contains(instance_B)) {
      fixed_model_instances.insert(instance_B);
    }
    auto X_AB = drake::math::RigidTransformd(pose.X_AB_quaternion(),
                                             pose.X_AB_translation());
    frame_relative_poses.push_back(std::make_tuple(&frame_A, &frame_B, X_AB));
  }
  std::expected<Eigen::VectorXd, std::string> q_result;
  bool need_multi_arm_deconfliction = false;
  if (!frame_relative_poses.empty()) {
    // Solve the IK problem
    motion::planning::IkPlannerOptions ik_options;
    ik_options.num_seeds = -1;  // use all cached seeds
    ik_options.num_random_seeds = 5;
    ik_options.insert_random_seed_into_cache = true;
    // If q_ref_opt is provided, we fix the idle joints to the
    // reference configuration. Otherwise, we don't fix the idle joints.
    ik_options.fix_idle_joints = q_ref_opt.has_value();
    ik_options.resolve_with_collision_avoidance = false;
    ik_options.ignore_multi_arm_collision = true;
    ik_options.select_seed_via_two_steps = false;
    motion::planning::IkPlanner::IkResult ik_result;
    if (only_use_passed_seed) {
      ik_result =
          ik_planner().SolveIk(frame_relative_poses, q_ref, 0, ik_options);
    } else {
      ik_result =
          ik_planner().SolveGlobalIk(frame_relative_poses, q_ref, ik_options);
    }
    // if q_opt has errors, then we give up.
    if (!ik_result.optimization_success()) {
      auto msg = fmt::format(
          "DracoPlanner:ResolveAnchorConf: IK optimization failed: {}",
          ik_result.failure_status_message());
      logging::log()->error("DracoPlanner:ResolveAnchorConf: {}", msg);
      if (has_draco_visualizer()) {
        auto failed_ik_frps_index =
            std::vector<std::tuple<drake::multibody::FrameIndex,
                                   drake::multibody::FrameIndex,
                                   drake::math::RigidTransformd>>();
        for (const auto& frp : frame_relative_poses) {
          failed_ik_frps_index.push_back(
              std::make_tuple(std::get<0>(frp)->index(),
                              std::get<1>(frp)->index(), std::get<2>(frp)));
        }
        const auto q_best_effort = ik_result.value();
        auto failed_ik = std::make_pair(q_best_effort, failed_ik_frps_index);
        AddToVisualizer(failed_ik, "Failed IK optimization");
      }
      return msg;
    } else if (ik_result.unavoidable_collision()) {
      auto msg = fmt::format(
          "DracoPlanner:ResolveAnchorConf: IK solution is in unavoidable "
          "collision: {}",
          ik_result.failure_status_message());
      logging::log()->error("DracoPlanner:ResolveAnchorConf: {}", msg);
      AddToVisualizer(ik_result.value(), "Colliding IK pose");
      return msg;
    } else if (ik_result.multiarm_collision()) {
      need_multi_arm_deconfliction = true;
      q_result = ik_result.value();  // Copy the result
    } else if (ik_result.self_collision()) {
      auto msg = fmt::format(
          "DracoPlanner:ResolveAnchorConf: IK solution is in self-collision: "
          "{}. You may turn on self_collision_resolve_with_constraint in ik "
          "planner options to resolve the self-collision (it may be slow).",
          ik_result.failure_status_message());
      logging::log()->error("DracoPlanner:ResolveAnchorConf: {}", msg);
      AddToVisualizer(ik_result.value(), "Self-colliding IK pose");
      return msg;
    } else {
      DRAKE_DEMAND(ik_result.is_valid());
      logging::log()->info(
          "DracoPlanner:ResolveAnchorConf: Successfully resolved the anchor "
          "configuration with {} poses ",
          frame_relative_poses.size());
      return ik_result.value();  // Return the valid configuration
    }
  } else {
    if (fixed_sysconf.empty()) {
      auto msg = fmt::format(
          "DracoPlanner:ResolveAnchorConf: No frame relative poses provided, "
          "and no fixed system configuration provided. Cannot resolve the "
          "anchor configuration. Please provide either frame relative poses "
          "or a fixed system configuration.");
      logging::log()->error("DracoPlanner:ResolveAnchorConf: {}", msg);
      return msg;
    }
    q_result = q_ref;
  }
  DRAKE_DEMAND(q_result.has_value());
  bool is_valid = robot_constraints().CheckSatisfied(q_result.value(), 0,
                                                     check_satisfied_options);
  // By now we have a solution. Let's just check if it is valid. If not,
  // check if we need to deconflict it, if not already done before.
  if (!is_valid && !need_multi_arm_deconfliction) {
    need_multi_arm_deconfliction =
        robot_constraints().CalcAndClassifyCollisions(q_result.value())
        == motion::RobotConstraints::CollisionType::kAcrossArmsOnly;
  }
  if (is_valid) {
    if (verbose) {
      logging::log()->info(
          "DracoPlanner:ResolveAnchorConf: Successfully resolved an incomplete "
          "anchor to a valid configuration with {} poses",
          frame_relative_poses.size());
    }
    return q_result.value();
  } else if (need_multi_arm_deconfliction) {
    logging::log()->info(
        "DracoPlanner:ResolveAnchorConf: The provided reference configuration "
        "is not valid. Trying to deconflict it using the thunder planner.");
    // convert to vector, for now
    const std::vector<drake::multibody::ModelInstanceIndex>
        fixed_model_instances_vec {fixed_model_instances.begin(),
                                   fixed_model_instances.end()};
    logging::log()->info(
        "DracoPlanner:ResolveAnchorConf: Multi arm collision detected, but "
        "deconflition will happen later during multi-configuration "
        "deconfliction. Returning the conflicted configuration.");
    return std::make_pair(q_result.value(), fixed_model_instances_vec);
  }
  // If we reach here, we have failed.
  // Show q_result.value() in meshcat
  AddToVisualizer(q_result.value(), "Invalid anchor");
  return fmt::format(
      "DracoPlanner:ResolveAnchorConf: Could not resolve the anchor conf");
}

}  // namespace planner
}  // namespace draco
