
#include "planning_service/draco/client_conversions.h"
#include "shokunin_common.h"

namespace fs = std::filesystem;
namespace psc = planning_service_client;

psc::planner::GeneralizedMultimodalPlanningProblem EastToPlateProblem() {
  // Goal anchor will be with the east arm reaching to a pose
  std::vector<psc::planner::Anchor> goal_anchors;
  drake::math::RollPitchYaw<double> rpy(0, M_PI, 0.6);
  Eigen::Quaterniond quaternion = rpy.ToQuaternion();
  const std::string frame_plate = "plate1__plate_root";
  const std::string frame_eef = "panda_east__tweezer_tip_link";
  auto frp_goal = psc::FrameRelativePose(
      frame_plate, frame_eef, Eigen::Vector3d(0.02, -0.03, 0.04), quaternion);
  const auto goal_anchor = psc::planner::Anchor(psc::SystemConf(), {frp_goal});
  goal_anchors.push_back(goal_anchor);
  for (int i = 1; i < 5; ++i) {
    double z_offset = -0.001 * i;
    auto frp_goal_iteration = psc::FrameRelativePose(
        frame_eef, frame_eef,
        frp_goal.X_AB_translation() + Eigen::Vector3d(0.0, 0.0, z_offset),
        quaternion);
    logging::log()->info("Anchor translation for iteration {}: {}", i,
                         frp_goal_iteration.X_AB_translation().transpose());
    // const auto goal_anchor_iteration =
    // psc::planner::Anchor(psc::SystemConf(), {frp_goal_iteration});
    // goal_anchors.push_back(goal_anchor_iteration);
  }
  return psc::planner::GeneralizedMultimodalPlanningProblem({}, goal_anchors);
}

psc::planner::GeneralizedMultimodalPlanningProblem WestToPickProblem() {
  psc::planner::Anchor goal_anchor;
  std::vector<psc::planner::Anchor> goal_anchors;
  // Goal anchor will be with the east arm reaching to a pose
  drake::math::RollPitchYaw<double> rpy(0, M_PI / 2, 0.6);
  Eigen::Quaterniond quaternion = rpy.ToQuaternion();
  const std::string frame_plate = "plate1__plate_root";
  const std::string frame_eef = "panda_west__tool_mount_attach_link";
  auto frp_goal = psc::FrameRelativePose(
      frame_plate, frame_eef, Eigen::Vector3d(0.03, -0.0, 0.15), quaternion);
  goal_anchor = psc::planner::Anchor({}, {frp_goal});
  goal_anchors.push_back(goal_anchor);
  for (int i = 1; i < 5; ++i) {
    double z_offset = -0.001 * i;
    auto frp_goal_iteration = psc::FrameRelativePose(
        frame_eef, frame_eef,
        Eigen::Vector3d(0.03, -0.0, 0.15) + Eigen::Vector3d(0.0, 0.0, z_offset),
        quaternion);
    logging::log()->info("Anchor translation for iteration {}: {}", i,
                         frp_goal_iteration.X_AB_translation().transpose());
    // const auto goal_anchor_iteration =
    // psc::planner::Anchor(psc::SystemConf(), {frp_goal_iteration});
    // goal_anchors.push_back(goal_anchor_iteration);
  }
  const auto& gen_mm_problem =
      psc::planner::GeneralizedMultimodalPlanningProblem({}, goal_anchors);
  return gen_mm_problem;
}

int main(int argc, char** argv) {
  CLI::App app {"Async Motions"};
  logging::create_log("Async Motions");
  app.require_subcommand(0);
  // Load the context from the CLI
  uint64_t context_id = 0;
  bool wait_to_vis = false;
  std::string test_options_filename;
  std::string test_results_filename;
  draco::VisualizerMode viz_mode;
  app.add_option("-c,--context", context_id, "Context ID")->required();
  app.add_option("--wait_for_viz", wait_to_vis, "Wait for visualization")
      ->default_val("false");
  app.add_option("-v, --viz_mode", viz_mode,
                 "Visualizer mode: 0=none, 1=draco, 2=native")
      ->default_val("0");
  CLI11_PARSE(app, argc, argv);
  logging::log()->info("GoToLinearMove with context ID: {}", context_id);
  auto planner = shokunin::MakeDracoPlannerFromContext(context_id, viz_mode);
  // Get the problems
  auto east_to_plate_problem = EastToPlateProblem();
  auto west_to_pick_problem = WestToPickProblem();
  // Start sysconf
  psc::SystemConf start_conf;
  Eigen::VectorXd q_start_east(7), q_start_west(7);
  q_start_east << -0.367, -0.057, 0.355, -2.233, -0.009, 2.240, 0.202;
  q_start_west << 0.381, -0.150, -0.245, -1.819, 0.041, 1.668, 0.144;
  start_conf["panda_east"] = psc::Conf(q_start_east);
  start_conf["panda_west"] = psc::Conf(q_start_west);
  // Solve east first, get the trajectory
  auto east_result = planner.SolvePlan(east_to_plate_problem, "EastToPlate",
                                       std::nullopt, start_conf);
  // Get the system timed trajectory
  if (east_result.is_success() == false) {
    logging::log()->error("Failed to solve east to plate problem: {}",
                          east_result.message());
    return 1;
  }
  auto east_sys_traj = east_result.system_timed_trajectory();
  psc::SystemTimedTrajectory active_traj;
  active_traj["panda_east"] = east_sys_traj.at("panda_east");
  //   double t_global_offset =
  //   east_sys_traj.at("panda_east").global_time_offset(); take time a bit back
  //   to fake the passage of time
  //   active_traj["panda_east"].SetGlobalTimeOffset(t_global_offset - 0.5);
  //   Now solve west to pick, using east arm trajectory as active trajectory
  psc::planner::PlanOptions plan_options;
  auto west_async_result =
      planner.SolvePlan(west_to_pick_problem, "Async_WestToPick", plan_options,
                        start_conf, active_traj);

  //   auto west_async_gen_mm_result = planner.SolveGeneralizedMultiModalPlan(
  //       west_to_pick_problem, start_conf, active_traj);
  //   const auto& west_async_ppt =
  //       drake::trajectories::PathParameterizedTrajectory<double>(
  //           west_async_gen_mm_result.value().trajectory.first,
  //           west_async_gen_mm_result.value().trajectory.second);
  //   auto west_sys_traj = draco::conversions::ToSystemTimedTrajectory(
  //       planner.time_optimal_spliner(), west_async_ppt);
  //   west_sys_traj["panda_west"].SetGlobalTimeOffset(t_global_offset);

  psc::SystemTimedTrajectory combined_traj;
  combined_traj["panda_east"] = east_sys_traj.at("panda_east");
  combined_traj["panda_west"] =
      west_async_result.system_timed_trajectory().at("panda_west");

  Eigen::VectorXd end_of_traj_sysconf_east =
      east_sys_traj.at("panda_east")
          .Value(east_sys_traj.at("panda_east").end_time());
  Eigen::VectorXd end_of_traj_sysconf_west =
      west_async_result.system_timed_trajectory()
          .at("panda_west")
          .Value(west_async_result.system_timed_trajectory()
                     .at("panda_west")
                     .end_time());
  psc::SystemConf end_of_traj_sysconf;
  end_of_traj_sysconf["panda_east"] = end_of_traj_sysconf_east;
  end_of_traj_sysconf["panda_west"] = end_of_traj_sysconf_west;
  // Solve west plan again and use end sysconf as start sysconf to see if we can
  // complete the plan
  auto west_result_from_end =
      planner.SolvePlan(west_to_pick_problem, "WestToPick_FromEnd",
                        plan_options, end_of_traj_sysconf);

  // Log if we have visualizer
  logging::log()->info("Has draco visualizer: {}, has meshcat: {}",
                       planner.has_draco_visualizer(),
                       planner.robot_model().meshcat() != nullptr);
  if (wait_to_vis
      && (planner.has_draco_visualizer()
          || planner.robot_model().meshcat() != nullptr)) {
    //   planner.mutable_draco_visualizer().Add(west_sys_traj, "Gen MM
    //   Problem");
    planner.mutable_draco_visualizer().Add(east_sys_traj, "Active Trajectory");
    planner.mutable_draco_visualizer().Add(
        west_async_result.system_timed_trajectory(), "Async Result");
    planner.mutable_draco_visualizer().Add(combined_traj,
                                           "Combined Trajectory");
    planner.mutable_draco_visualizer().Add(
        west_result_from_end.system_timed_trajectory(), "West Result From End");
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return 1;
}
