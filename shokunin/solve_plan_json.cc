#include "planning_service/comms/register_client_planning_problems.h"
#include "planning_service/draco/client_conversions.h"
#include "shokunin/shokunin_common.h"

// need CLI11
#include <CLI/CLI.hpp>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
  CLI::App app {"SolvePlan"};
  logging::create_log("SolvePlan");
  app.require_subcommand(0);
  // Load the context from the CLI
  uint64_t context_id = 0;
  std::string plan_path = "";
  bool update_artifacts = false;
  std::string frame;
  app.add_option("-c,--context", context_id, "Context ID");
  app.add_option("-p,--plan", plan_path, "Plan path")->required();
  app.add_option("-f,--frame", frame, "Frame");
  app.add_flag("--update_artifacts", update_artifacts,
               "Update roadmap and IRIS regions with the solution");
  CLI11_PARSE(app, argc, argv);
  const auto path = fs::path(plan_path);
  // Log if the file exists
  if (!fs::exists(path)) {
    logging::log()->error("File {} does not exist.", plan_path);
    return 0;
  }
  planning_service_client::planner::RegisterAllPlanningProblems();
  auto motion_def = planning_service_client::common::LoadFromJsonFile<
      planning_service_client::planner::MotionProblemDefinition>(path);
  if (context_id == 0) {
    context_id = motion_def.context_id().value();
    if (context_id == 0) {
      logging::log()->error(
          "No context ID provided in the CLI or the plan file. Please provide "
          "a context ID.");
      return 1;
    }
  }
  const auto name {motion_def.name().empty() ? "UnnamedPlan"
                                             : motion_def.name()};
  logging::log()->info("Solving plan {} for context ID {}", name, context_id);
  // Make the planner
  auto planner = shokunin::MakeDracoPlannerFromContext(
      context_id, draco::VisualizerMode::kDraco, true);
  const auto problem_ptr {motion_def.problem_clone()};
  const auto problem_proto_type =
      planning_service_client::ToProto(motion_def).problem_case();
  // TODO:(Sadra): Remove this switch statement and use a factory pattern
  // instead. This is a temporary solution.
  std::string label =
      fmt::format("{}_{}", name, magic_enum::enum_name(problem_proto_type));
  planning_service_client::planner::MotionPlanResult motion_plan_result;
  try {
    motion_plan_result = planner.SolvePlan(
        *problem_ptr, label, motion_def.maybe_plan_options(),
        motion_def.start_system_conf(), motion_def.maybe_active_trajectory());
  } catch (const std::exception& e) {
    logging::log()->error("Exception while solving the plan: {}", e.what());
    return 1;
  }
  // Let's see if we can solve the plan
  if (!motion_plan_result.is_success()) {
    logging::log()->error("Failed to solve the plan: {}",
                          motion_plan_result.message());
  }
  // --- Optionally update roadmap and IRIS regions ---
  if (update_artifacts) {
    try {
      auto& artifact_builder = planner.mutable_artifact_builder();
      auto iris_method = motion::iris::IrisBuilder::IrisMethod::kIrisNp2;
      const auto& robot_model = planner.robot_model();
      if (problem_proto_type
          == proto::MotionProblemDefinition::kStartToGoalProblem) {
        const auto& problem =
            static_cast<const psc::planner::StartToGoalProblem&>(*problem_ptr);
        const auto& q_start_sysconf = problem.start().system_conf();
        const auto& q_goal_sysconf = problem.goal().system_conf();
        Eigen::VectorXd q_start = draco::conversions::ToGeneralizedPosition(
            robot_model, q_start_sysconf);
        Eigen::VectorXd q_goal = draco::conversions::ToGeneralizedPosition(
            robot_model, q_goal_sysconf);

        bool success = artifact_builder.SolveProblemAndBuildRegions(
            q_start, q_goal, iris_method, true);
        if (success) {
          logging::log()->info(
              "Updated roadmap and IRIS regions with the new solution.");
          // Solve again and visualize the new result
          auto updated_result =
              planner.SolvePlan(problem, label, motion_def.maybe_plan_options(),
                                motion_def.start_system_conf(),
                                motion_def.maybe_active_trajectory());
          if (updated_result.is_success()) {
            auto updated_traj = updated_result.system_timed_trajectory();
            if (planner.has_draco_visualizer()) {
              planner.mutable_draco_visualizer().Add(
                  updated_traj, "Updated Solution Trajectory");
              logging::log()->info("Visualized updated solution trajectory.");
            }
          } else {
            logging::log()->error(
                "Failed to solve the plan again after updating artifacts: {}",
                updated_result.message());
          }
        } else {
          logging::log()->error(
              "Failed to update roadmap or IRIS regions with the new "
              "solution.");
        }
      } else if (problem_proto_type
                 == proto::MotionProblemDefinition::
                     kMultimodalPlanningProblem) {
        const auto& problem =
            static_cast<const psc::planner::MultimodalPlanningProblem&>(
                *problem_ptr);
        const auto& q_start_sysconf = problem.start().system_conf();
        Eigen::VectorXd q_start = draco::conversions::ToGeneralizedPosition(
            robot_model, q_start_sysconf);
        auto goal_conf_opt =
            planner.SolveSequentialAnchors({problem.goal()}, q_start);
        if (!goal_conf_opt.has_value()) {
          logging::log()->error("Failed to resolve goal anchor.");
          return 1;
        }
        DRAKE_DEMAND(goal_conf_opt.value().size() == 1);
        const Eigen::VectorXd& q_goal = goal_conf_opt.value().front();
        bool success = artifact_builder.SolveProblemAndBuildRegions(
            q_start, q_goal, iris_method, true);
        if (success) {
          logging::log()->info(
              "Updated roadmap and IRIS regions with the new solution.");
          // Solve again and visualize the new result
          auto updated_result =
              planner.SolvePlan(problem, label, motion_def.maybe_plan_options(),
                                motion_def.start_system_conf(),
                                motion_def.maybe_active_trajectory());
          if (updated_result.is_success()) {
            auto updated_traj = updated_result.system_timed_trajectory();
            if (planner.has_draco_visualizer()) {
              planner.mutable_draco_visualizer().Add(
                  updated_traj, "Updated Solution Trajectory");
              logging::log()->info("Visualized updated solution trajectory.");
            }
          } else {
            logging::log()->error(
                "Failed to solve the plan again after updating artifacts: {}",
                updated_result.message());
          }
        } else {
          logging::log()->error(
              "Failed to update roadmap or IRIS regions with the new "
              "solution.");
        }
      } else {
        logging::log()->warn(
            "--update_artifacts is only supported for StartToGoal and "
            "MultimodalPlanning problems.");
      }
    } catch (const std::exception& e) {
      logging::log()->error("Exception while updating artifacts: {}", e.what());
      return 1;
    }
  }
  // Let's get the pose of the end-effector throughout the trajectory
  if (!frame.empty() && motion_plan_result.is_success()) {
    // open a file to write the end-effector poses
    std::ofstream ofs("/logs/shokunin/end_effector_poses.txt");
    auto sys_traj = motion_plan_result.system_timed_trajectory();
    logging::log()->info("Getting end-effector pose for frame: {}", frame);
    double start_time, end_time;
    for (const auto& [key, traj] : sys_traj) {
      logging::log()->info("System: {}", key);
      start_time = traj.start_time();
      end_time = traj.end_time();
    }
    const double time_step = 0.01;
    for (double t = start_time; t <= end_time; t += time_step) {
      planning_service_client::SystemConf sys_conf;
      for (const auto& [key, traj] : sys_traj) {
        Eigen::VectorXd q_t = traj.Value(t);
        sys_conf[key] = q_t;
      }
      auto q = draco::conversions::ToGeneralizedPosition(planner.robot_model(),
                                                         sys_conf);
      auto X_WF = planner.CalcRelativePose(q, frame);
      logging::log()->info("Time: {:.2f}, Position: [{:.4f}, {:.4f}, {:.4f}]",
                           t, X_WF.translation().x(), X_WF.translation().y(),
                           X_WF.translation().z());
      ofs << "Time: " << t << ", Position: [" << X_WF.translation().x() << ", "
          << X_WF.translation().y() << ", " << X_WF.translation().z() << "]\n";
    }
    ofs.close();
  }
  // Wait until user kills the visualizer
  while (planner.has_draco_visualizer()
         && planner.draco_visualizer().IsRunning()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
