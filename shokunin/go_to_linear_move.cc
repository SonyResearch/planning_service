#include "shokunin_common.h"

namespace fs = std::filesystem;
namespace psc = planning_service_client;

namespace shokunin {

struct TestOptions {
  Eigen::VectorXd q_east_start = Eigen::VectorXd::Zero(7);
  Eigen::VectorXd q_west_start = Eigen::VectorXd::Zero(7);
  double z_plate = 0.1;
  std::string frame_plate = "plate1__plate_root";
  std::string frame_eef = "panda_east__finger_eef";
  double min_ee_x = -0.15;
  double max_ee_x = 0.15;
  double min_ee_y = -0.15;
  double max_ee_y = 0.15;
  double step_x = 0.07;
  double step_y = 0.07;
  double min_ee_yaw = -M_PI;
  double max_ee_yaw = M_PI;
  double step_yaw = M_PI / 2;
  double z_offset = 0.1;

  // Drake yaml serialization
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(q_east_start));
    a->Visit(DRAKE_NVP(q_west_start));
    a->Visit(DRAKE_NVP(z_plate));
    a->Visit(DRAKE_NVP(frame_plate));
    a->Visit(DRAKE_NVP(frame_eef));
    a->Visit(DRAKE_NVP(min_ee_x));
    a->Visit(DRAKE_NVP(max_ee_x));
    a->Visit(DRAKE_NVP(min_ee_y));
    a->Visit(DRAKE_NVP(max_ee_y));
    a->Visit(DRAKE_NVP(min_ee_yaw));
    a->Visit(DRAKE_NVP(max_ee_yaw));
    a->Visit(DRAKE_NVP(step_x));
    a->Visit(DRAKE_NVP(step_y));
    a->Visit(DRAKE_NVP(step_yaw));
    a->Visit(DRAKE_NVP(z_offset));
  }
};

struct TestResult {
  double x, y, yaw;
  bool success;
  double duration;
  std::string message;
  double configuration_twirl;

  // Drake yaml serialization
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(x));
    a->Visit(DRAKE_NVP(y));
    a->Visit(DRAKE_NVP(yaw));
    a->Visit(DRAKE_NVP(success));
    a->Visit(DRAKE_NVP(duration));
    a->Visit(DRAKE_NVP(message));
    a->Visit(DRAKE_NVP(configuration_twirl));
  }
};

struct TestResults {
  std::vector<TestResult> results;

  // Drake yaml serialization
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(results));
  }
};

psc::planner::MultimodalPlanningProblem CreateGoToPlateMultimodalProblem(
    const psc::SystemConf& start_conf, double ee_x, double ee_y, double ee_yaw,
    const TestOptions& options) {
  psc::planner::Anchor start_anchor, goal_anchor;
  start_anchor = psc::planner::Anchor(start_conf, {});
  // Goal anchor will be with the east arm reaching to a pose
  drake::math::RollPitchYaw<double> rpy(0, M_PI, ee_yaw);
  Eigen::Quaterniond quaternion = rpy.ToQuaternion();
  auto frp_goal = psc::FrameRelativePose(
      options.frame_plate, options.frame_eef,
      Eigen::Vector3d(ee_x, ee_y, options.z_plate), quaternion);
  goal_anchor = psc::planner::Anchor({}, {frp_goal});
  // FrameRelativePose of goal offset
  auto frp_goal_offset =
      psc::FrameRelativePose(options.frame_eef, options.frame_eef,
                             Eigen::Vector3d(0.0, 0.0, options.z_offset),
                             Eigen::Quaterniond(1, 0, 0, 0));
  return psc::planner::MultimodalPlanningProblem(start_anchor, goal_anchor, {},
                                                 {frp_goal_offset});
}

bool RunGoToLinearMove(const draco::planner::DracoPlanner& planner,
                       const std::string& test_options_filename,
                       const std::string& test_results_filename) {
  auto options = drake::yaml::LoadYamlFile<TestOptions>(test_options_filename);
  int num_problems = 0;
  int num_success = 0;
  TestResults all_results;
  psc::SystemConf start_conf;
  start_conf["panda_east"] = psc::Conf(options.q_east_start);
  start_conf["panda_west"] = psc::Conf(options.q_west_start);
  for (double x = options.min_ee_x; x < options.max_ee_x; x += options.step_x) {
    for (double y = options.min_ee_y; y < options.max_ee_y;
         y += options.step_y) {
      for (double yaw = options.min_ee_yaw; yaw < options.max_ee_yaw;
           yaw += options.step_yaw) {
        logging::log()->info("Testing ee_x: {}, ee_y: {}, ee_yaw: {}", x, y,
                             yaw);
        auto problem = shokunin::CreateGoToPlateMultimodalProblem(
            start_conf, x, y, yaw, options);
        num_problems++;
        std::string label =
            fmt::format("x[{:.2f}]_y[{:.2f}]_yaw[{:.2f}]", x, y, yaw);
        auto motion_plan_result =
            planner.SolvePlan(problem, label, std::nullopt, start_conf);
        TestResult result;
        result.x = x;
        result.y = y;
        result.yaw = yaw;
        result.success = motion_plan_result.is_success();
        result.message = motion_plan_result.message();
        if (!motion_plan_result.is_success()) {
          logging::log()->error("Failed to solve the plan: {}",
                                motion_plan_result.message());
        } else {
          const auto& sys_traj = motion_plan_result.system_timed_trajectory();
          result.duration = sys_traj.contains("panda_east")
                                ? sys_traj.at("panda_east").end_time()
                            : sys_traj.contains("panda_west")
                                ? sys_traj.at("panda_west").end_time()
                                : 0.0;
          result.configuration_twirl = CalcTrajectoryVelocityTwirl(sys_traj);
          num_success++;
        }
        all_results.results.push_back(result);
        // Send the result to native visualization
        if (planner.robot_model().meshcat()) {
          planner.robot_model().PublishMeshcatContext();
          auto meshcat = planner.robot_model().meshcat();
          auto cylinder =
              std::make_shared<drake::geometry::Cylinder>(0.002, 0.01);
          const auto& frame_plate =
              planner.robot_model().GetScopedFrameByName(options.frame_plate);
          const auto& plant = planner.robot_model().plant();
          const auto& model_plate_name =
              plant.GetModelInstanceName(frame_plate.model_instance());
          auto meshcat_frame_A_path =
              fmt::format("/drake/visual/{}/{}/problem_{}", model_plate_name,
                          options.frame_plate, num_problems);
          logging::log()->info("Setting meshcat object at path: {}",
                               meshcat_frame_A_path);
          auto color = result.success ? drake::geometry::Rgba(0, 1, 0, 0.8)
                                      : drake::geometry::Rgba(1, 0, 0, 0.8);
          meshcat->SetObject(meshcat_frame_A_path, *cylinder, color);
          drake::math::RigidTransformd X_plate_cylinder(
              Eigen::Quaterniond::Identity(),
              Eigen::Vector3d(x + 0.003 * std::cos(yaw),
                              y + 0.003 * std::sin(yaw), options.z_plate));
          meshcat->SetTransform(meshcat_frame_A_path, X_plate_cylinder);
        }
      }
    }
  }
  logging::log()->critical("RunGoToLinearMove: {}/{} problems solved",
                           num_success, num_problems);
  logging::log()->info("Saving results to: {}", test_results_filename);
  try {
    drake::yaml::SaveYamlFile<TestResults>(test_results_filename, all_results);
    logging::log()->info("Saved test results to: {}", test_results_filename);
  } catch (const std::exception& e) {
    logging::log()->error("Failed to save test results to: {}: {}",
                          test_results_filename, e.what());
  }
  return num_success == num_problems;
}

}  // namespace shokunin

int main(int argc, char** argv) {
  CLI::App app {"GoToLinearMove"};
  logging::create_log("GoToLinearMove");
  app.require_subcommand(0);
  // Load the context from the CLI
  uint64_t context_id = 0;
  bool wait_to_vis = false;
  std::string test_options_filename;
  std::string test_results_filename;
  draco::VisualizerMode viz_mode;
  app.add_option("-c,--context", context_id, "Context ID")->required();
  app.add_option("-o,--options", test_options_filename, "Test options filename")
      ->required();
  app.add_option("-r,--results", test_results_filename, "Test results filename")
      ->required();
  app.add_option("--wait_for_viz", wait_to_vis, "Wait for visualization")
      ->default_val("false");
  app.add_option("-v, --viz_mode", viz_mode,
                 "Visualizer mode: 0=none, 1=draco, 2=native")
      ->default_val("0");
  CLI11_PARSE(app, argc, argv);
  logging::log()->info("GoToLinearMove with context ID: {}", context_id);
  auto planner = shokunin::MakeDracoPlannerFromContext(context_id, viz_mode);
  bool all_success = shokunin::RunGoToLinearMove(planner, test_options_filename,
                                                 test_results_filename);
  // Log if we have visualizer
  logging::log()->info("Has draco visualizer: {}, has meshcat: {}",
                       planner.has_draco_visualizer(),
                       planner.robot_model().meshcat() != nullptr);
  if (wait_to_vis
      && (planner.has_draco_visualizer()
          || planner.robot_model().meshcat() != nullptr)) {
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  return all_success ? 0 : 1;
}
