#include <gtest/gtest.h>

#include "planning_service/common/string_utils.h"
#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

using motion::system_conf_t;

TEST(TestDracoPlanner, TestGeneralizedMultimodalPlan) {
  auto adapter = test::Franka();
  bool DEBUG_MESHCAT =
      false;  // Set to true to visualize in Meshcat for debugging
  if (DEBUG_MESHCAT) {
    adapter.robot_meshcat_params = motion::RobotMeshcatParams();
    adapter.robot_meshcat_params->port = 7000;
    adapter.robot_meshcat_params->visual = true;
    adapter.robot_meshcat_params->collision = true;
    adapter.options.visualizer_options.mode = VisualizerMode::kDraco;
  }
  const auto planner = DracoPlanner(adapter);
  std::string frame_A = "world";
  std::string frame_B = "franka_tool_location";
  Eigen::Vector3d amplitude_rpy {0.0, 0.0, 0.0};    // radians
  Eigen::Vector3d amplitude_xyz {0.00, 0.0, 0.05};  // meters
  int num_cycles = 10;
  auto twist = planning_service_client::planner::Twist(
      frame_A, frame_B, frame_A, amplitude_xyz, amplitude_rpy);
  auto problem = planning_service_client::planner::MaxCartesianAcceleration(
      twist, num_cycles);
  // Define start system configuration
  planning_service_client::SystemConf system_conf_start;
  Eigen::VectorXd q(7);
  q << -0.4, 0.50, -0.3, -1.8, -1.6, 2.0, -0.0;
  system_conf_start["franka"] = planning_service_client::Conf(q);
  const auto result =
      planner.SolvePlan(problem, "shaking", std::nullopt, system_conf_start);
  EXPECT_TRUE(result.is_success());
  auto traj = result.system_timed_trajectory().at("franka");
  EXPECT_TRUE(traj.Value(traj.start_time()).isApprox(q));
  // To see meshcat visualization, uncomment the following line
  while (DEBUG_MESHCAT) {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

}  // namespace planner
}  // namespace draco
