#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

namespace psc = planning_service_client;

TEST(TestDracoPlanner, OutOfViolation) {
  // When waypoints are given.
  const auto planner = DracoPlanner(test::Wallflower());
  // Let's put the robot in a position that is in collision.
  Eigen::VectorXd q(2);
  q << 0.1, 0.4;  // This is a position that is in collision.
  EXPECT_FALSE(planner.robot_constraints().CheckSatisfied(q));
  // Let's construct the start sysconf
  psc::SystemConf start_sysconf;
  start_sysconf["robot"] = q;
  // Let's construct the out of violation problem.
  psc::planner::OutOfViolation out_of_violation_problem(
      0.05, 0.05);  // Clearance and influence distance.
  // Let's solve the out of violation problem.
  auto result =
      planner.SolvePlan(out_of_violation_problem, "out_of_violation_test",
                        std::nullopt, start_sysconf);
  // Check that the result is successful.
  EXPECT_TRUE(result.is_success());
  // Check that the result has a valid configuration.
  auto traj = result.system_timed_trajectory();
  EXPECT_TRUE(traj.has_key("robot"));
  auto robot_traj =
      traj.at("robot");  // This is a client path parameterized trajectory.
  auto duration = robot_traj.duration();
  EXPECT_GT(duration, 0.0);
  // verify that the last configuration is not in collision
  auto last_conf = robot_traj.Value(duration);
  EXPECT_TRUE(planner.robot_constraints().CheckSatisfied(last_conf));
  // verify that the first configuration is the same as the start sysconf
  auto first_conf = robot_traj.Value(0.0);
  EXPECT_TRUE(first_conf.isApprox(start_sysconf["robot"].q(), 1e-6));
}

TEST(TestDracoPlanner, OutOfViolationWithAdditionalShapes) {
  auto draco_adapter = test::Wallflower();
  bool DEBUG_MESHCAT = false;
  if (DEBUG_MESHCAT) {
    draco_adapter.robot_meshcat_params = motion::RobotMeshcatParams();
    draco_adapter.robot_meshcat_params->port = 7000;
    draco_adapter.robot_meshcat_params->visual = true;
    draco_adapter.robot_meshcat_params->collision = true;
    draco_adapter.options.visualizer_options.mode = VisualizerMode::kDraco;
  }
  const auto planner = DracoPlanner(draco_adapter);
  // Let's put the robot in a position that is in collision.
  Eigen::VectorXd q(2);
  q << M_PI / 2, 0.35;  // This is a position that is in collision.
  // Let's construct the start sysconf
  psc::SystemConf start_sysconf;
  start_sysconf["robot"] = q;
  // Let's construct the out of violation problem.
  psc::planner::OutOfViolation out_of_violation_problem(
      0.05, 0.05, false);  // Clearance and influence distance.
  // Let's create a collision object: cylinder on top of the plate
  psc::planner::PlanOptions plan_options;
  psc::planner::CollisionOptions collision_options;
  psc::Cylinder cyl(0.25, 0.2);
  psc::ShapeInFrame cyl_in_frame(cyl);
  cyl_in_frame.set_frame("world");
  cyl_in_frame.set_translation(Eigen::Vector3d(0.05, 0.4, 0.3));
  cyl_in_frame.set_quaternion(Eigen::Quaterniond::Identity());
  collision_options.shapes.push_back(cyl_in_frame);
  plan_options.set_collision_options(collision_options);
  // Draco CheckSatisfied should fail with collision options
  EXPECT_FALSE(
      planner.CheckSatisfied({start_sysconf}, collision_options).satisfied());
  auto result =
      planner.SolvePlan(out_of_violation_problem, "out_of_violation_test",
                        plan_options, start_sysconf);
  if (result.is_success()) {
    logging::log()->info("OutOfViolation plan found successfully.");
  } else {
    logging::log()->error("OutOfViolation plan failed: {}", result.message());
  }
  if (DEBUG_MESHCAT) {
    // To see meshcat visualization, uncomment the following line
    while (true) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
  // Check that the result is successful.
  EXPECT_TRUE(result.is_success());
  auto system_traj = result.system_timed_trajectory();
  EXPECT_TRUE(system_traj.has_key("robot"));
  auto robot_traj = system_traj.at(
      "robot");  // This is a client path parameterized trajectory.
  auto duration = robot_traj.duration();
  EXPECT_GT(duration, 0.0);
  // verify that the last configuration is not in collision
  auto last_conf = robot_traj.Value(duration);
  psc::SystemConf last_sysconf;
  last_sysconf["robot"] = last_conf;
  EXPECT_TRUE(
      planner.CheckSatisfied({last_sysconf}, collision_options).satisfied());
}

}  // namespace planner
}  // namespace draco
