#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

TEST(TestDracoPlanner, TestFixedOffsetFramesMotion) {
  auto planner = DracoPlanner(test::DualPandas());
  planning_service_client::SystemConf sysconf;
  Eigen::VectorXd q_left(7), q_right(7);
  q_right << -0.4, 0.5, -0.2, -1.8, -1.6, 2.0, 0.0;
  q_left << 1.5, 0.6, 1.1, -2.5, 1.8, 1.6, 0.2;
  sysconf["franka_left"] = q_left;
  sysconf["franka_right"] = q_right;
  planning_service_client::planner::FixedFramesMotion problem(
      sysconf, "franka_right::franka_tool_location",
      "franka_left::franka_tool_location");
  // Get the pose of frame B relative to frame A
  auto draco_sysconf =
      conversions::ClientToDracoSystemConf(problem.system_conf());
  auto X_AB_nominal = planner.CalcRelativePose(draco_sysconf, problem.frame_B(),
                                               problem.frame_A());
  // Solve the problem
  const auto result = planner.SolvePlan(problem, "TestFixedOffsetFramesMotion",
                                        std::nullopt, sysconf);
  // Let's send the result to the visualizer
  EXPECT_TRUE(result.is_success());
  // Sample and check the trajectory
  const auto& systraj = result.system_timed_trajectory();
  auto traj = conversions::ToPathParameterizedTrajectory(
      planner.time_optimal_spliner(), systraj,
      conversions::ToGeneralizedBehavior::kThrowOnMissing);
  const auto& frame_A =
      planner.robot_model().GetScopedFrameByName(problem.frame_A());
  const auto& frame_B =
      planner.robot_model().GetScopedFrameByName(problem.frame_B());
  for (double t = traj.start_time(); t <= traj.end_time(); t += 0.01) {  // 10ms
    auto q = traj.value(t);
    auto X_AB =
        planner.robot_model().CalcRelativeTransform(q, frame_A, frame_B);
    const auto delta = X_AB_nominal.inverse() * X_AB;
    const auto delta_translation = delta.translation().norm();
    const auto delta_rotation =
        drake::math::RotationMatrixd(delta.rotation()).ToAngleAxis().angle();
    EXPECT_LT(delta_translation, 0.002);  // 2mm
    EXPECT_LT(delta_rotation, 0.003);     // 3 mill rad
  }
}

}  // namespace planner
}  // namespace draco
