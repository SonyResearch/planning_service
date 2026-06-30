#include <gtest/gtest.h>

#include "planning_service_client/planner/global_ik_problem.h"

namespace planning_service_client {
namespace planner {

TEST(FrameRelativePoses, ToProtoFromProto) {
  FrameRelativePose pose_1("frame_A",                              //
                           "frame_B",                              //
                           Eigen::Vector3d(1.0, 2.0, 3.0),         //
                           Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)  //
  );
  FrameRelativePose pose_2("frame_A",                              //
                           "frame_B",                              //
                           Eigen::Vector3d(4.0, 5.0, 6.0),         //
                           Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0)  //
  );

  SystemConf system_conf_seed;
  system_conf_seed["robot_1"] = Conf(Eigen::VectorXd::Ones(5));
  system_conf_seed["robot_2"] = Conf(Eigen::VectorXd::Ones(5) * 2.0);

  SystemConf system_conf_fixed;
  system_conf_fixed["robot_1"] = Conf(Eigen::VectorXd::Ones(5));

  GlobalIKProblem problem({pose_1,           //
                           pose_2},          //
                          system_conf_seed,  //
                          system_conf_fixed  //
  );

  auto msg = ToProto(problem);

  auto plan_back = FromProto<GlobalIKProblem>(msg);

  EXPECT_TRUE(plan_back.poses().size() == 2);
  EXPECT_TRUE(plan_back.poses()[0].frame_A() == pose_1.frame_A());
  EXPECT_TRUE(plan_back.poses()[0].frame_B() == pose_1.frame_B());
  EXPECT_TRUE(plan_back.poses()[0].X_AB_translation().isApprox(
      pose_1.X_AB_translation()));
  EXPECT_TRUE(plan_back.poses()[0].X_AB_quaternion().coeffs().isApprox(
      pose_1.X_AB_quaternion().coeffs()));

  EXPECT_TRUE(plan_back.ik_seed_system_conf_opt().has_value());
  EXPECT_TRUE(
      plan_back.ik_seed_system_conf_opt().value().at("robot_1").q().isApprox(
          system_conf_seed["robot_1"].q()));
  EXPECT_TRUE(
      plan_back.ik_seed_system_conf_opt().value().at("robot_2").q().isApprox(
          system_conf_seed["robot_2"].q()));
  EXPECT_TRUE(plan_back.fixed_system_conf_opt().has_value());
  EXPECT_TRUE(
      plan_back.fixed_system_conf_opt().value().at("robot_1").q().isApprox(
          system_conf_fixed["robot_1"].q()));
}

}  // namespace planner
}  // namespace planning_service_client
