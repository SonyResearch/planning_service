#include <gtest/gtest.h>

#include "planning_service_client/planner/max_cartesian_acceleration.h"

namespace planning_service_client {
namespace planner {

TEST(MaxCartesianAcceleration, ToProtoFromProto) {
  std::string frame_A = "frame_A";
  std::string frame_B = "frame_B";
  std::string frame_E = "frame_E";
  Eigen::Vector3d delta_xyz(1.0, 2.0, 3.0);
  Eigen::Vector3d delta_rpy(0.1, 0.2, 0.3);
  int num_cycles = 5;

  Twist twist(frame_A, frame_B, frame_E, delta_xyz, delta_rpy);
  MaxCartesianAcceleration problem(twist, num_cycles);

  auto proto = ToProto(problem);
  auto dut = FromProto<MaxCartesianAcceleration>(proto);

  EXPECT_EQ(dut.twist().frame_A(), frame_A);
  EXPECT_EQ(dut.twist().frame_B(), frame_B);
  EXPECT_EQ(dut.twist().frame_E(), frame_E);
  EXPECT_TRUE(dut.twist().delta_xyz().isApprox(delta_xyz));
  EXPECT_TRUE(dut.twist().delta_rpy().isApprox(delta_rpy));
  EXPECT_EQ(dut.num_cycles(), num_cycles);

  // Test default num_cycles
  MaxCartesianAcceleration problem_default(twist);
  EXPECT_EQ(problem_default.num_cycles(), 1);

  auto proto_default = ToProto(problem_default);
  auto dut_default = FromProto<MaxCartesianAcceleration>(proto_default);

  EXPECT_EQ(dut_default.twist().frame_A(), frame_A);
  EXPECT_EQ(dut_default.twist().frame_B(), frame_B);
  EXPECT_EQ(dut_default.twist().frame_E(), frame_E);
  EXPECT_TRUE(dut_default.twist().delta_xyz().isApprox(delta_xyz));
  EXPECT_TRUE(dut_default.twist().delta_rpy().isApprox(delta_rpy));
  EXPECT_EQ(dut_default.num_cycles(), 1);
}

}  // namespace planner
}  // namespace planning_service_client
