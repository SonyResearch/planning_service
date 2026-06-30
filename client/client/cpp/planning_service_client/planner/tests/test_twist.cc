#include <gtest/gtest.h>

#include "planning_service_client/planner/twist.h"

namespace planning_service_client {
namespace planner {

TEST(Twist, ToProtoFromProto) {
  std::string frame_A = "frame_A";
  std::string frame_B = "frame_B";
  std::string frame_E = "frame_E";
  Eigen::Vector3d delta_xyz(1.0, 2.0, 3.0);
  Eigen::Vector3d delta_rpy(0.1, 0.2, 0.3);

  Twist twist(frame_A, frame_B, frame_E, delta_xyz, delta_rpy);

  auto proto = ToProto(twist);
  auto dut = FromProto<Twist>(proto);

  EXPECT_EQ(dut.frame_A(), frame_A);
  EXPECT_EQ(dut.frame_B(), frame_B);
  EXPECT_EQ(dut.frame_E(), frame_E);
  EXPECT_TRUE(dut.delta_xyz().isApprox(delta_xyz));
  EXPECT_TRUE(dut.delta_rpy().isApprox(delta_rpy));
}

}  // namespace planner
}  // namespace planning_service_client
