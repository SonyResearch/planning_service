#include <gtest/gtest.h>

#include "planning_service_client/frame_relative_pose.h"

namespace planning_service_client {

TEST(FrameRelativePose, Basics) {
  Eigen::Vector3d translation(1.0, 2.0, 3.0);
  Eigen::Quaterniond quaternion(1.5, 0.5, 0.5, 0.5);
  std::string frame_A = "frame_A";
  std::string frame_B = "frame_B";
  FrameRelativePose frp(frame_A, frame_B, translation, quaternion);
}

TEST(FrameRelativePose, ToProtoFromProto) {
  Eigen::Vector3d translation(1.0, 2.0, 3.0);
  Eigen::Quaterniond quaternion(-0.5, 0.5, 0.5, 0.5);
  std::string frame_A = "frame_A";
  std::string frame_B = "frame_B";
  FrameRelativePose frp(frame_A, frame_B, translation, quaternion);
  proto::FrameRelativePose frp_pb = ToProto(frp);
  FrameRelativePose frp_from_pb = FromProto<FrameRelativePose>(frp_pb);
  EXPECT_EQ(frp.frame_A(), frp_from_pb.frame_A());
  EXPECT_EQ(frp.frame_B(), frp_from_pb.frame_B());
  // Make sure the frame names got copied correctly
  EXPECT_EQ(frame_A, frp_from_pb.frame_A());
  EXPECT_EQ(frame_B, frp_from_pb.frame_B());
  EXPECT_TRUE(translation.isApprox(frp_from_pb.X_AB_translation(), 1e-6));
  EXPECT_TRUE(quaternion.isApprox(frp_from_pb.X_AB_quaternion(), 1e-6));
}
}  // namespace planning_service_client
