/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include "planning_service/comms/plan_service.h"
#include "planning_service/comms/server_wrapper.h"
#include "testing_utils.h"

namespace comms {
namespace test {
class MotionPlanServerTest
    : public ServiceTest<MotionPlannerService, proto::MotionPlanner::Stub> {
  void SetService() {
    mgr_ = std::make_shared<service::planning::MotionPlanManager>(system_,
                                                                  options_);
    service_ = std::make_unique<MotionPlannerService>(mgr_);
  }
  void SetStub() {
    stub_ = proto::MotionPlanner::NewStub(channel_);
  }

 protected:
  std::shared_ptr<service::planning::MotionPlanManager> mgr_;
};

TEST_F(MotionPlanServerTest, CalcRelativePose) {
  proto::CalcRelativePoseRequest req;
  proto::CalcRelativePoseResponse resp;
  grpc::ClientContext context;
  // Default to 5 sec timeout.
  std::chrono::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
  context.set_deadline(deadline);
  req.mutable_context_id()->set_value(kIiwaBoxesHash);
  *req.mutable_system_conf() = psc::ToProto(system_conf_1);
  req.set_frame_a("world");
  req.set_frame_b("iiwa_link_ee");
  const auto status {stub_->CalcRelativePose(&context, req, &resp)};
  EXPECT_TRUE(status.ok()) << "Failed with error: " << status.error_message();
  proto::FrameRelativePose pose_proto;
  pose_proto.set_frame_a("world");
  pose_proto.set_frame_b("iiwa_link_ee");
  pose_proto.mutable_x_ab()->CopyFrom(resp.pose());
  const auto pose {psc::FromProto<psc::FrameRelativePose>(pose_proto)};
  Eigen::Vector3d translation {-0.0807311, -0.393704, 0.577188};
  Eigen::Vector4d rotation {0.851109, 0.233005, -0.445471, 0.151254};
  EXPECT_TRUE(pose.X_AB_translation().isApprox(translation, 1e-3))
      << fmt::format("Computed translation: {} does not match expected: {}",
                     pose.X_AB_translation().transpose(),
                     translation.transpose())
      << std::endl;
  EXPECT_TRUE(pose.X_AB_quaternion().coeffs().isApprox(rotation, 1e-3))
      << fmt::format("Computed rotation: {} does not match expected: {}",
                     pose.X_AB_quaternion().coeffs().transpose(),
                     rotation.transpose())
      << std::endl;
}

TEST_F(MotionPlanServerTest, CheckSatisfied) {
  proto::CheckSatisfiedRequest req;
  proto::CheckSatisfiedResponse resp;
  grpc::ClientContext context;
  // Default to 5 sec timeout.
  std::chrono::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
  context.set_deadline(deadline);
  req.mutable_context_id()->set_value(kIiwaBoxesHash);
  *req.add_system_conf_vec() = psc::ToProto(system_conf_1);
  const auto status {stub_->CheckSatisfied(&context, req, &resp)};
  EXPECT_TRUE(status.ok()) << "Error code! " << status.error_message();
  EXPECT_TRUE(resp.satisfied());
  EXPECT_TRUE(resp.unsatisfied_constraints().empty());
  EXPECT_TRUE(resp.offending_resource_names().empty());
  // Let's do one more with a configuration that violates the position
  // constraint.
  proto::CheckSatisfiedRequest req2;
  proto::CheckSatisfiedResponse resp2;
  grpc::ClientContext context2;
  // Default to 5 sec timeout.
  std::chrono::time_point deadline2 =
      std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
  context2.set_deadline(deadline2);
  req2.mutable_context_id()->set_value(kIiwaBoxesHash);
  auto system_conf_2_copy = system_conf_2;
  const auto q = system_conf_2_copy["iiwa"].q();
  auto q_copy = q;
  q_copy(0) = 10.0;  // This violates the position constraint on iiwa_joint_1,
                     // which is [-2.9671, 2.9671]
  system_conf_2_copy["iiwa"] = q_copy;
  *req2.add_system_conf_vec() = psc::ToProto(system_conf_2_copy);
  const auto status2 {stub_->CheckSatisfied(&context2, req2, &resp2)};
  EXPECT_TRUE(status2.ok()) << "Error code! " << status2.error_message();
  auto result2 = psc::FromProto<psc::CheckSatisfiedResponse>(resp2);
  EXPECT_FALSE(result2.satisfied());
  EXPECT_FALSE(result2.failed_constraint_strings().empty());
  // It should include the name "box"
  EXPECT_TRUE(result2.failed_constraint_strings().front().find("joint_limits")
              != std::string::npos);
  EXPECT_FALSE(result2.offending_resource_names().empty());
  // It should be "iiwa"
  EXPECT_EQ(result2.offending_resource_names().size(), 1);
  EXPECT_EQ(result2.offending_resource_names().front(), "iiwa");
}

TEST_F(MotionPlanServerTest, SetPoseInParentFrame) {
  proto::SetPoseInParentFrameResponse resp;
  grpc::ClientContext context;
  // Default to 5 sec timeout.
  std::chrono::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
  context.set_deadline(deadline);
  auto frp = planning_service_client::FrameRelativePose(
      "iiwa_link_ee", "calibrated_end_effector", Eigen::Vector3d(0.1, 0.2, 0.3),
      Eigen::Quaterniond(0.5, -0.5, 0.5, 0.5));
  proto::SetPoseInParentFrameRequest req;
  req.mutable_frame_relative_pose()->CopyFrom(
      planning_service_client::ToProto(frp));
  const auto status {stub_->SetPoseInParentFrame(&context, req, &resp)};
  EXPECT_TRUE(status.ok()) << "Error code! " << status.error_message();
}

}  // namespace test
}  // namespace comms
