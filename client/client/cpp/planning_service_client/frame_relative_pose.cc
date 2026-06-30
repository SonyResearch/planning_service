
#include "frame_relative_pose.h"

#include "planning_service_client/internal/client_throw.h"
namespace planning_service_client {
FrameRelativePose::FrameRelativePose(const std::string& frame_A,
                                     const std::string& frame_B,
                                     const Eigen::Vector3d& X_AB_translation,
                                     const Eigen::Quaterniond& X_AB_quaternion)
    : frame_A_(frame_A),
      frame_B_(frame_B),
      X_AB_translation_(X_AB_translation),
      X_AB_quaternion_(X_AB_quaternion) {}

proto::FrameRelativePose FrameRelativePose::ToProtoImpl() const {
  proto::FrameRelativePose frp_pb;
  // Populate the frame names
  frp_pb.set_frame_a(frame_A_);
  frp_pb.set_frame_b(frame_B_);
  // Populate the translation
  proto::Vector3* translation_pb = frp_pb.mutable_x_ab()->mutable_translation();
  translation_pb->set_x(X_AB_translation_.x());
  translation_pb->set_y(X_AB_translation_.y());
  translation_pb->set_z(X_AB_translation_.z());
  // Populate the rotation (Quaternion)
  proto::Quaternion* rotation_pb = frp_pb.mutable_x_ab()->mutable_quat();
  rotation_pb->set_w(X_AB_quaternion_.w());
  rotation_pb->set_x(X_AB_quaternion_.x());
  rotation_pb->set_y(X_AB_quaternion_.y());
  rotation_pb->set_z(X_AB_quaternion_.z());
  return frp_pb;
}

void FrameRelativePose::FromProtoImpl(const proto::FrameRelativePose& msg) {
  // Populate the frame names
  frame_A_ = msg.frame_a();
  frame_B_ = msg.frame_b();
  // Populate the translation
  const proto::Vector3& translation_pb = msg.x_ab().translation();
  Eigen::Vector3d translation(translation_pb.x(), translation_pb.y(),
                              translation_pb.z());
  // Populate the rotation (Quaternion)
  const proto::Quaternion& rotation_pb = msg.x_ab().quat();
  Eigen::Quaterniond quat(rotation_pb.w(), rotation_pb.x(), rotation_pb.y(),
                          rotation_pb.z());
  X_AB_translation_ = translation;
  X_AB_quaternion_ = quat;
}

proto::FrameRelativePosesVec FrameRelativePosesVec::ToProtoImpl() const {
  proto::FrameRelativePosesVec msg;
  for (const auto& pose : poses_) {
    *msg.add_poses() = ToProto(pose);
  }
  return msg;
}

void FrameRelativePosesVec::FromProtoImpl(
    const proto::FrameRelativePosesVec& msg) {
  poses_.clear();
  for (const auto& pose : msg.poses()) {
    poses_.push_back(FromProto<FrameRelativePose>(pose));
  }
}

}  // namespace planning_service_client
