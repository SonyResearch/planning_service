#include "twist.h"

#include "planning_service_client/internal/eigen_utils.h"

namespace planning_service_client {
namespace planner {

Twist::Twist(const std::string& frame_A, const std::string& frame_B,
             const std::string& frame_E, const Eigen::Vector3d& delta_xyz,
             const Eigen::Vector3d& delta_rpy)
    : frame_A_(frame_A),
      frame_B_(frame_B),
      frame_E_(frame_E),
      delta_xyz_(delta_xyz),
      delta_rpy_(delta_rpy) {}

proto::Twist Twist::ToProtoImpl() const {
  proto::Twist msg;
  msg.set_frame_a(frame_A_);
  msg.set_frame_b(frame_B_);
  msg.set_frame_e(frame_E_);
  *msg.mutable_delta_xyz() = internal::EigenVector3dToProto(delta_xyz_);
  *msg.mutable_delta_rpy() = internal::EigenVector3dToProto(delta_rpy_);
  return msg;
}

void Twist::FromProtoImpl(const proto::Twist& msg) {
  frame_A_ = msg.frame_a();
  frame_B_ = msg.frame_b();
  frame_E_ = msg.frame_e();
  delta_xyz_ = internal::ProtoToEigenVector3d(msg.delta_xyz());
  delta_rpy_ = internal::ProtoToEigenVector3d(msg.delta_rpy());
}

}  // namespace planner
}  // namespace planning_service_client
