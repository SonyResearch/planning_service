#include "planning_service_client/planner/anchor.h"

namespace planning_service_client {
namespace planner {

Anchor::Anchor(const SystemConf& system_conf,
               const std::vector<FrameRelativePose>& wayposes)
    : system_conf_(system_conf), poses_(wayposes) {}

proto::Anchor Anchor::ToProtoImpl() const {
  proto::Anchor msg;
  *msg.mutable_system_conf() = ToProto(system_conf_);
  for (const auto& pose : poses_) {
    *msg.add_poses() = ToProto(pose);
  }
  return msg;
}

void Anchor::FromProtoImpl(const proto::Anchor& msg) {
  system_conf_ = FromProto<SystemConf>(msg.system_conf());
  poses_.clear();
  for (const auto& pose_proto : msg.poses()) {
    poses_.push_back(FromProto<FrameRelativePose>(pose_proto));
  }
}

}  // namespace planner
}  // namespace planning_service_client
