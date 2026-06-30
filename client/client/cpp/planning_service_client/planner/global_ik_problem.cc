#include "global_ik_problem.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

GlobalIKProblem::GlobalIKProblem(
    const std::vector<FrameRelativePose>& poses,          //
    const std::optional<SystemConf> ik_seed_system_conf,  //
    const std::optional<SystemConf> fixed_system_conf)    //
    : poses_(poses),
      ik_seed_system_conf_opt_(ik_seed_system_conf),
      fixed_system_conf_opt_(fixed_system_conf) {}

proto::GlobalIKProblem GlobalIKProblem::ToProtoImpl() const {
  proto::GlobalIKProblem msg;
  msg.mutable_poses()->Clear();  // Make sure the vector is empty
  for (const auto& pose : poses_) {
    *msg.add_poses() = ToProto(pose);
  }
  if (ik_seed_system_conf_opt_.has_value()) {
    *msg.mutable_ik_seed_system_conf() =
        ToProto(ik_seed_system_conf_opt_.value());
  }
  if (fixed_system_conf_opt_.has_value()) {
    *msg.mutable_fixed_system_conf() = ToProto(fixed_system_conf_opt_.value());
  }
  return msg;
}

void GlobalIKProblem::FromProtoImpl(const proto::GlobalIKProblem& msg) {
  poses_.clear();  // Make sure the vector is empty
  for (const auto& pose : msg.poses()) {
    poses_.push_back(FromProto<FrameRelativePose>(pose));
  }
  if (msg.has_ik_seed_system_conf()) {
    ik_seed_system_conf_opt_ = FromProto<SystemConf>(msg.ik_seed_system_conf());
  }
  if (msg.has_fixed_system_conf()) {
    fixed_system_conf_opt_ = FromProto<SystemConf>(msg.fixed_system_conf());
  }
}

std::unique_ptr<PlanningProblemBase> GlobalIKProblem::DoClone() const {
  return std::make_unique<GlobalIKProblem>(*this);
}

void GlobalIKProblem::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_global_ik_problem()->CopyFrom(ToProto(*this));
}

CLIENT_REGISTER_PLANNING_PROBLEM(
    GlobalIKProblem,                                   //
    proto::MotionProblemDefinition::kGlobalIkProblem,  //
    global_ik_problem);                                //

}  // namespace planner
}  // namespace planning_service_client
