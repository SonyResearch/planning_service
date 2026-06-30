#include "fixed_frames_motion.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

FixedFramesMotion::FixedFramesMotion(const SystemConf& system_conf,
                                     const std::string& frame_A,
                                     const std::string& frame_B)
    : system_conf_(system_conf), frame_A_(frame_A), frame_B_(frame_B) {}

proto::FixedFramesMotionProblem FixedFramesMotion::ToProtoImpl() const {
  proto::FixedFramesMotionProblem msg;
  *msg.mutable_system_conf() = ToProto(system_conf_);
  msg.set_frame_a(frame_A_);
  msg.set_frame_b(frame_B_);
  return msg;
}

void FixedFramesMotion::FromProtoImpl(
    const proto::FixedFramesMotionProblem& msg) {
  system_conf_ = FromProto<SystemConf>(msg.system_conf());
  frame_A_ = msg.frame_a();
  frame_B_ = msg.frame_b();
}

std::unique_ptr<PlanningProblemBase> FixedFramesMotion::DoClone() const {
  return std::make_unique<FixedFramesMotion>(*this);
}

void FixedFramesMotion::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_fixed_frames_motion_problem()->CopyFrom(ToProto(*this));
}

CLIENT_REGISTER_PLANNING_PROBLEM(
    FixedFramesMotion,
    proto::MotionProblemDefinition::kFixedFramesMotionProblem,
    fixed_frames_motion_problem);

}  // namespace planner
}  // namespace planning_service_client
