#include "planning_service_client/planner/motion_problem_definition.h"

namespace planning_service_client {
namespace planner {

MotionProblemDefinition::MotionProblemDefinition(
    const ContextId& context_id, const PlanningProblemBase& problem,
    const std::optional<PlanOptions>& plan_options, const std::string& name)
    : context_id_(context_id),
      problem_(problem.Clone()),
      plan_options_(plan_options),
      name_(name) {}

proto::MotionProblemDefinition MotionProblemDefinition::ToProtoImpl() const {
  proto::MotionProblemDefinition msg;
  msg.mutable_context_id()->set_value(context_id_.value());
  problem_->AddToMotionProblemDefinitionProto(&msg);
  if (plan_options_) {
    msg.mutable_plan_options()->CopyFrom(ToProto(*plan_options_));
  }
  msg.set_name(name_);
  return msg;
}

void MotionProblemDefinition::FromProtoImpl(
    const proto::MotionProblemDefinition& msg) {
  context_id_ = FromProto<ContextId>(msg.context_id());
  problem_ = PlanningProblemRegistry::Create(msg);
  if (msg.has_plan_options()) {
    plan_options_ = FromProto<PlanOptions>(msg.plan_options());
  }
  name_ = msg.name();
}

}  // namespace planner
}  // namespace planning_service_client
