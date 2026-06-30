#include "out_of_violation.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

OutOfViolation::OutOfViolation(double configuration_clearance_norm,
                               double influence_distance, bool use_gradient,
                               std::optional<Twist> twist)
    : configuration_clearance_norm_(configuration_clearance_norm),
      influence_distance_(influence_distance),
      use_gradient_(use_gradient),
      twist_(use_gradient ? std::nullopt : twist) {}

proto::OutOfViolationProblem OutOfViolation::ToProtoImpl() const {
  proto::OutOfViolationProblem msg;
  msg.set_configuration_clearance_norm(configuration_clearance_norm_);
  msg.set_influence_distance(influence_distance_);
  msg.set_use_gradient(use_gradient_);
  if (twist_.has_value()) {
    *msg.mutable_twist() = ToProto(*twist_);
  }
  return msg;
}

void OutOfViolation::FromProtoImpl(const proto::OutOfViolationProblem& msg) {
  configuration_clearance_norm_ = msg.configuration_clearance_norm();
  influence_distance_ = msg.influence_distance();
  // Default to true when not explicitly set (proto3 bool defaults to false).
  use_gradient_ = msg.has_use_gradient() ? msg.use_gradient() : true;
  if (!use_gradient_ && msg.has_twist()) {
    twist_ = FromProto<Twist>(msg.twist());
  } else {
    twist_ = std::nullopt;
  }
}

std::unique_ptr<PlanningProblemBase> OutOfViolation::DoClone() const {
  return std::make_unique<OutOfViolation>(*this);
}

void OutOfViolation::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_out_of_violation_problem()->CopyFrom(ToProto(*this));
}

CLIENT_REGISTER_PLANNING_PROBLEM(
    OutOfViolation, proto::MotionProblemDefinition::kOutOfViolationProblem,
    out_of_violation_problem);

}  // namespace planner
}  // namespace planning_service_client
