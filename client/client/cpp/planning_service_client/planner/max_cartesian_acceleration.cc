#include "max_cartesian_acceleration.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

MaxCartesianAcceleration::MaxCartesianAcceleration(const Twist& twist,
                                                   int num_cycles)
    : twist_(twist), num_cycles_(num_cycles) {}

proto::MaxCartesianAccelerationProblem MaxCartesianAcceleration::ToProtoImpl()
    const {
  proto::MaxCartesianAccelerationProblem msg;
  *msg.mutable_twist() = ToProto(twist_);
  msg.set_num_cycles(num_cycles_);
  return msg;
}

void MaxCartesianAcceleration::FromProtoImpl(
    const proto::MaxCartesianAccelerationProblem& msg) {
  twist_ = FromProto<Twist>(msg.twist());
  num_cycles_ = msg.num_cycles();
}

std::unique_ptr<PlanningProblemBase> MaxCartesianAcceleration::DoClone() const {
  return std::make_unique<MaxCartesianAcceleration>(*this);
}

void MaxCartesianAcceleration::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_max_cartesian_acceleration_problem()->CopyFrom(ToProto(*this));
}

CLIENT_REGISTER_PLANNING_PROBLEM(
    MaxCartesianAcceleration,
    proto::MotionProblemDefinition::kMaxCartesianAccelerationProblem,
    max_cartesian_acceleration_problem);

}  // namespace planner
}  // namespace planning_service_client
