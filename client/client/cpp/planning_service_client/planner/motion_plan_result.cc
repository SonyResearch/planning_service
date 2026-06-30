
#include "motion_plan_result.h"

#include <cassert>

namespace planning_service_client {
namespace planner {

void MotionPlanResult::SetResourceResultType(const std::string& resource,
                                             MotionResultType result_type) {
  resource_result_types_[resource] = result_type;
}

MotionPlanResult::MotionResultType MotionPlanResult::GetResourceResultType(
    const std::string& resource) const {
  auto it = resource_result_types_.find(resource);
  if (it != resource_result_types_.end()) {
    return it->second;
  }
  return MotionResultType::kUndefined;
}

proto::MotionPlanResult MotionPlanResult::ToProtoImpl() const {
  CheckInvariants();
  proto::MotionPlanResult msg;
  msg.set_id(id());
  msg.set_is_success(is_success_);
  msg.set_message(message_);
  *msg.mutable_trajectory() = ToProto(system_timed_trajectory_);
  // Map SuccessStatus
  switch (success_status_) {
    case SuccessStatus::kUndefined:
      msg.set_success_status(proto::SUCCESS_STATUS_UNDEFINED);
      break;
    case SuccessStatus::kFull:
      msg.set_success_status(proto::SUCCESS_STATUS_FULL);
      break;
    case SuccessStatus::kStoppedShort:
      msg.set_success_status(proto::SUCCESS_STATUS_STOPPED_SHORT);
      break;
  }
  // Map FailureStatus
  switch (failure_status_) {
    case FailureStatus::kUndefined:
      msg.set_failure_status(proto::FAILURE_STATUS_UNDEFINED);
      break;
    case FailureStatus::kInvalidStart:
      msg.set_failure_status(proto::FAILURE_STATUS_INVALID_START);
      break;
    case FailureStatus::kInvalidGoal:
      msg.set_failure_status(proto::FAILURE_STATUS_INVALID_GOAL);
      break;
    case FailureStatus::kInverseKinematics:
      msg.set_failure_status(proto::FAILURE_STATUS_INVERSE_KINEMATICS);
      break;
    case FailureStatus::kAsyncCollision:
      msg.set_failure_status(proto::FAILURE_STATUS_ASYNC_COLLISION);
      break;
    case FailureStatus::kAsyncActiveResource:
      msg.set_failure_status(proto::FAILURE_STATUS_ASYNC_ACTIVE_RESOURCE);
      break;
    case FailureStatus::kGeneral:
      msg.set_failure_status(proto::FAILURE_STATUS_GENERAL);
      break;
  }
  return msg;
}

void MotionPlanResult::FromProtoImpl(const proto::MotionPlanResult& msg) {
  SetId(msg.id());
  is_success_ = msg.is_success();
  message_ = msg.message();
  system_timed_trajectory_ = FromProto<SystemTimedTrajectory>(msg.trajectory());
  // Map SuccessStatus
  switch (msg.success_status()) {
    case proto::SUCCESS_STATUS_UNDEFINED:
      success_status_ = SuccessStatus::kUndefined;
      break;
    case proto::SUCCESS_STATUS_FULL:
      success_status_ = SuccessStatus::kFull;
      break;
    case proto::SUCCESS_STATUS_STOPPED_SHORT:
      success_status_ = SuccessStatus::kStoppedShort;
      break;
    default:
      success_status_ = SuccessStatus::kUndefined;
      break;
  }
  // Map FailureStatus
  switch (msg.failure_status()) {
    case proto::FAILURE_STATUS_UNDEFINED:
      failure_status_ = FailureStatus::kUndefined;
      break;
    case proto::FAILURE_STATUS_INVALID_START:
      failure_status_ = FailureStatus::kInvalidStart;
      break;
    case proto::FAILURE_STATUS_INVALID_GOAL:
      failure_status_ = FailureStatus::kInvalidGoal;
      break;
    case proto::FAILURE_STATUS_INVERSE_KINEMATICS:
      failure_status_ = FailureStatus::kInverseKinematics;
      break;
    case proto::FAILURE_STATUS_ASYNC_COLLISION:
      failure_status_ = FailureStatus::kAsyncCollision;
      break;
    case proto::FAILURE_STATUS_ASYNC_ACTIVE_RESOURCE:
      failure_status_ = FailureStatus::kAsyncActiveResource;
      break;
    case proto::FAILURE_STATUS_GENERAL:
      failure_status_ = FailureStatus::kGeneral;
      break;
    default:
      failure_status_ = FailureStatus::kUndefined;
      break;
  }
  CheckInvariants();
}

void MotionPlanResult::CheckInvariants() const {
  // We either have a success or failure status
  if (is_success_) {
    assert(failure_status_ == FailureStatus::kUndefined);
    assert(success_status_ != SuccessStatus::kUndefined);
    assert(system_timed_trajectory_.size() > 0);
  } else {
    assert(success_status_ == SuccessStatus::kUndefined);
    assert(failure_status_ != FailureStatus::kUndefined);
    assert(system_timed_trajectory_.size() == 0);
  }
}

}  // namespace planner
}  // namespace planning_service_client
