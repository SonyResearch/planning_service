#pragma once

#include <Eigen/Dense>

#include "planning_service_client/planner/planner_base.h"
#include "planning_service_client/trajectories.h"

namespace planning_service_client {
namespace planner {

class MotionPlanResult : public PlannerResultBase<proto::MotionPlanResult> {
 public:
  MotionPlanResult() = default;

  bool is_success() const {
    return is_success_;
  }

  const SystemTimedTrajectory& system_timed_trajectory() const {
    return system_timed_trajectory_;
  }

  const std::string& message() const {
    return message_;
  }

  void SetMessage(const std::string& message) {
    message_ = message;
  }

  void SetSystemTimedTrajectory(
      const SystemTimedTrajectory& system_timed_trajectory) {
    system_timed_trajectory_ = system_timed_trajectory;
  }

  enum class MotionResultType {
    kUndefined = 0,
    kNewAndTargeted = 1,
    kNewNotTargeted = 2,
    kRunningNoUpdate = 3,
    kUpdate = 4,
    kEmpty = 5
  };

  void SetResourceResultType(const std::string& resource,
                             MotionResultType result_type);

  MotionResultType GetResourceResultType(const std::string& resource) const;

  enum class SuccessStatus { kUndefined = 0, kFull = 1, kStoppedShort = 2 };

  enum class FailureStatus {
    kUndefined = 0,
    kInvalidStart = 1,
    kInvalidGoal = 2,
    kInverseKinematics = 3,
    kAsyncCollision = 4,
    kAsyncActiveResource = 5,
    kGeneral = 100,
  };

  void SetSuccessStatus(SuccessStatus status = SuccessStatus::kFull) {
    success_status_ = status;
    is_success_ = (success_status_ != SuccessStatus::kUndefined);
  }

  SuccessStatus success_status() const {
    return success_status_;
  }

  void SetFailureStatus(FailureStatus status = FailureStatus::kGeneral) {
    failure_status_ = status;
    is_success_ = (failure_status_ == FailureStatus::kUndefined);
  }

  FailureStatus failure_status() const {
    return failure_status_;
  }

 private:
  proto::MotionPlanResult ToProtoImpl() const override;

  void FromProtoImpl(const proto::MotionPlanResult& msg) override;

  void CheckInvariants() const;

  bool is_success_ {false};

  SystemTimedTrajectory system_timed_trajectory_;

  // ToDo(@sadra) Convert this to SystemProperty and proto message if needed in
  // future. Right now, this is only for internal use so far.
  std::map<std::string, MotionResultType> resource_result_types_;

  std::string message_ {""};

  SuccessStatus success_status_ {SuccessStatus::kUndefined};

  FailureStatus failure_status_ {FailureStatus::kUndefined};
};

}  // namespace planner
}  // namespace planning_service_client
