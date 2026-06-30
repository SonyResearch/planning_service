#pragma once

#include <Eigen/Dense>

#include <string>

#include "planning_service_client/internal/proto_base.h"
#include "planning_service_client/planner/planner_base.h"
#include "planning_service_client/planner/twist.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Planning problem for maximum Cartesian acceleration between two
 * frames.
 * @param twist the twist specifying the frames and amplitudes
 * @param num_cycles number of cycles
 */
class MaxCartesianAcceleration final
    : public internal::ProtoBase<proto::MaxCartesianAccelerationProblem>,
      public PlanningProblemBase {
 public:
  MaxCartesianAcceleration() = default;

  MaxCartesianAcceleration(const Twist& twist, int num_cycles = 1);

  const Twist& twist() const {
    return twist_;
  }

  int num_cycles() const {
    return num_cycles_;
  }

 private:
  proto::MaxCartesianAccelerationProblem ToProtoImpl() const override;

  void FromProtoImpl(
      const proto::MaxCartesianAccelerationProblem& msg) override;

  std::unique_ptr<PlanningProblemBase> DoClone() const override;

  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const override;

  Twist twist_;
  int num_cycles_;
};

}  // namespace planner
}  // namespace planning_service_client
