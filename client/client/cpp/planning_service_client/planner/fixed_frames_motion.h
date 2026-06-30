#pragma once

#include <Eigen/Dense>

#include "planning_service_client/conf.h"
#include "planning_service_client/planner/planner_base.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Planning problem representing a motion from some SystemConf in
 * violation of the active constraints, to some configuration sufficiently clear
 * of those constraints.
 *
 */
class FixedFramesMotion final
    : public internal::ProtoBase<proto::FixedFramesMotionProblem>,
      public PlanningProblemBase {
 public:
  FixedFramesMotion() = default;

  FixedFramesMotion(const SystemConf& system_conf, const std::string& frame_A,
                    const std::string& frame_B);

  const SystemConf& system_conf() const {
    return system_conf_;
  }

  const std::string& frame_A() const {
    return frame_A_;
  }

  const std::string& frame_B() const {
    return frame_B_;
  }

 private:
  proto::FixedFramesMotionProblem ToProtoImpl() const override;

  void FromProtoImpl(const proto::FixedFramesMotionProblem& msg) override;

  std::unique_ptr<PlanningProblemBase> DoClone() const override;

  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const override;

  SystemConf system_conf_;
  std::string frame_A_;
  std::string frame_B_;
};

}  // namespace planner
}  // namespace planning_service_client
