#pragma once

#include <vector>

#include "planning_service_client/conf.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/internal/proto_base.h"
#include "planning_service_client/planner/planner_base.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Planning problem that solves a global inverse kinematics problem
 * @param poses the poses to solve the global IK problem for (can be more than
 * one for multiple end-effectors)
 * @param ik_seed_system_conf the system configuration to use as a seed for IK
 * @param fixed_system_conf the system configuration to use as a fixed
 * configuration for IK
 */
class GlobalIKProblem final
    : public internal::ProtoBase<proto::GlobalIKProblem>,
      public PlanningProblemBase {
 public:
  GlobalIKProblem() = default;
  GlobalIKProblem(
      const std::vector<FrameRelativePose>& poses,                         //
      const std::optional<SystemConf> ik_seed_system_conf = std::nullopt,  //
      const std::optional<SystemConf> fixed_system_conf = std::nullopt);

  const std::vector<FrameRelativePose>& poses() const {
    return poses_;
  }

  const std::optional<SystemConf>& ik_seed_system_conf_opt() const {
    return ik_seed_system_conf_opt_;
  }

  const std::optional<SystemConf>& fixed_system_conf_opt() const {
    return fixed_system_conf_opt_;
  }

 private:
  proto::GlobalIKProblem ToProtoImpl() const override;

  void FromProtoImpl(const proto::GlobalIKProblem& msg) override;

  std::unique_ptr<PlanningProblemBase> DoClone() const override;

  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const final;

  std::vector<FrameRelativePose> poses_;

  std::optional<SystemConf> ik_seed_system_conf_opt_;

  std::optional<SystemConf> fixed_system_conf_opt_;
};

}  // namespace planner
}  // namespace planning_service_client
