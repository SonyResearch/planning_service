#pragma once

#include <Eigen/Dense>

#include "planning_service_client/planner/anchor.h"
#include "planning_service_client/planner/planner_base.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Multimodal planning problem definition.
 *
 * This class encapsulates the start and goal states, as well as the
 * transformation poses for the cartsian linear moves used in multimodal
 * planning.
 */
class MultimodalPlanningProblem final
    : public internal::ProtoBase<proto::MultimodalPlanningProblem>,
      public PlanningProblemBase {
 public:
  MultimodalPlanningProblem() = default;

  MultimodalPlanningProblem(
      const Anchor& start, const Anchor& goal,
      std::vector<FrameRelativePose> start_transform_poses,
      std::vector<FrameRelativePose> goal_transform_poses,
      bool allow_async_partial_solutions = false);

  bool allow_async_partial_solutions() const {
    return allow_async_partial_solutions_;
  }

  const Anchor& start() const {
    return start_;
  }

  const Anchor& goal() const {
    return goal_;
  }

  const std::vector<FrameRelativePose>& start_transform_poses() const {
    return start_transform_poses_;
  }

  const std::vector<FrameRelativePose>& goal_transform_poses() const {
    return goal_transform_poses_;
  }

 private:
  proto::MultimodalPlanningProblem ToProtoImpl() const override;

  void FromProtoImpl(const proto::MultimodalPlanningProblem& msg) override;

  std::unique_ptr<PlanningProblemBase> DoClone() const final;

  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const final;

  Anchor start_;
  Anchor goal_;
  std::vector<FrameRelativePose> start_transform_poses_;
  std::vector<FrameRelativePose> goal_transform_poses_;
  bool allow_async_partial_solutions_ = false;
};
}  // namespace planner
}  // namespace planning_service_client
