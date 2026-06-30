#pragma once

#include <Eigen/Dense>

#include "planning_service_client/planner/anchor.h"
#include "planning_service_client/planner/planner_base.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {
/**
 * @brief Planning problem whose solution is a trajectory that traverses the
 * specified wayposes.
 * @param start the start anchor containing the initial poses, seed and fixed
 * partial sysconf
 * @param start_transform_poses the goal pose to plan to in a cartesian linear
 * fashion.
 *
 */
class CartesianLinearMoveProblem
    : public internal::ProtoBase<proto::CartesianLinearMoveProblem>,
      public PlanningProblemBase {
 public:
  CartesianLinearMoveProblem(
      const Anchor& start, std::vector<FrameRelativePose> start_transform_poses,
      bool allow_async_partial_solutions = false)
      : start_(start),
        start_transform_poses_(start_transform_poses),
        allow_async_partial_solutions_(allow_async_partial_solutions) {}

  // default constructor
  CartesianLinearMoveProblem() = default;
  const Anchor& start() const {
    return start_;
  }
  const std::vector<FrameRelativePose>& start_transform_poses() const {
    return start_transform_poses_;
  }

  bool allow_async_partial_solutions() const {
    return allow_async_partial_solutions_;
  }

 private:
  void FromProtoImpl(const proto::CartesianLinearMoveProblem& msg) override;
  proto::CartesianLinearMoveProblem ToProtoImpl() const override;
  std::unique_ptr<PlanningProblemBase> DoClone() const final;
  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const final;

  Anchor start_;
  std::vector<FrameRelativePose> start_transform_poses_;
  bool allow_async_partial_solutions_ = false;
};
}  // namespace planner
}  // namespace planning_service_client
