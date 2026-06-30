#pragma once

#include <Eigen/Dense>

#include "planning_service_client/planner/anchor.h"
#include "planning_service_client/planner/planner_base.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

class GeneralizedMultimodalPlanningProblem final
    : public internal::ProtoBase<proto::GeneralizedMultimodalPlanningProblem>,
      public PlanningProblemBase {
 public:
  GeneralizedMultimodalPlanningProblem() = default;

  GeneralizedMultimodalPlanningProblem(
      const std::vector<Anchor>& start_anchors,
      const std::vector<Anchor>& goal_anchors,
      bool fast_estimate_solution = false,
      bool allow_partial_start_anchor_solutions = false,
      bool allow_partial_goal_anchor_solutions = false,
      bool allow_async_partial_solutions = false,
      bool allow_update_active_arms = false)
      : start_anchors_(start_anchors),
        goal_anchors_(goal_anchors),
        fast_estimate_solution_(fast_estimate_solution),
        allow_partial_start_anchor_solutions_(
            allow_partial_start_anchor_solutions),
        allow_partial_goal_anchor_solutions_(
            allow_partial_goal_anchor_solutions),
        allow_async_partial_solutions_(allow_async_partial_solutions),
        allow_update_active_arms_(allow_update_active_arms) {}

  const std::vector<Anchor>& start_anchors() const {
    return start_anchors_;
  }
  const std::vector<Anchor>& goal_anchors() const {
    return goal_anchors_;
  }
  bool fast_estimate_solution() const {
    return fast_estimate_solution_;
  }
  bool allow_partial_start_anchor_solutions() const {
    return allow_partial_start_anchor_solutions_;
  }
  bool allow_partial_goal_anchor_solutions() const {
    return allow_partial_goal_anchor_solutions_;
  }
  bool allow_async_partial_solutions() const {
    return allow_async_partial_solutions_;
  }
  bool allow_update_active_arms() const {
    return allow_update_active_arms_;
  }

 private:
  proto::GeneralizedMultimodalPlanningProblem ToProtoImpl() const override;
  void FromProtoImpl(
      const proto::GeneralizedMultimodalPlanningProblem& msg) override;
  std::unique_ptr<PlanningProblemBase> DoClone() const final;
  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const final;
  std::vector<Anchor> start_anchors_;
  std::vector<Anchor> goal_anchors_;
  bool fast_estimate_solution_;
  bool allow_partial_start_anchor_solutions_ = false;
  bool allow_partial_goal_anchor_solutions_ = false;
  bool allow_async_partial_solutions_ = false;
  bool allow_update_active_arms_ = false;
};
}  // namespace planner
}  // namespace planning_service_client
