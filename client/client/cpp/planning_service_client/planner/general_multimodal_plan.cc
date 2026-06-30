#include "general_multimodal_plan.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

proto::GeneralizedMultimodalPlanningProblem
GeneralizedMultimodalPlanningProblem::ToProtoImpl() const {
  proto::GeneralizedMultimodalPlanningProblem msg;
  for (const auto& anchor : start_anchors_) {
    *msg.add_start_anchors() = ToProto(anchor);
  }
  for (const auto& anchor : goal_anchors_) {
    *msg.add_goal_anchors() = ToProto(anchor);
  }
  msg.set_fast_estimate_solution(fast_estimate_solution_);
  msg.set_allow_partial_start_anchor_solutions(
      allow_partial_start_anchor_solutions_);
  msg.set_allow_partial_goal_anchor_solutions(
      allow_partial_goal_anchor_solutions_);
  msg.set_allow_async_partial_solutions(allow_async_partial_solutions_);
  msg.set_allow_update_active_arms(allow_update_active_arms_);
  return msg;
}

void GeneralizedMultimodalPlanningProblem::FromProtoImpl(
    const proto::GeneralizedMultimodalPlanningProblem& msg) {
  start_anchors_.clear();
  for (const auto& anchor : msg.start_anchors()) {
    start_anchors_.emplace_back(FromProto<Anchor>(anchor));
  }
  goal_anchors_.clear();
  for (const auto& anchor : msg.goal_anchors()) {
    goal_anchors_.emplace_back(FromProto<Anchor>(anchor));
  }
  fast_estimate_solution_ = msg.fast_estimate_solution();
  allow_partial_start_anchor_solutions_ =
      msg.allow_partial_start_anchor_solutions();
  allow_partial_goal_anchor_solutions_ =
      msg.allow_partial_goal_anchor_solutions();
  allow_async_partial_solutions_ = msg.allow_async_partial_solutions();
  allow_update_active_arms_ = msg.allow_update_active_arms();
}

void GeneralizedMultimodalPlanningProblem::
    AddToMotionProblemDefinitionProtoImpl(
        proto::MotionProblemDefinition* msg) const {
  msg->mutable_generalized_multimodal_planning_problem()->CopyFrom(
      ToProto(*this));
}

std::unique_ptr<PlanningProblemBase>
GeneralizedMultimodalPlanningProblem::DoClone() const {
  return std::make_unique<GeneralizedMultimodalPlanningProblem>(*this);
}

// Registration macro (if needed)
CLIENT_REGISTER_PLANNING_PROBLEM(
    GeneralizedMultimodalPlanningProblem,
    proto::MotionProblemDefinition::kGeneralizedMultimodalPlanningProblem,
    generalized_multimodal_planning_problem);

}  // namespace planner
}  // namespace planning_service_client
