#include "multimodal_plan.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

MultimodalPlanningProblem::MultimodalPlanningProblem(
    const Anchor& start, const Anchor& goal,
    std::vector<FrameRelativePose> start_transform_poses,
    std::vector<FrameRelativePose> goal_transform_poses,
    bool allow_async_partial_solutions)
    : start_(start),
      goal_(goal),
      start_transform_poses_(std::move(start_transform_poses)),
      goal_transform_poses_(std::move(goal_transform_poses)),
      allow_async_partial_solutions_(allow_async_partial_solutions) {}

proto::MultimodalPlanningProblem MultimodalPlanningProblem::ToProtoImpl()
    const {
  proto::MultimodalPlanningProblem msg;
  *msg.mutable_start() = ToProto(start_);
  *msg.mutable_goal() = ToProto(goal_);
  msg.set_allow_async_partial_solutions(allow_async_partial_solutions_);
  for (const auto& pose : start_transform_poses_) {
    *msg.add_start_transform_poses() = ToProto(pose);
  }
  for (const auto& pose : goal_transform_poses_) {
    *msg.add_goal_transform_poses() = ToProto(pose);
  }
  return msg;
}

void MultimodalPlanningProblem::FromProtoImpl(
    const proto::MultimodalPlanningProblem& msg) {
  start_ = FromProto<Anchor>(msg.start());
  goal_ = FromProto<Anchor>(msg.goal());
  allow_async_partial_solutions_ = msg.allow_async_partial_solutions();
  start_transform_poses_.clear();
  for (const auto& pose : msg.start_transform_poses()) {
    start_transform_poses_.emplace_back(FromProto<FrameRelativePose>(pose));
  }
  goal_transform_poses_.clear();
  for (const auto& pose : msg.goal_transform_poses()) {
    goal_transform_poses_.emplace_back(FromProto<FrameRelativePose>(pose));
  }
}

void MultimodalPlanningProblem::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_multimodal_planning_problem()->CopyFrom(ToProto(*this));
}

std::unique_ptr<PlanningProblemBase> MultimodalPlanningProblem::DoClone()
    const {
  return std::make_unique<MultimodalPlanningProblem>(*this);
}

CLIENT_REGISTER_PLANNING_PROBLEM(
    MultimodalPlanningProblem,
    proto::MotionProblemDefinition::kMultimodalPlanningProblem,
    multimodal_planning_problem);

}  // namespace planner
}  // namespace planning_service_client
