#include "cartesian_linear_move_problem.h"

#include "planning_service_client/planner/planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

void CartesianLinearMoveProblem::FromProtoImpl(
    const proto::CartesianLinearMoveProblem& msg) {
  // Convert start anchor from proto
  start_ = FromProto<Anchor>(msg.start());
  allow_async_partial_solutions_ = msg.allow_async_partial_solutions();
  start_transform_poses_.clear();
  for (const auto& pose : msg.start_transform_poses()) {
    start_transform_poses_.push_back(FromProto<FrameRelativePose>(pose));
  }
}

proto::CartesianLinearMoveProblem CartesianLinearMoveProblem::ToProtoImpl()
    const {
  proto::CartesianLinearMoveProblem msg;
  *msg.mutable_start() = ToProto(start_);
  msg.set_allow_async_partial_solutions(allow_async_partial_solutions_);
  for (const auto& pose : start_transform_poses_) {
    *msg.add_start_transform_poses() = ToProto(pose);
  }
  return msg;
}
std::unique_ptr<PlanningProblemBase> CartesianLinearMoveProblem::DoClone()
    const {
  return std::make_unique<CartesianLinearMoveProblem>(*this);
}
void CartesianLinearMoveProblem::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_cartesian_linear_move_problem()->CopyFrom(ToProto(*this));
}
CLIENT_REGISTER_PLANNING_PROBLEM(
    CartesianLinearMoveProblem,
    proto::MotionProblemDefinition::kCartesianLinearMoveProblem,
    cartesian_linear_move_problem);

}  // namespace planner
}  // namespace planning_service_client
