#include "start_to_goal_plan.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

StartToGoalProblem::StartToGoalProblem(const Anchor& start, const Anchor& goal,
                                       const bool replace_invalid_goal,
                                       bool fast_estimate_solution,
                                       bool allow_async_partial_solutions)
    : start_(start),
      goal_(goal),
      replace_invalid_goal_(replace_invalid_goal),
      fast_estimate_solution_(fast_estimate_solution),
      allow_async_partial_solutions_(allow_async_partial_solutions) {}

proto::StartToGoalProblem StartToGoalProblem::ToProtoImpl() const {
  proto::StartToGoalProblem msg;
  *msg.mutable_start() = ToProto(start_);
  *msg.mutable_goal() = ToProto(goal_);
  msg.set_replace_invalid_goal(replace_invalid_goal_);
  msg.set_fast_estimate_solution(fast_estimate_solution_);
  msg.set_allow_async_partial_solutions(allow_async_partial_solutions_);
  return msg;
}

void StartToGoalProblem::FromProtoImpl(const proto::StartToGoalProblem& msg) {
  start_ = FromProto<Anchor>(msg.start());
  goal_ = FromProto<Anchor>(msg.goal());
  replace_invalid_goal_ = msg.replace_invalid_goal();
  fast_estimate_solution_ = msg.fast_estimate_solution();
  allow_async_partial_solutions_ = msg.allow_async_partial_solutions();
}

std::unique_ptr<PlanningProblemBase> StartToGoalProblem::DoClone() const {
  return std::make_unique<StartToGoalProblem>(*this);
}

void StartToGoalProblem::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_start_to_goal_problem()->CopyFrom(ToProto(*this));
}

CLIENT_REGISTER_PLANNING_PROBLEM(
    StartToGoalProblem, proto::MotionProblemDefinition::kStartToGoalProblem,
    start_to_goal_problem);

}  // namespace planner
}  // namespace planning_service_client
