#pragma once

#include <Eigen/Dense>

#include "planning_service_client/planner/anchor.h"
#include "planning_service_client/planner/planner_base.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Planning problem representing a motion from a start Anchor to a
 * goal Anchor, where an Anchor represents some robot state.
 *
 */
class StartToGoalProblem final
    : public internal::ProtoBase<proto::StartToGoalProblem>,
      public PlanningProblemBase {
 public:
  StartToGoalProblem() = default;

  StartToGoalProblem(const Anchor& start, const Anchor& goal,
                     bool replace_invalid_goal = false,
                     bool fast_estimate_solution = false,
                     bool allow_async_partial_solutions = false);

  const Anchor& start() const {
    return start_;
  }

  const Anchor& goal() const {
    return goal_;
  }

  bool replace_invalid_goal() const {
    return replace_invalid_goal_;
  }

  bool fast_estimate_solution() const {
    return fast_estimate_solution_;
  }

  bool allow_async_partial_solutions() const {
    return allow_async_partial_solutions_;
  }

 private:
  proto::StartToGoalProblem ToProtoImpl() const override;

  void FromProtoImpl(const proto::StartToGoalProblem& msg) override;

  std::unique_ptr<PlanningProblemBase> DoClone() const final;

  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const final;

  Anchor start_;
  Anchor goal_;
  bool replace_invalid_goal_;
  bool fast_estimate_solution_;
  bool allow_async_partial_solutions_ = false;
};

}  // namespace planner
}  // namespace planning_service_client
