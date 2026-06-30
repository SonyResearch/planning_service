#pragma once

#include "planning_service_client/context_id.h"
#include "planning_service_client/planner/plan_options.h"
#include "planning_service_client/planner/planner_base.h"
#include "planning_service_client/planner/planning_problem_registry.h"
#include "planning_service_client/trajectories.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Encapsulating class for planning problem definitions, which also
 * holds common properties, such as the context ID, boundary conditions, and
 * dynamic limits.
 *
 */
class MotionProblemDefinition
    : public internal::ProtoBase<proto::MotionProblemDefinition> {
 public:
  MotionProblemDefinition() = default;

  MotionProblemDefinition(
      const std::string_view name, const ContextId& context_id,
      const PlanningProblemBase& problem, const SystemConf& start_system_conf,
      const std::optional<SystemTimedTrajectory>& active_trajectory =
          std::nullopt,
      const std::optional<PlanOptions>& plan_options = std::nullopt)
      : name_(name),
        context_id_(context_id),
        problem_(problem.Clone()),
        start_system_conf_(start_system_conf),
        active_trajectory_(active_trajectory),
        plan_options_(plan_options) {}
  // Allow move assignment
  MotionProblemDefinition(MotionProblemDefinition&&) = default;
  MotionProblemDefinition& operator=(MotionProblemDefinition&&) = default;

  /* A problem definition must have both an ID and problem data. */
  bool complete() const {
    return context_id_ && problem_ != nullptr;
  }
  void set_name(const std::string_view name) {
    name_ = name.data();
  }
  const std::string& name() const {
    return name_;
  }
  void set_context_id(const ContextId& context_id) {
    context_id_ = context_id;
  }

  void set_problem(const PlanningProblemBase& problem) {
    problem_ = problem.Clone();
  }

  void set_plan_options(const PlanOptions& plan_options) {
    plan_options_ = plan_options;
  }

  void set_start_system_conf(const SystemConf& start_system_conf) {
    start_system_conf_ = start_system_conf;
  }

  void set_active_trajectory(const SystemTimedTrajectory& active_trajectory) {
    active_trajectory_ = active_trajectory;
  }

  const ContextId& context_id() const {
    return context_id_;
  }

  /** Gets the underlying planning problem. */
  std::unique_ptr<PlanningProblemBase> problem_clone() const {
    return problem_->Clone();
  }

  const SystemConf& start_system_conf() const {
    return start_system_conf_;
  }
  const std::optional<SystemTimedTrajectory>& maybe_active_trajectory() const {
    return active_trajectory_;
  }
  const std::optional<PlanOptions>& maybe_plan_options() const {
    return plan_options_;
  }

  bool has_active_trajectory() const {
    return active_trajectory_.has_value();
  }

  bool has_plan_options() const {
    return plan_options_.has_value();
  }

 private:
  proto::MotionProblemDefinition ToProtoImpl() const override {
    proto::MotionProblemDefinition msg;
    msg.set_name(name_);
    msg.mutable_context_id()->set_value(context_id_.value());
    problem_->AddToMotionProblemDefinitionProto(&msg);
    msg.mutable_start_system_conf()->CopyFrom(ToProto(start_system_conf_));
    if (active_trajectory_) {
      msg.mutable_active_trajectory()->CopyFrom(ToProto(*active_trajectory_));
    }
    if (plan_options_) {
      msg.mutable_plan_options()->CopyFrom(ToProto(*plan_options_));
    }
    return msg;
  }

  void FromProtoImpl(const proto::MotionProblemDefinition& msg) override {
    name_ = msg.name();
    context_id_ = FromProto<ContextId>(msg.context_id());
    problem_ = PlanningProblemRegistry::Create(msg);
    start_system_conf_ = FromProto<SystemConf>(msg.start_system_conf());
    if (msg.has_active_trajectory()) {
      active_trajectory_ =
          FromProto<SystemTimedTrajectory>(msg.active_trajectory());
    }
    if (msg.has_plan_options()) {
      plan_options_ = FromProto<PlanOptions>(msg.plan_options());
    }
  }
  std::string name_;
  ContextId context_id_;
  std::unique_ptr<PlanningProblemBase> problem_;
  SystemConf start_system_conf_;
  std::optional<SystemTimedTrajectory> active_trajectory_;
  std::optional<TrajectoryUpdateOptions> active_trajectory_update_options_;
  std::optional<PlanOptions> plan_options_;
};

}  // namespace planner
}  // namespace planning_service_client
