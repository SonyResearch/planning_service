#pragma once

#include <Eigen/Dense>

#include "planning_service_client/context_id.h"

namespace planning_service_client {
namespace planner {

template <typename Proto>
class PlannerResultBase : public internal::ProtoBase<Proto> {
 public:
  PlannerResultBase() = default;

  PlannerResultBase(uint32_t id) : id_(id) {}

  void SetId(uint32_t id) {
    id_ = id;
  }

  uint32_t id() const {
    return id_;
  }

 private:
  uint32_t id_ {0};
};

/**
 * @brief Base class for a motion planning problem definition, from which all
 * other problem definitions must inherit.
 *
 */
class PlanningProblemBase {
 public:
  PlanningProblemBase() = default;

  virtual ~PlanningProblemBase() = default;

  void AddToMotionProblemDefinitionProto(
      proto::MotionProblemDefinition* msg) const {
    AddToMotionProblemDefinitionProtoImpl(msg);
  }

  std::unique_ptr<PlanningProblemBase> Clone() const {
    return DoClone();
  }

 private:
  virtual std::unique_ptr<PlanningProblemBase> DoClone() const = 0;

  virtual void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const = 0;
};

}  // namespace planner
}  // namespace planning_service_client
