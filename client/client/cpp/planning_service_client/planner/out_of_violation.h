#pragma once

#include <Eigen/Dense>

#include <optional>

#include "planning_service_client/planner/planner_base.h"
#include "planning_service_client/planner/twist.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Planning problem representing a motion from some SystemConf in
 * violation of the active constraints, to some configuration sufficiently clear
 * of those constraints.
 *
 */
class OutOfViolation final
    : public internal::ProtoBase<proto::OutOfViolationProblem>,
      public PlanningProblemBase {
 public:
  OutOfViolation(double configuration_clearance_norm = 0.05,
                 double influence_distance = 0.05, bool use_gradient = true,
                 std::optional<Twist> twist = std::nullopt);

  double configuration_clearance_norm() const {
    return configuration_clearance_norm_;
  }

  double influence_distance() const {
    return influence_distance_;
  }

  bool use_gradient() const {
    return use_gradient_;
  }

  const std::optional<Twist>& twist() const {
    return twist_;
  }

 private:
  proto::OutOfViolationProblem ToProtoImpl() const override;

  void FromProtoImpl(const proto::OutOfViolationProblem& msg) override;

  std::unique_ptr<PlanningProblemBase> DoClone() const override;

  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const override;

  double configuration_clearance_norm_ = 0.05;
  double influence_distance_ = 0.05;
  bool use_gradient_ = true;
  std::optional<Twist> twist_ = std::nullopt;
};

}  // namespace planner
}  // namespace planning_service_client
