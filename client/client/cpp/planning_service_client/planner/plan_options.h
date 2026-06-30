#pragma once

#include "planning_service_client/planner/collision_options.h"
#include "planning_service_client/planner/dynamic_limits.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief Options controlling update behavior when extending an existing
 * trajectory with a new plan.
 *
 */
class TrajectoryUpdateOptions
    : public internal::ProtoBase<proto::TrajectoryUpdateOptions> {
 public:
  TrajectoryUpdateOptions() = default;
  // Step size for finding merge point between old and new trajectory.
  double merge_point_search_step_size;
  // If true, time-optimize the trajectory.
  bool time_optimal;

 private:
  proto::TrajectoryUpdateOptions ToProtoImpl() const override;
  void FromProtoImpl(const proto::TrajectoryUpdateOptions& msg) override;
};

class PlanOptions : public internal::ProtoBase<proto::PlanOptions> {
 public:
  PlanOptions() = default;

  PlanOptions(const CollisionOptions& collision_options,
              const DynamicLimits& dynamic_limits,
              const TrajectoryUpdateOptions& trajectory_update_options)
      : collision_options_(collision_options),
        dynamic_limits_(dynamic_limits),
        trajectory_update_options_(trajectory_update_options) {}

  PlanOptions(const CollisionOptions& collision_options,
              const DynamicLimits& dynamic_limits,
              const TrajectoryUpdateOptions& trajectory_update_options,
              double global_time)
      : collision_options_(collision_options),
        dynamic_limits_(dynamic_limits),
        trajectory_update_options_(trajectory_update_options),
        global_time_(global_time) {}

  void set_collision_options(const CollisionOptions& collision_options) {
    collision_options_ = collision_options;
  }
  void set_dynamic_limits(const DynamicLimits& dynamic_limits) {
    dynamic_limits_ = dynamic_limits;
  }
  void set_trajectory_update_options(
      const TrajectoryUpdateOptions& trajectory_update_options) {
    trajectory_update_options_ = trajectory_update_options;
  }
  void set_global_time(const double global_time) {
    global_time_ = global_time;
  }

  bool empty() const {
    return !collision_options_ && !dynamic_limits_
           && !trajectory_update_options_ && !global_time_;
  }

  const std::optional<CollisionOptions>& maybe_collision_options() const {
    return collision_options_;
  }

  const std::optional<DynamicLimits>& maybe_dynamic_limits() const {
    return dynamic_limits_;
  }

  const std::optional<TrajectoryUpdateOptions>&
  maybe_trajectory_update_options() const {
    return trajectory_update_options_;
  }

  const std::optional<double>& maybe_global_time() const {
    return global_time_;
  }

 private:
  proto::PlanOptions ToProtoImpl() const override;
  void FromProtoImpl(const proto::PlanOptions& msg) override;

  std::optional<CollisionOptions> collision_options_;
  std::optional<DynamicLimits> dynamic_limits_;
  std::optional<TrajectoryUpdateOptions> trajectory_update_options_;
  std::optional<double> global_time_;
};

}  // namespace planner
}  // namespace planning_service_client
