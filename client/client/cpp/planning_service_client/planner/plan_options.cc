
#include "plan_options.h"

namespace planning_service_client {
namespace planner {
proto::TrajectoryUpdateOptions TrajectoryUpdateOptions::ToProtoImpl() const {
  proto::TrajectoryUpdateOptions msg;
  msg.set_merge_point_search_step_size(merge_point_search_step_size);
  msg.set_time_optimal(time_optimal);
  return msg;
}

void TrajectoryUpdateOptions::FromProtoImpl(
    const proto::TrajectoryUpdateOptions& msg) {
  merge_point_search_step_size = msg.merge_point_search_step_size();
  time_optimal = msg.time_optimal();
}

proto::PlanOptions PlanOptions::ToProtoImpl() const {
  proto::PlanOptions msg;
  if (collision_options_) {
    msg.mutable_collision_options()->CopyFrom(ToProto(*collision_options_));
  }
  if (dynamic_limits_) {
    msg.mutable_dynamic_limits()->CopyFrom(ToProto(*dynamic_limits_));
  }
  if (trajectory_update_options_) {
    msg.mutable_trajectory_update_options()->CopyFrom(
        ToProto(*trajectory_update_options_));
  }
  if (global_time_) {
    msg.set_global_time(*global_time_);
  }
  return msg;
}
void PlanOptions::FromProtoImpl(const proto::PlanOptions& msg) {
  if (msg.has_collision_options()) {
    collision_options_ = FromProto<CollisionOptions>(msg.collision_options());
  }
  if (msg.has_dynamic_limits()) {
    dynamic_limits_ = FromProto<DynamicLimits>(msg.dynamic_limits());
  }
  if (msg.has_trajectory_update_options()) {
    trajectory_update_options_ =
        FromProto<TrajectoryUpdateOptions>(msg.trajectory_update_options());
  }
  if (msg.has_global_time()) {
    global_time_ = msg.global_time();
  }
}

}  // namespace planner
}  // namespace planning_service_client
