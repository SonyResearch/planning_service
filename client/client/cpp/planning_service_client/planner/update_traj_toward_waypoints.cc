
#include "update_traj_toward_waypoints.h"

#include "planning_problem_registry.h"

namespace planning_service_client {
namespace planner {

UpdateTrajTowardWaypointsProblem::UpdateTrajTowardWaypointsProblem(
    const SystemTimedTrajectory& current_trajectory,
    const std::vector<SystemConf>& waypoints, double time_now,
    const std::vector<double>& suggested_segment_durations,
    double merge_point_search_step_size, bool time_optimal,
    const std::optional<SystemConf>& waypoint_wiggle_room)
    : current_trajectory_(current_trajectory),
      waypoints_(waypoints),
      time_now_(time_now),
      suggested_segment_durations_(suggested_segment_durations),
      merge_point_search_step_size_(merge_point_search_step_size),
      time_optimal_(time_optimal),
      waypoint_wiggle_room_(waypoint_wiggle_room) {
  CheckInvariants();
}

UpdateTrajTowardWaypointsProblem::UpdateTrajTowardWaypointsProblem(
    const SystemTimedTrajectory& current_trajectory,
    const std::vector<FrameRelativePose>& wayposes, double time_now,
    const std::vector<double>& suggested_segment_durations,
    double merge_point_search_step_size, bool time_optimal,
    std::optional<SystemConf> waypoint_wiggle_room)
    : current_trajectory_(current_trajectory),
      wayposes_(wayposes),
      time_now_(time_now),
      suggested_segment_durations_(suggested_segment_durations),
      merge_point_search_step_size_(merge_point_search_step_size),
      time_optimal_(time_optimal),
      waypoint_wiggle_room_(waypoint_wiggle_room) {
  CheckInvariants();
}

proto::UpdateTrajTowardWaypointsProblem
UpdateTrajTowardWaypointsProblem::ToProtoImpl() const {
  proto::UpdateTrajTowardWaypointsProblem msg;
  msg.set_time_now(time_now_);
  for (const auto& waypoint : waypoints_) {
    auto* waypoint_proto = msg.add_waypoints();
    waypoint_proto->CopyFrom(ToProto(waypoint));
  }
  for (const auto& waypose : wayposes_) {
    auto* waypose_proto = msg.add_wayposes();
    waypose_proto->CopyFrom(ToProto(waypose));
  }
  for (const auto& suggested_time_interval : suggested_segment_durations_) {
    msg.add_suggested_segment_durations(suggested_time_interval);
  }
  msg.set_merge_point_search_step_size(merge_point_search_step_size_);
  auto* current_trajectory_proto = msg.mutable_current_trajectory();
  current_trajectory_proto->CopyFrom(ToProto(current_trajectory_));
  msg.set_time_optimal(time_optimal_);
  if (waypoint_wiggle_room_) {
    *msg.mutable_waypoint_wiggle_room() = ToProto(*waypoint_wiggle_room_);
  }
  return msg;
}

void UpdateTrajTowardWaypointsProblem::FromProtoImpl(
    const proto::UpdateTrajTowardWaypointsProblem& msg) {
  time_now_ = msg.time_now();
  waypoints_.clear();
  wayposes_.clear();
  for (const auto& waypoint_proto : msg.waypoints()) {
    auto waypoint = FromProto<SystemConf>(waypoint_proto);
    waypoints_.push_back(waypoint);
  }
  for (const auto& waypose_proto : msg.wayposes()) {
    auto waypose = FromProto<FrameRelativePose>(waypose_proto);
    wayposes_.push_back(waypose);
  }
  suggested_segment_durations_.clear();
  for (const auto& suggested_time_interval :
       msg.suggested_segment_durations()) {
    suggested_segment_durations_.push_back(suggested_time_interval);
  }
  merge_point_search_step_size_ = msg.merge_point_search_step_size();
  current_trajectory_ =
      FromProto<SystemTimedTrajectory>(msg.current_trajectory());
  time_optimal_ = msg.time_optimal();
  if (msg.has_waypoint_wiggle_room()) {
    waypoint_wiggle_room_ = FromProto<SystemConf>(msg.waypoint_wiggle_room());
  } else {
    waypoint_wiggle_room_ = std::nullopt;
  }
  CheckInvariants();
}

void UpdateTrajTowardWaypointsProblem::AddToMotionProblemDefinitionProtoImpl(
    proto::MotionProblemDefinition* msg) const {
  msg->mutable_update_traj_toward_waypoints_problem()->CopyFrom(ToProto(*this));
}

std::unique_ptr<PlanningProblemBase> UpdateTrajTowardWaypointsProblem::DoClone()
    const {
  return std::make_unique<UpdateTrajTowardWaypointsProblem>(*this);
}

void UpdateTrajTowardWaypointsProblem::CheckInvariants() const {
  CLIENT_THROW_INVALID_ARGUMENT_UNLESS(waypoints_.size() == 0
                                       || wayposes_.size() == 0);
  CLIENT_THROW_INVALID_ARGUMENT_UNLESS(
      suggested_segment_durations_.size() == waypoints_.size() - 1
      || suggested_segment_durations_.size() == wayposes_.size() - 1
      || suggested_segment_durations_.size() == 0);
  CLIENT_THROW_INVALID_ARGUMENT_UNLESS(merge_point_search_step_size_ > 0.0);
  // all values in suggested_segment_durations_ must be positive
  CLIENT_THROW_INVALID_ARGUMENT_UNLESS(
      std::all_of(suggested_segment_durations_.begin(),
                  suggested_segment_durations_.end(), [](double val) {
                    return val > 0.0;
                  }));
}

CLIENT_REGISTER_PLANNING_PROBLEM(
    UpdateTrajTowardWaypointsProblem,
    proto::MotionProblemDefinition::kUpdateTrajTowardWaypointsProblem,
    update_traj_toward_waypoints_problem);

}  // namespace planner
}  // namespace planning_service_client
