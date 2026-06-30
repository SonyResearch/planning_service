#pragma once

#include <Eigen/Dense>

#include "planning_service_client/conf.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/internal/client_throw.h"
#include "planning_service_client/planner/planner_base.h"
#include "planning_service_client/trajectories.h"

namespace planning_service_client {
namespace planner {

/** A planning problem to update a trajectory toward new waypoints.
 * The trajectory is updated to pass through the waypoints at the specified
 * time intervals. The trajectory is updated by searching for a merge point
 * from the current trajectory to a spline that passes through the waypoints.
 */
class UpdateTrajTowardWaypointsProblem final
    : public internal::ProtoBase<proto::UpdateTrajTowardWaypointsProblem>,
      public PlanningProblemBase {
 public:
  UpdateTrajTowardWaypointsProblem() = default;

  /**
   * @brief Waypoint constructor.
   *
   * @param current_trajectory The current trajectory.
   * @param waypoints The waypoints to update the trajectory toward.
   * @param time_now The current time.
   * @param suggested_segment_durations The suggested time intervals between the
   * waypoints. The planner exactly follows the ratio of the suggested time
   * intervals to construct the new spline.
   * @param merge_point_search_step_size The step size for searching the merge
   * point from the current trajectory to the a spline that passes through the
   * waypoints.
   */
  UpdateTrajTowardWaypointsProblem(
      const SystemTimedTrajectory& current_trajectory,
      const std::vector<SystemConf>& waypoints, double time_now,
      const std::vector<double>& suggested_segment_durations = {},
      double merge_point_search_step_size = 0.1, bool time_optimal = false,
      const std::optional<SystemConf>& waypoint_wiggle_room = std::nullopt);

  /**
   * @brief Waypose constructor.
   *
   * @param current_trajectory The current trajectory.
   * @param wayposes The wayposes to update the trajectory toward.
   * @param time_now The current time.
   * @param suggested_segment_durations The suggested time intervals between the
   * waypoints. The planner exactly follows the ratio of the suggested time
   * intervals to construct the new spline.
   * @param merge_point_search_step_size The step size for searching the merge
   * point from the current trajectory to the a spline that passes through the
   * waypoints.
   */
  UpdateTrajTowardWaypointsProblem(
      const SystemTimedTrajectory& current_trajectory,
      const std::vector<FrameRelativePose>& wayposes, double time_now,
      const std::vector<double>& suggested_segment_durations = {},
      double merge_point_search_step_size = 0.1, bool time_optimal = false,
      std::optional<SystemConf> waypoint_wiggle_room = std::nullopt);

  /** Getters for current_trajectory */
  const SystemTimedTrajectory& current_trajectory() const {
    return current_trajectory_;
  }

  /** Returns waypoints. Returns empty vector if wayposes are used */
  const std::vector<SystemConf>& waypoints() const {
    return waypoints_;
  }

  /** Returns wayposes. Returns empty vector if waypoints are used */
  const std::vector<FrameRelativePose>& wayposes() const {
    return wayposes_;
  }

  double time_now() const {
    return time_now_;
  }

  const std::vector<double>& suggested_segment_durations() const {
    return suggested_segment_durations_;
  }

  double merge_point_search_step_size() const {
    return merge_point_search_step_size_;
  }

  bool time_optimal() const {
    return time_optimal_;
  }

  const std::optional<SystemConf>& waypoint_wiggle_room() const {
    return waypoint_wiggle_room_;
  }

 private:
  proto::UpdateTrajTowardWaypointsProblem ToProtoImpl() const override;

  void FromProtoImpl(
      const proto::UpdateTrajTowardWaypointsProblem& msg) override;

  std::unique_ptr<PlanningProblemBase> DoClone() const final;

  void AddToMotionProblemDefinitionProtoImpl(
      proto::MotionProblemDefinition* msg) const final;

  void CheckInvariants() const;

  SystemTimedTrajectory current_trajectory_;
  std::vector<SystemConf> waypoints_;
  std::vector<FrameRelativePose> wayposes_;
  double time_now_;
  std::vector<double> suggested_segment_durations_;
  double merge_point_search_step_size_;
  bool time_optimal_;
  std::optional<SystemConf> waypoint_wiggle_room_;
};

}  // namespace planner
}  // namespace planning_service_client
