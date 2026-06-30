#include <gtest/gtest.h>

#include "planning_service_client/planner/update_traj_toward_waypoints.h"
#include "test_utils.h"

namespace planning_service_client {
namespace planner {

TEST(UpdateTrajTowardWaypointsProblem, Basics) {
  SystemTimedTrajectory sys_timed_traj;
  sys_timed_traj["left"] = test::Traj1();
  sys_timed_traj["right"] = test::Traj2();
  // Push a couple of waypoints
  std::vector<SystemConf> waypoints;
  SystemConf waypoint;
  waypoint["left"] = Eigen::MatrixXd::Zero(4, 1);
  waypoint["right"] = Eigen::MatrixXd::Zero(4, 1);
  waypoints.push_back(waypoint);
  waypoint["left"] = Eigen::MatrixXd::Ones(4, 1);
  waypoint["right"] = Eigen::MatrixXd::Ones(4, 1);
  waypoints.push_back(waypoint);
  EXPECT_NO_THROW(UpdateTrajTowardWaypointsProblem(sys_timed_traj, waypoints,
                                                   0.0,
                                                   {
                                                       0.2,
                                                   },
                                                   0.1));
  // negative value in suggested time intervals
  EXPECT_THROW(UpdateTrajTowardWaypointsProblem(sys_timed_traj, waypoints, 0.0,
                                                {-0.1}, 0.1),
               std::invalid_argument);
  // negative value for merge point search step size
  EXPECT_THROW(UpdateTrajTowardWaypointsProblem(sys_timed_traj, waypoints, 0.0,
                                                {0.2}, -0.1),
               std::invalid_argument);
}

TEST(UpdateTrajTowardWaypointsProblem, ToProtoFromProto1) {
  SystemTimedTrajectory sys_timed_traj;
  sys_timed_traj["left"] = test::Traj1();
  sys_timed_traj["right"] = test::Traj2();
  // Push a couple of waypoints
  std::vector<SystemConf> waypoints;
  SystemConf waypoint;
  waypoint["left"] = Eigen::MatrixXd::Zero(4, 1);
  waypoint["right"] = 2.0 * Eigen::MatrixXd::Ones(4, 1);
  waypoints.push_back(waypoint);
  waypoint["left"] = Eigen::MatrixXd::Ones(4, 1);
  waypoint["right"] = 3.0 * Eigen::MatrixXd::Ones(4, 1);
  waypoints.push_back(waypoint);
  // Set the parameters
  double time_now = 0.0;
  std::vector<double> suggested_segment_durations = {0.2};
  double merge_point_search_step_size = 0.1;
  bool time_optimal = true;
  SystemConf waypoint_wiggle_room;
  waypoint_wiggle_room["left"] = Eigen::MatrixXd::Ones(4, 1) * 0.1;
  waypoint_wiggle_room["right"] = Eigen::MatrixXd::Ones(4, 1) * 0.2;
  // Construct the update plan object
  UpdateTrajTowardWaypointsProblem dut(
      sys_timed_traj, waypoints, time_now, suggested_segment_durations,
      merge_point_search_step_size, time_optimal, waypoint_wiggle_room);
  // Convert to proto
  auto msg = ToProto(dut);
  // Convert back from proto
  auto dut_back = FromProto<UpdateTrajTowardWaypointsProblem>(msg);
  // Check that the two objects are the same
  EXPECT_EQ(0.0, dut_back.time_now());
  auto waypoints_back = dut_back.waypoints();
  EXPECT_EQ(2, waypoints_back.size());
  EXPECT_TRUE(
      waypoints_back[0]["left"].q().isApprox(Eigen::MatrixXd::Zero(4, 1)));
  EXPECT_TRUE(waypoints_back[0]["right"].q().isApprox(
      2.0 * Eigen::MatrixXd::Ones(4, 1)));
  EXPECT_TRUE(
      waypoints_back[1]["left"].q().isApprox(Eigen::MatrixXd::Ones(4, 1)));
  EXPECT_TRUE(waypoints_back[1]["right"].q().isApprox(
      3.0 * Eigen::MatrixXd::Ones(4, 1)));
  auto sys_timed_traj_back = dut_back.current_trajectory();
  auto traj_left_back = sys_timed_traj_back["left"];
  auto traj_right_back = sys_timed_traj_back["right"];
  EXPECT_TRUE(
      traj_left_back.path().IsCloseTo(sys_timed_traj["left"].path(), 1e-6));
  EXPECT_TRUE(traj_left_back.time_scaling().IsCloseTo(
      sys_timed_traj["left"].time_scaling(), 1e-6));
  EXPECT_TRUE(
      traj_right_back.path().IsCloseTo(sys_timed_traj["right"].path(), 1e-6));
  EXPECT_TRUE(traj_right_back.time_scaling().IsCloseTo(
      sys_timed_traj["right"].time_scaling(), 1e-6));
  EXPECT_EQ(1, dut_back.suggested_segment_durations().size());
  EXPECT_EQ(0.2, dut_back.suggested_segment_durations()[0]);
  EXPECT_EQ(0.1, dut_back.merge_point_search_step_size());
  EXPECT_TRUE(dut_back.time_optimal());
  EXPECT_TRUE(dut_back.waypoint_wiggle_room().has_value());
  EXPECT_TRUE(dut_back.waypoint_wiggle_room()->has_key("left"));
  EXPECT_TRUE(dut_back.waypoint_wiggle_room()->has_key("right"));
  EXPECT_TRUE(dut_back.waypoint_wiggle_room()->at("left").q().isApprox(
      Eigen::MatrixXd::Ones(4, 1) * 0.1));
  EXPECT_TRUE(dut_back.waypoint_wiggle_room()->at("right").q().isApprox(
      Eigen::MatrixXd::Ones(4, 1) * 0.2));
}

TEST(UpdateTrajTowardWaypointsProblem, ToProtoFromProto2) {
  SystemTimedTrajectory sys_timed_traj;
  sys_timed_traj["left"] = test::Traj1();
  sys_timed_traj["right"] = test::Traj2();
  // Push a couple of waypoints
  std::vector<FrameRelativePose> wayposes;
  wayposes.push_back(FrameRelativePose("A", "B", Eigen::Vector3d(0.0, 0.0, 0.0),
                                       Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0)));
  wayposes.push_back(FrameRelativePose("C", "D", Eigen::Vector3d(1.0, 2.0, 3.0),
                                       Eigen::Quaterniond(0.5, 0.5, 0.5, 0.5)));
  wayposes.push_back(FrameRelativePose("E", "F", Eigen::Vector3d(0.0, 0.0, 0.0),
                                       Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)));
  // Construct the update plan object
  UpdateTrajTowardWaypointsProblem dut(sys_timed_traj, wayposes, 0.0);
  // Convert to proto
  auto msg = ToProto(dut);
  // Convert back from proto
  auto dut_back = FromProto<UpdateTrajTowardWaypointsProblem>(msg);
  // Check that the wayposes back are the same as the wayposes
  auto wayposes_back = dut_back.wayposes();
  EXPECT_EQ(3, wayposes_back.size());
  EXPECT_EQ("A", wayposes_back[0].frame_A());
  EXPECT_EQ("B", wayposes_back[0].frame_B());
  EXPECT_TRUE(wayposes_back[1].X_AB_translation().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_EQ("E", wayposes_back[2].frame_A());
  EXPECT_EQ("F", wayposes_back[2].frame_B());
  EXPECT_TRUE(wayposes_back[2].X_AB_quaternion().isApprox(
      Eigen::Quaterniond(0.0, 0.0, 0.0, 1.0)));
}

}  // namespace planner
}  // namespace planning_service_client
