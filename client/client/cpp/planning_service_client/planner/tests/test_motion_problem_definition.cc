#include <gtest/gtest.h>

#include <google/protobuf/util/message_differencer.h>

#include "planning_service_client/planner/motion_problem_definition.h"
#include "planning_service_client/planner/multimodal_plan.h"
#include "planning_service_client/planner/start_to_goal_plan.h"
#include "planning_service_client/planner/update_traj_toward_waypoints.h"
#include "test_utils.h"

namespace planning_service_client {
namespace planner {
namespace pb = google::protobuf;
TEST(PlanOptions, Empty) {
  PlanOptions plan_options;
  EXPECT_TRUE(plan_options.empty());
  DynamicLimits dynamic_limits(1.0, 0.9, 0.8,
                               {{"frame_A", 1.0}, {"frame_B", 2.0}});
  plan_options.set_dynamic_limits(dynamic_limits);
  EXPECT_FALSE(plan_options.empty());
  plan_options = PlanOptions();
  EXPECT_TRUE(plan_options.empty());
  CollisionOptions collision_options;
  plan_options.set_collision_options(collision_options);
  EXPECT_FALSE(plan_options.empty());
}

TEST(PlanOptions, ToProto) {
  DynamicLimits dynamic_limits(1.0, 0.9, 0.8,
                               {{"frame_A", 1.0}, {"frame_B", 2.0}});
  CollisionOptions collision_options;
  collision_options.paddings.emplace_back(CollisionPair("body_A", "body_B"),
                                          0.1);
  PlanOptions plan_options;
  plan_options.set_dynamic_limits(dynamic_limits);
  plan_options.set_collision_options(collision_options);
  auto msg {ToProto(plan_options)};
  EXPECT_TRUE(msg.has_dynamic_limits());
  EXPECT_TRUE(msg.has_collision_options());
  const auto expected_dynamic_limits {ToProto(dynamic_limits)};
  EXPECT_TRUE(pb::util::MessageDifferencer::Equals(msg.dynamic_limits(),
                                                   expected_dynamic_limits));
}
TEST(PlanOptions, FromProto) {
  proto::PlanOptions msg;
  const DynamicLimits dynamic_limits(1.0, 0.9, 0.8,
                                     {{"frame_A", 1.0}, {"frame_B", 2.0}});
  msg.mutable_dynamic_limits()->CopyFrom(ToProto(dynamic_limits));
  PlanOptions plan_options = FromProto<PlanOptions>(msg);
  EXPECT_TRUE(plan_options.maybe_dynamic_limits().has_value());
  EXPECT_FALSE(plan_options.maybe_collision_options().has_value());
  const auto& actual_dynamic_limits =
      plan_options.maybe_dynamic_limits().value();
  EXPECT_EQ(actual_dynamic_limits.safety_factor_velocity,
            dynamic_limits.safety_factor_velocity);
  EXPECT_EQ(actual_dynamic_limits.safety_factor_acceleration,
            dynamic_limits.safety_factor_acceleration);
  EXPECT_EQ(actual_dynamic_limits.safety_factor_torque,
            dynamic_limits.safety_factor_torque);
  EXPECT_EQ(actual_dynamic_limits.cartesian_velocity_limits.size(),
            dynamic_limits.cartesian_velocity_limits.size());
}
TEST(MotionProblemDefinition, StartToGoalProblem) {
  // Test the MotionProblemDefinition class
  ContextId context_id(123, "test");
  auto start_to_goal_problem = test::RandomStartToGoalProblem();
  MotionProblemDefinition dut("test_mpd", context_id, start_to_goal_problem,
                              test::RandomSystemConf(2));
  auto msg = ToProto(dut);
  EXPECT_EQ(dut.name(), msg.name());
  EXPECT_EQ(dut.context_id().value(), context_id.value());
  // Let's get the problem from the proto
  auto dut_back = FromProto<MotionProblemDefinition>(msg);
  EXPECT_EQ(dut_back.context_id().value(), context_id.value());
  EXPECT_TRUE(
      dynamic_cast<StartToGoalProblem*>(dut_back.problem_clone().get()));
  EXPECT_FALSE(dynamic_cast<UpdateTrajTowardWaypointsProblem*>(
      dut_back.problem_clone().get()));
  EXPECT_FALSE(
      dynamic_cast<MultimodalPlanningProblem*>(dut_back.problem_clone().get()));
}

TEST(MotionProblemDefinition, UpdateTrajTowardWaypointsProblem) {
  // Test the MotionProblemDefinition class
  ContextId context_id(123, "test");
  auto update_problem = test::RandomUpdateTrajTowardWaypointsProblem();
  MotionProblemDefinition dut("test_mpd", context_id, update_problem,
                              test::RandomSystemConf(2));
  auto msg = ToProto(dut);
  EXPECT_EQ(dut.name(), msg.name());
  EXPECT_EQ(dut.context_id().value(), context_id.value());
  // Let's get the problem from the proto
  auto dut_back = FromProto<MotionProblemDefinition>(msg);
  EXPECT_EQ(dut_back.context_id().value(), context_id.value());
  EXPECT_TRUE(dynamic_cast<UpdateTrajTowardWaypointsProblem*>(
      dut_back.problem_clone().get()));
  EXPECT_FALSE(
      dynamic_cast<StartToGoalProblem*>(dut_back.problem_clone().get()));
  EXPECT_FALSE(
      dynamic_cast<MultimodalPlanningProblem*>(dut_back.problem_clone().get()));
}

TEST(MotionProblemDefinition, PlanOptions) {
  ContextId context_id(123, "test");
  auto start_to_goal_problem = test::RandomStartToGoalProblem();
  MotionProblemDefinition mpd("test_mpd", context_id, start_to_goal_problem,
                              test::RandomSystemConf(2));
  DynamicLimits dynamic_limits(1.0, 0.9, 0.8,
                               {{"frame_A", 1.0}, {"frame_B", 2.0}});
  CollisionOptions collision_options;
  collision_options.paddings.emplace_back(CollisionPair("body_A", "body_B"),
                                          0.1);
  PlanOptions plan_options;
  plan_options.set_dynamic_limits(dynamic_limits);
  plan_options.set_collision_options(collision_options);
  mpd.set_plan_options(plan_options);
  const auto msg {ToProto(mpd)};
  const auto new_mpd {FromProto<MotionProblemDefinition>(msg)};
  EXPECT_TRUE(new_mpd.maybe_plan_options().has_value());
  const auto& new_plan_options = new_mpd.maybe_plan_options().value();
  EXPECT_TRUE(new_plan_options.maybe_dynamic_limits().has_value());
  EXPECT_TRUE(new_plan_options.maybe_collision_options().has_value());
}

TEST(PlanOptions, GlobalTimeEmpty) {
  PlanOptions plan_options;
  EXPECT_TRUE(plan_options.empty());
  plan_options.set_global_time(1.5);
  EXPECT_FALSE(plan_options.empty());
  EXPECT_TRUE(plan_options.maybe_global_time().has_value());
  EXPECT_DOUBLE_EQ(plan_options.maybe_global_time().value(), 1.5);
  plan_options = PlanOptions();
  EXPECT_TRUE(plan_options.empty());
  EXPECT_FALSE(plan_options.maybe_global_time().has_value());
}

TEST(PlanOptions, GlobalTimeConstructor) {
  DynamicLimits dynamic_limits(1.0, 0.9, 0.8,
                               {{"frame_A", 1.0}, {"frame_B", 2.0}});
  CollisionOptions collision_options;
  TrajectoryUpdateOptions trajectory_update_options;
  trajectory_update_options.merge_point_search_step_size = 0.1;
  trajectory_update_options.time_optimal = true;
  const double global_time = 3.14;
  PlanOptions plan_options(collision_options, dynamic_limits,
                           trajectory_update_options, global_time);
  EXPECT_TRUE(plan_options.maybe_global_time().has_value());
  EXPECT_DOUBLE_EQ(plan_options.maybe_global_time().value(), global_time);
}

TEST(PlanOptions, GlobalTimeProto) {
  // ToProto
  PlanOptions plan_options;
  plan_options.set_global_time(2.71);
  auto msg = ToProto(plan_options);
  EXPECT_TRUE(msg.has_global_time());
  EXPECT_DOUBLE_EQ(msg.global_time(), 2.71);
  // FromProto
  proto::PlanOptions msg2;
  msg2.set_global_time(1.23);
  PlanOptions plan_options2 = FromProto<PlanOptions>(msg2);
  EXPECT_TRUE(plan_options2.maybe_global_time().has_value());
  EXPECT_DOUBLE_EQ(plan_options2.maybe_global_time().value(), 1.23);
  // RoundTrip
  PlanOptions plan_options3;
  plan_options3.set_global_time(9.99);
  auto msg3 = ToProto(plan_options3);
  PlanOptions plan_options3_back = FromProto<PlanOptions>(msg3);
  EXPECT_TRUE(plan_options3_back.maybe_global_time().has_value());
  EXPECT_DOUBLE_EQ(plan_options3_back.maybe_global_time().value(), 9.99);
}

TEST(TrajectoryUpdateOptions, Proto) {
  TrajectoryUpdateOptions options;
  options.merge_point_search_step_size = 0.1;
  options.time_optimal = true;
  auto msg = ToProto(options);
  EXPECT_DOUBLE_EQ(msg.merge_point_search_step_size(),
                   options.merge_point_search_step_size);
  EXPECT_EQ(msg.time_optimal(), options.time_optimal);
  const auto options_back = FromProto<TrajectoryUpdateOptions>(msg);
  EXPECT_DOUBLE_EQ(options_back.merge_point_search_step_size,
                   options.merge_point_search_step_size);
  EXPECT_EQ(options_back.time_optimal, options.time_optimal);
}

TEST(MotionProblemDefinition, StartTrajectory) {
  ContextId context_id(123, "test");
  auto start_to_goal_problem = test::RandomStartToGoalProblem();
  SystemConf start_sysconf = test::RandomSystemConf(2);
  SystemTimedTrajectory active_trajectory {{"a", test::Traj1()}};
  PlanOptions plan_options;
  plan_options.set_trajectory_update_options(TrajectoryUpdateOptions());
  MotionProblemDefinition mpd("test_mpd", context_id, start_to_goal_problem,
                              start_sysconf, active_trajectory, plan_options);
  const auto msg {ToProto(mpd)};
  EXPECT_TRUE(msg.has_start_system_conf());
  EXPECT_TRUE(msg.has_active_trajectory());
  EXPECT_TRUE(msg.plan_options().has_trajectory_update_options());
  const auto new_mpd {FromProto<MotionProblemDefinition>(msg)};
  EXPECT_TRUE(new_mpd.maybe_active_trajectory().has_value());
  const auto& new_active_trajectory = new_mpd.maybe_active_trajectory().value();
  EXPECT_EQ(new_active_trajectory.at("a").path().dim(),
            active_trajectory.at("a").path().dim());
  EXPECT_EQ(new_active_trajectory.at("a").time_scaling().dim(),
            active_trajectory.at("a").time_scaling().dim());
  EXPECT_EQ(new_mpd.start_system_conf().size(), start_sysconf.size());
}

// TODO (Sadra):: Add a test for MultimodalPlanningProblem. Is this needed?

}  // namespace planner
}  // namespace planning_service_client
