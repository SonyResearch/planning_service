#include <gtest/gtest.h>

#include "planning_service_client/planner/motion_plan_result.h"
#include "test_utils.h"
namespace planning_service_client {
namespace planner {

TEST(MotionPlanResult, MotionResultType) {
  SystemTimedTrajectory sys_timed_traj;
  sys_timed_traj["left"] = test::Traj1();
  sys_timed_traj["right"] = test::Traj2();
  MotionPlanResult mpr;
  mpr.SetId(42);
  mpr.SetSystemTimedTrajectory(sys_timed_traj);
  mpr.SetMessage("All good");
  // Default is new
  EXPECT_EQ(mpr.GetResourceResultType("left"),
            MotionPlanResult::MotionResultType::kUndefined);
  // Set it to something else
  mpr.SetResourceResultType(
      "left", MotionPlanResult::MotionResultType::kRunningNoUpdate);
  EXPECT_EQ(mpr.GetResourceResultType("left"),
            MotionPlanResult::MotionResultType::kRunningNoUpdate);
  // Set success status
  mpr.SetSuccessStatus(MotionPlanResult::SuccessStatus::kFull);
  EXPECT_TRUE(mpr.is_success());
  EXPECT_EQ(mpr.success_status(), MotionPlanResult::SuccessStatus::kFull);
  // Set failure status
  mpr.SetFailureStatus(MotionPlanResult::FailureStatus::kUndefined);
  EXPECT_TRUE(mpr.is_success());
  EXPECT_EQ(mpr.failure_status(), MotionPlanResult::FailureStatus::kUndefined);
  // Check invariants should pass. So we can convert to proto
  EXPECT_NO_THROW(ToProto(mpr));
  // Now set inconsistent statuses
  mpr.SetSystemTimedTrajectory(SystemTimedTrajectory());
  mpr.SetFailureStatus(MotionPlanResult::FailureStatus::kInvalidGoal);
  // Converting to proto should throw
  EXPECT_DEATH(ToProto(mpr), "CheckInvariants");
  // Need to make success undefined to fix
  mpr.SetSuccessStatus(MotionPlanResult::SuccessStatus::kUndefined);
  EXPECT_NO_THROW(ToProto(mpr));
  EXPECT_FALSE(mpr.is_success());
}

TEST(UpdateTrajTowardWaypointsProblem, ToProtoFromProto) {
  SystemTimedTrajectory sys_timed_traj;
  sys_timed_traj["left"] = test::Traj1();
  sys_timed_traj["right"] = test::Traj2();
  // set success and message
  bool is_success = true;
  std::string message = "test is successful";
  uint32_t id = 123;
  MotionPlanResult motion_plan_result;
  motion_plan_result.SetId(id);
  motion_plan_result.SetMessage(message);
  motion_plan_result.SetSystemTimedTrajectory(sys_timed_traj);
  motion_plan_result.SetSuccessStatus(MotionPlanResult::SuccessStatus::kFull);
  EXPECT_TRUE(motion_plan_result.is_success());
  motion_plan_result.SetFailureStatus(
      MotionPlanResult::FailureStatus::kUndefined);
  auto msg = ToProto(motion_plan_result);
  auto motion_plan_result_back = FromProto<MotionPlanResult>(msg);
  EXPECT_EQ(motion_plan_result_back.id(), id);
  EXPECT_EQ(motion_plan_result_back.is_success(), is_success);
}

}  // namespace planner
}  // namespace planning_service_client
