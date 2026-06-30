#include <gtest/gtest.h>

#include "planning_service_client/planner/start_to_goal_plan.h"

namespace planning_service_client {
namespace planner {

TEST(Anchor, ToProtoFromProto) {
  SystemConf system_conf_start;
  system_conf_start["robot_1"] = Conf(Eigen::VectorXd::Ones(5));
  std::vector<FrameRelativePose> wayposes;
  wayposes.push_back(FrameRelativePose("frame_A", "frame_B",
                                       Eigen::Vector3d(1, 2, 3),
                                       Eigen::Quaterniond(1, 0, 0, 0)));
  wayposes.push_back(FrameRelativePose("frame_B", "frame_C",
                                       Eigen::Vector3d(4, 5, 6),
                                       Eigen::Quaterniond(1, 0, 0, 0)));
  Anchor start(system_conf_start, wayposes);
  SystemConf system_conf_goal = system_conf_start;
  system_conf_goal["robot_2"] = Conf(Eigen::VectorXd::Ones(5) * 2.0);
  Anchor goal(system_conf_goal, std::vector<FrameRelativePose>());
  bool replace_invalid_goal = true;
  bool fast_estimate_solution = true;
  bool allow_async_partial_solutions = true;
  StartToGoalProblem plan(start, goal, replace_invalid_goal,
                          fast_estimate_solution,
                          allow_async_partial_solutions);
  auto msg = ToProto(plan);
  auto plan_back = FromProto<StartToGoalProblem>(msg);
  EXPECT_TRUE(plan_back.start().system_conf().at("robot_1").q().isApprox(
      system_conf_start["robot_1"].q()));
  EXPECT_FALSE(plan_back.start().system_conf().has_key("robot_2"));
  EXPECT_TRUE(plan_back.goal().system_conf().at("robot_1").q().isApprox(
      system_conf_goal["robot_1"].q()));
  EXPECT_TRUE(plan_back.goal().system_conf().at("robot_2").q().isApprox(
      system_conf_goal["robot_2"].q()));
  EXPECT_EQ(plan_back.replace_invalid_goal(), replace_invalid_goal);
  EXPECT_EQ(plan_back.fast_estimate_solution(), fast_estimate_solution);
  EXPECT_EQ(plan_back.allow_async_partial_solutions(),
            allow_async_partial_solutions);
  // Test cloning
  auto plan_clone = plan.Clone();
  EXPECT_TRUE(plan_clone != nullptr);
  auto plan_clone_cast = dynamic_cast<StartToGoalProblem*>(plan_clone.get());
  EXPECT_TRUE(plan_clone_cast != nullptr);
  EXPECT_TRUE(plan_clone_cast->start().system_conf().at("robot_1").q().isApprox(
      system_conf_start["robot_1"].q()));
  EXPECT_FALSE(plan_clone_cast->start().system_conf().has_key("robot_2"));
  EXPECT_TRUE(plan_clone_cast->goal().system_conf().at("robot_1").q().isApprox(
      system_conf_goal["robot_1"].q()));
  EXPECT_TRUE(plan_clone_cast->goal().system_conf().at("robot_2").q().isApprox(
      system_conf_goal["robot_2"].q()));
  EXPECT_EQ(plan_clone_cast->replace_invalid_goal(), replace_invalid_goal);
  EXPECT_EQ(plan_clone_cast->fast_estimate_solution(), fast_estimate_solution);
  EXPECT_EQ(plan_clone_cast->allow_async_partial_solutions(),
            allow_async_partial_solutions);
}

TEST(StartToGoalProblem, ToProtoFromProto) {
  SystemConf system_conf_start;
  system_conf_start["robot_1"] = Conf(Eigen::VectorXd::Ones(5));
  system_conf_start["robot_2"] = Conf(Eigen::VectorXd::Ones(5) * 2.0);
  std::vector<FrameRelativePose> wayposes;
  wayposes.push_back(FrameRelativePose("frame_A", "frame_B",
                                       Eigen::Vector3d(1, 2, 3),
                                       Eigen::Quaterniond(1, 0, 0, 0)));
  wayposes.push_back(FrameRelativePose("frame_B", "frame_C",
                                       Eigen::Vector3d(4, 5, 6),
                                       Eigen::Quaterniond(1, 0, 0, 0)));
  Anchor start(system_conf_start, wayposes);
  SystemConf system_conf_goal = system_conf_start;
  system_conf_goal["robot_1"] = Conf(Eigen::VectorXd::Ones(5) * 2.0);
  system_conf_goal["robot_2"] = Conf(Eigen::VectorXd::Ones(5) * 3.0);
  Anchor goal(system_conf_goal, std::vector<FrameRelativePose>());
  bool replace_invalid_goal = true;
  bool fast_estimate_solution = true;
  bool allow_async_partial_solutions = true;

  StartToGoalProblem plan(start, goal, replace_invalid_goal,
                          fast_estimate_solution,
                          allow_async_partial_solutions);

  auto msg = ToProto(plan);

  auto plan_back = FromProto<StartToGoalProblem>(msg);
  EXPECT_TRUE(plan_back.start().system_conf().at("robot_1").q().isApprox(
      system_conf_start["robot_1"].q()));
  EXPECT_TRUE(plan_back.start().system_conf().at("robot_2").q().isApprox(
      system_conf_start["robot_2"].q()));
  EXPECT_TRUE(plan_back.goal().system_conf().at("robot_1").q().isApprox(
      system_conf_goal["robot_1"].q()));
  EXPECT_TRUE(plan_back.goal().system_conf().at("robot_2").q().isApprox(
      system_conf_goal["robot_2"].q()));
  EXPECT_EQ(plan_back.replace_invalid_goal(), replace_invalid_goal);
  EXPECT_EQ(plan_back.fast_estimate_solution(), fast_estimate_solution);
  EXPECT_EQ(plan_back.allow_async_partial_solutions(),
            allow_async_partial_solutions);
}
}  // namespace planner
}  // namespace planning_service_client
