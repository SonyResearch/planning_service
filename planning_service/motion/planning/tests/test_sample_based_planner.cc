

/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <ctime>
#include <fstream>

#include "planning_service/motion/planning/thunder_planner.h"

using robot_conf_t = Eigen::VectorXd;
using robot_conf_vec_t = std::vector<robot_conf_t>;  // "trajectory"
using system_conf_t = std::map<std::string, robot_conf_t>;

namespace motion {
namespace planning {
namespace ompl {

class ThunderPlannerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    logging::log()->set_level(spdlog::level::debug);
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/wallflower/dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    robot_model_ = std::make_unique<RobotModel>(xml_file, dmd);
    ConstraintsAdapter constraints_adapter;
    constraints_adapter.plan_name = "test";
    constraints_adapter.collision_checker = CollisionCheckerAdapter {};
    // create robot_constraints_
    robot_constraints_ = std::make_unique<RobotConstraints>(
        *robot_model_, constraints_adapter, num_threads_);
    sample_options_.parallel = false;
  }
  const int num_threads_ {
      static_cast<int>(std::thread::hardware_concurrency())};
  std::unique_ptr<drake::RandomGenerator> gen_ {
      std::make_unique<drake::RandomGenerator>(0)};
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  SampleOptions sample_options_;
};

TEST_F(ThunderPlannerTest, AttemptRecallWithThunder) {
  const auto q_start_eigen {
      robot_constraints_->GenerateSamples(gen_.get(), 1, sample_options_)
          .at(0)};
  const auto q_goal_eigen {
      robot_constraints_->GenerateSamples(gen_.get(), 1, sample_options_)
          .at(0)};
  // create thunder parameters
  ThunderParameters thunder_parameters;
  // Set the db_file_path to the context_dir
  thunder_parameters.max_scratch_planning_time = 1.0;
  std::string db_file_path = "test_data/wallflower/no_prm.dat";
  // create sample based planner
  ThunderPlanner thunder_planner(*robot_constraints_, thunder_parameters,
                                 db_file_path);
  // attempt recall with thunder
  auto result = thunder_planner.SolveRecallPlan(q_start_eigen, q_goal_eigen);
  // No db was passed so this should fail
  EXPECT_FALSE(result.has_value());
}

TEST_F(ThunderPlannerTest, PlanWithThunderParallel) {
  const auto q_start_eigen {
      robot_constraints_->GenerateSamples(gen_.get(), 1, sample_options_)[0]};
  const auto q_goal_eigen {
      robot_constraints_->GenerateSamples(gen_.get(), 1, sample_options_)[0]};

  // create thunder parameters
  ThunderParameters thunder_parameters;
  thunder_parameters.max_scratch_planning_time = 2.0;
  // create sample based planner
  ThunderPlanner thunder_planner(*robot_constraints_, thunder_parameters);
  // plan with thunder parallel and save results
  auto result =
      thunder_planner.SolveParallelPlan(q_start_eigen, q_goal_eigen, true);
  // assert results not empty
  EXPECT_TRUE(result.has_value());
  EXPECT_FALSE(result.value().empty());
  // try recall and assert result not empty
  auto result_recall =
      thunder_planner.SolveRecallPlan(q_start_eigen, q_goal_eigen);
  logging::log()->info("result_recall size: {}", result_recall.value().size());
  EXPECT_TRUE(result_recall.has_value());
}

TEST_F(ThunderPlannerTest, SolveProblemsAndSave) {
  const auto n_problems {5};
  const auto start_samples {robot_constraints_->GenerateSamples(
      gen_.get(), n_problems, sample_options_)};
  const auto goal_samples {robot_constraints_->GenerateSamples(
      gen_.get(), n_problems, sample_options_)};
  // create a vector of PlanningProblem objects
  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> planning_problems;
  for (size_t i = 0; i < start_samples.size(); ++i) {
    planning_problems.emplace_back(start_samples[i], goal_samples[i]);
  }

  for (const auto& planning_problem : planning_problems) {
    // create thunder parameters
    ThunderParameters thunder_parameters;
    thunder_parameters.max_scratch_planning_time = 2.0;
    // create sample based planner
    ThunderPlanner thunder_planner(*robot_constraints_, thunder_parameters);
    const auto& [start, goal] {planning_problem};
    auto result = thunder_planner.SolveRecallPlan(start, goal);
    if (!result.has_value()) {
      result = thunder_planner.SolveParallelPlan(start, goal, true);
    }
    EXPECT_TRUE(result.has_value());
  }

  ThunderParameters thunder_load_parameters;
  ThunderPlanner thunder_load(*robot_constraints_, thunder_load_parameters);
  conf_edge_vec_t edges {thunder_load.GetRoadmapEdges()};
  EXPECT_FALSE(edges.empty());
}

TEST_F(ThunderPlannerTest, GetDataBaseEdges) {
  // create thunder parameters
  ThunderParameters thunder_parameters;
  thunder_parameters.max_scratch_planning_time = 2.0;
  // create sample based planner
  ThunderPlanner thunder_planner(*robot_constraints_, thunder_parameters);

  conf_edge_vec_t edges {thunder_planner.GetRoadmapEdges()};
  EXPECT_FALSE(edges.empty());
}
}  // namespace ompl
}  // namespace planning
}  // namespace motion
