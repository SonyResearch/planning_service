

/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/common/file_utils.h"
#include "planning_service/motion/planning/artifact_builder.h"

namespace fs = std::filesystem;
namespace motion {
namespace planning {

class ArtifactBuilderTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const std::string package_xml_path {
        "planning_service/test_data/package.xml"};
    const fs::path context_dir_parent {"planning_service/test_data"};
    auto context_dir = context_dir_parent / system_name_;
    auto temp_context_dir_ = common::utils::temp_dir() / system_name_;
    fs::create_directories(temp_context_dir_);
    fs::copy(context_dir, temp_context_dir_,
             fs::copy_options::recursive | fs::copy_options::update_existing);
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            temp_context_dir_ / "dmd.yaml")};
    robot_model_ = std::make_unique<motion::RobotModel>(package_xml_path, dmd);
    ConstraintsAdapter constraints_adapter {};
    constraints_adapter.plan_name = "test";
    constraints_adapter.collision_checker = CollisionCheckerAdapter {};
    const auto n_threads {std::thread::hardware_concurrency()};
    robot_constraints_ = std::make_unique<RobotConstraints>(
        *robot_model_, constraints_adapter, n_threads);
    ompl::ThunderParameters thunder_parameters {};
    auto db_file_path = temp_context_dir_ / "thunder_prm.dat";
    auto thunder_planner = std::make_unique<ompl::ThunderPlanner>(
        *robot_constraints_, thunder_parameters, db_file_path.string());
    const std::string regions_file =
        (temp_context_dir_ / "iris_regions.yaml").string();
    auto iris_builder_options {
        drake::yaml::LoadYamlFile<iris::IrisBuilderOptions>(
            "planning_service/test_data/iris_builder_options.yaml")};
    artifact_builder_ = std::make_unique<ArtifactBuilder>(
        std::move(thunder_planner), iris_builder_options, regions_file,
        temp_context_dir_, temp_context_dir_ / "problems");
  }

  void TearDown() override {
    fs::remove_all(temp_context_dir_.parent_path());
  }

  fs::path temp_context_dir_;
  std::string system_name_ {"NONE"};
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  std::unique_ptr<ArtifactBuilder> artifact_builder_;
};

class ArtifactBuilderDualPandaTest : public ArtifactBuilderTest {
 protected:
  void SetUp() override {
    system_name_ = "dual_pandas";
    ArtifactBuilderTest::SetUp();
  }
};

class ArtifactBuilder2DPrismaticTest : public ArtifactBuilderTest {
 protected:
  void SetUp() override {
    system_name_ = "2d_prismatic_robot";
    ArtifactBuilderTest::SetUp();
  }
};

class ArtifactBuilderWallflowerTest : public ArtifactBuilderTest {
 protected:
  void SetUp() override {
    system_name_ = "wallflower";
    ArtifactBuilderTest::SetUp();
  }
};

TEST_F(ArtifactBuilder2DPrismaticTest, Ctor) {
  // Problem directories should have been created
  EXPECT_TRUE(fs::is_directory(artifact_builder_->config_problems_dir()));
}

TEST_F(ArtifactBuilder2DPrismaticTest, AddConfsToRoadmap) {
  const auto num_vertices_before = artifact_builder_->GetNumRoadmapVertices();
  Eigen::VectorXd new_conf(2);
  new_conf << 0.75, 0.5;
  artifact_builder_->AddConfsToRoadmap({new_conf});
  const auto num_vertices_after = artifact_builder_->GetNumRoadmapVertices();
  EXPECT_EQ(num_vertices_after, num_vertices_before + 1)
      << "num_vertices_after: " << num_vertices_after
      << ", num_vertices_before: " << num_vertices_before;
  // Check that the last vertex is the one we added
  const auto& added_conf = artifact_builder_->thunder_planner().GetVertexConf(
      num_vertices_after - 1);
  logging::log()->info("Added conf: {}", added_conf.transpose());
  EXPECT_TRUE(added_conf.isApprox(new_conf));
  // Adding the same conf again should not increase the number of vertices
  artifact_builder_->AddConfsToRoadmap({new_conf});
  const auto num_vertices_after_adding_same =
      artifact_builder_->GetNumRoadmapVertices();
  logging::log()->info("Number of vertices after adding same conf: {}",
                       num_vertices_after_adding_same);
  EXPECT_EQ(num_vertices_after_adding_same, num_vertices_after);
}

TEST_F(ArtifactBuilderWallflowerTest, SolveProblemAndBuildRegions_CoversPath) {
  // Example start and goal for the 2d prismatic robot
  Eigen::VectorXd q_start(2);
  q_start << -2.5, 0.35;
  Eigen::VectorXd q_goal(2);
  q_goal << 2.5, 0.25;

  // Populate the roadmap using parallel planning
  auto parallel_solution_opt =
      artifact_builder_->GetSampleBasedSolution(q_start, q_goal, false, true);
  ASSERT_TRUE(parallel_solution_opt.has_value());

  // Get the recall solution path
  auto recall_solution_opt =
      artifact_builder_->GetSampleBasedSolution(q_start, q_goal, true, false);
  ASSERT_TRUE(recall_solution_opt.has_value());

  // Solve the problem and build regions
  bool success = artifact_builder_->SolveProblemAndBuildRegions(
      q_start, q_goal, iris::IrisBuilder::IrisMethod::kIrisNp2, false);
  EXPECT_TRUE(success);

  // Check that every configuration in the path is covered by at least one
  // region
  const auto& regions =
      artifact_builder_->iris_builder().adapter().regions_vec();
  ASSERT_FALSE(regions.empty());

  // Make sure all the recall_solution path edges are covered by regions
  for (size_t i = 0; i < recall_solution_opt.value().size() - 1; ++i) {
    const auto& q1 = recall_solution_opt.value()[i];
    const auto& q2 = recall_solution_opt.value()[i + 1];
    bool edge_covered =
        artifact_builder_->iris_builder().inspector().IsEdgeCoveredByRegions(
            {q1, q2});
    EXPECT_TRUE(edge_covered);
  }
}

TEST_F(ArtifactBuilderDualPandaTest, Ctor) {
  // Problem directories should have been created
  EXPECT_TRUE(fs::is_directory(artifact_builder_->config_problems_dir()));
}

TEST(ArtifactBuilder, EmptyCtor) {
  // Should throw because the thunder planner is not valid
  const std::string package_xml_path {"planning_service/test_data/package.xml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          "planning_service/test_data/2d_prismatic_robot/dmd.yaml")};
  auto robot_model =
      std::make_unique<motion::RobotModel>(package_xml_path, dmd);
  ConstraintsAdapter constraints_adapter {};
  constraints_adapter.plan_name = "test";
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  auto robot_constraints =
      std::make_unique<RobotConstraints>(*robot_model, constraints_adapter, 1);
  ompl::ThunderParameters thunder_parameters {};
  std::string db_file_path =
      "planning_service/test_data/2d_prismatic_robot/thunder_prm.dat";
  auto thunder_planner = std::make_unique<ompl::ThunderPlanner>(
      *robot_constraints, thunder_parameters, db_file_path);
  const std::string regions_file =
      "planning_service/test_data/2d_prismatic_robot/iris_regions.yaml";
  auto iris_builder_options {
      drake::yaml::LoadYamlFile<iris::IrisBuilderOptions>(
          "planning_service/test_data/iris_builder_options.yaml")};
  fs::path empty;
  EXPECT_THROW(ArtifactBuilder(std::move(thunder_planner), iris_builder_options,
                               regions_file, empty, empty / "problems"),
               std::runtime_error);
}

}  // namespace planning
}  // namespace motion
