/*
 * Copyright © 2025 Sony Research. All rights reserved.
 */

#include <drake/geometry/optimization/geodesic_convexity.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/geometry/shape_specification.h>
#include <drake/solvers/binding.h>
#include <drake/solvers/solve.h>
#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "planning_service/motion/iris/iris_builder.h"

namespace motion {
namespace iris {

class TestIrisBuilder : public ::testing::Test {
 protected:
  TestIrisBuilder() {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/wallflower/more_walls.dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
        std::make_pair("robot", 0)};  // only the first AA joint is continuous
    bool implicit_parallelism = true;
    robot_model_ = std::make_unique<RobotModel>(xml_file, dmd, std::nullopt,
                                                continuous_revolute_joints,
                                                implicit_parallelism);
    ConstraintsAdapter constraints_adapter;
    CollisionCheckerAdapter collision_checker_adapter;
    constraints_adapter.collision_checker = collision_checker_adapter;
    RobotConstraints robot_constraints {*robot_model_, constraints_adapter, 1};
    robot_constraints_ =
        std::make_unique<RobotConstraints>(*robot_model_, constraints_adapter);
    iris_regions_adapter_ = IrisRegionsAdapter();  // empty adapter
    std::string iris_regions_file =
        "temp";  // This test always starts with an empty adapter
    IrisRegionsAdapter empty_adapter;
    drake::yaml::SaveYamlFile(iris_regions_file, empty_adapter);
    IrisBuilderOptions options;
    options.repair_regions = true;
    options.use_generation_from_cliques = false;
    options.drake_iris_np_options.require_sample_point_is_contained = true;
    drake::yaml::SaveYamlFile(
        "planning_service/test_data/iris_builder_options.yaml", options);
    builder_wo_clique_ = std::make_unique<IrisBuilder>(
        *robot_constraints_, options, iris_regions_file);
    // Make another iris_builder with samples to guide cliques.
    drake::RandomGenerator gen(0);
    SampleOptions sample_options;
    sample_options.parallel = false;
    auto samples =
        robot_constraints_->GenerateSamples(&gen, 10, sample_options);
    options.use_generation_from_cliques = true;
    builder_w_clique_ = std::make_unique<IrisBuilder>(
        *robot_constraints_, options, iris_regions_file, samples);
  };

  const RobotConstraints& robot_constraints() const {
    return *robot_constraints_;
  }

  const IrisBuilder& builder(bool clique = false) const {
    if (!clique) {
      return *builder_wo_clique_;
    }
    return *builder_w_clique_;
  }

  IrisBuilder& mutable_builder(bool clique = false) {
    if (!clique) {
      return *builder_wo_clique_;
    }
    return *builder_w_clique_;
  }

 private:
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  std::unique_ptr<IrisBuilder> builder_wo_clique_, builder_w_clique_;
  IrisRegionsAdapter iris_regions_adapter_;
};

TEST_F(TestIrisBuilder, CalcIrisFromConfig) {
  // Calculate Iris Region from a configuration using 3 different algorithms.
  const Eigen::Vector2d q = Eigen::Vector2d(0, 0.3);
  auto polytope_np = builder().CalcIrisNpFromConfig(q);
  EXPECT_TRUE(polytope_np.has_value());
  auto polytope_np2 = builder().CalcIrisNp2FromConfig(q);
  EXPECT_TRUE(polytope_np2.has_value());
  auto polytope_zo = builder().CalcIrisZoFromConfig(q);
  EXPECT_TRUE(polytope_zo.has_value());
}

TEST_F(TestIrisBuilder, CalcIrisRegionFromConfig) {
  const auto q_1 = Eigen::Vector2d(0, 0.2);
  const auto q_2 = Eigen::Vector2d(M_PI / 3, 0.3);
  auto polytope_1 = builder().CalcIrisRegionFromConfig(q_1);
  auto polytope_2 = builder().CalcIrisRegionFromConfig(q_2);
  EXPECT_EQ(polytope_1.has_value(), true);
  EXPECT_EQ(polytope_2.has_value(), true);
  mutable_builder().AddRegion(*polytope_1, "test_region_1");
  mutable_builder().AddRegion(*polytope_2, "test_region_2");
  // Now add a third region in the first one should fail
  const auto q_3 = Eigen::Vector2d(
      0, 0.25);  // close to q_1,  should be inside the first region
  EXPECT_TRUE(builder().inspector().InsideAnyRegion(q_3));
  auto polytope_3 =
      builder().CalcIrisRegionFromConfig(q_3, IrisBuilder::IrisMethod::kIrisNp);
  EXPECT_FALSE(polytope_3.has_value());
  // Get 1000 samples from robot constraints
  drake::RandomGenerator gen(0);
  SampleOptions sample_options;
  sample_options.parallel = false;
  auto q_vec = robot_constraints().GenerateSamples(&gen, 100, sample_options);
  EXPECT_GE(q_vec.size(), 100);
  auto inspector = IrisInspector(robot_constraints(), builder().adapter());
  inspector.SetPointsForIrisCoverageEvaluation(q_vec);
  inspector.EvaluateCoverage();
}

TEST_F(TestIrisBuilder, CalcIrisRegionFromEdge) {
  const auto q_1 = Eigen::Vector2d(0, 0.25);
  const auto q_2 = Eigen::Vector2d(M_PI / 2, 0.2);
  auto polytope_np = builder(true).CalcIrisRegionFromEdge(
      {q_1, q_2}, IrisBuilder::IrisMethod::kIrisNp);
  auto polytope_np2 = builder(true).CalcIrisRegionFromEdge(
      {q_1, q_2}, IrisBuilder::IrisMethod::kIrisNp2);
  auto polytope_zo = builder(true).CalcIrisRegionFromEdge(
      {q_1, q_2}, IrisBuilder::IrisMethod::kIrisZo);
  EXPECT_EQ(polytope_np.has_value(), true);
  EXPECT_EQ(polytope_np2.has_value(), true);
  EXPECT_EQ(polytope_zo.has_value(), true);
  // Double check edge is contained within the polytope
  EXPECT_TRUE(polytope_np->PointInSet(q_1) && polytope_np->PointInSet(q_2));
  EXPECT_TRUE(polytope_np2->PointInSet(q_1) && polytope_np2->PointInSet(q_2));
  EXPECT_TRUE(polytope_zo->PointInSet(q_1) && polytope_zo->PointInSet(q_2));
}

TEST_F(TestIrisBuilder, CalcIrisRegionFromEdgeWithCliques) {
  const auto q_1 = Eigen::Vector2d(0, 0.25);
  const auto q_2 = Eigen::Vector2d(M_PI / 2, 0.22);
  auto polytope_np = builder().CalcIrisRegionFromEdge(
      {q_1, q_2}, IrisBuilder::IrisMethod::kIrisNp);
  auto polytope_np2 = builder().CalcIrisRegionFromEdge(
      {q_1, q_2}, IrisBuilder::IrisMethod::kIrisNp2);
  auto polytope_zo = builder().CalcIrisRegionFromEdge(
      {q_1, q_2}, IrisBuilder::IrisMethod::kIrisZo);
  EXPECT_EQ(polytope_np.has_value(), true);
  EXPECT_EQ(polytope_np2.has_value(), true);
  EXPECT_EQ(polytope_zo.has_value(), true);
  // Double check edge is contained within the polytope
  EXPECT_TRUE(polytope_np->PointInSet(q_1) && polytope_np->PointInSet(q_2));
  EXPECT_TRUE(polytope_np2->PointInSet(q_1) && polytope_np2->PointInSet(q_2));
  EXPECT_TRUE(polytope_zo->PointInSet(q_1) && polytope_zo->PointInSet(q_2));
}

TEST_F(TestIrisBuilder, BuildFromConfigs) {
  std::map<std::string, Eigen::VectorXd> dut;
  dut.emplace("config_1", Eigen::Vector2d(M_PI, 0.2));
  dut.emplace("config_2", Eigen::Vector2d(-M_PI, 0.4));
  dut.emplace("config_3", Eigen::Vector2d(0, 0.3));
  mutable_builder(true).BuildFromConfigs(dut, IrisBuilder::IrisMethod::kIrisZo);
  // Check that at least 2 regions were created
  EXPECT_GE(builder(true).adapter().regions_vec().size(), 2);
  // Check that the configs are covered by the regions
  for (const auto& [_, config] : dut) {
    EXPECT_TRUE(builder(true).inspector().InsideAnyRegion(config));
  }
}

TEST_F(TestIrisBuilder, BuildFromEdges) {
  std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd>> dut;
  // This edge is invalid, it should not be added to the builder
  dut.emplace("edge_1", std::make_pair(Eigen::Vector2d(0, 0.2),
                                       Eigen::Vector2d(M_PI, 0.4)));
  // These edges are valid, they should be added to the builder
  dut.emplace("edge_2", std::make_pair(Eigen::Vector2d(0, 0.2),
                                       Eigen::Vector2d(M_PI / 2, 0.2)));
  dut.emplace("edge_3", std::make_pair(Eigen::Vector2d(-M_PI / 2, 0.4),
                                       Eigen::Vector2d(0, 0.3)));
  mutable_builder(true).BuildFromEdges(dut, IrisBuilder::IrisMethod::kIrisZo);
  // Check that at least 2 regions were created
  EXPECT_GE(builder(true).adapter().regions_vec().size(), 2);
  EXPECT_TRUE(
      builder(true).inspector().IsEdgeCoveredByRegions(dut.at("edge_2")));
  EXPECT_TRUE(
      builder(true).inspector().IsEdgeCoveredByRegions(dut.at("edge_3")));
  // Check that the edge_1 is not covered by any region
  EXPECT_FALSE(
      builder(true).inspector().IsEdgeCoveredByRegions(dut.at("edge_1")));
}

TEST_F(TestIrisBuilder, BuildFromPath) {
  std::vector<Eigen::VectorXd> conf_sequence;
  conf_sequence.emplace_back(Eigen::Vector2d(0, 0.3));
  conf_sequence.emplace_back(Eigen::Vector2d(M_PI / 2, 0.2));
  conf_sequence.emplace_back(Eigen::Vector2d(M_PI, 0.2));
  conf_sequence.emplace_back(Eigen::Vector2d(M_PI, 0.4));
  mutable_builder().BuildFromPath(conf_sequence);
  // Check that at least 2 regions were created
  EXPECT_GE(builder().adapter().regions_vec().size(), 2);
  // Check that the path configs are covered by the regions
  for (const auto& config : conf_sequence) {
    EXPECT_TRUE(builder().inspector().InsideAnyRegion(config));
  }
}

TEST_F(TestIrisBuilder, BuildFromPath_Comparison) {
  std::vector<Eigen::VectorXd> conf_sequence;
  conf_sequence.emplace_back(Eigen::Vector2d(-M_PI / 2, 0.2));
  conf_sequence.emplace_back(Eigen::Vector2d(M_PI / 2, 0.2));
  mutable_builder().BuildFromPath(conf_sequence);
  mutable_builder(true).BuildFromPath(conf_sequence,
                                      IrisBuilder::IrisMethod::kIrisZo);
  // Let's make a random inspector
  drake::RandomGenerator gen(2);
  SampleOptions sample_options;
  sample_options.parallel = false;
  auto q_vec = robot_constraints().GenerateSamples(&gen, 100, sample_options);
  auto inspector_wo_clique =
      IrisInspector(robot_constraints(), builder().adapter());
  inspector_wo_clique.SetPointsForIrisCoverageEvaluation(q_vec);
  auto coverage_wo_clique = inspector_wo_clique.EvaluateCoverage();
  auto inspector_w_clique =
      IrisInspector(robot_constraints(), builder(true).adapter());
  inspector_w_clique.SetPointsForIrisCoverageEvaluation(q_vec);
  auto coverage_w_clique = inspector_w_clique.EvaluateCoverage();
  EXPECT_GE(coverage_w_clique.volume_covered,
            coverage_wo_clique.volume_covered);
  logging::log()->info("Coverage without cliques: {}", coverage_wo_clique);
  logging::log()->info("Coverage with cliques: {}", coverage_w_clique);
}

TEST_F(TestIrisBuilder, RepairRegionViaSampling) {
  // Make a dummy region that is the full domain
  const auto region_center_invalid =
      drake::geometry::optimization::HPolyhedron::MakeBox(
          Eigen::Vector2d(0, 0.2), Eigen::Vector2d(M_PI, 0.4));
  // Repairing would throw because region center is in collision
  EXPECT_THROW(builder().RepairRegionViaSampling(region_center_invalid, {}, 1),
               std::runtime_error);
  const auto region = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0, 0.2), Eigen::Vector2d(M_PI / 2, 0.4));
  // Repair the region
  auto [repaired_region, repair_required] =
      builder().RepairRegionViaSampling(region, {}, 1000);
  EXPECT_TRUE(repair_required);
  // Center of the maximum volume inscribed ellipsoid should be inside the
  // repaired region
  auto original_ellipsoid_center =
      region.MaximumVolumeInscribedEllipsoid().center();
  EXPECT_TRUE(repaired_region.PointInSet(original_ellipsoid_center));
  // Check that the new region is within the original region
  EXPECT_TRUE(repaired_region.ContainedIn(region));
  // Repairing the region again would not need repairing
  auto [repaired_region_2, repair_required_2] =
      builder().RepairRegionViaSampling(repaired_region, {}, 1000);
  EXPECT_FALSE(repair_required_2);
  // Confirm that the region is still the same
  EXPECT_TRUE(repaired_region_2.ContainedIn(repaired_region));
  // And the reverse should also be true
  EXPECT_TRUE(repaired_region.ContainedIn(repaired_region_2));
  // Test the case that we did not use ellipsoid for repairing
  auto [repaired_region_3, repair_required_3] =
      builder().RepairRegionViaSampling(region, {}, 1000, 0, false);
  EXPECT_TRUE(repair_required_3);
  drake::RandomGenerator gen(0);
  double region_volume = region.CalcVolumeViaSampling(&gen).volume;
  double repaired_with_ellipsoid_volume =
      repaired_region.CalcVolumeViaSampling(&gen).volume;
  double repaired_without_ellipsoid_volume =
      repaired_region_3.CalcVolumeViaSampling(&gen).volume;
  logging::log()->info(
      "Original region volume: {}, repaired with ellipsoid volume: {}, "
      "repaired without ellipsoid volume: {}",
      region_volume, repaired_with_ellipsoid_volume,
      repaired_without_ellipsoid_volume);
  EXPECT_GT(repaired_with_ellipsoid_volume, repaired_without_ellipsoid_volume);
}

TEST(IrisBuilder, ArmWithGripper) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.position_constraints =
      std::vector<PositionConstraintAdapter> {};
  constraints_adapter.angle_constraints =
      std::vector<AngleBetweenVectorsConstraintAdapter> {};
  // Add a position constraint for the gripper
  constraints_adapter.position_constraints->push_back(
      PositionConstraintAdapter {
          .frame_A = "world",
          .frame_B = "panda_east__tool_mount_base_link",
          .position_BQ = Eigen::Vector3d::Zero(),
          .position_AQ_lower = Eigen::Vector3d(-0.4, -0.5, 0.2),
          .position_AQ_upper = Eigen::Vector3d(0.4, 0.5, 0.6)});
  // Add an angle between vectors constraint for the panda_link 0, which should
  // not rotate
  constraints_adapter.angle_constraints->push_back(
      AngleBetweenVectorsConstraintAdapter {
          .frame_A = "world",
          .frame_B = "panda_east__tool_mount_base_link",
          .a_A = Eigen::Vector3d::UnitZ(),
          .b_B = -Eigen::Vector3d::UnitZ(),
          .angle_lower = 0.0,
          .angle_upper = M_PI / 2});
  CollisionCheckerAdapter collision_checker_adapter;
  constraints_adapter.collision_checker = collision_checker_adapter;
  const auto robot_constraints {
      RobotConstraints(*robot_model, constraints_adapter)};
  auto iris_builder = IrisBuilder(robot_constraints, IrisBuilderOptions(),
                                  "temp_iris_arm_with_gripper.yaml");
  Eigen::VectorXd q(8);
  q << 0.0, 0.0, -1.0, -1.5, 0.0, 1.2, 1.5, 0.0;
  auto polytope_opt = iris_builder.CalcIrisRegionFromConfig(q);
  ASSERT_TRUE(polytope_opt.has_value());
  const auto& polytope = *polytope_opt;
  // Check that the polytope is in 8D
  EXPECT_EQ(polytope.ambient_dimension(), 8);
  // Sample some points in the polytope and check that they satisfy the
  // constraints
  drake::RandomGenerator gen(0);
  const int num_samples = 100;
  auto q_sample = q;
  for (int i = 0; i < num_samples; ++i) {
    q_sample = polytope.UniformSample(&gen, q_sample);
    EXPECT_TRUE(robot_constraints.CheckSatisfied(q_sample));
  }
}

}  // namespace iris
}  // namespace motion
