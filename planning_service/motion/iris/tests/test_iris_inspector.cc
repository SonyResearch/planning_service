/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <drake/geometry/optimization/geodesic_convexity.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/geometry/shape_specification.h>
#include <drake/solvers/binding.h>
#include <drake/solvers/solve.h>
#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "planning_service/motion/iris/iris_inspector.h"

namespace motion {
namespace iris {

class TestIrisInspector : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/alfred/sp.dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    const std::string kAncillaryArmName {"ancillary_arm"};
    const auto aa_j0 = std::make_pair(kAncillaryArmName, 0);
    const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
        aa_j0};  // only the first AA joint is continuous
    robot_model_ = std::make_unique<RobotModel>(xml_file, dmd, std::nullopt,
                                                continuous_revolute_joints);
    ConstraintsAdapter constraints_adapter;
    CollisionCheckerAdapter collision_checker_adapter;
    constraints_adapter.collision_checker = collision_checker_adapter;
    robot_constraints_ =
        std::make_unique<RobotConstraints>(*robot_model_, constraints_adapter);
    // Load the IrisRegionsAdapter from YAML
    const std::string iris_regions_adapter_file {
        "planning_service/test_data/alfred/regions_4keys.yaml"};
    auto iris_regions_adapter = drake::yaml::LoadYamlFile<IrisRegionsAdapter>(
        iris_regions_adapter_file);
    // Now make an empty IrisAdapter and add the regions one by one to benefit
    // from the computation of intersections and aabbs
    iris_regions_adapter_ = IrisRegionsAdapter();
    for (const auto& region : iris_regions_adapter.regions_vec()) {
      iris_regions_adapter_.AddRegion(
          region.set(), region.name(), region.constraints_hash(),
          robot_model_->continuous_revolute_joint_indices(), 1, 0);
    }
    iris_inspector_ = std::make_unique<IrisInspector>(*robot_constraints_,
                                                      iris_regions_adapter_);
  }
  auto& inspector() {
    return *iris_inspector_;
  }

  const auto& robot_model() const {
    return *robot_model_;
  }

 private:
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  IrisRegionsAdapter iris_regions_adapter_;
  std::unique_ptr<IrisInspector> iris_inspector_;
};

TEST_F(TestIrisInspector, MaybeCalcEdgeToRegions) {
  drake::RandomGenerator gen {0};
  SampleOptions options;
  options.parallel = false;
  const auto q_vec =
      inspector().robot_constraints().GenerateSamples(&gen, 1, options);
  const auto& q = q_vec.at(0);
  EXPECT_FALSE(inspector().InsideAnyRegion(q));
  const auto closest_point_result {inspector().MaybeCalcEdgeToRegions(q)};
  EXPECT_TRUE(closest_point_result.has_value());
  const auto& [closest_set_index, closest_point] {closest_point_result.value()};
  EXPECT_TRUE(inspector().InsideAnyRegion(closest_point));
}

TEST_F(TestIrisInspector, GetRegionsContainingConfig) {
  for (const auto& intersection :
       inspector().iris_regions_adapter().intersections_vec().value()) {
    const int i = intersection.index_one();
    const int j = intersection.index_two();
    const auto& polytope_1 =
        inspector().iris_regions_adapter().regions_vec()[i].set();
    const auto& polytope_2 =
        inspector().iris_regions_adapter().regions_vec()[j].set();
    drake::solvers::MathematicalProgram prog;
    auto x =
        prog.NewContinuousVariables(robot_model().plant().num_positions(), "x");
    // To deal with numerical issues, we need to shrink the polytopes a bit
    auto polytope_1_shrinked = drake::geometry::optimization::HPolyhedron(
        polytope_1.A(),
        polytope_1.b() - 1e-4 * Eigen::VectorXd::Ones(polytope_1.b().size()));
    auto polytope_2_shrinked = drake::geometry::optimization::HPolyhedron(
        polytope_2.A(),
        polytope_2.b() - 1e-4 * Eigen::VectorXd::Ones(polytope_2.b().size()));
    polytope_1_shrinked.AddPointInSetConstraints(&prog, x);
    polytope_2_shrinked.AddPointInSetConstraints(&prog, x);
    const auto result = drake::solvers::Solve(prog);
    EXPECT_TRUE(result.is_success());
    const auto x_val = result.GetSolution(x);
    // Check if the point is inside both regions
    EXPECT_TRUE(polytope_1.PointInSet(x_val));
    EXPECT_TRUE(polytope_2.PointInSet(x_val));
    const auto region_ids = inspector().GetRegionsContainingConfig(x_val);
    EXPECT_TRUE(region_ids.size() >= 2);
    for (const auto& region_id : region_ids) {
      if (i != region_id) {
        bool intersection_found_i {false};
        for (const auto& intersection :
             inspector().iris_regions_adapter().intersections_vec().value()) {
          if ((intersection.index_one() == i
               && intersection.index_two() == region_id)
              || (intersection.index_one() == region_id
                  && intersection.index_two() == i)) {
            intersection_found_i = true;
            break;
          }
        }
        EXPECT_TRUE(intersection_found_i);
      }
      if (j != region_id) {
        bool intersection_found_j {false};
        for (const auto& intersection :
             inspector().iris_regions_adapter().intersections_vec().value()) {
          if ((intersection.index_one() == j
               && intersection.index_two() == region_id)
              || (intersection.index_one() == region_id
                  && intersection.index_two() == j)) {
            intersection_found_j = true;
            break;
          }
        }
        EXPECT_TRUE(intersection_found_j);
      }
    }
    // check both regions are in the set
    EXPECT_TRUE(std::find(region_ids.begin(), region_ids.end(), i)
                != region_ids.end());
    EXPECT_TRUE(std::find(region_ids.begin(), region_ids.end(), j)
                != region_ids.end());
    return;
  }
}

TEST_F(TestIrisInspector, EvaluateCoverage) {
  drake::RandomGenerator generator(0);
  SampleOptions options;
  options.parallel = false;
  std::vector<Eigen::VectorXd> points =
      inspector().robot_constraints().GenerateSamples(&generator, 10, options);
  inspector().SetPointsForIrisCoverageEvaluation(points);
  // Evaluate the coverage.
  const auto coverage_result = inspector().EvaluateCoverage();
  // Check the coverage result.
  logging::log()->info("Coverage result: \n{}", coverage_result);
  // Calc the coverage again, should be cached. Should not take long.
  auto start = std::chrono::high_resolution_clock::now();
  const auto coverage_result2 = inspector().EvaluateCoverage();
  logging::log()->info("Coverage result: \n{}", coverage_result2);
  auto end = std::chrono::high_resolution_clock::now();
  auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
  logging::log()->info("Calculation time with cache: {} ms", duration.count());
  // Should take way less than 10 ms
  EXPECT_LE(duration.count(), 10);
}

TEST(IrisInspector, ConfigAgainstSingleRegion) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter;
  constraints_adapter.collision_checker = collision_checker_adapter;
  auto iris_region = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  IrisRegionsAdapter iris_regions_adapter;
  iris_regions_adapter.AddRegion(iris_region, "single_region", 1);
  auto robot_constraints =
      RobotConstraints(*robot_model, constraints_adapter, 1);
  auto iris_builder =
      std::make_unique<IrisInspector>(robot_constraints, iris_regions_adapter);
  // test when the start and end are the same
  const Eigen::Vector2d q_a = Eigen::Vector2d(0.5, 0.5);
  const auto result_a = iris_builder->EvalConfigAgainstIrisRegion(
      q_a, iris_builder->iris_regions_adapter().regions_vec().at(0), false,
      false);
  EXPECT_TRUE(result_a.inside);
  EXPECT_FALSE(result_a.check_satisfied.has_value());
  EXPECT_FALSE(result_a.connecting_set.has_value());
  // test InsideAnyRegion
  EXPECT_TRUE(iris_builder->InsideAnyRegion(q_a));
  // test EvalConfigAgainstIrisRegions
  const auto regions_results = iris_builder->EvalConfigAgainstIrisRegions(q_a);
  EXPECT_EQ(regions_results.containing_regions_indices.size(), 1);
  EXPECT_EQ(regions_results.containing_regions_indices.at(0), 0);
  EXPECT_FALSE(regions_results.visible_region_index.has_value());
  EXPECT_FALSE(regions_results.visible_point.has_value());
}

TEST(IrisInspector, CalcClosestValidConfToRegions) {
  // Need dual pandas for this test
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_pandas/dmd.yaml"};
  auto dmd =
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file);
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  auto iris_regions_adapter = drake::yaml::LoadYamlFile<IrisRegionsAdapter>(
      "planning_service/test_data/dual_pandas/iris_regions.yaml");
  auto robot_constraints = RobotConstraints(*robot_model, ConstraintsAdapter());
  auto iris_builder =
      std::make_unique<IrisInspector>(robot_constraints, iris_regions_adapter);
  // Now we have an iris builder with 21 regions
  Eigen::VectorXd q_right(7), q_left(7);
  q_right << 2.5, -0.2, -1.1, -0.5, 0.2, 1.5, -2.5;
  q_left << 0.7, -0.7, -2.2, -1.8, -1.1, 0.6, -0.9;
  Eigen::VectorXd q(14);
  q << q_right, q_left;
  // we want the right arm to be fixed
  std::vector<drake::multibody::ModelInstanceIndex> fixed_model_instances {
      robot_model->plant().GetModelInstanceByName("franka_right")};
  const auto closest_conf_opt =
      iris_builder->CalcClosestValidConfToRegions(q, fixed_model_instances);
  EXPECT_TRUE(closest_conf_opt.has_value());
  // check that the closest conf on the right is the same as the number above
  const auto closest_conf = closest_conf_opt.value();
  const auto sys_conf = robot_model->ToSystemConf(closest_conf);
  const auto closest_conf_right = sys_conf.at("franka_right");
  EXPECT_TRUE(closest_conf_right.isApprox(q_right));
}

class TestSmallIrisInspector : public ::testing::Test {
 protected:
  TestSmallIrisInspector() {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/wallflower/dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
        std::make_pair("robot", 0)};  // only the first AA joint is continuous
    robot_model_ = std::make_unique<RobotModel>(xml_file, dmd, std::nullopt,
                                                continuous_revolute_joints);
    ConstraintsAdapter constraints_adapter;
    CollisionCheckerAdapter collision_checker_adapter;
    constraints_adapter.collision_checker = collision_checker_adapter;
    RobotConstraints robot_constraints {*robot_model_, constraints_adapter, 1};
    // set 1, q0: from 0 to 90 degrees
    drake::geometry::optimization::HPolyhedron set1 =
        drake::geometry::optimization::HPolyhedron::MakeBox(
            Eigen::Vector2d(0, 0.2), Eigen::Vector2d(M_PI / 2.0, 0.35));
    // set 2: q0: from 60 to 180 degrees
    drake::geometry::optimization::HPolyhedron set2 =
        drake::geometry::optimization::HPolyhedron::MakeBox(
            Eigen::Vector2d(-5.0 * M_PI / 3.0, 0.3),
            Eigen::Vector2d(-M_PI, 0.35));
    // set 3: q0: from -90 to 45 degrees
    drake::geometry::optimization::HPolyhedron set3 =
        drake::geometry::optimization::HPolyhedron::MakeBox(
            Eigen::Vector2d(3.0 * M_PI / 2, -0.1),
            Eigen::Vector2d(9.0 * M_PI / 4, 0.5));
    robot_constraints_ =
        std::make_unique<RobotConstraints>(*robot_model_, constraints_adapter);
    const auto& crji = robot_model_->continuous_revolute_joint_indices();
    iris_regions_adapter_ = IrisRegionsAdapter();
    // Add the regions to the adapter
    iris_regions_adapter_.AddRegion(set1, "set1", 0, crji, 1, 0);
    iris_regions_adapter_.AddRegion(set2, "set2", 1, crji, 1, 0);
    iris_regions_adapter_.AddRegion(set3, "set3", 2, crji, 1, 0);
    iris_inspector_ = std::make_unique<IrisInspector>(*robot_constraints_,
                                                      iris_regions_adapter_);
  };

  const RobotModel& robot_model() const {
    return *robot_model_;
  }
  const IrisInspector& inspector() const {
    return *iris_inspector_;
  }

 private:
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  std::unique_ptr<IrisInspector> iris_inspector_;
  IrisRegionsAdapter iris_regions_adapter_;
};

TEST_F(TestSmallIrisInspector, EvalConfigAgainsIrisRegion1) {
  const Eigen::Vector2d q = Eigen::Vector2d(0, 0.2);
  // Against first region: it is inside
  const auto& iris_region_0 =
      inspector().iris_regions_adapter().regions_vec().at(0);
  const auto result_0 =
      inspector().EvalConfigAgainstIrisRegion(q, iris_region_0, false, true);
  EXPECT_TRUE(result_0.inside);
  EXPECT_FALSE(result_0.check_satisfied.has_value());
  EXPECT_FALSE(result_0.connecting_set.has_value());
  EXPECT_FALSE(result_0.distance_upper_bound.has_value());
  // Against third region: it is also inside
  const auto& iris_region_2 =
      inspector().iris_regions_adapter().regions_vec().at(2);
  const auto result_2 =
      inspector().EvalConfigAgainstIrisRegion(q, iris_region_2, false, true);
  EXPECT_TRUE(result_2.inside);
  EXPECT_FALSE(result_2.check_satisfied.has_value());
  EXPECT_FALSE(result_2.connecting_set.has_value());
  EXPECT_FALSE(result_2.distance_upper_bound.has_value());
}

TEST_F(TestSmallIrisInspector, EvalConfigAgainsIrisRegion2) {
  const Eigen::Vector2d q = Eigen::Vector2d(0, 0.2);
  // Against second region: it is not inside. The distance_upper_bound must be
  // 0.2 + M_PI/6
  const auto& iris_region_1 =
      inspector().iris_regions_adapter().regions_vec().at(1);
  const auto result_1 =
      inspector().EvalConfigAgainstIrisRegion(q, iris_region_1, false, false);
  EXPECT_FALSE(result_1.inside);
  EXPECT_FALSE(result_1.check_satisfied.has_value());
  EXPECT_FALSE(result_1.connecting_set.has_value());
  EXPECT_TRUE(result_1.distance_upper_bound.has_value());
  EXPECT_NEAR(result_1.distance_upper_bound.value(), M_PI / 3, 1e-6);
  // Let's check the visibility
  const auto result_1_visible =
      inspector().EvalConfigAgainstIrisRegion(q, iris_region_1, true, true);
  EXPECT_FALSE(result_1_visible.inside);
  EXPECT_TRUE(result_1_visible.check_satisfied.has_value()
              && result_1_visible.check_satisfied.value());
  EXPECT_TRUE(result_1_visible.connecting_set.has_value());
  const auto& vertices = result_1_visible.connecting_set.value().vertices();
  EXPECT_TRUE(vertices.col(0).isApprox(q));
  // The second one is visible and on the boundary of the set, but wrapped
  EXPECT_TRUE(vertices.col(1).isApprox(Eigen::Vector2d(M_PI / 3.0, 0.3), 1e-4));
}

TEST_F(TestSmallIrisInspector, EvalConfigAgainsIrisRegion3) {
  // Choose a point that will have multiple wrappings: q0 at 120 degrees against
  // the third region On one side, it will have 120 to -90: wrapped distance of
  // 150 degrees. On the other side, it will have 120 to 45: wrapped distance of
  // 75 degrees. the second one is shorter.
  const Eigen::Vector2d q_a = Eigen::Vector2d(2.0 * M_PI / 3.0, 0.2);
  const auto& iris_region_2 =
      inspector().iris_regions_adapter().regions_vec().at(2);
  const auto result_2_a =
      inspector().EvalConfigAgainstIrisRegion(q_a, iris_region_2, true, true);
  EXPECT_FALSE(result_2_a.inside);
  EXPECT_TRUE(result_2_a.check_satisfied.has_value()
              && result_2_a.check_satisfied.value());
  EXPECT_TRUE(result_2_a.connecting_set.has_value());
  const auto& vertices_a = result_2_a.connecting_set.value().vertices();
  EXPECT_TRUE(vertices_a.col(0).isApprox(q_a));
  // The second one is visible and on the boundary of the set, but wrapped to be
  // close to 2*M_PI/3. It will be pi/4.
  logging::log()->info("vertices_a: \n{}", vertices_a);
  EXPECT_TRUE(vertices_a.col(1).isApprox(Eigen::Vector2d(M_PI / 4, 0.2), 1e-4));
  // Now choose a point that will be closer to -90 degrees. Set q0 at 180
  // degrees. then it will have 180 to -90: wrapped distance of 90 degrees,
  // shorter than 180 to 45. For fun, make it -M_PI. Then the visible must be
  // -M_PI/2.
  const Eigen::Vector2d q_b = Eigen::Vector2d(-M_PI, 0.2);
  const auto result_2_b =
      inspector().EvalConfigAgainstIrisRegion(q_b, iris_region_2, true, true);
  EXPECT_FALSE(result_2_b.inside);
  EXPECT_TRUE(result_2_b.check_satisfied.has_value()
              && result_2_b.check_satisfied.value());
  EXPECT_TRUE(result_2_b.connecting_set.has_value());
  const auto& vertices_b = result_2_b.connecting_set.value().vertices();
  EXPECT_TRUE(vertices_b.col(0).isApprox(q_b));
  EXPECT_TRUE(
      vertices_b.col(1).isApprox(Eigen::Vector2d(-M_PI / 2.0, 0.2), 1e-4));
}

TEST_F(TestSmallIrisInspector, EvalConfigAgainsIrisRegions) {
  // Pick a point that is inside the first and third regions, but not the
  // second.
  const Eigen::Vector2d q_a = Eigen::Vector2d(0, 0.2);
  auto result = inspector().EvalConfigAgainstIrisRegions(q_a);
  EXPECT_EQ(result.containing_regions_indices.size(), 2);
  EXPECT_EQ(result.containing_regions_indices.at(0), 0);
  EXPECT_EQ(result.containing_regions_indices.at(1), 2);
  EXPECT_FALSE(result.visible_region_index.has_value());
  EXPECT_FALSE(result.visible_point.has_value());
  // Pick a point that is visible to the second region.
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.2 * M_PI, 0.2);
  result = inspector().EvalConfigAgainstIrisRegions(q_b);
  EXPECT_EQ(result.containing_regions_indices.size(), 0);
  EXPECT_TRUE(result.visible_region_index.has_value());
  EXPECT_TRUE(result.visible_point.has_value());
  EXPECT_EQ(result.visible_region_index.value(), 1);
  EXPECT_TRUE(
      result.visible_point.value().isApprox(Eigen::Vector2d(M_PI, 0.3), 1e-4));
}

TEST_F(TestSmallIrisInspector, IsValidViaSampling) {
  const auto& regions = inspector().iris_regions_adapter().regions_vec();
  // The first region has a collision with wall
  EXPECT_FALSE(inspector().IsValidViaSampling(regions.at(0), 1e3));
  // The second region is valid, so it should return true
  EXPECT_TRUE(inspector().IsValidViaSampling(regions.at(1), 1e3));
  // The third region is out of bounds, so it should not be valid
  EXPECT_FALSE(inspector().IsValidViaSampling(regions.at(2), 1e3));
}

}  // namespace iris
}  // namespace motion
