/*
 * Copyright © 2025 Sony Group Corporation. All rights reserved.
 */

#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/optimization/geodesic_convexity.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/geometry/shape_specification.h>
#include <drake/solvers/binding.h>
#include <drake/solvers/solve.h>
#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "planning_service/motion/iris/iris_adapter.h"

namespace motion {
namespace iris {

TEST(IrisRegionsAdapter, Basics) {
  auto dut = IrisRegionsAdapter();
  size_t hash {1};
  // Let's make some regions
  std::map<std::string, drake::geometry::optimization::HPolyhedron>
      iris_regions_map;
  iris_regions_map.emplace(
      "region_1", drake::geometry::optimization::HPolyhedron::MakeL1Ball(2));
  iris_regions_map.emplace(
      "region_2", drake::geometry::optimization::HPolyhedron::MakeBox(
                      Eigen::Vector2d(0.0, 0.0), Eigen::Vector2d(1.0, 1.0)));
  iris_regions_map.emplace(
      "region_3", drake::geometry::optimization::HPolyhedron::MakeBox(
                      Eigen::Vector2d(0.6, 0.6), Eigen::Vector2d(1.5, 1.5)));
  iris_regions_map.emplace(
      "region_4",
      drake::geometry::optimization::HPolyhedron::MakeBox(
          Eigen::Vector2d(-1.2, -0.2), Eigen::Vector2d(-0.99, 0.2)));
  // 1 intersects with 2, 3, 4, and 2 with 3
  for (const auto& [name, region] : iris_regions_map) {
    const auto* new_iris_region = dut.AddRegion(region, name, hash);
    EXPECT_TRUE(new_iris_region != nullptr);
    EXPECT_TRUE(new_iris_region->name() == name);
    EXPECT_TRUE(new_iris_region->constraints_hash() == hash);
    EXPECT_TRUE(new_iris_region->index() == int(dut.regions_vec().size() - 1));
  }
  EXPECT_TRUE(iris_regions_map.size() == dut.regions_vec().size());
  EXPECT_TRUE(iris_regions_map.size() == dut.GetConvexSets().size());
  // Check indices
  for (size_t i = 0; i < dut.regions_vec().size(); ++i) {
    const auto index = int(i);
    EXPECT_TRUE(dut.regions_vec()[i].index() == index);
    const auto& name = dut.regions_vec()[i].name();
    EXPECT_EQ(iris_regions_map.count(name), 1);
    EXPECT_EQ(hash, dut.regions_vec()[i].constraints_hash());
  }
  // Test saving and loading the adapter from a file
  const std::string adapter_file = "temp";
  drake::yaml::SaveYamlFile(adapter_file, dut);
  // load the adapter file again
  const auto dut_loaded {
      drake::yaml::LoadYamlFile<IrisRegionsAdapter>(adapter_file)};
  EXPECT_EQ(dut_loaded.regions_vec().size(), dut.regions_vec().size());
  EXPECT_EQ(dut_loaded.GetConvexSets().size(), dut.GetConvexSets().size());
  EXPECT_TRUE(dut_loaded.intersections_vec().has_value());
  EXPECT_EQ(dut_loaded.intersections_vec().value().size(),
            dut.intersections_vec().value().size());
  // get the graphviz string
  const auto graphviz_string = dut.CalcGraphVizString();
  logging::log()->debug("Graphviz string:\n {}", graphviz_string);
  // Check the intersections from the loaded adapter
  const auto& intersections_vec = dut_loaded.intersections_vec();
  EXPECT_TRUE(intersections_vec.has_value());
  EXPECT_TRUE(intersections_vec.value().size() > 0);
  for (const auto& intersection : intersections_vec.value()) {
    EXPECT_GE(intersection.index_one(), 0);
    EXPECT_GE(intersection.index_two(), 0);
    EXPECT_LT(intersection.index_one(), int(dut.regions_vec().size()));
    EXPECT_LT(intersection.index_two(), int(dut.regions_vec().size()));
    EXPECT_EQ(
        intersection.offset().size(),
        dut.regions_vec()[intersection.index_one()].set().ambient_dimension());
    // For this problem, the offset must be zero
    EXPECT_TRUE(intersection.offset().isZero());
    EXPECT_EQ(intersection.intersection_samples().size(), 1);
    // Since offset is zero, we double check intersections exist
    const auto& polytope_1 =
        dut.regions_vec().at(intersection.index_one()).set();
    const auto& polytope_2 =
        dut.regions_vec().at(intersection.index_two()).set();
    EXPECT_TRUE(polytope_1.IntersectsWith(polytope_2));
  }
  // 1 intersects with 2, 3, 4, and 2 with 3
  EXPECT_EQ(intersections_vec.value().size(), 3);
  const auto& intersection_1 = intersections_vec.value().at(0);
  const auto& intersection_2 = intersections_vec.value().at(1);
  const auto& intersection_3 = intersections_vec.value().at(2);
  EXPECT_TRUE(intersection_1.index_one() == 0
              && intersection_1.index_two() == 1);
  EXPECT_TRUE(intersection_2.index_one() == 1
              && intersection_2.index_two() == 2);
  EXPECT_TRUE(intersection_3.index_one() == 0
              && intersection_3.index_two() == 3);
  // Get the adapter string
  const auto adapter_string = drake::yaml::SaveYamlString(dut);
  logging::log()->debug("Adapter string:\n {}", adapter_string);
}

TEST(IrisRegionsAdapter, CalcEdgeIntersectionWithRegion) {
  const Eigen::Vector2d v1 {-1.0, 2.0};
  const Eigen::Vector2d v2 {-1.0, 0.0};
  const Eigen::Vector2d v3 {0.0, 0.0};
  const Eigen::Vector2d v4 {0.0, 2.0};
  const Eigen::Vector2d v5 {0.0, -2.0};

  std::pair<Eigen::Vector2d, Eigen::Vector2d> e12 {
      v1, v2};  // second vertex inside, intersection on boundary
  std::pair<Eigen::Vector2d, Eigen::Vector2d> e23 {v2,
                                                   v3};  // both vertices inside
  std::pair<Eigen::Vector2d, Eigen::Vector2d> e34 {v3,
                                                   v4};  // first vertex inside
  std::pair<Eigen::Vector2d, Eigen::Vector2d> e35 {v3,
                                                   v5};  // first vertex inside
  std::pair<Eigen::Vector2d, Eigen::Vector2d> e45 {
      v4, v5};  // both vertices outside; intersection exists
  std::pair<Eigen::Vector2d, Eigen::Vector2d> e14 {
      v1, v4};  // both vertices outside; no intersection

  const auto ones_2d = Eigen::Vector2d::Ones(2);
  const auto box =
      drake::geometry::optimization::HPolyhedron::MakeBox(-ones_2d, ones_2d);
  const auto& intersection_e12_opt {
      IrisRegionsAdapter::CalcEdgeIntersectionWithRegion(e12, box)};
  ASSERT_TRUE(intersection_e12_opt.has_value());
  const auto& e12_intersect_t_ {intersection_e12_opt.value()};
  EXPECT_LE(std::abs(e12_intersect_t_.first - 0.5), 1e-3);
  EXPECT_LE(std::abs(e12_intersect_t_.second - 1.0), 1e-3);
  // Make sure e12_intersect is inside the polytope
  const auto& e12_v1_intersect {(1 - e12_intersect_t_.first) * v1
                                + e12_intersect_t_.first * v2};
  const auto& e12_v2_intersect {(1 - e12_intersect_t_.second) * v1
                                + e12_intersect_t_.second * v2};
  EXPECT_TRUE(box.PointInSet(e12_v1_intersect, 0.0));
  EXPECT_TRUE(box.PointInSet(e12_v2_intersect, 0.0));
  // Make sure that the intersection is fully in the polytope, i.e., when we
  // intersect it again it does not change
  const auto& double_intersection_e12_opt {
      IrisRegionsAdapter::CalcEdgeIntersectionWithRegion(
          std::pair(e12_v1_intersect, e12_v2_intersect), box)};
  ASSERT_TRUE(double_intersection_e12_opt.has_value());
  const auto& e12_double_intersect {double_intersection_e12_opt.value()};
  EXPECT_LE(std::abs(e12_double_intersect.first), 1e-3);
  EXPECT_LE(std::abs(e12_double_intersect.second - 1.0), 1e-3);

  // edge e14
  const auto& intersection_e14_opt {
      IrisRegionsAdapter::CalcEdgeIntersectionWithRegion(e14, box)};
  ASSERT_FALSE(intersection_e14_opt.has_value());

  // edge e23 completely inside
  const auto& intersection_e23_opt {
      IrisRegionsAdapter::CalcEdgeIntersectionWithRegion(e23, box)};
  ASSERT_TRUE(intersection_e23_opt.has_value());
  const auto& e23_intersect_t_ {intersection_e23_opt.value()};

  EXPECT_LE(std::abs(e23_intersect_t_.first), 1e-3);
  EXPECT_LE(std::abs(e23_intersect_t_.second - 1.0), 1e-3);
  const auto& e23_v2_intersect {(1 - e23_intersect_t_.first) * v2
                                + e23_intersect_t_.first * v3};
  const auto& e23_v3_intersect {(1 - e23_intersect_t_.second) * v2
                                + e23_intersect_t_.second * v3};

  EXPECT_TRUE(box.PointInSet(e23_v2_intersect, 0.0));
  EXPECT_TRUE(box.PointInSet(e23_v3_intersect, 0.0));

  // edge e34 first vertex inside
  const auto& intersection_e34_opt {
      IrisRegionsAdapter::CalcEdgeIntersectionWithRegion(e34, box)};
  ASSERT_TRUE(intersection_e34_opt.has_value());
  const auto& e34_intersect_t_ {intersection_e34_opt.value()};
  EXPECT_LE(std::abs(e34_intersect_t_.first), 1e-3);  // moved
  EXPECT_LE(e34_intersect_t_.second, 1 - 1e-1);       // did not move
  const auto& e34_v3_intersect {(1 - e34_intersect_t_.first) * v3
                                + e34_intersect_t_.first * v4};
  const auto& e34_v4_intersect {(1 - e34_intersect_t_.second) * v3
                                + e34_intersect_t_.second * v4};
  EXPECT_TRUE(box.PointInSet(e34_v3_intersect, 0.0));
  EXPECT_TRUE(box.PointInSet(e34_v4_intersect, 0.0));

  // edge e_45 both outside, intersection exists
  const auto& intersection_e45_opt {
      IrisRegionsAdapter::CalcEdgeIntersectionWithRegion(e45, box)};
  ASSERT_TRUE(intersection_e45_opt.has_value());
  const auto& e45_intersect_t_ {intersection_e45_opt.value()};
  EXPECT_GE(e45_intersect_t_.first, 1e-1);       // moved
  EXPECT_LE(e45_intersect_t_.second, 1 - 1e-1);  // moved
  const auto& e45_v4_intersect {(1 - e45_intersect_t_.first) * v4
                                + e45_intersect_t_.first * v5};
  const auto& e45_v5_intersect {(1 - e45_intersect_t_.second) * v4
                                + e45_intersect_t_.second * v5};
  EXPECT_TRUE(box.PointInSet(e45_v4_intersect, 0.0));
  EXPECT_TRUE(box.PointInSet(e45_v5_intersect, 0.0));
}

}  // namespace iris
}  // namespace motion
