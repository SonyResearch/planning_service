#include <gtest/gtest.h>

#include "planning_service/motion/planning/internal/geodesic_math.h"

namespace motion {
namespace internal {

TEST(GeodesicMath, CalcWrapMultiple) {
  // Angles 0, -0.5pi, -pi + eps, 0.5pi, pi - eps are all within pi distance
  // from 0
  const double eps = 1e-6;
  EXPECT_EQ(CalcWrapMultiple(0.0, 0.0), 0);
  EXPECT_EQ(CalcWrapMultiple(-0.5 * M_PI, 0.0), 0);
  EXPECT_EQ(CalcWrapMultiple(-M_PI + eps, 0.0), 0);
  EXPECT_EQ(CalcWrapMultiple(0.5 * M_PI, 0.0), 0);
  EXPECT_EQ(CalcWrapMultiple(M_PI - eps, 0.0), 0);
  // if query angle is 1.5pi, and reference angle is 0.2pi,
  // then the distance becomes 1.5pi-0.2pi = 1.3pi. So need to
  // subtract 2pi from query angle to make it within pi distance
  // from the reference angle: 1.5pi - 2pi = -0.5pi
  // So the wrap multiple is -1
  EXPECT_EQ(CalcWrapMultiple(1.5 * M_PI, 0.2 * M_PI), -1);
  // if query angle is -1.5pi, and reference angle is 0.2pi,
  // then the distance becomes -1.5pi-0.2pi = -1.7pi. So need to
  // add 2pi to query angle to make it within pi distance
  // from the reference angle: -1.5pi + 2pi = 0.5pi
  // So the wrap multiple is 1
  EXPECT_EQ(CalcWrapMultiple(-1.5 * M_PI, 0.2 * M_PI), 1);
  // Given two random numbers, the query_angle + wrap_multiple*2pi
  // should be within pi distance from the reference angle.
  const double query_angle = -434.89;
  const double reference_angle = 532.6;
  const int wrap_multiple = CalcWrapMultiple(query_angle, reference_angle);
  const double wrapped_query_angle = query_angle + wrap_multiple * 2 * M_PI;
  EXPECT_LE(std::abs(wrapped_query_angle - reference_angle), M_PI);
}

TEST(GeodesicMath, WrapConfiguration) {
  const Eigen::Vector3d q_ref {8.0, 2.2, -3.1};
  const std::vector<int> continuous_revolute_joint_indices = {1, 2};
  Eigen::VectorXd q = Eigen::Vector3d(0.0, 2.2 + 2 * M_PI, -3.1 - 4 * M_PI);
  WrapConfiguration(q_ref, continuous_revolute_joint_indices, &q);
  EXPECT_NEAR(q(0), 0.0, 1e-6);
  EXPECT_NEAR(q(1), 2.2, 1e-6);
  EXPECT_NEAR(q(2), -3.1, 1e-6);
  // Bad size should throw
  Eigen::Vector2d q_ref_bad_size {0.0, 2.2};
  EXPECT_THROW(
      WrapConfiguration(q_ref_bad_size, continuous_revolute_joint_indices, &q),
      std::exception);
}

TEST(GeodesicMath, CartesianProductOfWrapMultiples) {
  // Let's have a map from joint index to a vector of wrap multiples
  std::map<int, std::vector<int>> joint_to_offset_wraps = {
      {1, {1, 2}}, {2, {3}}, {3, {4, 5}}, {4, {6, 7}}};
  // The Cartesian product of wrap multiples for the above map is:
  // [{1: 1, 2: 3, 3: 4, 4: 6}, {1: 1, 2: 3, 3: 4, 4: 7},
  //  {1: 1, 2: 3, 3: 5, 4: 6}, {1: 1, 2: 3, 3: 5, 4: 7},
  //  {1: 2, 2: 3, 3: 4, 4: 6}, {1: 2, 2: 3, 3: 4, 4: 7},
  //  {1: 2, 2: 3, 3: 5, 4: 6}, {1: 2, 2: 3, 3: 5, 4: 7},
  std::vector<std::map<int, int>> result =
      CartesianProductOfWrapMultiples(joint_to_offset_wraps);
  EXPECT_EQ(result.size(), 8);
  for (const auto& map : result) {
    EXPECT_EQ(map.size(), 4);
    EXPECT_TRUE(map.find(1) != map.end());
    EXPECT_TRUE(map.find(2) != map.end());
    EXPECT_TRUE(map.find(3) != map.end());
    EXPECT_TRUE(map.find(4) != map.end());
  }
  // check equaivalncey of the maps
  auto is_equal = [](const std::map<int, int>& a, const std::map<int, int>& b) {
    if (a.size() != b.size()) {
      return false;
    }
    for (const auto& [key, value] : a) {
      if (b.find(key) == b.end() || b.at(key) != value) {
        return false;
      }
    }
    return true;
  };
  // Check membership of the result
  std::map<int, int> expected_member = {{1, 1}, {2, 3}, {3, 4}, {4, 6}};
  EXPECT_TRUE(std::any_of(result.begin(), result.end(),
                          [&expected_member, &is_equal](const auto& map) {
                            return is_equal(map, expected_member);
                          }));
  std::map<int, int> expected_non_member = {{1, 1}, {2, 2}, {3, 4}, {4, 6}};
  EXPECT_FALSE(std::any_of(result.begin(), result.end(),
                           [&expected_non_member, &is_equal](const auto& map) {
                             return is_equal(map, expected_non_member);
                           }));
}

TEST(GeodesicMath, CalcWrapMultiplesPerJoint) {
  const Eigen::Vector2d q {-2.2 * M_PI, 1.7 * M_PI};
  const drake::geometry::optimization::Hyperrectangle aabb(
      Eigen::Vector2d {0.0, -M_PI}, Eigen::Vector2d {0.9 * M_PI, -0.2 * M_PI});
  const std::vector<int> continuous_revolute_joint_indices = {0, 1};
  const bool exit_if_inside_impossible = false;
  // For joint 0, a wrap_multiple of 1 brings it to 0.2pi distance from the
  // lower bound, and a wrap_multiple of 2 brings it to 0.9pi distance from the
  // upper bound. Both are within pi distance from the bounds. So the wrap
  // multiples are 1 and 2. For joint 1, a wrap_multiple of -1 actually brings
  // it to within the bounds. So the wrap multiple is -1 and is the only wrap
  // multiple.
  std::map<int, std::vector<int>> result = CalcWrapMultiplesPerJoint(
      q, aabb, continuous_revolute_joint_indices, exit_if_inside_impossible);
  // Expected result is: {0: {1, 2}, 1: {-1}}
  EXPECT_EQ(result.size(), 2);
  EXPECT_TRUE(result.find(0) != result.end());
  EXPECT_TRUE(result.find(1) != result.end());
  EXPECT_EQ(result.at(0).size(), 2);
  EXPECT_EQ(result.at(1).size(), 1);
  EXPECT_EQ(result.at(0)[0], 1);
  EXPECT_EQ(result.at(0)[1], 2);
  EXPECT_EQ(result.at(1)[0], -1);
}

TEST(GeodesicMath, MultipleWrapsPossible) {
  std::map<int, std::vector<int>> joint_to_offset_wraps_1 = {
      {1, {1, 2}}, {2, {3}}, {3, {4, 5}}, {4, {6, 7}}};
  EXPECT_TRUE(MultipleWrapsPossible(joint_to_offset_wraps_1));
  std::map<int, std::vector<int>> joint_to_offset_wraps_2 = {
      {1, {1}}, {2, {3}}, {3, {4, -1}}, {4, {6}}};
  EXPECT_TRUE(MultipleWrapsPossible(joint_to_offset_wraps_2));
  std::map<int, std::vector<int>> joint_to_offset_wraps_3 = {
      {1, {1}}, {2, {3}}, {3, {4}}, {4, {6}}};
  EXPECT_FALSE(MultipleWrapsPossible(joint_to_offset_wraps_3));
  // Empty map
  std::map<int, std::vector<int>> joint_to_offset_wraps_4 = {};
  EXPECT_FALSE(MultipleWrapsPossible(joint_to_offset_wraps_4));
  // Invalid
  std::map<int, std::vector<int>> joint_to_offset_wraps_5 = {
      {1, {1, 2, 3}}, {2, {3}}, {3, {4, 5}}, {4, {6, 7}}};
  EXPECT_THROW(MultipleWrapsPossible(joint_to_offset_wraps_5), std::exception);
}

TEST(GeodesicMath, CalcManhattanDistanceToAABB) {
  const drake::geometry::optimization::Hyperrectangle aabb(
      Eigen::Vector2d {-0.2, 0.3}, Eigen::Vector2d {0.5, 0.7});
  const Eigen::Vector2d q_1 {0, 0};       // 0 + 0.3 = 0.3
  const Eigen::Vector2d q_2 {-0.3, 0.1};  // 0.1 + 0.2 = 0.3
  const Eigen::Vector2d q_3 {0.0, 0.5};   // 0 + 0 = 0
  const Eigen::Vector2d q_4 {0.6, 0.3};   // 0.1 + 0 = 0.1
  EXPECT_NEAR(CalcManhattanDistanceToAABB(q_1, aabb), 0.3, 1e-6);
  EXPECT_NEAR(CalcManhattanDistanceToAABB(q_2, aabb), 0.3, 1e-6);
  EXPECT_NEAR(CalcManhattanDistanceToAABB(q_3, aabb), 0.0, 1e-6);
  EXPECT_NEAR(CalcManhattanDistanceToAABB(q_4, aabb), 0.1, 1e-6);
  // invalid input
  const Eigen::Vector3d q_5 {0.0, 0.0, 0.0};
  EXPECT_THROW(CalcManhattanDistanceToAABB(q_5, aabb), std::exception);
}

TEST(GeodesicMath, CalcSortedWrappedConfigs) {
  const drake::geometry::optimization::Hyperrectangle aabb(
      Eigen::Vector2d {-0.5 * M_PI, 0.1 * M_PI},
      Eigen::Vector2d {0.4 * M_PI, M_PI});
  const Eigen::Vector2d q {-1.4 * M_PI, 0.2 * M_PI};
  // No wrapping: the distance would be 0.9pi + 0 = 0.9pi
  std::map<int, int> wrap_map_1 = {{0, 0}, {1, 0}};
  // Add 2*pi to joint 0: the distance would be 0.2pi + 0 = 0.2pi
  std::map<int, int> wrap_map_2 = {{0, 1}, {1, 0}};
  // Add 2*pi to joint 0 and 2*pi to joint 1: the distance would be 0.2pi
  // + 1.2*pi = 1.4pi
  std::map<int, int> wrap_map_3 = {{0, 1}, {1, 1}};
  std::vector<std::map<int, int>> joint_to_offset_wraps = {
      wrap_map_1, wrap_map_2, wrap_map_3};
  std::vector<Eigen::VectorXd> wrapped_configs =
      CalcSortedWrappedConfigs(q, aabb, joint_to_offset_wraps);
  EXPECT_EQ(wrapped_configs.size(), 3);
  const Eigen::Vector2d q_0 {q + 2 * M_PI * Eigen::Vector2d {0, 0}};
  const Eigen::Vector2d q_1 {q + 2 * M_PI * Eigen::Vector2d {1, 0}};
  const Eigen::Vector2d q_2 {q + 2 * M_PI * Eigen::Vector2d {1, 1}};
  EXPECT_TRUE(wrapped_configs[0].isApprox(q_1));
  EXPECT_TRUE(wrapped_configs[1].isApprox(q_0));
  EXPECT_TRUE(wrapped_configs[2].isApprox(q_2));
  EXPECT_NEAR(CalcManhattanDistanceToAABB(wrapped_configs[0], aabb), 0.2 * M_PI,
              1e-6);
  EXPECT_NEAR(CalcManhattanDistanceToAABB(wrapped_configs[1], aabb), 0.9 * M_PI,
              1e-6);
  EXPECT_NEAR(CalcManhattanDistanceToAABB(wrapped_configs[2], aabb), 1.4 * M_PI,
              1e-6);
}

}  // namespace internal
}  // namespace motion
