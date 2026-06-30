#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/geometry/shape_specification.h>
#include <drake/solvers/binding.h>
#include <drake/solvers/solve.h>
#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "planning_service/motion/iris/clique_generator.h"

namespace motion {
namespace iris {

class TestCliqueGenerator : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/wallflower/dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    robot_model_ = std::make_unique<RobotModel>(xml_file, dmd);
    ConstraintsAdapter constraints_adapter;
    CollisionCheckerAdapter collision_checker_adapter;
    constraints_adapter.collision_checker = collision_checker_adapter;
    robot_constraints_ =
        std::make_unique<RobotConstraints>(*robot_model_, constraints_adapter);
    // Generate random samples
    drake::RandomGenerator generator(1);
    SampleOptions options;
    options.parallel = false;
    auto q_vec = robot_constraints_->GenerateSamples(&generator, 10, options);
    clique_generator_ =
        std::make_unique<CliqueGenerator>(*robot_constraints_, q_vec);
    // Define an adjacency matrix
    Eigen::SparseMatrix<bool> adjacency_matrix(q_vec.size(), q_vec.size());
    for (int i = 0; i < static_cast<int>(q_vec.size()); ++i) {
      for (int j = i + 1; j < static_cast<int>(q_vec.size()); ++j) {
        if (robot_constraints_->CheckSatisfiedEdge(q_vec[i], q_vec[j], 0.005)) {
          adjacency_matrix.insert(i, j) = true;
          adjacency_matrix.insert(j, i) = true;  // Add both directions
        }
      }
    }
    // Create a clique generator with the adjacency matrix
    clique_generator_adjacency_matrix_ = std::make_unique<CliqueGenerator>(
        *robot_constraints_, q_vec, adjacency_matrix);
  }

  const CliqueGenerator& dut() const {
    return *clique_generator_;
  }

  const CliqueGenerator& dut_adjacency_matrix() const {
    return *clique_generator_adjacency_matrix_;
  }

 private:
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  std::unique_ptr<CliqueGenerator> clique_generator_;
  std::unique_ptr<CliqueGenerator> clique_generator_adjacency_matrix_;
};

TEST_F(TestCliqueGenerator, Ctor) {
  EXPECT_EQ(dut().vertices().size(), 10);
  EXPECT_GT(dut().adjacency_matrix().nonZeros(), 0);
  EXPECT_LE(dut().adjacency_matrix().nonZeros(), 90);
}

TEST_F(TestCliqueGenerator, CalcCliqueEllipsoidAroundConfig) {
  const auto q = Eigen::Vector2d {0.0, 0.2};
  auto clique_ellipsoid = dut().CalcCliqueEllipsoidAroundConfig(q);
  EXPECT_TRUE(clique_ellipsoid.has_value());
  // Test with a big existing set
  drake::geometry::optimization::ConvexSets existing_convex_sets;
  // Make a box
  auto box_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {M_PI / 3, 0.2}, Eigen::Vector2d {M_PI, 0.4});
  auto box_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {-M_PI, 0.2}, Eigen::Vector2d {-M_PI / 3, 0.4});
  existing_convex_sets.emplace_back(box_1.Clone());
  existing_convex_sets.emplace_back(box_2.Clone());
  auto clique_ellipsoid_with_existing =
      dut().CalcCliqueEllipsoidAroundConfig(q, existing_convex_sets);
  EXPECT_TRUE(clique_ellipsoid_with_existing.has_value());
}

TEST_F(TestCliqueGenerator, CalcCliqueEllipsoidAroundEdge) {
  const auto q1 = Eigen::Vector2d {0.0, 0.2};
  const auto q2 = Eigen::Vector2d {M_PI / 2, 0.3};
  auto clique_ellipsoid = dut().CalcCliqueEllipsoidAroundEdge({q1, q2});
  EXPECT_TRUE(clique_ellipsoid.has_value());
  // Test with a big existing set
  drake::geometry::optimization::ConvexSets existing_convex_sets;
  // Make a box
  auto box_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {M_PI / 3, 0.2}, Eigen::Vector2d {M_PI, 0.4});
  auto box_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {-M_PI, 0.2}, Eigen::Vector2d {-M_PI / 3, 0.4});
  existing_convex_sets.emplace_back(box_1.Clone());
  existing_convex_sets.emplace_back(box_2.Clone());
  auto clique_ellipsoid_with_existing =
      dut().CalcCliqueEllipsoidAroundEdge({q1, q2}, existing_convex_sets);
  EXPECT_TRUE(clique_ellipsoid_with_existing.has_value());
}

TEST_F(TestCliqueGenerator, CalcCliqueEllipsoidAroundConfigWithVertexIndex) {
  // Test with a valid vertex index
  int vertex_index = 0;
  auto clique_ellipsoid =
      dut_adjacency_matrix().CalcCliqueEllipsoidAroundConfig(vertex_index);
  EXPECT_TRUE(clique_ellipsoid.has_value());

  // Test with existing convex sets
  drake::geometry::optimization::ConvexSets existing_convex_sets;
  auto box_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {M_PI / 3, 0.2}, Eigen::Vector2d {M_PI, 0.4});
  auto box_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {-M_PI, 0.2}, Eigen::Vector2d {-M_PI / 3, 0.4});
  existing_convex_sets.emplace_back(box_1.Clone());
  existing_convex_sets.emplace_back(box_2.Clone());

  auto clique_ellipsoid_with_existing =
      dut_adjacency_matrix().CalcCliqueEllipsoidAroundConfig(
          vertex_index, existing_convex_sets);
  EXPECT_TRUE(clique_ellipsoid_with_existing.has_value());

  // Test with invalid vertex indices
  EXPECT_THROW(dut_adjacency_matrix().CalcCliqueEllipsoidAroundConfig(-1),
               std::exception);
  EXPECT_THROW(dut_adjacency_matrix().CalcCliqueEllipsoidAroundConfig(
                   dut_adjacency_matrix().vertices().size()),
               std::exception);
}

TEST_F(TestCliqueGenerator, CalcCliqueEllipsoidAroundEdgeWithVertexIndices) {
  // Test with valid vertex indices
  std::pair<int, int> vertex_indices = {0, 1};
  auto clique_ellipsoid =
      dut_adjacency_matrix().CalcCliqueEllipsoidAroundEdge(vertex_indices);
  // Note: This might return nullopt if vertices 0 and 1 are not connected
  // in the adjacency matrix, which is fine for this test

  // Test with existing convex sets
  drake::geometry::optimization::ConvexSets existing_convex_sets;
  auto box_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {M_PI / 3, 0.2}, Eigen::Vector2d {M_PI, 0.4});
  auto box_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d {-M_PI, 0.2}, Eigen::Vector2d {-M_PI / 3, 0.4});
  existing_convex_sets.emplace_back(box_1.Clone());
  existing_convex_sets.emplace_back(box_2.Clone());

  auto clique_ellipsoid_with_existing =
      dut_adjacency_matrix().CalcCliqueEllipsoidAroundEdge(
          vertex_indices, existing_convex_sets);
  // This also might return nullopt depending on connectivity

  // Test with invalid vertex indices
  std::pair<int, int> invalid_vertex_indices = {-1, 0};
  EXPECT_THROW(dut_adjacency_matrix().CalcCliqueEllipsoidAroundEdge(
                   invalid_vertex_indices),
               std::exception);
  invalid_vertex_indices = {0, dut_adjacency_matrix().vertices().size()};
  EXPECT_THROW(dut_adjacency_matrix().CalcCliqueEllipsoidAroundEdge(
                   invalid_vertex_indices),
               std::exception);
  invalid_vertex_indices = {0, 0};
  EXPECT_THROW(dut_adjacency_matrix().CalcCliqueEllipsoidAroundEdge(
                   invalid_vertex_indices),
               std::exception);
}

TEST_F(TestCliqueGenerator,
       CalcCliqueEllipsoidAroundEdgeWithVertexIndicesConnectivity) {
  // Find two connected vertices in the adjacency matrix
  const auto& adj_matrix = dut_adjacency_matrix().adjacency_matrix();
  std::pair<int, int> connected_vertices = {-1, -1};

  // Search for a connected pair
  for (int i = 0; i < adj_matrix.rows() && connected_vertices.first == -1;
       ++i) {
    for (int j = i + 1; j < adj_matrix.cols(); ++j) {
      if (adj_matrix.coeff(i, j)) {
        connected_vertices = {i, j};
        break;
      }
    }
  }

  if (connected_vertices.first != -1) {
    // Test with connected vertices
    auto clique_ellipsoid =
        dut_adjacency_matrix().CalcCliqueEllipsoidAroundEdge(
            connected_vertices);
    // Should succeed or fail gracefully based on available vertices

    // Test the same pair with empty existing sets
    drake::geometry::optimization::ConvexSets empty_sets;
    auto clique_ellipsoid_empty =
        dut_adjacency_matrix().CalcCliqueEllipsoidAroundEdge(connected_vertices,
                                                             empty_sets);
    // Should have same result as above
    EXPECT_EQ(clique_ellipsoid.has_value(), clique_ellipsoid_empty.has_value());
  }
}

}  // namespace iris
}  // namespace motion
