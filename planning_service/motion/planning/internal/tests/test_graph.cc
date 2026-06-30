#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/optimization/hyperrectangle.h>
#include <gtest/gtest.h>

#include "planning_service/common/logging.h"
#include "planning_service/motion/planning/internal/graph.h"

namespace motion {
namespace internal {

TEST(DistanceFunc, Basics) {
  const auto l2_norm_metric =
      GraphOfConfigs::DistanceFunc::L2NormMetric(Eigen::VectorXd::Ones(3));
  const auto l1_norm_metric =
      GraphOfConfigs::DistanceFunc::L1NormMetric(Eigen::VectorXd::Ones(3));
  const auto l_inf_norm_metric =
      GraphOfConfigs::DistanceFunc::LInfNormMetric(Eigen::VectorXd::Ones(3));
  const Eigen::VectorXd a = Eigen::VectorXd::Zero(3);
  const Eigen::VectorXd b = Eigen::VectorXd::Ones(3);
  EXPECT_EQ(l2_norm_metric(a, b), std::sqrt(3));
  EXPECT_EQ(l1_norm_metric(a, b), 3);
  EXPECT_EQ(l_inf_norm_metric(a, b), 1);
  // reverse order
  EXPECT_EQ(l2_norm_metric(b, a), std::sqrt(3));
  EXPECT_EQ(l1_norm_metric(b, a), 3);
  EXPECT_EQ(l_inf_norm_metric(b, a), 1);
  // wrong size
  const Eigen::VectorXd c = Eigen::VectorXd::Zero(4);
  EXPECT_THROW(l2_norm_metric(a, c), std::runtime_error);
  EXPECT_THROW(l1_norm_metric(a, c), std::runtime_error);
  EXPECT_THROW(l_inf_norm_metric(a, c), std::runtime_error);
  // same point should have 0 distance
  EXPECT_EQ(l2_norm_metric(b, b), 0);
  EXPECT_EQ(l1_norm_metric(b, b), 0);
  EXPECT_EQ(l_inf_norm_metric(b, b), 0);
}

TEST(DistanceFunc, WithWrapping) {
  // Declare joint 1 as continuous revolute joint
  const auto l2_norm_metric =
      GraphOfConfigs::DistanceFunc::L2NormMetric(Eigen::VectorXd::Ones(2), {1});
  const auto l1_norm_metric =
      GraphOfConfigs::DistanceFunc::L1NormMetric(Eigen::VectorXd::Ones(2), {1});
  const auto l_inf_norm_metric = GraphOfConfigs::DistanceFunc::LInfNormMetric(
      Eigen::VectorXd::Ones(2), {1});
  const Eigen::Vector2d a(0, 0);
  const Eigen::Vector2d b(0, 2 * M_PI);
  EXPECT_EQ(l2_norm_metric(a, b), 0);
  EXPECT_EQ(l1_norm_metric(a, b), 0);
  EXPECT_EQ(l_inf_norm_metric(a, b), 0);
  const Eigen::Vector2d c(5, 2 * M_PI - 1.0);
  EXPECT_EQ(l2_norm_metric(a, c), std::sqrt(26));
  EXPECT_EQ(l1_norm_metric(a, c), 6);
  EXPECT_EQ(l_inf_norm_metric(a, c), 5);
}

using Vertex = drake::geometry::optimization::GraphOfConvexSets::Vertex;
using Edge = drake::geometry::optimization::GraphOfConvexSets::Edge;
using ConvexSet = drake::geometry::optimization::ConvexSet;
using Point = drake::geometry::optimization::Point;
using Hyperrectangle = drake::geometry::optimization::Hyperrectangle;

TEST(GraphOfConfigs, Basics) {
  auto gcc = GraphOfConfigs(2);
  auto* v1 = gcc.AddVertex(Point(Eigen::Vector2d(0, 0)), "v1");
  auto* v2 = gcc.AddVertex(Point(Eigen::Vector2d(1, 1)), "v2");
  auto* v3 = gcc.AddVertex(Point(Eigen::Vector2d(0, 1)), "v3");
  EXPECT_EQ(gcc.Vertices().size(), 3);
  // Add a vertex that is not a point
  EXPECT_THROW(gcc.AddVertex(Hyperrectangle(Eigen::Vector2d(0, 0),
                                            Eigen::Vector2d(1, 1))),
               std::runtime_error);
  // Add an edge
  gcc.AddEdge(v1, v2);
  gcc.AddEdge(v2, v3);
  EXPECT_EQ(gcc.Edges().size(), 2);
  // Remove a Vertex
  gcc.RemoveVertex(v1);
  EXPECT_EQ(gcc.Vertices().size(), 2);
  EXPECT_EQ(gcc.Edges().size(), 1);
  EXPECT_EQ(v2->outgoing_edges().size(), 1);
  EXPECT_EQ(v2->incoming_edges().size(), 0);
}

TEST(GraphOfConfigs, StronglyConnectedComponents) {
  auto gcc = GraphOfConfigs(2);
  auto* v1 = gcc.AddVertex(Point(Eigen::Vector2d(0, 0)), "v1");
  auto* v2 = gcc.AddVertex(Point(Eigen::Vector2d(1, 1)), "v2");
  auto* v3 = gcc.AddVertex(Point(Eigen::Vector2d(0, 1)), "v3");
  auto* v4 = gcc.AddVertex(Point(Eigen::Vector2d(1, 0)), "v4");
  auto* v5 = gcc.AddVertex(Point(Eigen::Vector2d(2, 0)), "v5");
  gcc.AddEdge(v1, v2);
  gcc.AddEdge(v2, v3);
  gcc.AddEdge(v3, v1);
  gcc.AddEdge(v4, v5);
  gcc.AddEdge(v5, v4);
  const auto sccs = gcc.CalcStronglyConnectedComponents();
  EXPECT_EQ(sccs.size(), 2);
  EXPECT_EQ(sccs[0].size(), 3);
  // It includes v1, v2, and v3
  EXPECT_TRUE(std::find(sccs[0].begin(), sccs[0].end(), v1) != sccs[0].end());
  EXPECT_TRUE(std::find(sccs[0].begin(), sccs[0].end(), v2) != sccs[0].end());
  EXPECT_TRUE(std::find(sccs[0].begin(), sccs[0].end(), v3) != sccs[0].end());
  EXPECT_EQ(sccs[1].size(), 2);
  // It includes v4 and v5
  EXPECT_TRUE(std::find(sccs[1].begin(), sccs[1].end(), v4) != sccs[1].end());
  EXPECT_TRUE(std::find(sccs[1].begin(), sccs[1].end(), v5) != sccs[1].end());
  // Add a cycle
  gcc.AddEdge(v3, v4);
  gcc.AddEdge(v5, v1);
  const auto sccs2 = gcc.CalcStronglyConnectedComponents();
  EXPECT_EQ(sccs2.size(), 1);
  // It includes all vertices
  EXPECT_EQ(sccs2[0].size(), 5);
  // Add a sixth vertex that is alone
  auto* v6 = gcc.AddVertex(Point(Eigen::Vector2d(3, 0)), "v6");
  const auto sccs3 = gcc.CalcStronglyConnectedComponents();
  EXPECT_EQ(sccs3.size(), 2);
  EXPECT_EQ(sccs3[0].size(), 5);
  EXPECT_EQ(sccs3[1].size(), 1);
  EXPECT_TRUE(std::find(sccs3[1].begin(), sccs3[1].end(), v6)
              != sccs3[1].end());
}

TEST(GraphOfConfigs, WeightedEdges) {
  auto gcc = GraphOfConfigs(2, {0}, Eigen::VectorXd::Ones(2),
                            GraphOfConfigs::MetricType::kL2Norm);
  auto* v1 = gcc.AddVertex(Point(Eigen::Vector2d(0, 0)), "v1");
  auto* v2 = gcc.AddVertex(Point(Eigen::Vector2d(3, 4)), "v2");
  auto* v3 = gcc.AddVertex(Point(Eigen::Vector2d(12 * M_PI + 1.0, 0)), "v3");
  auto* edge_12 = gcc.AddEdge(v1, v2);
  auto* edge_13 = gcc.AddEdge(v1, v3);
  auto* edge_32 = gcc.AddEdge(v3, v2);
  EXPECT_EQ(gcc.GetEdgeCost(edge_12), 5);
  // wrapping makes the distance 1
  EXPECT_EQ(gcc.GetEdgeCost(edge_13), 1);
  // joint 0 distance isl 2.0, joint 1 distance is 4.0
  EXPECT_NEAR(gcc.GetEdgeCost(edge_32), std::sqrt(20), 1e-6);
}

TEST(GraphOfConfigs, CalcShortestPath) {
  auto gcc = GraphOfConfigs(2, {0}, Eigen::VectorXd::Ones(2),
                            GraphOfConfigs::MetricType::kL2Norm);
  auto* v1 = gcc.AddVertex(Point(Eigen::Vector2d(0, 0)), "v1");
  auto* v2 = gcc.AddVertex(Point(Eigen::Vector2d(1.0, -1)), "v2");
  auto* v3 = gcc.AddVertex(Point(Eigen::Vector2d(2.0, 0)), "v3");
  auto* v4 = gcc.AddVertex(Point(Eigen::Vector2d(2.0, -1.0)), "v4");
  auto* v5 = gcc.AddVertex(Point(Eigen::Vector2d(3, 2.0)), "v5");
  // adding 12 * M_PI will not make a difference, because it is full rotation
  // and joint 0 is declared as continuous revolute joint
  auto* v6 = gcc.AddVertex(Point(Eigen::Vector2d(3 + 12 * M_PI, -1)), "v6");
  auto* v7 = gcc.AddVertex(Point(Eigen::Vector2d(5, 0)), "v7");
  //             v5
  //          ↗      ↘
  // v1 → → v3         v7
  //  ↘     ↓       ↗
  //   v2 → v4 → v6
  gcc.AddEdge(v1, v2);
  gcc.AddEdge(v1, v3);
  gcc.AddEdge(v3, v5);
  gcc.AddEdge(v3, v4);
  gcc.AddEdge(v5, v7);
  gcc.AddEdge(v2, v4);
  gcc.AddEdge(v4, v6);
  gcc.AddEdge(v6, v7);
  const auto [cost, optimal_path] = gcc.CalcShortestPath(v1, v7);
  // candidate paths:
  std::vector<const Vertex*> path1 = {v1, v3, v5, v7};
  std::vector<const Vertex*> path2 = {v1, v3, v4, v6, v7};
  std::vector<const Vertex*> path3 = {v1, v2, v4, v6, v7};
  auto total_cost = [](const std::vector<const Vertex*>& path,
                       const GraphOfConfigs& gcc) {
    double cost = 0;
    for (int i = 0; i < std::ssize(path) - 1; ++i) {
      const auto* v = path[i];
      const auto* next_v = path[i + 1];
      // verify that the edge exists
      bool edge_exists = false;
      for (const auto* edge : v->outgoing_edges()) {
        if (&(edge->v()) == next_v) {
          cost += gcc.GetEdgeCost(edge);
          edge_exists = true;
          break;
        }
      }
      if (!edge_exists) {
        throw std::runtime_error("Edge does not exist from " + v->name()
                                 + " to " + next_v->name());
      }
    }
    return cost;
  };
  const double cost1 = total_cost(path1, gcc);
  const double cost2 = total_cost(path2, gcc);
  const double cost3 = total_cost(path3, gcc);
  const double optimal_cost = total_cost(optimal_path, gcc);
  // the best path is path 1
  EXPECT_EQ(optimal_cost, cost3);
  // the other paths are not optimal
  EXPECT_LE(optimal_cost, cost1);
  EXPECT_LE(optimal_cost, cost2);
  // The optimal path should have the same cost as the returned cost
  EXPECT_EQ(cost, optimal_cost);
}

}  // namespace internal
}  // namespace motion
