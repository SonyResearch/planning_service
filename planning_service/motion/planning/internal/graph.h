#pragma once

#include <Eigen/Dense>
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/point.h>

#include <map>
#include <vector>

namespace motion {
namespace internal {

class GraphOfConfigs
    : private drake::geometry::optimization::GraphOfConvexSets {
 public:
  class DistanceFunc {
   public:
    double operator()(const Eigen::VectorXd& a,
                      const Eigen::VectorXd& b) const {
      DRAKE_THROW_UNLESS(a.rows() == dim_);
      DRAKE_THROW_UNLESS(b.rows() == dim_);
      return distance_func_(a, b);
    }

    static DistanceFunc L2NormMetric(
        const Eigen::VectorXd& weights,
        std::vector<int> continuous_revolute_joint_indices = {});

    static DistanceFunc LInfNormMetric(
        const Eigen::VectorXd& weights,
        std::vector<int> continuous_revolute_joint_indices = {});

    static DistanceFunc L1NormMetric(
        const Eigen::VectorXd& weights,
        std::vector<int> continuous_revolute_joint_indices = {});

   private:
    DistanceFunc(
        const int dim,
        std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>
            distance_func)
        : dim_(dim), distance_func_(distance_func) {}

    const int dim_;
    const std::function<double(const Eigen::VectorXd&, const Eigen::VectorXd&)>
        distance_func_;

    friend class GraphOfConfigs;
  };

  enum class MetricType { kL1Norm, kL2Norm, kLInfNorm };

  GraphOfConfigs(const int dim,
                 std::vector<int> continuous_revolute_joint_indices = {},
                 const std::optional<Eigen::VectorXd>& weights = std::nullopt,
                 MetricType metric_type = MetricType::kL2Norm);

  drake::geometry::optimization::GraphOfConvexSets::Vertex* AddVertex(
      const drake::geometry::optimization::ConvexSet& set,
      std::string name = "");

  drake::geometry::optimization::GraphOfConvexSets::Edge* AddEdge(
      drake::geometry::optimization::GraphOfConvexSets::Vertex* start,
      drake::geometry::optimization::GraphOfConvexSets::Vertex* goal,
      std::string name = "");

  double GetEdgeCost(
      const drake::geometry::optimization::GraphOfConvexSets::Edge* edge) const;

  // Returns the strongly connected components of the graph.
  std::vector<std::vector<
      const drake::geometry::optimization::GraphOfConvexSets::Vertex*>>
  CalcStronglyConnectedComponents() const;

  // Returns the shortest path from start to goal using A* algorithm.
  std::pair<double, std::vector<const drake::geometry::optimization::
                                    GraphOfConvexSets::Vertex*>>
  CalcShortestPath(
      const drake::geometry::optimization::GraphOfConvexSets::Vertex* start,
      const drake::geometry::optimization::GraphOfConvexSets::Vertex* goal,
      std::function<
          void(drake::geometry::optimization::GraphOfConvexSets::Vertex*)>*
          add_edges_func = nullptr) const;

  // Return the underlying metric.
  const DistanceFunc& metric() const {
    return distance_func_;
  }

  using drake::geometry::optimization::GraphOfConvexSets::Edges;
  using drake::geometry::optimization::GraphOfConvexSets::GetGraphvizString;
  using drake::geometry::optimization::GraphOfConvexSets::RemoveEdge;
  using drake::geometry::optimization::GraphOfConvexSets::RemoveVertex;
  using drake::geometry::optimization::GraphOfConvexSets::Vertices;

 private:
  const int dim_;
  DistanceFunc distance_func_ {
      DistanceFunc::L2NormMetric(Eigen::VectorXd::Ones(0))};
  std::map<const drake::geometry::optimization::GraphOfConvexSets::Edge*,
           double>
      edge_cost_;
};

}  // namespace internal
}  // namespace motion
