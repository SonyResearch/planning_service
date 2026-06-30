#include "planning_service/motion/planning/internal/graph.h"

#include "planning_service/common/logging.h"
#include "planning_service/motion/planning/internal/geodesic_math.h"

namespace motion {
namespace internal {

using Vertex = drake::geometry::optimization::GraphOfConvexSets::Vertex;
using Edge = drake::geometry::optimization::GraphOfConvexSets::Edge;
using ConvexSet = drake::geometry::optimization::ConvexSet;
using Point = drake::geometry::optimization::Point;

using DistanceFunc = GraphOfConfigs::DistanceFunc;
using MetricType = GraphOfConfigs::MetricType;

DistanceFunc DistanceFunc::L2NormMetric(
    const Eigen::VectorXd& weights,
    std::vector<int> continuous_revolute_joint_indices) {
  return DistanceFunc(
      weights.rows(), [weights, continuous_revolute_joint_indices](
                          const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        auto a_copy {a};
        for (const auto& index : continuous_revolute_joint_indices) {
          const double wrap_multiple = CalcWrapMultiple(a(index), b(index));
          a_copy(index) += wrap_multiple * 2.0 * M_PI;
        }
        return (a_copy - b).cwiseProduct(weights).norm();
      });
}

DistanceFunc DistanceFunc::LInfNormMetric(
    const Eigen::VectorXd& weights,
    std::vector<int> continuous_revolute_joint_indices) {
  return DistanceFunc(
      weights.rows(), [weights, continuous_revolute_joint_indices](
                          const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        auto a_copy {a};
        for (const auto& index : continuous_revolute_joint_indices) {
          const double wrap_multiple = CalcWrapMultiple(a(index), b(index));
          a_copy(index) += wrap_multiple * 2.0 * M_PI;
        }
        return (a_copy - b).cwiseProduct(weights).lpNorm<Eigen::Infinity>();
      });
}

DistanceFunc DistanceFunc::L1NormMetric(
    const Eigen::VectorXd& weights,
    std::vector<int> continuous_revolute_joint_indices) {
  return DistanceFunc(
      weights.rows(), [weights, continuous_revolute_joint_indices](
                          const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        auto a_copy {a};
        for (const auto& index : continuous_revolute_joint_indices) {
          const double wrap_multiple = CalcWrapMultiple(a(index), b(index));
          a_copy(index) += wrap_multiple * 2.0 * M_PI;
        }
        return (a_copy - b).cwiseProduct(weights).lpNorm<1>();
      });
}

namespace {
void strongconnect(const Vertex* vertex, const GraphOfConfigs& graph,
                   std::vector<const Vertex*>& stack, int* counter,
                   std::map<const Vertex*, int>& index,
                   std::map<const Vertex*, bool>& on_stack,
                   std::map<const Vertex*, int>& lowlink,
                   std::vector<std::vector<const Vertex*>>& scc_vec) {
  index[vertex] = *counter;
  lowlink[vertex] = *counter;
  *counter = *counter + 1;
  on_stack[vertex] = true;
  stack.push_back(vertex);
  for (const auto& edge : vertex->outgoing_edges()) {
    const auto* neighbor = &(edge->v());
    if (index.count(neighbor) == 0) {
      strongconnect(neighbor, graph, stack, counter, index, on_stack, lowlink,
                    scc_vec);
      lowlink.at(vertex) = std::min(lowlink.at(vertex), lowlink.at(neighbor));
    } else if (on_stack.at(neighbor)) {
      lowlink.at(vertex) = std::min(lowlink.at(vertex), index.at(neighbor));
    }
  }
  if (lowlink.at(vertex) == index.at(vertex)) {
    std::vector<const Vertex*> scc;
    while (true) {
      const auto* w = stack.back();
      on_stack[w] = false;
      scc.push_back(w);
      stack.pop_back();
      if (w == vertex) {
        break;
      }
    }
    scc_vec.push_back(scc);
  }
}
}  // namespace

namespace {
DistanceFunc MakeDistanceFunction(
    const int dim, const std::vector<int>& continuous_revolute_joint_indices,
    const std::optional<Eigen::VectorXd>& weights, MetricType metric_type) {
  const Eigen::VectorXd weights_copy =
      weights.value_or(Eigen::VectorXd::Ones(dim));
  DRAKE_THROW_UNLESS(dim > 0);
  DRAKE_THROW_UNLESS(weights_copy.rows() == dim);
  switch (metric_type) {
    case MetricType::kL1Norm:
      return DistanceFunc::L1NormMetric(weights_copy,
                                        continuous_revolute_joint_indices);
    case MetricType::kL2Norm:
      return DistanceFunc::L2NormMetric(weights_copy,
                                        continuous_revolute_joint_indices);
    case MetricType::kLInfNorm:
      return DistanceFunc::LInfNormMetric(weights_copy,
                                          continuous_revolute_joint_indices);
    default:
      throw std::runtime_error("Unknown metric type");
  }
}
}  // namespace

GraphOfConfigs::GraphOfConfigs(
    const int dim, std::vector<int> continuous_revolute_joint_indices,
    const std::optional<Eigen::VectorXd>& weights, MetricType metric_type)
    : drake::geometry::optimization::GraphOfConvexSets(),
      dim_(dim),
      distance_func_ {MakeDistanceFunction(
          dim, continuous_revolute_joint_indices, weights, metric_type)} {
  DRAKE_DEMAND(dim_ > 0);
  DRAKE_DEMAND(distance_func_.dim_ == dim_);
}

Vertex* GraphOfConfigs::AddVertex(const ConvexSet& set, std::string name) {
  const auto* point = dynamic_cast<const Point*>(&set);
  if (point == nullptr) {
    throw std::runtime_error("GraphOfConfigs only supports Point vertices");
  }
  return drake::geometry::optimization::GraphOfConvexSets::AddVertex(*point,
                                                                     name);
}

Edge* GraphOfConfigs::AddEdge(Vertex* start, Vertex* goal, std::string name) {
  const auto start_config_opt = start->set().MaybeGetPoint();
  DRAKE_DEMAND(start_config_opt.has_value());
  const auto& start_config = start_config_opt.value();
  const auto goal_config_opt = goal->set().MaybeGetPoint();
  DRAKE_DEMAND(goal_config_opt.has_value());
  const auto& goal_config = goal_config_opt.value();
  const double cost = distance_func_(start_config, goal_config);
  auto* edge = drake::geometry::optimization::GraphOfConvexSets::AddEdge(
      start, goal, name);
  edge_cost_[edge] = cost;
  return edge;
}

double GraphOfConfigs::GetEdgeCost(
    const drake::geometry::optimization::GraphOfConvexSets::Edge* edge) const {
  return edge_cost_.at(edge);
}

std::vector<std::vector<
    const drake::geometry::optimization::GraphOfConvexSets::Vertex*>>
GraphOfConfigs::CalcStronglyConnectedComponents() const {
  // Using Tarjan's algorithm.
  // Check
  // https://en.wikipedia.org/wiki/Tarjan%27s_strongly_connected_components_algorithm
  std::vector<std::vector<const Vertex*>> sccs;
  int counter = 0;
  std::map<const Vertex*, int> index;
  std::vector<const Vertex*> stack;
  std::map<const Vertex*, bool> on_stack;
  std::map<const Vertex*, int> lowlink;
  // now let's implement the algorithm
  for (const auto* vertex : Vertices()) {
    if (index.count(vertex) == 0) {
      strongconnect(vertex, *this, stack, &counter, index, on_stack, lowlink,
                    sccs);
    }
  }
  return sccs;
}

std::pair<double,
          std::vector<
              const drake::geometry::optimization::GraphOfConvexSets::Vertex*>>
GraphOfConfigs::CalcShortestPath(
    const drake::geometry::optimization::GraphOfConvexSets::Vertex* start,
    const drake::geometry::optimization::GraphOfConvexSets::Vertex* goal,
    std::function<
        void(drake::geometry::optimization::GraphOfConvexSets::Vertex*)>*
        add_edges_func) const {
  // print the graph
  logging::log()->trace("GraphOfConfigs:CalcShortestPath: Graph: {}",
                        GetGraphvizString());
  const auto now = std::chrono::system_clock::now();
  std::vector<const Vertex*> open_set {start};
  std::map<const Vertex*, const Vertex*> came_from;
  std::map<const Vertex*, double> g_score;
  std::map<const Vertex*, double> f_score;
  for (const auto* vertex : Vertices()) {
    g_score[vertex] = std::numeric_limits<double>::infinity();
    f_score[vertex] = std::numeric_limits<double>::infinity();
  }
  const auto start_config_opt = start->set().MaybeGetPoint();
  DRAKE_DEMAND(start_config_opt.has_value());
  const auto& start_config = start_config_opt.value();
  const auto goal_config_opt = goal->set().MaybeGetPoint();
  DRAKE_DEMAND(goal_config_opt.has_value());
  const auto& goal_config = goal_config_opt.value();
  f_score[start] = distance_func_(start_config, goal_config);
  g_score[start] = 0.0;
  while (!open_set.empty()) {
    // get the vertex with the lowest f_score
    const Vertex* current = nullptr;
    double min_f_score = std::numeric_limits<double>::infinity();
    for (const auto* vertex : open_set) {
      logging::log()->trace(
          "GraphOfConfigs:CalcShortestPath: Evaluating node: {}",
          vertex->name());
      if (f_score[vertex] < min_f_score) {
        min_f_score = f_score[vertex];
        current = vertex;
        logging::log()->trace(
            "GraphOfConfigs:CalcShortestPath: Adding node: {}",
            current->name());
      }
    }
    if (current == goal) {
      std::vector<const Vertex*> path;
      const Vertex* v = goal;
      while (v != start) {
        path.push_back(v);
        v = came_from.at(v);
      }
      path.push_back(start);
      std::reverse(path.begin(), path.end());
      const auto end = std::chrono::system_clock::now();
      logging::log()->info(
          "GraphOfConfigs:CalcShortestPath: A* Found path in {} ms",
          std::chrono::duration_cast<std::chrono::milliseconds>(end - now)
              .count());
      return {g_score[goal], path};
    }
    open_set.erase(std::remove(open_set.begin(), open_set.end(), current),
                   open_set.end());
    if (add_edges_func && *add_edges_func) {
      (*add_edges_func)(const_cast<Vertex*>(current));
    }
    for (const auto* edge : current->outgoing_edges()) {
      const Vertex* neighbor = &(edge->v());
      double tentative_g_score = g_score[current] + GetEdgeCost(edge);
      if (tentative_g_score < g_score[neighbor]) {
        came_from[neighbor] = current;
        g_score[neighbor] = tentative_g_score;
        const auto neighbor_config_opt = neighbor->set().MaybeGetPoint();
        DRAKE_DEMAND(neighbor_config_opt.has_value());
        const auto& neighbor_config = neighbor_config_opt.value();
        f_score[neighbor] =
            g_score[neighbor] + distance_func_(neighbor_config, goal_config);
        if (std::find(open_set.begin(), open_set.end(), neighbor)
            == open_set.end()) {
          open_set.push_back(neighbor);
        }
      }
    }
  }
  throw std::runtime_error("No path found");
}

}  // namespace internal
}  // namespace motion
