#pragma once

#include <drake/common/copyable_unique_ptr.h>
#include <drake/geometry/optimization/geodesic_convexity.h>
#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>

#include "convex_hull_gcs.h"
#include "internal/graph.h"
#include "planning_service/motion/iris/iris_inspector.h"

namespace motion {
namespace planning {

/** Determines the type of connecting set to use when the connecting
 configuration is outside of the GCS regions and there is a visible point inside
 the GCS regions. */
enum class ConnectingSetType {
  /** Connects two configurations with a VPolytope that two vertices are
   the connecting configuration and the visible point. */
  kVPolytope,

  /** Connects two configurations with a narrow box the main diagonal of which
   is the line segment between the two configurations. */
  kNarrowBox,

  /** Connects two configurations with a narrow ellipsoid the main diagonal of
   which is the line segment between the two configurations. */
  kNarrowEllipsoid,
};

struct GcsPlannerOptions {
  drake::geometry::optimization::GraphOfConvexSetsOptions drake_gcs_options;

  /** The type of connecting set to use when connecting two configurations.
  0 - VPolytope
    1 - NarrowBox
    2 - NarrowEllipsoid */
  int connecting_set_type {1};

  /** The epsilon used to inflate the connecting set. This is to ensure that the
   connecting set has a non-zero measure intersection with the GCS to avoid
   numerical issues. The exact implementation depends on the connecting set
   type. */
  double connection_set_epsilon {1e-2};

  /** Whether to use convex hulls of intersections for planning. */
  bool use_convex_hull_gcs {false};

  /** The type of penalization to use for the connecting set.
   0 - Energy
   1 - Length */
  int cost_type {0};

  /** Whether to lazily add edges in the graph of configs. This can
   significantly speed up the loading time of the planner, but may result in
   slower computation of paths. */
  bool lazy_gcc_edges {true};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(drake_gcs_options));
    a->Visit(DRAKE_NVP(connecting_set_type));
    a->Visit(DRAKE_NVP(connection_set_epsilon));
    a->Visit(DRAKE_NVP(use_convex_hull_gcs));
    a->Visit(DRAKE_NVP(cost_type));
    a->Visit(DRAKE_NVP(lazy_gcc_edges));
  }
};

class TestGcsPlannerBase;

/**
Gcs planner for a fixed set of constraints. The planner is constructed from a
set of iris regions and a set of robot constraints. The planner can be used to
compute a path between two configurations that satisfies the constraints.

@param robot_constraints: robot with constraints
@param iris_regions_adapter: iris regions
@param joint_velocity_bound: joint velocity bound
@param options: options for the planner
*/
class SingleModeGcsPlanner {
 public:
  /** GcsPlanner is constructed from a set of iris regions */
  SingleModeGcsPlanner(const RobotConstraints& robot_constraints,
                       const iris::IrisRegionsAdapter& iris_regions_adapter,
                       const Eigen::VectorXd& joint_velocity_bound,
                       const GcsPlannerOptions& options = GcsPlannerOptions());

  /** Calculate the optimal path between two configurations
  @param q1: start configuration
    @param q2: end configuration
    @return: optimal path between q1 and q2, if one exists.
    */
  std::optional<drake::trajectories::CompositeTrajectory<double>>
  CalcOptimalPath(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2);

  /** Calculate the optimal path between two configurations using the convex
  hulls of intersections. While this may increase the length of resulting
  trajectories, it is likely to produce trajectories in much shorter time
  @param q1: start configuration
    @param q2: end configuration
    @return: optimal path between q1 and q2, if one exists.
    */
  std::optional<drake::trajectories::CompositeTrajectory<double>>
  CalcOptimalPathCHulls(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2);

  /** Calculates a fast estimate of the path length between two configurations.
   * @param q1: start configuration
   * @param q2: end configuration
   * @return: the estimated path length (guaranteed to be an upper bound) if
   * successful, otherwise std::nullopt.
   */
  std::optional<double> FastEstimatePathLength(const Eigen::VectorXd& q1,
                                               const Eigen::VectorXd& q2);

  /** Return the underlying metric in the graph of configs. */
  // ToDo(@sadra): Since this is being used in the public API, we should
  // move it from the internal namespace to a public namespace.
  const internal::GraphOfConfigs::DistanceFunc& metric() const {
    return graph_of_configs_->metric();
  }

 private:
  /** helper class for unit testing private methods */
  friend class TestGcsPlannerBase;

  enum class TerminalType { kStart, kEnd };

  // The following structure is not used in the public API.
  struct TerminalNode {
    // Pointer to the subgraph of the configuration in the graph of convex sets
    drake::planning::trajectory_optimization::GcsTrajectoryOptimization::
        Subgraph* q_subgraph {nullptr};

    // Pointer to the subgraph of visible line graph of convex sets
    drake::planning::trajectory_optimization::GcsTrajectoryOptimization::
        Subgraph* visible_gcs_subgraph {nullptr};

    // Pointer to the subgraph of visible line graph of convex sets
    std::unique_ptr<drake::geometry::optimization::ConvexSet> visible_gcs_set {
        nullptr};  // Corrected initialization

    // Pointer to the vertex of the configuration in the graph of configs
    drake::geometry::optimization::GraphOfConvexSets::Vertex* q_gcc {nullptr};

    // Pointer to the vertex of the visible configuration in the graph of
    // configs
    drake::geometry::optimization::GraphOfConvexSets::Vertex* q_visible_gcc {
        nullptr};

    // Visible region index, used to reconstruct the path
    std::optional<int> visible_region_index {std::nullopt};
  };

  // Clean up the terminal nodes
  void RemoveTerminalNode(TerminalNode* terminal_node);

  // Go from the graph of configs to the graph of convex sets
  std::vector<const drake::geometry::optimization::GraphOfConvexSets::Vertex*>
  ConvertToGcsVertices(
      const std::vector<
          const drake::geometry::optimization::GraphOfConvexSets::Vertex*>&
          path,
      const TerminalNode& start_terminal,
      const TerminalNode& end_terminal) const;

  // Go from the graph of configs to the graph of convex sets
  std::vector<int> ConvertToCHullVertices(
      const std::vector<
          const drake::geometry::optimization::GraphOfConvexSets::Vertex*>&
          path) const;

  static std::unique_ptr<drake::geometry::optimization::ConvexSet>
  CreateConnectingSet(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
                      const ConnectingSetType& connecting_set_type,
                      const double epsilon);

  // Add a terminal node to the graph of convex sets and configs
  TerminalNode AddTerminal(const Eigen::VectorXd& q,
                           const TerminalType& terminal_type);

  // Add a connecting set for a visible point outside of the GCS
  TerminalNode DoAddConnectingSet(
      const Eigen::VectorXd& q,
      const std::pair<int, Eigen::VectorXd>& visible_result,
      const TerminalType& terminal_type);

  // Add a configuration that is in the interior of a region
  TerminalNode DoAddConfig(const std::vector<int>& region_indices,
                           const Eigen::VectorXd& q,
                           const TerminalType& terminal_type);

  std::optional<
      std::vector<drake::geometry::optimization::GraphOfConvexSets::Vertex*>>
  CalcOptimalPathVertices(const Eigen::VectorXd& q1,
                          const Eigen::VectorXd& q2) const;

  std::unique_ptr<
      drake::planning::trajectory_optimization::GcsTrajectoryOptimization>
      gcs_traj_opt_;

  const RobotConstraints& robot_constraints_;
  const iris::IrisRegionsAdapter& iris_regions_adapter_;
  std::unique_ptr<iris::IrisInspector> iris_inspector_;
  GcsPlannerOptions gcs_options_;
  std::unique_ptr<planning::GraphOfConvexHulls> convex_hull_gcs_;
  std::unique_ptr<internal::GraphOfConfigs> graph_of_configs_;
  std::map<const drake::geometry::optimization::GraphOfConvexSets::Vertex*,
           std::pair<int, int>>
      vertex_to_intersection_;
  // Function that adds edge to a graph of configs vertex
  std::function<void(drake::geometry::optimization::GraphOfConvexSets::Vertex*)>
      add_edges_func_;
};

}  // namespace planning
}  // namespace motion
