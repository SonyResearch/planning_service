#pragma once

#include <drake/geometry/optimization/convex_hull.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/point.h>
#include <drake/planning/trajectory_optimization/gcs_trajectory_optimization.h>

#include <map>
#include <optional>
#include <set>
#include <tuple>
#include <vector>

#include "planning_service/motion/iris/iris_inspector.h"

namespace motion {
namespace planning {

using ConvexHull = drake::geometry::optimization::ConvexHull;
using Subgraph = drake::planning::trajectory_optimization::
    GcsTrajectoryOptimization::Subgraph;
/** This is an external class used for managing the construction of convex hulls
 * of intersections, as well as determining trajectories through these convex
 * hulls. The planner is constructed from an iris_regions_adapter from which the
 * intersections vector must be provided. If no intersections are found int the
 * intersections vector, null trajectories will be returned.
 * @param iris_regions_adapter contains the vector of intersections from which
 * convex hulls are constructed */
class GraphOfConvexHulls {
 public:
  /** Light wrapper class used to describe a convex hull. In our cases, all
   * convex hulls are constructed from three regions or two intersections which
   * share an intermediate region. In order to construct a convex hull through a
   * convex set, two intersections must share a convex set.
   * @param convex_hull the convex hull being stored
   * @param start_index index of the edge convex
   * set (must be smaller than end_index)
   * @param mid_index the index of the
   * convex set that both intersections from which the convex hull was
   * constructed share
   * @param end_index the other index of the edge convex set
   * (must be larger than start_index)
   * @param add_index the index of the convex
   * hull as a whole (order of construction) */
  class ConvexHullUnit {
   public:
    ConvexHullUnit(const ConvexHull convex_hull, const int start_index,
                   const int mid_index, const int end_index,
                   const int add_index)
        : convex_hull_(convex_hull),
          start_index_(start_index),
          mid_index_(mid_index),
          end_index_(end_index),
          add_index_(add_index) {}

    const ConvexHull& convex_hull() const {
      return convex_hull_;
    }
    int start_index() const {
      return start_index_;
    }
    int mid_index() const {
      return mid_index_;
    }
    int end_index() const {
      return end_index_;
    }
    int add_index() const {
      return add_index_;
    }

   private:
    const ConvexHull convex_hull_;
    const int start_index_;
    const int mid_index_;
    const int end_index_;
    const int add_index_;
  };

  // Constructor
  GraphOfConvexHulls(const iris::IrisRegionsAdapter& iris_regions_adapter);

  /** Solves a convex restriction from q1 to q2 through the sets specified in
   * desired_sets.
   * @param q1 the starting sets (configuration and visibility)
   * @param q2 the ending sets (configuration and visibility)
   * @param desired_sets the ordered indices of convex sets through which the
   * trajectory will pass
   * @returns trajectory if one can be found
   * pre-condition: desired sets must use the indices from the iris_region
   * adapter to describe the desired sets
   */
  std::optional<drake::trajectories::CompositeTrajectory<double>>
  SolveConvexRestriction(const drake::geometry::optimization::ConvexSets& q1,
                         const drake::geometry::optimization::ConvexSets& q2,
                         std::vector<int> desired_sets);

 private:
  /** Helper function to construct the adjacency matrix for our convex hulls */
  void ConstructCHullAdj();

  /** Add a terminal node to the graph (will be constructed as a point and a
   * convex hull containing the point and the desired_intersection)
   * @param q convex sets the first of which is the starting terminal and the
   * remainder of which may be visibility graphs
   * @param desired_intersection the intersection (specified through indices) to
   * which q is connected through a convex set
   * @param start true if this is a start terminal, false if this is an end
   * terminal
   */
  void AddTerminalNode(drake::geometry::optimization::ConvexSets q,
                       const std::pair<int, int> desired_intersection,
                       const bool start);

  /** Add a terminal node to the graph (will be constructed as a point and a
   * convex hull containing the point and the desired_intersection where
   * desired_intersection represents a convex hull)
   * @param q convex sets the first of which is the starting terminal and the
   * remainder of which may be visibility graphs
   * @param desired_intersection the convex hull that the starting terminal is
   * connected to
   * @param start true if this is a start terminal, false if this is an end
   * terminal
   */
  void AddTerminalNode(drake::geometry::optimization::ConvexSets q,
                       const std::tuple<int, int, int> desired_intersection,
                       const bool start);
  /** Removes all nodes that are not the primary convex hull subgraph */
  void RemTerminalNode();

  /** Helper function used to update the index mapping in the gcs_traj_opt
   * subgraph list */
  void UpdateSubgraphMap();

  std::unique_ptr<
      drake::planning::trajectory_optimization::GcsTrajectoryOptimization>
      gcs_traj_opt_;
  const iris::IrisRegionsAdapter& iris_regions_adapter_;
  std::vector<ConvexHullUnit> convex_hulls_;
  std::map<std::tuple<int, int, int>, int> index_mapping_;
  std::set<std::tuple<int, int, int>> key_set_;
  std::vector<std::vector<int>> adj_matrix_;
  std::vector<std::pair<int, int>> adj_list_;
  std::map<std::string, int> subgraph_to_index_;
  Subgraph* main_regions_;
  Subgraph* start_region_;  // The starting region (convex hull) that connects
                            // the starting q to the convex hull graph. Stored
                            // so that a directed edge can be added from the
                            // start region to the end region (in the case that
                            // no additional convex hulls are needed)
};

}  // namespace planning
}  // namespace motion
