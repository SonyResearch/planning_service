/*
 * Copyright © 2024 Sony Research. All rights reserved.
 */

/// @file clique_generator.h

#pragma once

#include <drake/common/random.h>
#include <drake/common/type_safe_index.h>
#include <drake/geometry/optimization/convex_set.h>
#include <drake/geometry/optimization/hyperellipsoid.h>
#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/planning/graph_algorithms/max_clique_solver_via_greedy.h>

#include "planning_service/motion/robot_constraints.h"

namespace motion {
namespace iris {

class CliqueGenerator {
 public:
  /** @brief Constructor for CliqueGenerator
    * @param robot_constraints The robot constraints to use for collision
   checking
    * @param configurations The configurations to use for generating the clique
    * @param adjacency_matrix Optional adjacency matrix to use instead of
   collision
    * checking
    * @param step The step size to use for collision checking edges
    * If the adjacency matrix is provided, this parameter is ignored.
    */
  CliqueGenerator(
      const RobotConstraints& robot_constraints,
      const std::vector<Eigen::VectorXd>& configurations,
      std::optional<Eigen::SparseMatrix<bool>> adjacency_matrix = std::nullopt,
      double step = 0.005);

  /**
   * @brief Calculate a clique ellipsoid around a configuration specified by
   * vertex index. Uses the adjacency matrix to determine connectivity instead
   * of collision checking.
   *
   * @param vertex_index Index of the vertex in the vertices_ array
   * @param existing_convex_sets Existing convex sets to avoid overlap
   * @return std::optional<drake::geometry::optimization::Hyperellipsoid>
   */
  std::optional<drake::geometry::optimization::Hyperellipsoid>
  CalcCliqueEllipsoidAroundConfig(
      int vertex_index,
      const drake::geometry::optimization::ConvexSets& existing_convex_sets =
          drake::geometry::optimization::ConvexSets()) const;

  /**
   * @brief Calculate a clique ellipsoid around an edge specified by vertex
   * indices. Uses the adjacency matrix to determine connectivity instead of
   * collision checking.
   *
   * @param vertex_indices Pair of vertex indices in the vertices_ array
   * @param existing_convex_sets Existing convex sets to avoid overlap
   * @return std::optional<drake::geometry::optimization::Hyperellipsoid>
   */
  std::optional<drake::geometry::optimization::Hyperellipsoid>
  CalcCliqueEllipsoidAroundEdge(
      const std::pair<int, int>& vertex_indices,
      const drake::geometry::optimization::ConvexSets& existing_convex_sets =
          drake::geometry::optimization::ConvexSets()) const;

  /** Grows a clique around a given configuration and returns a hyperellipsoid
   * that contains the clique.
   * @param q the configuration to grow the clique around
   * @param min_clique_size the minimum size of the clique to grow
   * @return the clique that was grown, or an empty matrix if no clique was
   * found.
   */
  std::optional<drake::geometry::optimization::Hyperellipsoid>
  CalcCliqueEllipsoidAroundConfig(
      const Eigen::VectorXd& q,
      const drake::geometry::optimization::ConvexSets& existing_convex_sets =
          drake::geometry::optimization::ConvexSets()) const;

  std::optional<drake::geometry::optimization::Hyperellipsoid>
  CalcCliqueEllipsoidAroundEdge(
      const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
      const drake::geometry::optimization::ConvexSets& existing_convex_sets =
          drake::geometry::optimization::ConvexSets()) const;

  const std::vector<Eigen::VectorXd>& vertices() const {
    return vertices_;
  }

  const Eigen::SparseMatrix<bool>& adjacency_matrix() const {
    return adjacency_matrix_;
  }

 private:
  const RobotConstraints& robot_constraints_;
  const std::vector<Eigen::VectorXd> vertices_;
  const Eigen::SparseMatrix<bool> adjacency_matrix_;
};

}  // namespace iris
}  // namespace motion
