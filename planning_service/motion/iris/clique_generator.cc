#include "clique_generator.h"

namespace motion {
namespace iris {

namespace {
Eigen::SparseMatrix<bool> MakeAdjacencyMatrix(
    const RobotConstraints& robot_constraints,
    const std::vector<Eigen::VectorXd>& configurations, double step) {
  Eigen::SparseMatrix<bool> adjacency_matrix(configurations.size(),
                                             configurations.size());
  int num_pairs =
      std::ssize(configurations) * (std::ssize(configurations) - 1) / 2;
  // Magic formula: the more dimensions, the more logs will be printed.
  double logging_step =
      (20
       - std::min(
           19,
           robot_constraints.robot_model().holonomic_mapping().minimal_dim()))
      / 100.0;
  double percentage_completed = 0.0;
  int progress_pairs = 0;
  for (int i = 0; i < std::ssize(configurations); ++i) {
    for (int j = i + 1; j < std::ssize(configurations); ++j) {
      if (progress_pairs > logging_step * num_pairs) {
        logging::log()->info(
            "CliqueGenerator: MakeEdgesFromConfs: {}% done with {} pairs",
            int(percentage_completed * 100), num_pairs);
        percentage_completed += logging_step;
        progress_pairs = 0;
      }
      if (robot_constraints.CheckSatisfiedEdge(configurations[i],
                                               configurations[j], step)) {
        adjacency_matrix.insert(i, j) = true;
        adjacency_matrix.insert(j, i) = true;  // Add both directions
      }
      ++progress_pairs;
    }
  }
  return adjacency_matrix;
}

std::vector<int> FindUncoveredVertices(
    const std::vector<Eigen::VectorXd>& vertices,
    const drake::geometry::optimization::ConvexSets& existing_convex_sets) {
  std::vector<int> valid_vertices;
  for (int i = 0; i < std::ssize(vertices); ++i) {
    bool is_covered = false;
    for (const auto& convex_set : existing_convex_sets) {
      if (convex_set->PointInSet(vertices[i])) {
        is_covered = true;
        break;
      }
    }
    if (!is_covered) {
      valid_vertices.push_back(i);
    }
  }
  return valid_vertices;
}
}  // namespace

CliqueGenerator::CliqueGenerator(
    const RobotConstraints& robot_constraints,
    const std::vector<Eigen::VectorXd>& configurations,
    std::optional<Eigen::SparseMatrix<bool>> adjacency_matrix, double step)
    : robot_constraints_(robot_constraints),
      vertices_(configurations),
      adjacency_matrix_(
          adjacency_matrix.has_value()
              ? adjacency_matrix.value()
              : MakeAdjacencyMatrix(robot_constraints, configurations, step)) {
  // Demand that the vertices are not empty
  DRAKE_THROW_UNLESS(!vertices_.empty());
  // demand that the adjacency matrix is square and has the same size as
  // the configurations vector
  DRAKE_DEMAND(adjacency_matrix_.rows() == adjacency_matrix_.cols());
  DRAKE_DEMAND(adjacency_matrix_.rows() == std::ssize(vertices_));
  DRAKE_DEMAND(adjacency_matrix_.cols() == std::ssize(vertices_));
  logging::log()->info(
      "CliqueGenerator initialized with {} vertices and {} edges.",
      vertices_.size(), adjacency_matrix_.nonZeros());
  logging::log()->debug(
      "CliqueGenerator:Adjacency matrix is {}x{} with {} non-zero entries",
      adjacency_matrix_.rows(), adjacency_matrix_.cols(),
      adjacency_matrix_.nonZeros());
}

std::optional<drake::geometry::optimization::Hyperellipsoid>
CliqueGenerator::CalcCliqueEllipsoidAroundConfig(
    int vertex_index,
    const drake::geometry::optimization::ConvexSets& existing_convex_sets)
    const {
  int dim = robot_constraints_.robot_model().holonomic_mapping().minimal_dim();
  int minimum_dim = dim + 1;  // Minimum dimension for the clique to be valid

  DRAKE_THROW_UNLESS(vertex_index >= 0 && vertex_index < std::ssize(vertices_));

  const Eigen::VectorXd& q = vertices_[vertex_index];

  // First, remove points that are already covered by existing convex sets.
  const auto uncovered_vertices =
      FindUncoveredVertices(vertices_, existing_convex_sets);
  if (uncovered_vertices.empty()) {
    logging::log()->info(
        "CalcCliqueEllipsoidAroundConfig: All vertices are covered by existing "
        "convex sets. This is good news! But it means no new clique will be "
        "formed.");
    return std::nullopt;
  }

  // Find visible vertices using the adjacency matrix instead of collision
  // checks
  std::vector<int> visible_vertices;
  for (int i = 0; i < std::ssize(uncovered_vertices); ++i) {
    // Skip the vertex itself
    if (uncovered_vertices[i] == vertex_index) continue;
    int uncovered_vertex_idx = uncovered_vertices[i];
    // Check if there's an edge in the adjacency matrix
    if (adjacency_matrix_.coeff(vertex_index, uncovered_vertex_idx)) {
      visible_vertices.push_back(uncovered_vertex_idx);
    }
  }

  if (std::ssize(visible_vertices) < minimum_dim) {
    logging::log()->error(
        "CalcCliqueEllipsoidAroundConfig: Not enough visible vertices to form "
        "a clique around vertex {}. "
        "Found {} visible vertices, but need more than {}.",
        vertex_index, visible_vertices.size(), minimum_dim);
    return std::nullopt;
  }

  // Now let's form a subgraph with the visible vertices and the given vertex
  int n = std::ssize(visible_vertices);
  Eigen::SparseMatrix<bool> subgraph(n + 1, n + 1);

  // Add edges between the given vertex (index n in subgraph) and all visible
  // vertices
  for (int i = 0; i < n; ++i) {
    subgraph.insert(i, n) = true;
    subgraph.insert(n, i) = true;
  }

  // Add edges between visible vertices based on the original adjacency matrix
  for (int i = 0; i < n; ++i) {
    for (int j = i + 1; j < n; ++j) {
      int vertex_i = visible_vertices[i];
      int vertex_j = visible_vertices[j];
      if (adjacency_matrix_.coeff(vertex_i, vertex_j)) {
        subgraph.insert(i, j) = true;
        subgraph.insert(j, i) = true;
      }
    }
  }

  drake::planning::graph_algorithms::MaxCliqueSolverViaGreedy max_clique_solver;
  auto max_clique = max_clique_solver.SolveMaxClique(subgraph);

  DRAKE_THROW_UNLESS(
      max_clique[max_clique.size()
                 - 1]);  // Ensure the last vertex is the clique,
                         // otherwise there is a bug in the solver
  const int m = std::count(max_clique.begin(), max_clique.end(), true);
  if (m < minimum_dim) {
    logging::log()->error(
        "CliqueGenerator: Not enough vertices in the maximum clique to form an "
        "ellipsoid. "
        "Found {} vertices, but need at least {}.",
        m, minimum_dim);
    return std::nullopt;
  }
  // Retrieve the vertices of the maximum clique
  Eigen::MatrixXd clique_matrix(dim, m);
  int col = 0;
  for (int i = 0; i < n; ++i) {
    if (max_clique[i]) {
      int index = visible_vertices[i];
      logging::log()->info(
          "CliqueGenerator: Found vertex {} in the maximum clique with "
          "coordinates: {}",
          index, vertices_[index].transpose());
      clique_matrix.col(col++) = vertices_[index];
    }
  }
  DRAKE_DEMAND(col == m - 1);  // Ensure we have the correct number of columns
  // Add the configuration q to the clique matrix
  clique_matrix.col(col) = q;
  logging::log()->info(
      "CliqueGenerator: Found {} vertices in the maximum clique. clique_matrix "
      "is of size {}x{}",
      m, clique_matrix.rows(), clique_matrix.cols());
  logging::log()->trace("Clique matrix:\n{}", clique_matrix);
  return drake::geometry::optimization::Hyperellipsoid::
      MinimumVolumeCircumscribedEllipsoid(clique_matrix);
}

std::optional<drake::geometry::optimization::Hyperellipsoid>
CliqueGenerator::CalcCliqueEllipsoidAroundConfig(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::ConvexSets& existing_convex_sets)
    const {
  int dim = robot_constraints_.robot_model().holonomic_mapping().minimal_dim();
  int minimum_dim = dim + 1;  // Minimum dimension for the clique to be valid
  DRAKE_THROW_UNLESS(q.size() == dim);
  // First, remove points that are already covered by existing convex sets.
  auto uncovered_vertices =
      FindUncoveredVertices(vertices_, existing_convex_sets);
  if (uncovered_vertices.empty()) {
    logging::log()->info(
        "CalcCliqueEllipsoidAroundConfig: All vertices are covered by existing "
        "convex sets. This is good news! But it means no new clique will be "
        "formed.");
    return std::nullopt;
  }
  // Now let's insert q to see which vertices form edges with it
  std::vector<int> visible_vertices;
  for (int i = 0; i < std::ssize(uncovered_vertices); ++i) {
    if (robot_constraints_.CheckSatisfiedEdge(
            q, vertices_[uncovered_vertices[i]], 0.005)) {
      visible_vertices.push_back(uncovered_vertices[i]);
    }
  }
  if (std::ssize(visible_vertices) < minimum_dim) {
    logging::log()->error(
        "CalcCliqueEllipsoidAroundConfig: Not enough visible vertices to form "
        "a clique around the configuration. "
        "Found {} visible vertices, but need more than {}.",
        visible_vertices.size(), minimum_dim);
    return std::nullopt;
  }
  // Now let's form a subgraph with the visible vertices and q
  int n = std::ssize(visible_vertices);
  Eigen::SparseMatrix<bool> subgraph(n + 1, n + 1);
  for (int i = 0; i < n; ++i) {
    subgraph.insert(i, n) = true;
    subgraph.insert(n, i) = true;
    for (int j = i + 1; j < n; ++j) {
      if (adjacency_matrix_.coeff(visible_vertices[i], visible_vertices[j])) {
        subgraph.insert(i, j) = true;
        subgraph.insert(j, i) = true;
      }
    }
  }
  drake::planning::graph_algorithms::MaxCliqueSolverViaGreedy max_clique_solver;
  auto max_clique = max_clique_solver.SolveMaxClique(subgraph);
  DRAKE_THROW_UNLESS(max_clique[max_clique.size() - 1]
                     == true);  // Ensure the last vertex is the clique,
                                // otherwise there is a bug in the solver
  const int m = std::count(max_clique.begin(), max_clique.end(), true);
  if (m < minimum_dim) {
    logging::log()->error(
        "CliqueGenerator: Not enough vertices in the maximum clique to form an "
        "ellipsoid. "
        "Found {} vertices, but need at least {}.",
        m, minimum_dim);
    return std::nullopt;
  }
  // Retrieve the vertices of the maximum clique
  Eigen::MatrixXd clique_matrix(dim, m);
  int col = 0;
  for (int i = 0; i < n; ++i) {
    if (max_clique[i]) {
      int index = visible_vertices[i];
      logging::log()->trace(
          "CliqueGenerator: Found vertex {} in the maximum clique with "
          "coordinates: {}",
          index, vertices_[index].transpose());
      clique_matrix.col(col++) = vertices_[index];
    }
  }
  DRAKE_DEMAND(col == m - 1);  // Ensure we have the correct number of columns
  // Add the configuration q to the clique matrix
  clique_matrix.col(col) = q;
  logging::log()->info(
      "CliqueGenerator: Found {} vertices in the maximum clique. clique_matrix "
      "is of size {}x{}",
      m, clique_matrix.rows(), clique_matrix.cols());
  logging::log()->trace("Clique matrix:\n{}", clique_matrix);
  return drake::geometry::optimization::Hyperellipsoid::
      MinimumVolumeCircumscribedEllipsoid(clique_matrix);
}

std::optional<drake::geometry::optimization::Hyperellipsoid>
CliqueGenerator::CalcCliqueEllipsoidAroundEdge(
    const std::pair<int, int>& vertex_indices,
    const drake::geometry::optimization::ConvexSets& existing_convex_sets)
    const {
  int dim = robot_constraints_.robot_model().holonomic_mapping().minimal_dim();
  const int minimum_dim =
      dim + 1;  // Minimum dimension for the clique to be valid

  const auto& [vertex_index1, vertex_index2] = vertex_indices;

  // Validate vertex indices
  DRAKE_THROW_UNLESS(vertex_index1 >= 0
                     && vertex_index1 < std::ssize(vertices_));
  DRAKE_THROW_UNLESS(vertex_index2 >= 0
                     && vertex_index2 < std::ssize(vertices_));
  DRAKE_THROW_UNLESS(vertex_index1 != vertex_index2);

  // Check that the two vertices are connected in the adjacency matrix
  if (!adjacency_matrix_.coeff(vertex_index1, vertex_index2)) {
    logging::log()->error(
        "CliqueGenerator:CalcCliqueEllipsoidAroundEdge: Vertices {} and {} are "
        "not connected in the adjacency matrix. Cannot form a clique around "
        "this edge.",
        vertex_index1, vertex_index2);
    return std::nullopt;
  }

  const Eigen::VectorXd& q1 = vertices_[vertex_index1];
  const Eigen::VectorXd& q2 = vertices_[vertex_index2];

  // First, remove points that are already covered by existing convex sets.
  const auto uncovered_vertices =
      FindUncoveredVertices(vertices_, existing_convex_sets);
  if (uncovered_vertices.empty()) {
    logging::log()->info(
        "CalcCliqueEllipsoidAroundEdge: All vertices are covered by existing "
        "convex sets. This is good news! But it means no new clique will be "
        "formed.");
    return std::nullopt;
  }

  // Find vertices that are connected to both vertex_index1 and vertex_index2
  std::vector<int> visible_vertices;
  for (int i = 0; i < std::ssize(uncovered_vertices); ++i) {
    int uncovered_vertex_idx = uncovered_vertices[i];
    // Skip the edge vertices themselves
    if (uncovered_vertex_idx == vertex_index1
        || uncovered_vertex_idx == vertex_index2) {
      continue;
    }
    // Check if the uncovered vertex is connected to both edge vertices
    if (adjacency_matrix_.coeff(vertex_index1, uncovered_vertex_idx)
        && adjacency_matrix_.coeff(vertex_index2, uncovered_vertex_idx)) {
      visible_vertices.push_back(uncovered_vertex_idx);
    }
  }

  if (std::ssize(visible_vertices) + 2 < minimum_dim) {
    logging::log()->error(
        "CalcCliqueEllipsoidAroundEdge: Not enough visible vertices to form "
        "a clique around the edge ({}, {}). "
        "Found {} visible vertices, but need at least {}.",
        vertex_index1, vertex_index2, visible_vertices.size(), minimum_dim - 2);
    return std::nullopt;
  }

  // Now let's form a subgraph with the visible vertices and the two edge
  // vertices
  int n = std::ssize(visible_vertices);
  Eigen::SparseMatrix<bool> subgraph(n + 2, n + 2);

  // Add edges between visible vertices and both edge vertices
  for (int i = 0; i < n; ++i) {
    subgraph.insert(i, n) = true;      // Edge to first vertex
    subgraph.insert(i, n + 1) = true;  // Edge to second vertex
    subgraph.insert(n, i) = true;      // Symmetric edge
    subgraph.insert(n + 1, i) = true;  // Symmetric edge

    // Add edges between visible vertices based on the original adjacency matrix
    for (int j = i + 1; j < n; ++j) {
      int vertex_i = visible_vertices[i];
      int vertex_j = visible_vertices[j];
      if (adjacency_matrix_.coeff(vertex_i, vertex_j)) {
        subgraph.insert(i, j) = true;
        subgraph.insert(j, i) = true;
      }
    }
  }

  // Add edge between the two original vertices
  subgraph.insert(n, n + 1) = true;
  subgraph.insert(n + 1, n) = true;

  drake::planning::graph_algorithms::MaxCliqueSolverViaGreedy max_clique_solver;
  const auto max_clique = max_clique_solver.SolveMaxClique(subgraph);

  // The last two vertices in max_clique should correspond to vertex_index1 and
  // vertex_index2
  DRAKE_THROW_UNLESS(max_clique[n]);
  DRAKE_THROW_UNLESS(max_clique[n + 1]);

  int m = std::count(max_clique.begin(), max_clique.end(), true);
  if (m < minimum_dim) {
    logging::log()->error(
        "CliqueGenerator: Not enough vertices in the maximum clique to form an "
        "ellipsoid. "
        "Found {} vertices, but need at least {}.",
        m, minimum_dim);
    return std::nullopt;
  }

  // Retrieve the vertices of the maximum clique
  Eigen::MatrixXd clique_matrix(dim, m);
  int col = 0;
  for (int i = 0; i < n; ++i) {
    if (max_clique[i]) {
      int index = visible_vertices[i];
      logging::log()->trace(
          "CliqueGenerator: Found vertex {} in the maximum clique with "
          "coordinates: {}",
          index, vertices_[index].transpose());
      clique_matrix.col(col++) = vertices_[index];
    }
  }

  // Add the two edge vertices to the clique matrix
  clique_matrix.col(col++) = q1;
  clique_matrix.col(col) = q2;
  DRAKE_DEMAND(col == m - 1);  // Ensure we have the correct number of columns

  logging::log()->info(
      "CliqueGenerator: Found {} vertices in the maximum clique around edge "
      "({}, {}). "
      "clique_matrix is of size {}x{}",
      m, vertex_index1, vertex_index2, clique_matrix.rows(),
      clique_matrix.cols());
  logging::log()->trace("Clique matrix:\n{}", clique_matrix);

  return drake::geometry::optimization::Hyperellipsoid::
      MinimumVolumeCircumscribedEllipsoid(clique_matrix);
}

std::optional<drake::geometry::optimization::Hyperellipsoid>
CliqueGenerator::CalcCliqueEllipsoidAroundEdge(
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
    const drake::geometry::optimization::ConvexSets& existing_convex_sets)
    const {
  int dim = robot_constraints_.robot_model().holonomic_mapping().minimal_dim();
  int minimum_dim = dim + 1;  // Minimum dimension for the clique to be valid
  const auto& [q1, q2] = edge;
  DRAKE_THROW_UNLESS(q1.size() == dim && q2.size() == dim);
  if (!robot_constraints_.CheckSatisfiedEdge(q1, q2, 0.005)) {
    logging::log()->error(
        "CliqueGenerator:CalcCliqueEllipsoidAroundEdge: Edge ({}, {}) is not "
        "satisfied by the robot constraints. Cannot form a clique.",
        q1.transpose(), q2.transpose());
    return std::nullopt;
  }
  // First, remove points that are already covered by existing convex sets.
  auto uncovered_vertices =
      FindUncoveredVertices(vertices_, existing_convex_sets);
  if (uncovered_vertices.empty()) {
    logging::log()->info(
        "CalcCliqueEllipsoidAroundConfig: All vertices are covered by existing "
        "convex sets. This is good news! But it means no new clique will be "
        "formed.");
    return std::nullopt;
  }
  // Now let's insert q1 and q2 to see which vertices form edges with them
  std::vector<int> visible_vertices;
  for (int i = 0; i < std::ssize(uncovered_vertices); ++i) {
    if (robot_constraints_.CheckSatisfiedEdge(
            q1, vertices_[uncovered_vertices[i]], 0.005)
        && robot_constraints_.CheckSatisfiedEdge(
            q2, vertices_[uncovered_vertices[i]], 0.005)) {
      visible_vertices.push_back(uncovered_vertices[i]);
    }
  }
  if (std::ssize(visible_vertices) + 1 <= minimum_dim) {
    logging::log()->error(
        "CalcCliqueEllipsoidAroundEdge: Not enough visible vertices to form "
        "a clique around the edge. "
        "Found {} visible vertices, but need at least {}.",
        visible_vertices.size(), minimum_dim - 1);
    return std::nullopt;
  }
  // Now let's form a subgraph with the visible vertices and q1, q2
  int n = std::ssize(visible_vertices);
  Eigen::SparseMatrix<bool> subgraph(n + 2, n + 2);
  for (int i = 0; i < n; ++i) {
    subgraph.insert(i, n) = true;
    subgraph.insert(i, n + 1) = true;
    subgraph.insert(n, i) = true;
    subgraph.insert(n + 1, i) = true;
    for (int j = i + 1; j < n; ++j) {
      if (adjacency_matrix_.coeff(visible_vertices[i], visible_vertices[j])) {
        subgraph.insert(i, j) = true;
        subgraph.insert(j, i) = true;
      }
    }
  }
  subgraph.insert(n, n + 1) = true;  // Add edge between q1 and q2
  subgraph.insert(n + 1, n) = true;  // Add edge between q2 and q1
  drake::planning::graph_algorithms::MaxCliqueSolverViaGreedy max_clique_solver;
  auto max_clique = max_clique_solver.SolveMaxClique(subgraph);
  // The last two vertices in max_clique should correspond to q1 and q2
  DRAKE_THROW_UNLESS(max_clique[n] == true);
  DRAKE_THROW_UNLESS(max_clique[n + 1] == true);
  int m = std::count(max_clique.begin(), max_clique.end(), true);
  if (m < minimum_dim) {
    logging::log()->error(
        "CliqueGenerator: Not enough vertices in the maximum clique to form an "
        "ellipsoid. "
        "Found {} vertices, but need at least {}.",
        m, minimum_dim);
    return std::nullopt;
  }
  // Retrieve the vertices of the maximum clique
  Eigen::MatrixXd clique_matrix(dim, m);
  int col = 0;
  for (int i = 0; i < n; ++i) {
    if (max_clique[i]) {
      int index = visible_vertices[i];
      logging::log()->trace(
          "CliqueGenerator: Found vertex {} in the maximum clique with "
          "coordinates: {}",
          index, vertices_[index].transpose());
      clique_matrix.col(col++) = vertices_[index];
    }
  }
  // Add q1 and q2 to the clique matrix
  clique_matrix.col(col++) = q1;
  clique_matrix.col(col) = q2;
  DRAKE_DEMAND(col == m - 1);  // Ensure we have the correct number of columns
  logging::log()->info(
      "CliqueGenerator: Found {} vertices in the maximum clique. clique_matrix "
      "is of size {}x{}",
      m, clique_matrix.rows(), clique_matrix.cols());
  logging::log()->trace("Clique matrix:\n{}", clique_matrix);
  return drake::geometry::optimization::Hyperellipsoid::
      MinimumVolumeCircumscribedEllipsoid(clique_matrix);
}

}  // namespace iris
}  // namespace motion
