/*
 * Copyright © 2024 Dexai Robotics. All rights reserved.
 */

/// @file thunder_planner.h
#pragma once

#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/tools/thunder/Thunder.h>

#include "sample_based_planner.h"

namespace motion {
namespace planning {
namespace ompl {

struct ThunderParameters {
  /** the resolution at which to check the validity of a path */

  double validity_check_resolution {0.005};

  /** used by SPARSdb as a radius for the nearest neighbor search */
  double sparse_delta_fraction_base {0.16};

  /** number of parallel plans to run by Thunder */
  unsigned int num_parallel_plans {0};

  /** the minimum radius of clearance from other nodes, for each node in the
   * roadmap */
  double roadmap_granularity_base {0.3};

  /** compute the cost of the solution in ompl when comparing solutions */
  bool set_compute_solution_cost {true};

  /** when loading a roadmap, recompute edge costs. This is expensive. */
  bool migration_on_load {false};

  /** use our own dense roadmap instead of the sparse roadmap */
  bool dense_roadmap {true};

  /** use cost as edge weight in roadmap, if cost availeble */
  bool use_cost_in_roadmap {true};

  /** add new edges with their cost as weight */
  bool add_edge_with_cost {true};

  /** planning time upper bound for scratch planning */
  double max_scratch_planning_time {10.0};

  /** planning time upper bound for recall planning */
  double max_recall_planning_time {1.0};

  /** Log all planner output to info, else debug */
  int verbosity {2};

  // Drake YAML serialization
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(validity_check_resolution));
    a->Visit(DRAKE_NVP(sparse_delta_fraction_base));
    a->Visit(DRAKE_NVP(num_parallel_plans));
    a->Visit(DRAKE_NVP(roadmap_granularity_base));
    a->Visit(DRAKE_NVP(migration_on_load));
    a->Visit(DRAKE_NVP(max_scratch_planning_time));
    a->Visit(DRAKE_NVP(max_recall_planning_time));
    a->Visit(DRAKE_NVP(verbosity));
  }
};

// ThunderPlanner inherits from ::ompl::tools::Thunder and owns
// a SampleBasedPlanningContext member variable.

class ThunderPlanner : ::ompl::tools::Thunder {
 public:
  /** RobotModel-Constraints constructor. */
  ThunderPlanner(const RobotConstraints& robot_constraints,
                 const ThunderParameters& thunder_parameters,
                 const std::string& db_file_path = "");

  const Eigen::VectorXd& GetVertexConf(int vertex_index) const;

  std::vector<int> CalcSortedVertexIndicesNearConf(
      const Eigen::VectorXd& q_start) const;

  /** Given a configuration, find the nearest valid configuration in the
   * roadmap. */
  /** @param q_invalid the configuration to search from (typically invalid)
   * @param fixed_model_instance_indices the model instance indices to be
   * considered fixed.
   * @param offset an offset to be added to the search direction
   * @param search_step_size the step size to use when searching towards the
   * nearest configuration
   * @param maybe_q_start an optional starting configuration to consider. If
   * provided, the function will only return a configuration that the edge from
   * the starting configuration to it is valid.
   * @return the nearest valid configuration in the roadmap */
  std::optional<Eigen::VectorXd> CalcNearestValidConf(
      const Eigen::VectorXd& q_invalid,
      const std::vector<drake::multibody::ModelInstanceIndex>&
          fixed_model_instance_indices = {},
      double offset = 0.01, double search_step_size = 0.01,
      std::optional<Eigen::VectorXd> maybe_q_start = std::nullopt,
      int num_random_samples = 10) const;

  /** An overload where multiple configurations are provided, and at least
   * one of them is invalid. It finds the nearest valid configuration that
   * satisfies all fixed model instance indices. Using this function means that
   * fixed_model_instance_indices is populated, otherwise it does not make
   * sense. */
  std::optional<Eigen::VectorXd> CalcNearestValidConf(
      const std::vector<Eigen::VectorXd>& q_invalid_vec,
      const std::set<drake::multibody::ModelInstanceIndex>&
          fixed_model_instance_indices,
      double offset = 0.01, double search_step_size = 0.01,
      std::optional<Eigen::VectorXd> maybe_q_start = std::nullopt,
      int num_random_samples = 10) const;

  std::optional<std::vector<Eigen::VectorXd>> SolveRecallPlan(
      const Eigen::VectorXd& start, const Eigen::VectorXd& goal);

  std::optional<std::vector<Eigen::VectorXd>> SolveParallelPlan(
      const Eigen::VectorXd& start, const Eigen::VectorXd& goal,
      const bool save_solution_to_database = false);

  /** Get all edges stored in the roadmap database. */
  const conf_edge_vec_t GetRoadmapEdges();

  /**
   * @brief Get the vertex indices of the largest clique containing the edge
   * between the two given vertices. The function does not include the edge
   * vertex indices in the returned clique.
   * @param from_vertex_index the index of the first vertex
   * @param to_vertex_index the index of the second vertex
   * @return the indices of the remaining vertices in the largest clique
   * containing the edge
   */
  std::vector<int> GetLargestCliqueContainingEdge(
      unsigned int from_vertex_index, unsigned int to_vertex_index) const;

  /** Save roadmap data as a .dat file at the given path. */
  void SaveThunderDataToDatabase(const bool do_post_processing = false);

  std::shared_ptr<SampleBasedPlanningContext> planning_context() const {
    return planning_context_;
  }

  ::ompl::tools::ThunderDBPtr experience_database() const {
    return experienceDB_;
  }

  ::ompl::base::SpaceInformationPtr space_information() const {
    return planning_context_->space_information();
  }

  void SetThunderParameters(const ThunderParameters& thunder_parameters) {
    thunder_parameters_ = thunder_parameters;
  }

  const ThunderParameters& thunder_parameters() const {
    return thunder_parameters_;
  }

  /**
   * @brief Get the adjacency matrix from the roadmap for the given vertices.
   *
   * @param vertex_indices
   * @return Sparse matrix where the (i,j) entry is true if there is an edge
   * between the i-th and j-th vertices in the given vertex_indices.
   */
  Eigen::SparseMatrix<bool> SparseAdjacencyMatrixOfVertices(
      const std::vector<unsigned int>& vertex_indices) const;

  /**
   * @brief Get the adjacency matrix for the whole roadmap.
   * @return Sparse matrix where the (i,j) entry is true if there is an edge
   * between the i-th and j-th vertices in the roadmap.
   */
  Eigen::SparseMatrix<bool> SparseAdjacencyMatrixOfRoadmap() const {
    // create a vector of vertex indices
    std::vector<unsigned int> vertex_indices;
    vertex_indices.reserve(planner_data_->numVertices());
    for (unsigned int i = 0; i < planner_data_->numVertices(); i++) {
      vertex_indices.push_back(i);
    }
    auto matrix = SparseAdjacencyMatrixOfVertices(vertex_indices);
    logging::log()->debug(
        "ThunderPlanner:SparseAdjacencyMatrixOfRoadmap: Created adjacency "
        "matrix of size {}x{} with {} non-zero entries",
        matrix.rows(), matrix.cols(), matrix.nonZeros());
    return matrix;
  }

  /**
   * @brief Configure the Thunder planner by setting various parameters and
   * assigning the filepath for the database.
   *
   * @param thunder_parameters Parameters to be passed to the Thunder planner
   */
  void SetupThunderPlanner(const ThunderParameters& thunder_parameters);

  /**
   * @brief Load the roadmap from disk, validate it, and save it back.
   *
   */
  void ReloadAndValidateLocalRoadmap();

  void GetPlannerData(ob::PlannerData& planner_data) {
    experienceDB_->getSPARSdb()->getPlannerData(planner_data);
  }

  void AddConfsToRoadmap(const std::vector<Eigen::VectorXd>& confs);

  /**
   * @brief Get a sparse representation of the largest subgraph of the roadmap
   * that is connected to all the vertices in the given list.
   *
   * @param included_vertex_indices the vertices to which the returned subgraph
   * should be connected
   * @return const Eigen::SparseMatrix<double>
   */
  std::vector<unsigned int> GetConnectedVertices(
      std::vector<unsigned int> included_vertex_indices) const;

  /** @brief Get the configuration space distance between two configurations
   * @param conf_A the first configuration
   * @param conf_B the second configuration
   * @return the distance between the two configurations
   */
  double CalcConfigSpaceDistance(const Eigen::VectorXd& conf_A,
                                 const Eigen::VectorXd& conf_B) const;

  const ob::PlannerData& planner_data() const {
    return *planner_data_;
  }

  const std::vector<Eigen::VectorXd>& vertices_confs() const {
    return vertices_confs_;
  }

  std::string GetDatabaseFilePath() const {
    return db_file_path_;
  }

 private:
  static std::shared_ptr<SampleBasedPlanningContext>
  makeSampleBasedPlanningContext(const RobotConstraints& robot_constraints) {
    return std::make_shared<SampleBasedPlanningContext>(robot_constraints);
  }

  ThunderPlanner(std::shared_ptr<SampleBasedPlanningContext> ctx,
                 const ThunderParameters& thunder_parameters,
                 const std::string& db_file_path = "");
  /**
   * @brief Validate the start and goal states, convert them to their OMPL
   * representations, and sets them in the OMPL problem definition.
   *
   * @param start Start state
   * @param goal Goal state
   */
  void SetupProblemDefinition(const Eigen::VectorXd& start,
                              const Eigen::VectorXd& goal);

  std::shared_ptr<SampleBasedPlanningContext> planning_context_;
  ThunderParameters thunder_parameters_;
  const std::string db_file_path_;
  std::unique_ptr<ob::PlannerData> planner_data_;
  std::vector<Eigen::VectorXd> vertices_confs_;

  friend class ThunderPlannerRoadmapTest;
};

}  // namespace ompl
}  // namespace planning
}  // namespace motion
