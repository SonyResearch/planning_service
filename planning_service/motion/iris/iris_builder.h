/*
 * Copyright © 2024 Sony Research. All rights reserved.
 */

/// @file iris_builder.h

#pragma once

#include <drake/common/random.h>
#include <drake/common/schema/stochastic.h>
#include <drake/common/type_safe_index.h>
#include <drake/geometry/optimization/convex_set.h>
#include <drake/geometry/optimization/graph_of_convex_sets.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/geometry/optimization/iris.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/planning/iris/iris_np2.h>
#include <drake/planning/iris/iris_zo.h>

#include <filesystem>

#include <magic_enum/magic_enum.hpp>

#include "clique_generator.h"
#include "iris_inspector.h"

namespace fs = std::filesystem;

namespace motion {
namespace iris {
enum class ValidConfigMethod {
  kEdge, /** Sample along an edge from the candidate to the center. */
  kOpt,  /** Solve an optimization problem to find a valid configuration. */
};

struct IrisBuilderOptions {
  drake::planning::IrisZoOptions drake_iris_zo_options =
      drake::planning::IrisZoOptions();

  drake::geometry::optimization::IrisOptions drake_iris_np_options =
      drake::geometry::optimization::IrisOptions();

  drake::planning::IrisNp2Options drake_iris_np2_options =
      drake::planning::IrisNp2Options();

  // if true, then the existing regions will be treated
  bool existing_regions_as_obstacle {true};

  /** If the volume of a region is less than this value, then it will be
   * discarded. */
  double minimum_region_volume {0.0};

  /** The distance to use for sampling the edge for CheckSatisfied. */
  double edge_sample_step {0.005};

  /** If true, then the regions will be repaired before being saved. */
  bool repair_regions {false};

  /** If true, then the regions will be repaired using hyperplanes guided by
   * the ellipsoid. If false, then the additional hyperplanes will be added
   * by linearization of the robot constraints at the sampled points. The latter
   * is more conservative, but also may finish faster. */
  bool repair_via_ellipsoid {true};

  ValidConfigMethod valid_config_method {ValidConfigMethod::kEdge};

  /** The number of samples to use for repairing the regions. Would not have any
  effect if repair_regions is false. */
  int num_samples_for_repair {1000};

  /** The overlap between the partitions when using PartitionConvexSet to break
  a region into smaller regions such that the length of each on axis
  corresponding to any continuous revolute joint is less than pi. See
  PartitionConvexSet in Drake for more information. */
  double partition_overlap {1e-6};

  /** The number of configuration samples to take from each region intersection.
    Each configuration will be added to the graph of configurations. */
  int intersection_samples {1};

  /** The number of samples to be skipped in-between samples selected from
   * intersections */
  int mixing_steps {3};

  /**
   * The rank tolerance used for computing the
   * MinimumVolumeCircumscribedEllipsoid of a clique. See
   * @MinimumVolumeCircumscribedEllipsoid.
   */
  double rank_tol_for_minimum_volume_circumscribed_ellipsoid {1e-4};

  /** If true, then the generation of Iris Regions from cliques will be used, by
   * calculating the largest clique containing the edge as opposed to generating
   * from the edge itself. */
  bool use_generation_from_cliques {false};

  /** If true, then the configurations that are already in the Iris Regions
   * will be excluded from the cliques. This is useful to avoid generating
   * ellipsoids towards already existing regions, which can lead to
   * too much wasteful overlapping
   */
  bool exclude_existing_regions_from_cliques {true};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(drake_iris_zo_options));
    a->Visit(DRAKE_NVP(drake_iris_np_options));
    a->Visit(DRAKE_NVP(drake_iris_np2_options));
    a->Visit(DRAKE_NVP(existing_regions_as_obstacle));
    a->Visit(DRAKE_NVP(minimum_region_volume));
    a->Visit(DRAKE_NVP(edge_sample_step));
    a->Visit(DRAKE_NVP(repair_regions));
    a->Visit(DRAKE_NVP(repair_via_ellipsoid));
    std::string method_name {magic_enum::enum_name(valid_config_method)};
    a->Visit(drake::MakeNameValue("valid_config_method", &method_name));
    const auto valid_config_method_opt {
        magic_enum::enum_cast<ValidConfigMethod>(method_name)};
    if (!valid_config_method_opt.has_value()) {
      throw std::runtime_error(
          fmt::format("Invalid valid_config_method: {}", method_name));
    }
    valid_config_method = valid_config_method_opt.value();
    a->Visit(DRAKE_NVP(num_samples_for_repair));
    a->Visit(DRAKE_NVP(partition_overlap));
    a->Visit(DRAKE_NVP(intersection_samples));
    a->Visit(DRAKE_NVP(mixing_steps));
    a->Visit(DRAKE_NVP(use_generation_from_cliques));
    a->Visit(DRAKE_NVP(exclude_existing_regions_from_cliques));
  }
};

/** A class that builds Iris Regions from a robot model and robot constraints.
An Iris region is a HPolyhedron in the configuration space of the robot such
that all of its points are satisfying the given robot constraints. For more
information about Iris Regions, please refer to the following papers:
https://arxiv.org/pdf/2205.03690.pdf
https://groups.csail.mit.edu/robotics-center/public_papers/Deits14.pdf
*/
class IrisBuilder {
 public:
  /** Constructor for IrisBuilder
   @param robot_constraints the robot constraints
   @param iris_builder_options the iris meta options
   @param iris_regions_adapter the iris regions adapter
   note that the iris meta options are optional and if not provided, the default
    options will be used.
   */
  IrisBuilder(
      const RobotConstraints& robot_constraints,
      const IrisBuilderOptions& iris_builder_options,
      const std::string& iris_regions_adapter_filename,
      const std::optional<std::vector<Eigen::VectorXd>>& clique_configs =
          std::nullopt,
      std::optional<Eigen::SparseMatrix<bool>> adjacency_matrix = std::nullopt);

  /** Postprocesses the Iris Regions by repairing them, and partitioning them
   * into smaller regions if needed. Then it will add the regions to the Iris
   * Regions Adapter, and also saves the file. */
  void AddRegion(const drake::geometry::optimization::HPolyhedron& set,
                 const std::string& name);

  /**
   * @brief Get the vertex index from a configuration within a tolerance radius.
   *
   * Searches through the clique generator's vertices to find the closest vertex
   * that is within the specified tolerance radius from the given configuration.
   *
   * @param q The query configuration
   * @param tolerance_radius Maximum allowable distance from the configuration
   * @return std::optional<int> The index of the closest vertex within
   * tolerance, or std::nullopt if no vertex is found within the tolerance
   * radius
   */
  std::optional<int> GetVertexIndexFromConfiguration(
      const Eigen::VectorXd& q, double tolerance_radius) const;

  /** Calculates an Iris Region from the robot configuration using
   * IrisNp algorithm.
   * @param q the robot configuration to calculate the Iris Region from
   * @return If successful, the Iris Region. Otherwise, nullopt.
   * This method uses the IrisNp algorithm, which is a basic version of the Iris
   * algorithm. It is designed to handle simple scenarios and provide a quick
   * solution.
   */

  /**
   * @brief Get vertex indices for an edge from a configuration pair.
   *
   * Finds the closest vertices in the roadmap to each configuration in the
   * pair, ensuring both are within the tolerance radius and that the vertices
   * are connected in the roadmap's adjacency matrix.
   *
   * @param edge Pair of configurations representing an edge in configuration
   * space
   * @param tolerance_radius Maximum allowable distance from each configuration
   * to its nearest vertex
   * @return std::optional<std::pair<int, int>> Pair of connected vertex
   * indices, or std::nullopt if no suitable connected pair is found
   */
  std::optional<std::pair<int, int>> GetEdgeVertexIndicesFromConfigurationPair(
      const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
      double tolerance_radius) const;

  std::optional<drake::geometry::optimization::HPolyhedron>
  CalcIrisNpFromConfig(
      const Eigen::VectorXd& q,
      const std::optional<drake::geometry::optimization::Hyperellipsoid>&
          starting_ellipsoid = std::nullopt,
      const std::optional<Eigen::MatrixXd> containment_points =
          std::nullopt) const;

  /** Calculates an Iris Region from the robot configuration using
   * IrisNp2 algorithm.
   * @param q the robot configuration to calculate the Iris Region from
   * @return If successful, the Iris Region. Otherwise, nullopt.
   * This method uses the IrisNp2 algorithm, which is a more advanced
   * version of the IrisNp algorithm. It is designed to handle more complex
   * scenarios and provide better performance.
   */
  std::optional<drake::geometry::optimization::HPolyhedron>
  CalcIrisNp2FromConfig(
      const Eigen::VectorXd& q,
      const std::optional<drake::geometry::optimization::Hyperellipsoid>&
          starting_ellipsoid = std::nullopt,
      const std::optional<Eigen::MatrixXd> containment_points =
          std::nullopt) const;

  /** Calculates an Iris Region from the robot configuration using
   * IrisZo algorithm.
   * @param q the robot configuration to calculate the Iris Region from
   * @return If successful, the Iris Region. Otherwise, nullopt.
   */
  std::optional<drake::geometry::optimization::HPolyhedron>
  CalcIrisZoFromConfig(
      const Eigen::VectorXd& q,
      const std::optional<drake::geometry::optimization::Hyperellipsoid>&
          starting_ellipsoid = std::nullopt,
      const std::optional<Eigen::MatrixXd> containment_points =
          std::nullopt) const;

  enum class IrisMethod { kIrisNp, kIrisNp2, kIrisZo };

  /** Calculates Iris Regions from a configuration.
   * @param q the configuration to calculate the Iris Regions from
   * @param method the method to use for calculating the Iris Region, if not
   * provided, the default method (iris_np2) will be used.
   * @return If successful, the Iris Regions. Otherwise, nullopt.
   * This method will calculate the Iris Regions from the configuration using
   */
  std::optional<drake::geometry::optimization::HPolyhedron>
  CalcIrisRegionFromConfig(
      const Eigen::VectorXd& q,
      const IrisMethod& method = IrisMethod::kIrisNp2) const;

  /** Calculates Iris Regions from a configuration.
   * @param q the configuration to calculate the Iris Regions from
   * @param method the method to use for calculating the Iris Region, if not
   * provided, the default method (iris_np2) will be used.
   * @return If successful, the Iris Regions. Otherwise, nullopt.
   */
  std::optional<drake::geometry::optimization::HPolyhedron>
  CalcIrisRegionFromEdge(
      const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
      const IrisMethod& method = IrisMethod::kIrisNp2) const;

  /** helper for CalcAndAddIrisRegionFromSeedConfig */
  void BuildFromConfigs(
      const std::map<std::string, Eigen::VectorXd>& config_name_map,
      const IrisMethod& method = IrisMethod::kIrisNp2);

  /** Builds Iris Regions from a set of edges, the case when
the edges are not named. */
  void BuildFromConfigs(const std::vector<Eigen::VectorXd>& configs,
                        const IrisMethod& method = IrisMethod::kIrisNp2,
                        const std::string& prefix = "config");

  /** Builds Iris Regions from a set of edges. */
  void BuildFromEdges(
      const std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd>>&
          edge_map,
      const IrisMethod& method = IrisMethod::kIrisNp2);

  /** Builds Iris Regions from a set of edges, the case when
  the edges are not named. */
  void BuildFromEdges(
      const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
      const IrisMethod& method = IrisMethod::kIrisNp2,
      const std::string& prefix = "edge");

  /** Calculates Iris regions from a path (an ordered sequence of
   * configurations).| Creates edges between any two consecutive configurations,
   * generates an IrisRegion from each uncovered edge, and adds them to the
   * member Iris regions.
   * @param path the path to calculate the Iris Regions from
   */
  void BuildFromPath(const std::vector<Eigen::VectorXd>& path,
                     const IrisMethod& method = IrisMethod::kIrisNp2);

  /**
   * @brief Attempts to repair a given region using a configuration.
   *
   * This method takes an HPolyhedron region and a configuration vector q.
   * If q is not in the region or already satisfies the robot constraints,
   * the region is returned unchanged and the boolean is false.
   * Otherwise, the method finds the closest satisfying configuration to q,
   * and attempts to repair the region either by intersecting with a correction
   * polytope or by adding a hyperplane using the ellipsoid method.
   *
   * @param region The region to attempt to repair.
   * @param q The configuration to use for repair.
   * @param repair_via_ellipsoid If true, use ellipsoid-based repair; otherwise,
   * use linearized constraints.
   * @param options Options for constraint satisfaction checking.
   * @return std::pair<drake::geometry::optimization::HPolyhedron, bool>
   *         - The repaired region (or the original if no repair was performed).
   *         - A boolean indicating whether a repair was performed.
   */
  std::pair<drake::geometry::optimization::HPolyhedron, bool>
  RepairRegionWithConfig(
      const drake::geometry::optimization::HPolyhedron& region,
      const Eigen::VectorXd& q, bool repair_via_ellipsoid,
      ValidConfigMethod valid_config_method,
      const CheckSatisfiedOptions& options, int thread_num = 0) const;

  /**
   * @brief
   *
   * @param region The region to repair
   * @param repair_configurations Optional configurations to use for repairing
   * the region
   * @param sample_size The number of samples to take for repairing the region
   * @return std::pair<drake::geometry::optimization::HPolyhedron, bool>
   * Polyhedron and flag indicating whether the region required repair or not
   */
  std::pair<drake::geometry::optimization::HPolyhedron, bool>
  RepairRegionViaSampling(
      const drake::geometry::optimization::HPolyhedron& region,
      const std::vector<Eigen::VectorXd>& repair_configurations = {},
      int sample_size = 5000, int thread_num = 0,
      bool repair_via_ellipsoid = true) const;

  /** Post processes the Iris Region to make it more accurate. It is a wrapper
  around multiple steps that are needed
  @param region the Iris Region to post process
  @return a vector of Iris Regions that are the result of the post processing.
  The vector
  */
  std::vector<drake::geometry::optimization::HPolyhedron> PostProcessRegion(
      const drake::geometry::optimization::HPolyhedron& region) const;

  /** Given a vector of configurations, calculates the one that expands the
  visibility-based coverage of iris regions by the most. */
  std::optional<Eigen::VectorXd> EvalBestExpansionConf(
      std::vector<Eigen::VectorXd> q_vec) const;

  motion::iris::IrisInspector::CoverageResult EvaluateCoverage() const {
    return inspector_->EvaluateCoverage();
  }

  /** Getter (read-only) for the underlying robot constraints. */
  const RobotConstraints& robot_constraints() const {
    return robot_constraints_;
  }

  const IrisRegionsAdapter& adapter() const {
    return adapter_;
  }

  /** Getter (read-only) for the underlying iris meta options. */
  const IrisBuilderOptions& options() const {
    return options_;
  }

  const IrisInspector& inspector() const {
    return *inspector_;
  }

  const fs::path& adapter_file() const {
    return adapter_file_;
  }

  void SetIrisRegionsAdapter(const IrisRegionsAdapter& iris_regions_adapter) {
    adapter_ = iris_regions_adapter;
    // Update the inspector with the new adapter
    inspector_ = std::make_unique<IrisInspector>(robot_constraints_, adapter_,
                                                 options_.edge_sample_step);
  }

  void SetPointsForIrisCoverageEvaluation(
      const std::vector<Eigen::VectorXd>& config_points) {
    inspector_->SetPointsForIrisCoverageEvaluation(config_points);
  }

 private:
  const RobotConstraints& robot_constraints_;
  const IrisBuilderOptions options_;
  const fs::path adapter_file_;
  const drake::geometry::optimization::HPolyhedron domain_;
  IrisRegionsAdapter adapter_;
  std::unique_ptr<IrisInspector> inspector_;
  std::unique_ptr<CliqueGenerator> clique_generator_;
};

}  // namespace iris
}  // namespace motion
