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

#include <filesystem>

#include "iris_adapter.h"
#include "planning_service/motion/robot_constraints.h"

namespace motion {
namespace iris {

using conf_edge_t = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

class IrisInspector {
 public:
  /** Constructor for IrisInspector
   @param robot_constraints the robot constraints
   @param iris_regions_adapter the iris regions adapter
   note that the iris meta options are optional and if not provided, the default
    options will be used.
   */
  IrisInspector(const RobotConstraints& robot_constraints,
                const IrisRegionsAdapter& iris_regions_adapter,
                double check_satisfied_edge_step = 0.005);
  /**
   * @brief Checks to see if the edge is inside the region. This is equivalent
   * to both the vertices being inside the region as the region is convex.
   *
   * @param edge The edge to check
   * @param region The region against which the edge is checked
   */
  bool IsEdgeInsideRegion(
      const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
      const IrisRegionsAdapter::IrisRegion& region) const;

  /**
   * @brief Checks if an edge is covered completeley by any one iris region
   *
   * @param edge the edge to check
   * @param regions the regions that might cover the edge
   * @return true if edge is covered by any one region, false otherwise
   */
  bool IsEdgeCoveredByAnyIrisRegion(
      const conf_edge_t& edge,
      const std::vector<IrisRegionsAdapter::IrisRegion>& regions) const;

  /** Checks if an Iris Region is valid by sampling random configurations.
   @param region the Iris Region to check
   @param sample_size the number of samples to check
    @return true if the region is valid, false otherwise
    */
  bool IsValidViaSampling(const IrisRegionsAdapter::IrisRegion& region,
                          const int sample_size = 1000) const;

  /** Returns convex sets as a vector of pointers */
  const drake::geometry::optimization::ConvexSets GetConvexSets() const {
    return iris_regions_adapter_.GetConvexSets();
  }

  /** Provides infomration on how the IRIS regions cover a specific set of
  configurations. */
  struct CoverageResult {
    /** The number of configurations that are covered by the volume of the iris
    regions. If a configuration is volume_covered, it means that it is inside at
    least one of the iris regions. */
    int volume_covered;

    /** The number of configurations that are covered by the visibility of the
    iris regions. If a configuration is visibility-covered, it means that all
    the points on the shortest line segment between the
    configuration and one of the iris regions satisfies the constraints. */
    int visibility_covered;

    /** The number of configurations that are not covered by the iris regions by
     * either volume or visibility. */
    int uncovered;

    /** The number of configurations that violate the constraints. They do
     * **not** count as uncovered. */
    int violates_constraints;
  };

  void SetPointsForIrisCoverageEvaluation(
      const std::vector<Eigen::VectorXd>& config_points);

  /** Evaluates the coverage of the iris regions. It only evaluates the coverage
   * for the newly added region.
   * @return The coverage result.
   */
  CoverageResult EvaluateCoverage();

  /** Data structure for the result of EvalConfigAgainstRegion
  @param inside true if the point is inside the Iris Region, false otherwise
  @param check_satisfied if provided, then it is true if the point satisfies
  the constraints, false otherwise
  @param visible_point if provided, then it is the closest point in the region
  to the configuration such that the line segment between the configuration and
  the visible point satisfies the constraints
  */
  struct ConfigAgainstRegionResult {
    bool inside {false};
    std::optional<bool> check_satisfied {std::nullopt};
    std::optional<Eigen::VectorXd> visible_point {std::nullopt};
  };

  /** Checks the properties of a configuration with respect to an Iris Region.
   @param q the configuration to check
   @param region the Iris Region to check
   @param check_satisfied if true, then the function will check if the
   configuration satisfies the constraints
   @param check_visibility if true, then the function will check if the
   configuration is visible from the robot base
   @return a struct with the result of the check
   @warning Will be deprecated soon. Use EvalConfigAgainstIrisRegion.
   */
  ConfigAgainstRegionResult EvalConfigAgainstRegion(
      const Eigen::VectorXd& q,
      const drake::geometry::optimization::HPolyhedron& region,
      const bool check_satisfied = false,
      const bool check_visibility = false) const;

  /** Structure for the result of EvalConfigAgainstIrisRegion
  @param inside true if the point is inside the Iris Region, false otherwise
  @param check_satisfied if provided, then it is true if the point satisfies
  the constraints, false otherwise
  @param connecting_set if provided, is the convex set represented by
  a VPolytope that connects the configuration to the Iris Region.
  */
  struct ConfigAgainstIrisRegionResult {
    bool inside {false};
    std::optional<bool> check_satisfied {std::nullopt};
    std::optional<drake::geometry::optimization::VPolytope> connecting_set {
        std::nullopt};
    /** An upper bound on the distance between the configuration and the
    Iris Region. Only available if connecting_set is not computed. */
    std::optional<double> distance_upper_bound {std::nullopt};
  };

  /** Checks the properties of a configuration with respect to an Iris Region.
  @param q the configuration to check
  @param region the Iris Region to check
  @param check_satisfied if true, then the function will check if the
  configuration satisfies the constraints
  @param check_visibility if true, then the function will check if the
  configuration is visible from the robot base
  @return a struct with the result of the check.
  */
  ConfigAgainstIrisRegionResult EvalConfigAgainstIrisRegion(
      const Eigen::VectorXd& q,
      const IrisRegionsAdapter::IrisRegion& iris_region,
      const bool check_satisfied = false,
      const bool check_visibility = false) const;

  /** Returns true if the configuration q is inside any of the iris regions */
  bool InsideAnyRegion(const Eigen::VectorXd& q) const;

  bool IsEdgeCoveredByRegions(
      const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge) const;

  /** Evaluates the point against all the iris regions. */
  struct ConfigAgainstIrisRegionsResult {
    // The regions that contain the configuration
    std::vector<int> containing_regions_indices {};
    std::optional<int> visible_region_index {std::nullopt};
    std::optional<Eigen::VectorXd> visible_point {std::nullopt};
  };

  /** Evaluates the properties of a configuration with respect to the Iris
  Regions.
  @param q the configuration to check
  */
  ConfigAgainstIrisRegionsResult EvalConfigAgainstIrisRegions(
      const Eigen::VectorXd& q) const;

  /** TODO(@sadraddini) To be Deprecated */
  std::vector<int> GetRegionsContainingConfig(const Eigen::VectorXd& q) const;

  /** Given a configuration q, calculates a closest point in the Iris Regions
  such that the line segment between q and the closest point satisfies the
  constraints.
  @param q the configuration
  @return If succcesful, a pair of the closest convex set index and the
  closest point in that convex set. Returns nullopt if not successful.
  TODO(@sadraddini) To be Deprecated.
   */
  std::optional<std::pair<int, Eigen::VectorXd>> MaybeCalcEdgeToRegions(
      const Eigen::VectorXd& q) const;

  /** Finds the closest valid configuration to the given configuration that is
   * inside or close to the Iris regions.
   *
   * @param q The configuration to find the closest valid configuration to.
   * @param iris_region The iris region to find the closest valid configuration
   * to.
   * @param fixed_model_instances The model instances that are fixed in the
   * configuration, and those that should not be moved.
   * @return If successful, a pair of the closest valid configuration and the
   * distance to the given configuration. Returns nullopt if not successful.
   */
  std::optional<std::pair<Eigen::VectorXd, double>>
  CalcClosestValidConfToRegion(
      const Eigen::VectorXd& q,
      const IrisRegionsAdapter::IrisRegion& iris_region,
      const std::vector<drake::multibody::ModelInstanceIndex>
          fixed_model_instances = {}) const;

  /** Finds the closest valid configuration to the given configuration that is
   * inside or close to the Iris regions.
   *
   * @param q The configuration to find the closest valid configuration to.
   * @param fixed_model_instances The model instances that are fixed in the
   * configuration, and those that should not be moved.
   * @return If successful, the closest valid configuration. Returns nullopt if
   * not successful.
   */
  std::optional<Eigen::VectorXd> CalcClosestValidConfToRegions(
      const Eigen::VectorXd& q,
      const std::vector<drake::multibody::ModelInstanceIndex>
          fixed_model_instances = {}) const;

  /** Getter (read-only) for the underlying robot constraints. */
  const RobotConstraints& robot_constraints() const {
    return robot_constraints_;
  }

  const IrisRegionsAdapter& iris_regions_adapter() const {
    return iris_regions_adapter_;
  }

 private:
  const RobotConstraints& robot_constraints_;
  const IrisRegionsAdapter& iris_regions_adapter_;
  const double check_satisfied_edge_step_;

  // For IRIS coverage
  enum class CoverageResultType {
    kCoveredByVolume,
    kCoveredByVisibility,
    kUncovered,
    kViolatesConstraints
  };

  std::vector<Eigen::VectorXd> config_points_;
  std::vector<CoverageResultType> coverage_result_types_;
  std::vector<int> evaluated_iris_regions_indices_;
};

}  // namespace iris
}  // namespace motion

/// \cond DO_NOT_DOCUMENT
template <>
struct fmt::formatter<motion::iris::IrisInspector::CoverageResult> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(const motion::iris::IrisInspector::CoverageResult& result,
              FormatContext& ctx) const {
    int total_points = result.uncovered + result.volume_covered
                       + result.visibility_covered
                       + result.violates_constraints;
    const double volume_coverage_percentage =
        100.0 * (result.volume_covered)
        / static_cast<double>(total_points - result.violates_constraints);
    const double visibility_coverage_percentage =
        100.0 * (result.visibility_covered)
        / static_cast<double>(total_points - result.violates_constraints);
    return fmt::format_to(
        ctx.out(),
        "Volume covered: \t\t{}/{} ({:.2f}%) \nVisibility covered: \t\t{}/{} "
        "({:.2f}%) \nUncovered: \t\t\t {}/{} ({:.2f}%)\nViolates constraints: "
        "\t\t {}/{} ({:.2f}%)",
        result.volume_covered, total_points - result.violates_constraints,
        volume_coverage_percentage, result.visibility_covered,
        total_points - result.violates_constraints,
        visibility_coverage_percentage, result.uncovered,
        total_points - result.violates_constraints,
        100.0 * result.uncovered
            / static_cast<double>(total_points - result.violates_constraints),
        result.violates_constraints, total_points,
        100.0 * result.violates_constraints
            / static_cast<double>(total_points));
  }
};
/// \endcond
