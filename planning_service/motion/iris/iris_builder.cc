#include "iris_builder.h"

#include <drake/geometry/optimization/affine_ball.h>
#include <drake/geometry/optimization/geodesic_convexity.h>
#include <drake/geometry/optimization/hyperellipsoid.h>
#include <drake/geometry/optimization/intersection.h>
#include <drake/planning/iris/iris_common.h>
#include <drake/solvers/choose_best_solver.h>
#include <drake/solvers/clarabel_solver.h>
#include <drake/solvers/clp_solver.h>
#include <drake/solvers/gurobi_solver.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/mosek_solver.h>
#include <drake/solvers/osqp_solver.h>
#include <drake/solvers/scs_solver.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/solver_interface.h>

namespace motion {
namespace iris {

// Helper functions
namespace {
// Insert the robot constraints into the IrisBuilderOptions
IrisBuilderOptions InsertRobotConstraintsIntoIrisBuilderOptions(
    const RobotConstraints& robot_constraints,
    const IrisBuilderOptions& iris_builder_options) {
  IrisBuilderOptions options {iris_builder_options};
  options.drake_iris_np_options.prog_with_additional_constraints =
      robot_constraints.iris_prog_no_collision_constraints(0);
  options.drake_iris_zo_options.sampled_iris_options
      .prog_with_additional_constraints =
      robot_constraints.iris_prog_no_collision_constraints(0);
  options.drake_iris_np2_options.sampled_iris_options
      .prog_with_additional_constraints =
      robot_constraints.iris_prog_no_collision_constraints(0);
  // If mimic joints are present, then set the option to use them.
  if (!robot_constraints.robot_model().holonomic_mapping().is_identity()) {
    const auto& parameterization = robot_constraints.robot_model()
                                       .holonomic_mapping()
                                       .iris_parameterization_function();
    // regular iris does not support parameterization (apparently will never do)
    options.drake_iris_zo_options.parameterization = parameterization;
    options.drake_iris_np2_options.parameterization = parameterization;
  }
  return options;
}

// Colorful logging function to report the calculated polytope
void ReportPolytope(const drake::geometry::optimization::HPolyhedron& polytope,
                    std::chrono::system_clock::time_point start,
                    const std::string& method_name) {
  const int random_seed = 0;
  logging::log()->info(
      fmt::format(fg(FMT_GREEN),
                  "IrisBuilder:ReportPolytope: calculated polytope with "
                  "{} hyperplanes using {} from seed in {} ms",
                  polytope.b().size(), method_name,
                  std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now() - start)
                      .count()));
  drake::RandomGenerator gen(random_seed);
  const drake::geometry::optimization::SampledVolume sampled_volume {
      polytope.CalcVolumeViaSampling(&gen, 1e-1, 1e5)};
  const double volume {sampled_volume.volume};
  logging::log()->info(
      "IrisBuilder:ReportPolytope: volume of the polytope is {} with "
      "relative accuracy {} and {} samples",
      volume, sampled_volume.rel_accuracy, sampled_volume.num_samples);
}

// Casts the intersection of multiple polytopes to a single HPolyhedron
drake::geometry::optimization::HPolyhedron GetIntersectionAsHPolyhedron(
    const drake::geometry::optimization::Intersection& intersection) {
  DRAKE_DEMAND(intersection.num_elements() > 1);
  const auto* base_polytope {
      dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(
          &intersection.element(0))};
  drake::geometry::optimization::HPolyhedron intersection_polytope {
      *base_polytope};
  DRAKE_DEMAND(base_polytope != nullptr);
  for (int i = 1; i < intersection.num_elements(); ++i) {
    const auto* polytope {
        dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(
            &intersection.element(i))};
    DRAKE_DEMAND(polytope != nullptr);
    intersection_polytope = intersection_polytope.Intersection(*polytope, true);
  }
  return intersection_polytope;
}

std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>
MakeEdgesFromConfSequence(const std::vector<Eigen::VectorXd>& conf_sequence,
                          const double conf_equality_tolerance = 1e-5) {
  std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>> edges;
  for (size_t i = 0; i < conf_sequence.size() - 1; ++i) {
    if ((conf_sequence[i] - conf_sequence[i + 1]).norm()
        < conf_equality_tolerance) {
      continue;  // Skip identical or nearly identical configurations
    }
    edges.push_back({conf_sequence[i], conf_sequence[i + 1]});
  }
  return edges;
}

}  // namespace

IrisBuilder::IrisBuilder(
    const RobotConstraints& robot_constraints,
    const IrisBuilderOptions& iris_builder_options,
    const std::string& iris_regions_adapter_file,
    const std::optional<std::vector<Eigen::VectorXd>>& clique_configs,
    std::optional<Eigen::SparseMatrix<bool>> adjacency_matrix)
    : robot_constraints_ {robot_constraints},
      options_ {InsertRobotConstraintsIntoIrisBuilderOptions(
          robot_constraints, iris_builder_options)},
      adapter_file_ {fs::path(iris_regions_adapter_file)},
      domain_ {drake::geometry::optimization::HPolyhedron::MakeBox(
          robot_constraints.robot_model().holonomic_mapping().Reduce(
              robot_constraints.robot_model().plant().GetPositionLowerLimits()),
          robot_constraints.robot_model().holonomic_mapping().Reduce(
              robot_constraints.robot_model()
                  .plant()
                  .GetPositionUpperLimits()))},
      // if file exists, load it, otherwise create an empty adapter
      adapter_ {
          fs::exists(adapter_file_)
              ? drake::yaml::LoadYamlFile<IrisRegionsAdapter>(adapter_file_)
              : IrisRegionsAdapter()},
      // create an inspector with the robot constraints and the iris regions
      inspector_ {
          std::make_unique<IrisInspector>(robot_constraints_, adapter_)} {
  if (clique_configs.has_value() && !clique_configs->empty()) {
    logging::log()->info(
        "IrisBuilder: Creating a clique generator with {} configurations",
        clique_configs->size());
    logging::log()->info(
        "IrisBuilder: The adjacency matrix is {}",
        adjacency_matrix.has_value()
            ? fmt::format("{}x{} with {} non-zeros", adjacency_matrix->rows(),
                          adjacency_matrix->cols(),
                          adjacency_matrix->nonZeros())
            : "not provided, will be computed");
    clique_generator_ = std::make_unique<CliqueGenerator>(
        robot_constraints_, clique_configs.value(), adjacency_matrix);
    logging::log()->info(
        "IrisBuilder: created with a clique generator with {} "
        "vertices and {} edges",
        clique_generator_->vertices().size(),
        clique_generator_->adjacency_matrix().nonZeros());
  } else {
    logging::log()->info("IrisBuilder: created without a clique generator");
  }
  logging::log()->info(
      "IrisBuilder: created with robot constraints hash {} and {} regions, and "
      "with a clique generator {}",
      robot_constraints_.constraints_hash(), adapter_.regions_vec().size(),
      clique_generator_ ? "enabled" : "disabled");
}

void IrisBuilder::AddRegion(
    const drake::geometry::optimization::HPolyhedron& region,
    const std::string& name) {
  const auto processed_regions = PostProcessRegion(region);
  for (int i = 0; i < drake::ssize(processed_regions); ++i) {
    const auto& processed_region = processed_regions.at(i);
    const auto processed_name {
        processed_regions.size() > 1 ? fmt::format("{}_{}", name, i) : name};
    adapter_.AddRegion(
        processed_region, processed_name, robot_constraints_.constraints_hash(),
        robot_constraints_.robot_model().continuous_revolute_joint_indices(),
        options_.intersection_samples, options_.mixing_steps);
  }
  // Save data to the file
  drake::yaml::SaveYamlFile(adapter_file_.string(), adapter_);
  // Update the inspector with the new adapter, and evaluate coverage
  inspector_ = std::make_unique<IrisInspector>(robot_constraints_, adapter_);
  inspector_->EvaluateCoverage();
}

std::optional<drake::geometry::optimization::HPolyhedron>
IrisBuilder::CalcIrisNpFromConfig(
    const Eigen::VectorXd& q,
    const std::optional<drake::geometry::optimization::Hyperellipsoid>&
        starting_ellipsoid,
    const std::optional<Eigen::MatrixXd> containment_points) const {
  drake::geometry::optimization::IrisOptions iris_options {
      options_.drake_iris_np_options};
  if (options_.existing_regions_as_obstacle) {
    iris_options.configuration_obstacles = inspector_->GetConvexSets();
  }
  iris_options.starting_ellipse = starting_ellipsoid;
  if (containment_points.has_value()) {
    std::function<bool(const drake::geometry::optimization::HPolyhedron&)>
        containment_check =
            [&containment_points](
                const drake::geometry::optimization::HPolyhedron& polytope) {
              // All columns must be inside the polytope
              for (int i = 0; i < containment_points->cols(); ++i) {
                if (!polytope.PointInSet((containment_points->col(i)))) {
                  return true;  // Return true if any point is outside
                }
              }
              return false;  // Return false if all points are inside
            };
    iris_options.termination_func = std::move(containment_check);
  }
  auto start {std::chrono::system_clock::now()};
  const auto& plant {robot_constraints_.robot_model().plant()};
  auto& plant_context {robot_constraints_.mutable_plant_context()};
  if (!robot_constraints_.robot_model().holonomic_mapping().is_identity()) {
    throw std::runtime_error(
        "IrisBuilder:CalcIrisNpFromConfig: Regular IrisNp does not support "
        "mimic joints or holonomic mappings. Use IrisNp2 or IrisZo instead.");
  }
  plant.SetPositions(&plant_context, q);
  try {
    const auto polytope = drake::geometry::optimization::IrisNp(
        plant, plant_context, iris_options);
    ReportPolytope(polytope, start, "IrisNp from Config");
    return polytope;
  } catch (const std::exception& e) {
    logging::log()->error(
        "IrisBuilder:CalcIrisNpFromConfig: failed to calculate Iris region "
        "for configuration {} due to exception: {}",
        q.transpose(), e.what());
  }
  return std::nullopt;
}

std::optional<int> IrisBuilder::GetVertexIndexFromConfiguration(
    const Eigen::VectorXd& q, double tolerance_radius) const {
  if (clique_generator_ == nullptr) {
    logging::log()->error(
        "IrisBuilder:GetVertexIndexFromConfiguration: No clique generator "
        "available, cannot search for vertex index");
    return std::nullopt;
  }
  const auto& vertices = clique_generator_->vertices();
  DRAKE_THROW_UNLESS(vertices.size() > 0);
  int closest_vertex_index = -1;
  double min_distance = std::numeric_limits<double>::max();
  // Search through all vertices to find the closest one within tolerance
  for (int i = 0; i < std::ssize(vertices); ++i) {
    const auto& vertex = vertices[i];
    // Check dimension compatibility
    DRAKE_DEMAND(vertex.size() == q.size());
    // Calculate Euclidean distance
    double distance = (vertex - q).norm();
    // Check if this vertex is closer and within tolerance
    if (distance <= tolerance_radius && distance < min_distance) {
      min_distance = distance;
      closest_vertex_index = i;
    }
  }
  if (closest_vertex_index == -1) {
    logging::log()->info(
        "IrisBuilder:GetVertexIndexFromConfiguration: No vertex found within "
        "tolerance radius {} from configuration {}",
        tolerance_radius, q.transpose());
    return std::nullopt;
  }
  logging::log()->info(
      "IrisBuilder:GetVertexIndexFromConfiguration: Found closest vertex {} "
      "at distance {} from configuration {}",
      closest_vertex_index, min_distance, q.transpose());

  return closest_vertex_index;
}

std::optional<std::pair<int, int>>
IrisBuilder::GetEdgeVertexIndicesFromConfigurationPair(
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
    double tolerance_radius) const {
  if (clique_generator_ == nullptr) {
    logging::log()->error(
        "IrisBuilder:GetEdgeVertexIndicesFromConfigurationPair: No clique "
        "generator available, cannot search for vertex indices");
    return std::nullopt;
  }
  const auto& vertices = clique_generator_->vertices();
  const auto& adjacency_matrix = clique_generator_->adjacency_matrix();
  DRAKE_THROW_UNLESS(vertices.size() > 0);
  const auto& [q1, q2] = edge;
  // Find closest vertices to each configuration
  auto vertex_index1_opt =
      GetVertexIndexFromConfiguration(q1, tolerance_radius);
  auto vertex_index2_opt =
      GetVertexIndexFromConfiguration(q2, tolerance_radius);
  if (!vertex_index1_opt.has_value() || !vertex_index2_opt.has_value()) {
    logging::log()->info(
        "IrisBuilder:GetEdgeVertexIndicesFromConfigurationPair: Could not "
        "find vertices within tolerance {} for both configurations ({}, {})",
        tolerance_radius, q1.transpose(), q2.transpose());
    return std::nullopt;
  }
  int vertex_index1 = vertex_index1_opt.value();
  int vertex_index2 = vertex_index2_opt.value();
  // Check if the vertices are the same
  if (vertex_index1 == vertex_index2) {
    logging::log()->info(
        "IrisBuilder:GetEdgeVertexIndicesFromConfigurationPair: Both "
        "configurations map to the same vertex {}, cannot form an edge",
        vertex_index1);
    return std::nullopt;
  }
  // Check if the vertices are connected in the adjacency matrix
  if (!adjacency_matrix.coeff(vertex_index1, vertex_index2)) {
    logging::log()->info(
        "IrisBuilder:GetEdgeVertexIndicesFromConfigurationPair: Vertices {} "
        "and {} are not connected in the roadmap adjacency matrix",
        vertex_index1, vertex_index2);
    return std::nullopt;
  }
  logging::log()->info(
      "IrisBuilder:GetEdgeVertexIndicesFromConfigurationPair: Successfully "
      "found connected vertex pair ({}, {}) for edge configurations ({}, {})",
      vertex_index1, vertex_index2, q1.transpose(), q2.transpose());

  return std::make_pair(vertex_index1, vertex_index2);
}

std::optional<drake::geometry::optimization::HPolyhedron>
IrisBuilder::CalcIrisNp2FromConfig(
    const Eigen::VectorXd& q,
    const std::optional<drake::geometry::optimization::Hyperellipsoid>&
        starting_ellipsoid,
    const std::optional<Eigen::MatrixXd> containment_points) const {
  drake::planning::IrisNp2Options iris_options {
      options_.drake_iris_np2_options};
  auto start {std::chrono::system_clock::now()};
  const auto& collision_checker = robot_constraints_.collision_checker();
  std::optional<drake::geometry::optimization::Hyperellipsoid>
      starting_ellipsoid_copy = starting_ellipsoid;
  if (starting_ellipsoid.has_value()) {
    starting_ellipsoid_copy = starting_ellipsoid.value();
  } else {
    // Default radius for the starting ellipsoid
    double radius = 0.005;
    starting_ellipsoid_copy =
        drake::geometry::optimization::Hyperellipsoid::MakeHypersphere(radius,
                                                                       q);
  }
  DRAKE_DEMAND(starting_ellipsoid_copy.has_value());
  // Iris Np2 does not support containment points directly,
  // so we use the starting ellipsoid to help the sample point is contained,
  // but no guarantee. The following is implemented but Drake might throw.
  iris_options.sampled_iris_options.containment_points = containment_points;
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  if (!robot_constraints_.CheckSatisfied(q, 0, check_satisfied_options)) {
    logging::log()->error(
        "IrisBuilder:CalcIrisNp2FromConfig: configuration {} does not satisfy "
        "the constraints, cannot calculate IrisNp2 region",
        q.transpose());
    return std::nullopt;
  }
  const auto* collision_checker_scene_graph =
      dynamic_cast<const drake::planning::SceneGraphCollisionChecker*>(
          &collision_checker);
  if (!collision_checker_scene_graph) {
    logging::log()->error(
        "IrisBuilder:CalcIrisNp2FromConfig: collision checker is not a "
        "SceneGraphCollisionChecker, cannot calculate IrisNp2 region");
    return std::nullopt;
  }
  try {
    // Check if the collision checker is indeed a SceneGraphCollisionChecker
    auto polytope = drake::planning::IrisNp2(*collision_checker_scene_graph,
                                             *starting_ellipsoid_copy, domain_,
                                             iris_options);
    ReportPolytope(polytope, start, "IrisNp2 from Config");
    return polytope;
  } catch (const std::exception& e) {
    logging::log()->error(
        "IrisBuilder:CalcIrisNp2FromConfig: failed to calculate IrisNp2 region "
        "for configuration {} due to exception: {}",
        q.transpose(), e.what());
  }
  return std::nullopt;
}

std::optional<drake::geometry::optimization::HPolyhedron>
IrisBuilder::CalcIrisZoFromConfig(
    const Eigen::VectorXd& q,
    const std::optional<drake::geometry::optimization::Hyperellipsoid>&
        starting_ellipsoid,
    const std::optional<Eigen::MatrixXd> containment_points) const {
  drake::planning::IrisZoOptions iris_options {options_.drake_iris_zo_options};
  auto start {std::chrono::system_clock::now()};
  const auto& collision_checker = robot_constraints_.collision_checker();
  std::optional<drake::geometry::optimization::Hyperellipsoid>
      starting_ellipsoid_copy = starting_ellipsoid;
  if (starting_ellipsoid.has_value()) {
    starting_ellipsoid_copy = starting_ellipsoid.value();
  } else {
    // Default radius for the starting ellipsoid
    double radius = 0.005;
    starting_ellipsoid_copy =
        drake::geometry::optimization::Hyperellipsoid::MakeHypersphere(radius,
                                                                       q);
  }
  DRAKE_DEMAND(starting_ellipsoid_copy.has_value());
  iris_options.sampled_iris_options.containment_points = containment_points;
  try {
    // Check if the collision checker is indeed a SceneGraphCollisionChecker
    auto polytope = drake::planning::IrisZo(
        collision_checker, *starting_ellipsoid_copy, domain_, iris_options);
    ReportPolytope(polytope, start, "IrisZo from Config");
    return polytope;
  } catch (const std::exception& e) {
    logging::log()->error(
        "IrisBuilder:CalcIrisZoFromConfig: failed to calculate IrisZo region "
        "for configuration {} due to exception: {}",
        q.transpose(), e.what());
  }
  return std::nullopt;
}

std::optional<drake::geometry::optimization::HPolyhedron>
IrisBuilder::CalcIrisRegionFromConfig(
    const Eigen::VectorXd& q, const IrisBuilder::IrisMethod& method) const {
  // Check if the configuration satisfies the constraints
  CheckSatisfiedOptions check_satisfied_options;
  check_satisfied_options.verbose = true;
  if (!robot_constraints_.CheckSatisfied(q, 0, check_satisfied_options)) {
    logging::log()->error(
        "IrisBuilder:CalcIrisNpFromConfig: configuration {} does not satisfy "
        "the constraints, cannot calculate IrisNp region",
        q.transpose());
    return std::nullopt;
  }
  // Check if the q is already covered by existing convex sets
  if (inspector_->InsideAnyRegion(q)) {
    logging::log()->info(
        "IrisBuilder:CalcIrisNpFromConfig: configuration {} is already inside "
        "an existing Iris region. Not calculating IrisNp region.",
        q.transpose());
    return std::nullopt;
  }
  std::optional<drake::geometry::optimization::Hyperellipsoid>
      starting_ellipsoid;
  const auto existing_regions = inspector_->GetConvexSets();
  bool clique_ellipsoid = true;
  if (options_.use_generation_from_cliques) {
    if (clique_generator_ == nullptr) {
      logging::log()->error(
          "IrisBuilder:CalcIrisRegionFromConfig: No clique generator "
          "available, cannot generate a clique around configuration {}",
          q.transpose());
      return std::nullopt;
    }
    logging::log()->info(
        "IrisBuilder:CalcIrisRegionFromConfig: using clique generator to "
        "generate a clique around configuration {}",
        q.transpose());
    auto vertex_index_opt = GetVertexIndexFromConfiguration(q, 1e-2);
    if (vertex_index_opt.has_value()) {
      if (options_.exclude_existing_regions_from_cliques) {
        starting_ellipsoid = clique_generator_->CalcCliqueEllipsoidAroundConfig(
            vertex_index_opt.value(), existing_regions);
      } else {
        starting_ellipsoid = clique_generator_->CalcCliqueEllipsoidAroundConfig(
            vertex_index_opt.value());
      }
    } else {
      if (options_.exclude_existing_regions_from_cliques) {
        starting_ellipsoid = clique_generator_->CalcCliqueEllipsoidAroundConfig(
            q, existing_regions);
      } else {
        starting_ellipsoid =
            clique_generator_->CalcCliqueEllipsoidAroundConfig(q);
      }
    }
  }
  clique_ellipsoid = starting_ellipsoid.has_value();
  std::optional<Eigen::MatrixXd> containment_points;
  switch (method) {
    case IrisBuilder::IrisMethod::kIrisNp: {
      if (clique_ellipsoid
          && options_.drake_iris_np_options.require_sample_point_is_contained) {
        containment_points = Eigen::MatrixXd(q.size(), 1);
        containment_points->col(0) = q;
      }
      return CalcIrisNpFromConfig(q, starting_ellipsoid, containment_points);
    }
    case IrisBuilder::IrisMethod::kIrisNp2: {
      return CalcIrisNp2FromConfig(q, starting_ellipsoid);
    }
    case IrisBuilder::IrisMethod::kIrisZo: {
      if (clique_ellipsoid
          && options_.drake_iris_zo_options.sampled_iris_options
                 .require_sample_point_is_contained) {
        containment_points = Eigen::MatrixXd(q.size(), 2);
        containment_points->col(0) = q;
        DRAKE_DEMAND(starting_ellipsoid.has_value());
        containment_points->col(1) = starting_ellipsoid->center();
      }
      return CalcIrisZoFromConfig(q, starting_ellipsoid, containment_points);
    }
    default: {
      logging::log()->error(
          "IrisBuilder:CalcIrisRegionFromConfig: unknown Iris method: {}",
          static_cast<int>(method));
      return std::nullopt;
    }
  }
  return std::nullopt;  // This line is unreachable, but added to avoid compiler
                        // warnings
}

std::optional<drake::geometry::optimization::HPolyhedron>
IrisBuilder::CalcIrisRegionFromEdge(
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
    const IrisMethod& method) const {
  const auto& [q1, q2] = edge;
  if (!robot_constraints_.CheckSatisfiedEdge(q1, q2, 0.005)) {
    logging::log()->error(
        "IrisBuilder:CalcIrisRegionFromEdge: edge ({}, {}) does not satisfy "
        "the constraints, cannot calculate Iris region",
        q1.transpose(), q2.transpose());
    return std::nullopt;
  }
  // Check if the edge is already covered by existing convex sets
  if (inspector_->IsEdgeCoveredByRegions(edge)) {
    logging::log()->info(
        "IrisBuilder:CalcIrisRegionFromEdge: edge ({}, {}) is already inside "
        "existing Iris region. Not calculating Iris region.",
        q1.transpose(), q2.transpose());
    return std::nullopt;
  }
  std::optional<drake::geometry::optimization::Hyperellipsoid>
      starting_ellipsoid;
  const auto existing_regions = inspector_->GetConvexSets();
  if (options_.use_generation_from_cliques) {
    if (clique_generator_ == nullptr) {
      logging::log()->error(
          "IrisBuilder:CalcIrisRegionFromEdge: No clique generator "
          "available, cannot generate a clique around edge ({}, {})",
          q1.transpose(), q2.transpose());
      return std::nullopt;
    }
    logging::log()->info(
        "IrisBuilder:CalcIrisRegionFromEdge: using clique generator to "
        "generate a clique around edge ({}, {})",
        q1.transpose(), q2.transpose());
    std::optional<std::pair<int, int>> vertex_indices_opt =
        GetEdgeVertexIndicesFromConfigurationPair(edge, 1e-2);
    if (vertex_indices_opt.has_value()) {
      const auto& index_pair {vertex_indices_opt.value()};
      if (options_.exclude_existing_regions_from_cliques) {
        starting_ellipsoid = clique_generator_->CalcCliqueEllipsoidAroundEdge(
            index_pair, existing_regions);
      } else {
        starting_ellipsoid =
            clique_generator_->CalcCliqueEllipsoidAroundEdge(index_pair);
      }
    } else {
      if (options_.exclude_existing_regions_from_cliques) {
        starting_ellipsoid = clique_generator_->CalcCliqueEllipsoidAroundEdge(
            edge, existing_regions);
      } else {
        starting_ellipsoid =
            clique_generator_->CalcCliqueEllipsoidAroundEdge(edge);
      }
    }
  } else {
    logging::log()->info(
        "IrisBuilder:CalcIrisRegionFromEdge: use_generation_from_cliques is "
        "disabled");
  }
  if (!starting_ellipsoid.has_value()) {
    logging::log()->info(
        "IrisBuilder:CalcIrisRegionFromEdge: Not using cliques: {} OR clique "
        "could "
        "not find an ellipsoid, using a default segment ellipsoid",
        options_.use_generation_from_cliques);
    auto affine_ball = drake::geometry::optimization::AffineBall::
        MakeAffineBallFromLineSegment(q1, q2, 0.005);
    starting_ellipsoid =
        drake::geometry::optimization::Hyperellipsoid(affine_ball);
  }
  DRAKE_DEMAND(starting_ellipsoid.has_value());
  const auto q_center = (q1 + q2) / 2.0;
  switch (method) {
    case IrisBuilder::IrisMethod::kIrisNp: {
      Eigen::MatrixXd containment_points(q_center.size(), 2);
      containment_points.col(0) = q1;
      containment_points.col(1) = q2;
      return CalcIrisNpFromConfig(q_center, starting_ellipsoid,
                                  containment_points);
    }
    case IrisBuilder::IrisMethod::kIrisNp2: {
      Eigen::MatrixXd containment_points(q_center.size(), 3);
      containment_points.col(0) = q1;
      containment_points.col(1) = q2;
      containment_points.col(2) = starting_ellipsoid->center();
      return CalcIrisNp2FromConfig(q_center, starting_ellipsoid,
                                   containment_points);
    }
    case IrisBuilder::IrisMethod::kIrisZo: {
      Eigen::MatrixXd containment_points(q_center.size(), 3);
      containment_points.col(0) = q1;
      containment_points.col(1) = q2;
      containment_points.col(2) = starting_ellipsoid->center();
      return CalcIrisZoFromConfig(q_center, starting_ellipsoid,
                                  containment_points);
    }
    default: {
      logging::log()->error(
          "IrisBuilder:CalcIrisRegionFromEdge: unknown Iris method: {}",
          static_cast<int>(method));
      return std::nullopt;
    }
  }
  return std::nullopt;  // This line is unreachable, but added to avoid compiler
                        // warnings
}

void IrisBuilder::BuildFromConfigs(
    const std::map<std::string, Eigen::VectorXd>& config_name_map,
    const IrisMethod& method) {
  logging::log()->info("IrisBuilder:BuildFromConfigs");
  for (const auto& [name, q] : config_name_map) {
    if (!inspector_->InsideAnyRegion(q)) {
      logging::log()->info(
          "IrisBuilder:BuildFromConfigs: using seed config named {}", name);
      const auto polytope_opt = CalcIrisRegionFromConfig(q, method);
      if (polytope_opt.has_value()) {
        AddRegion(polytope_opt.value(), name);
      } else {
        logging::log()->error(
            "IrisBuilder:BuildFromConfigs: failed to calculate Iris region "
            "for configuration {} named {}",
            q.transpose(), name);
      }
    } else {
      logging::log()->info(
          "IrisBuilder:BuildFromConfigs: configuration {} named {} is "
          "already inside an existing Iris region. Skipping it.",
          q.transpose(), name);
    }
  }
  logging::log()->info(
      "IrisBuilder:BuildFromConfigs: finished building Iris regions from "
      "configs. Found {} regions",
      adapter_.regions_vec().size());
  if (adapter_.regions_vec().empty()) {
    logging::log()->warn(
        "IrisBuilder:BuildFromConfigs: no regions were built from the configs");
  }
}

void IrisBuilder::BuildFromConfigs(const std::vector<Eigen::VectorXd>& configs,
                                   const IrisMethod& method,
                                   const std::string& prefix) {
  logging::log()->info("IrisBuilder:BuildFromConfigs with prefix {}", prefix);
  std::map<std::string, Eigen::VectorXd> name_to_config_map {};
  int counter = 0;
  for (const auto& config : configs) {
    const std::string name = fmt::format("{}_{}", prefix, counter);
    counter++;
    name_to_config_map.emplace(name, config);
  }
  BuildFromConfigs(name_to_config_map, method);
}

void IrisBuilder::BuildFromEdges(
    const std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd>>&
        edge_map,
    const IrisMethod& method) {
  logging::log()->info("IrisBuilder:BuildFromEdges");
  for (const auto& [name, edge] : edge_map) {
    if (!inspector_->IsEdgeCoveredByRegions(edge)) {
      logging::log()->info("IrisBuilder:BuildFromEdges: using edge named {}",
                           name);
      std::optional<drake::geometry::optimization::HPolyhedron> polytope_opt =
          CalcIrisRegionFromEdge(edge, method);
      if (polytope_opt.has_value()) {
        AddRegion(polytope_opt.value(), name);
      } else {
        logging::log()->error(
            "IrisBuilder:BuildFromEdges: failed to calculate Iris region for "
            "edge {}",
            name);
      }
    } else {
      logging::log()->info(
          "IrisBuilder:BuildFromEdges: edge {} is already covered by "
          "existing Iris regions  ",
          name);
    }
  }
  logging::log()->info(
      "IrisBuilder:BuildFromEdges: finished building Iris regions from edges. "
      "Found {} regions",
      adapter_.regions_vec().size());
  if (adapter_.regions_vec().empty()) {
    logging::log()->warn(
        "IrisBuilder:BuildFromEdges: no regions were built from the edges");
  }
}

void IrisBuilder::BuildFromEdges(
    const std::vector<std::pair<Eigen::VectorXd, Eigen::VectorXd>>& edges,
    const IrisMethod& method, const std::string& prefix) {
  logging::log()->info("IrisBuilder:BuildFromEdges with prefix {}", prefix);
  std::map<std::string, std::pair<Eigen::VectorXd, Eigen::VectorXd>>
      name_to_edge_map {};
  int counter = 0;
  for (const auto& edge : edges) {
    const std::string name = fmt::format("{}_{}", prefix, counter);
    counter++;
    name_to_edge_map.emplace(name, edge);
  }
  BuildFromEdges(name_to_edge_map, method);
}

void IrisBuilder::BuildFromPath(
    const std::vector<Eigen::VectorXd>& conf_sequence,
    const IrisMethod& method) {
  logging::log()->info("IrisBuilder:BuildFromPath with {} configurations",
                       conf_sequence.size());
  const auto edges {MakeEdgesFromConfSequence(conf_sequence)};
  BuildFromEdges(edges, method, "path");
}

std::pair<drake::geometry::optimization::HPolyhedron, bool>
IrisBuilder::RepairRegionWithConfig(
    const drake::geometry::optimization::HPolyhedron& region,
    const Eigen::VectorXd& q, bool repair_via_ellipsoid,
    ValidConfigMethod valid_config_method, const CheckSatisfiedOptions& options,
    int thread_num) const {
  // If sample is not even in the region, nothing to repair here.
  if (!region.PointInSet(q)) {
    logging::log()->debug(
        "IrisBuilder:RepairRegionWithConfig: q is not in the region");
    return {region, false};
  }
  // If q already satisfies constraints, no repair required.
  if (robot_constraints_.CheckSatisfied(q, thread_num, options)) {
    return {region, false};
  }
  const auto max_ellipsoid = region.MaximumVolumeInscribedEllipsoid();
  const auto q_ellipsoid_center = max_ellipsoid.center();
  // Find closest satisfying configuration
  std::optional<Eigen::VectorXd> q_closest_opt;
  switch (valid_config_method) {
    case ValidConfigMethod::kOpt: {
      q_closest_opt =
          robot_constraints_.CalcClosestSatisfyingConfiguration(q, thread_num);
      break;
    }
    case ValidConfigMethod::kEdge: {
      const auto step {0.001};
      q_closest_opt =
          robot_constraints_.CalcClosestSatisfyingConfigurationOnEdge(
              q, q_ellipsoid_center, thread_num, step, options);
      break;
    }
    default: {
      logging::log()->error(
          "IrisBuilder:RepairRegionWithConfig: unknown ValidConfigMethod: {}",
          magic_enum::enum_name(valid_config_method));
      return {region, false};
    }
  }
  if (!q_closest_opt.has_value()) {
    logging::log()->error(
        "IrisBuilder:RepairRegionWithConfig: failed to find closest "
        "satisfying configuration for sample {}",
        q.transpose());
    return {region, false};
  }
  const auto& q_closest = q_closest_opt.value();
  // Start from the provided region and produce a repaired copy
  drake::geometry::optimization::HPolyhedron repaired_region {region};
  if (repair_via_ellipsoid) {
    // Compute max ellipsoid and add hyperplane pushing region toward center
    const auto poly_a = max_ellipsoid.A() * (q_closest - q_ellipsoid_center);
    double poly_b =
        (max_ellipsoid.A() * q_closest).dot(q_closest - q_ellipsoid_center);
    const auto poly_a_normalized = poly_a.normalized();
    poly_b /= poly_a.norm();
    poly_b -= options_.drake_iris_zo_options.sampled_iris_options
                  .configuration_space_margin;

    Eigen::MatrixXd new_A(repaired_region.A().rows() + 1,
                          repaired_region.A().cols());
    new_A.topRows(repaired_region.A().rows()) = repaired_region.A();
    new_A.bottomRows(1) = poly_a_normalized.transpose();
    Eigen::VectorXd new_b(repaired_region.b().rows() + 1);
    new_b.head(repaired_region.b().rows()) = repaired_region.b();
    new_b(repaired_region.b().rows()) = poly_b;
    repaired_region = drake::geometry::optimization::HPolyhedron(new_A, new_b);
  } else {
    // Linearize constraints at the closest configuration and intersect
    const auto correction_polytope =
        robot_constraints_.FindSatisfactionHPolyhedron(q_closest, 0);
    repaired_region = repaired_region.Intersection(correction_polytope);
  }
  logging::log()->info(
      "IrisBuilder:RepairRegionWithConfig: produced repaired region with {} "
      "hyperplanes",
      repaired_region.A().rows());
  if (repaired_region.IsEmpty()) {
    logging::log()->error(
        "IrisBuilder:RepairRegionWithConfig: repaired region is empty");
    throw std::runtime_error(
        "IrisBuilder:RepairRegionWithConfig: repaired region is empty");
  }
  return {repaired_region, true};
}

std::pair<drake::geometry::optimization::HPolyhedron, bool>
IrisBuilder::RepairRegionViaSampling(
    const drake::geometry::optimization::HPolyhedron& region,
    const std::vector<Eigen::VectorXd>& repair_configurations, int sample_size,
    int thread_num, bool repair_via_ellipsoid) const {
  drake::RandomGenerator gen(
      static_cast<size_t>(options_.drake_iris_np_options.random_seed));
  Eigen::VectorXd q {region.ChebyshevCenter()};
  drake::geometry::optimization::HPolyhedron repaired_region {region};
  std::vector<Eigen::VectorXd> q_vec;
  CheckSatisfiedOptions options;
  options.verbose = true;
  bool repair_required {false};
  auto max_ellipsoid = region.MaximumVolumeInscribedEllipsoid();
  if (repair_via_ellipsoid) {
    // The center must be valid, otherwise the ellipsoid technique can not work
    if (!robot_constraints_.CheckSatisfied(max_ellipsoid.center(), thread_num,
                                           options)) {
      logging::log()->error(
          "IrisBuilder:RepairRegionViaSampling: max ellipsoid center {} "
          "is not a valid configuration",
          max_ellipsoid.center().transpose());
      throw std::runtime_error(
          "IrisBuilder:RepairRegionViaSampling: max ellipsoid center is "
          "not a valid configuration. Don't use ellipsoid repair "
          "technique if the center is not valid.");
    }
  }
  int repair_count = 0;
  auto all_repair_configurations {repair_configurations};
  int i = 0;
  while (i < sample_size) {
    const auto q_sampled = region.UniformSample(&gen, q);
    if (!repaired_region.PointInSet(q_sampled)) {
      logging::log()->debug(
          "IrisBuilder:RepairRegionViaSampling: q is not in the region");
      continue;
    }
    all_repair_configurations.push_back(q_sampled);
    i++;
  }
  for (const auto& q_repair : all_repair_configurations) {
    const auto& [repaired_candidate, did_repair] = RepairRegionWithConfig(
        repaired_region, q_repair, repair_via_ellipsoid,
        options_.valid_config_method, options, thread_num);
    if (!did_repair) {
      // Either q satisfied constraints already, q was not repairable, or
      // closest config couldn't be found.
      continue;
    }
    repair_required = true;
    repair_count++;
    repaired_region = repaired_candidate;
  }
  logging::log()->info(
      "IrisBuilder:RepairRegionViaSampling: repaired region has {} "
      "hyperplanes after {}/{} samples led to repair",
      repaired_region.A().rows(), repair_count,
      all_repair_configurations.size());
  return std::make_pair(repaired_region, repair_required);
}

std::vector<drake::geometry::optimization::HPolyhedron>
IrisBuilder::PostProcessRegion(
    const drake::geometry::optimization::HPolyhedron& region) const {
  std::unique_ptr<drake::geometry::optimization::ConvexSet> convex_set {
      nullptr};
  if (options_.repair_regions) {
    const auto& [repaired_region, _] {
        RepairRegionViaSampling(region, {}, options_.num_samples_for_repair, 0,
                                options_.repair_via_ellipsoid)};
    // The input argument for PartitionConvexSet is a
    // drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet> We
    // use Clone to get a copyable_unique_ptr, as it returns a std::unique_ptr
    // to the base class. Calling copy constructor on a region is not ideal but
    // much cheaper relative to repair or partitioning them.
    // https://drake.mit.edu/doxygen_cxx/classdrake_1_1copyable__unique__ptr.html
    convex_set = repaired_region.Clone();
  } else {
    // See the comment above for the use of Clone
    convex_set = region.Clone();
  }
  const auto partitioned_convex_sets {
      drake::geometry::optimization::PartitionConvexSet(
          *convex_set,
          robot_constraints_.robot_model().continuous_revolute_joint_indices(),
          options_.partition_overlap)};
  logging::log()->info(
      "IrisBuilder:PostProcessRegion: partitioned into {} convex sets",
      partitioned_convex_sets.size());
  std::vector<drake::geometry::optimization::HPolyhedron>
      partitioned_regions {};
  if (partitioned_convex_sets.size() == 1) {
    const auto* polytope {
        dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(
            partitioned_convex_sets.front().get())};
    DRAKE_DEMAND(polytope != nullptr);
    partitioned_regions.push_back(*polytope);
    return partitioned_regions;
  }
  for (const auto& convex_set : partitioned_convex_sets) {
    const auto* intersection_of_polytopes {
        dynamic_cast<const drake::geometry::optimization::Intersection*>(
            convex_set.get())};
    DRAKE_DEMAND(intersection_of_polytopes != nullptr);
    const auto polytope_of_intersections {
        GetIntersectionAsHPolyhedron(*intersection_of_polytopes)};
    partitioned_regions.push_back(polytope_of_intersections);
  }
  logging::log()->info(
      "IrisBuilder:PostProcessRegion: partitioned into {} regions",
      partitioned_regions.size());
  return partitioned_regions;
}

// std::optional<Eigen::VectorXd> IrisBuilder::EvalBestExpansionConf(
//     std::vector<Eigen::VectorXd> q_vec) const {
//   // first we evaluate all the configurations
//   std::vector<size_t> indices_covered, indices_visible, indices_not_visible;
//   for (size_t i = 0; i < q_vec.size(); ++i) {
//     if (InsideAnyRegion(q_vec.at(i))) {
//       indices_covered.push_back(i);
//     } else {
//       if (MaybeCalcEdgeToRegions(q_vec.at(i)).has_value()) {
//         indices_visible.push_back(i);
//       } else {
//         indices_not_visible.push_back(i);
//       }
//     }
//   }
//   logging::log()->info(
//       "IrisBuilder:EvalBestExpansionConf: {} covered, {} visible, {} not "
//       "visible",
//       indices_covered.size(), indices_visible.size(),
//       indices_not_visible.size());
//   // now see for each visible, how many non-visibles are visible
//   int best_visible_index {-1};
//   int best_num_visible {0};
//   for (size_t i = 0; i < indices_visible.size(); ++i) {
//     int num_visible {0};
//     const auto& q_visible = q_vec.at(indices_visible.at(i));
//     for (size_t j = 0; j < indices_not_visible.size(); ++j) {
//       const auto& q_not_visible = q_vec.at(indices_not_visible.at(j));
//       if (robot_constraints_.CheckSatisfiedEdge(
//               q_visible, q_not_visible,
//               iris_builder_options_.edge_sample_step)) {
//         ++num_visible;
//       }
//     }
//     logging::log()->info(
//         "IrisBuilder:EvalBestExpansionConf: visible index {} has {} "
//         "non-visible visible",
//         indices_visible.at(i), num_visible);
//     if (num_visible > best_num_visible) {
//       best_visible_index = indices_visible.at(i);
//       best_num_visible = num_visible;
//     }
//   }
//   if (best_visible_index == -1) {
//     if (best_num_visible == 0) {
//       logging::log()->error(
//           "IrisBuilder:EvalBestExpansionConf: no visible index found");
//     }
//     return std::nullopt;
//   }
//   logging::log()->info(
//       "IrisBuilder:EvalBestExpansionConf: best visible index {} has {} "
//       "non-visible visible",
//       best_visible_index, best_num_visible);
//   return q_vec.at(best_visible_index);
// }

}  // namespace iris
}  // namespace motion
