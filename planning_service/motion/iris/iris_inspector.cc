#include "iris_inspector.h"

#include <drake/geometry/optimization/geodesic_convexity.h>
#include <drake/geometry/optimization/intersection.h>
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

#include <random>

#include "planning_service/motion/planning/internal/geodesic_math.h"

namespace motion {
namespace iris {

IrisInspector::IrisInspector(const RobotConstraints& robot_constraints,
                             const IrisRegionsAdapter& iris_regions_adapter,
                             double check_satisfied_edge_step)
    : robot_constraints_(robot_constraints),
      iris_regions_adapter_(iris_regions_adapter),
      check_satisfied_edge_step_(check_satisfied_edge_step) {
  logging::log()->info("IrisInspector: {} regions and {} intersections loaded",
                       iris_regions_adapter_.regions_vec().size(),
                       iris_regions_adapter_.intersections_vec().has_value()
                           ? iris_regions_adapter_.intersections_vec()->size()
                           : 0);
}

bool IrisInspector::IsEdgeInsideRegion(
    const conf_edge_t& edge,
    const IrisRegionsAdapter::IrisRegion& region) const {
  return EvalConfigAgainstIrisRegion(edge.first, region, false, false).inside
         && EvalConfigAgainstIrisRegion(edge.second, region, false, false)
                .inside;
}

bool IrisInspector::IsEdgeCoveredByAnyIrisRegion(
    const conf_edge_t& edge,
    const std::vector<IrisRegionsAdapter::IrisRegion>& regions) const {
  for (const auto& region : regions) {
    if (IsEdgeInsideRegion(edge, region)) {
      return true;
    }
  }
  return false;
}

bool IrisInspector::IsValidViaSampling(
    const IrisRegionsAdapter::IrisRegion& region, const int sample_size) const {
  drake::RandomGenerator gen(0);
  Eigen::VectorXd q {region.set().ChebyshevCenter()};
  std::vector<Eigen::VectorXd> q_vec;
  for (int i = 0; i < sample_size; ++i) {
    q_vec.push_back(q);
    q = region.set().UniformSample(&gen, q);
  }
  CheckSatisfiedOptions options;
  options.verbose = true;
  return robot_constraints_.CheckSatisfied(q_vec, options);
}

void IrisInspector::SetPointsForIrisCoverageEvaluation(
    const std::vector<Eigen::VectorXd>& config_points) {
  config_points_ = config_points;
  coverage_result_types_.resize(config_points_.size(),
                                CoverageResultType::kUncovered);
  evaluated_iris_regions_indices_ = {};
}

IrisInspector::CoverageResult IrisInspector::EvaluateCoverage() {
  const long int n_points = drake::ssize(config_points_);
  if (n_points == 0) {
    logging::log()->warn(
        "IrisInspector:EvaluateCoverage: No points to evaluate coverage for.");
    return {0, 0, 0, 0};
  }
  logging::log()->info(
      "IrisInspector:EvaluateCoverage: Evaluating coverage for {} points",
      n_points);
  for (const auto& iris_region : iris_regions_adapter().regions_vec()) {
    if (std::find(evaluated_iris_regions_indices_.begin(),
                  evaluated_iris_regions_indices_.end(), iris_region.index())
        != evaluated_iris_regions_indices_.end()) {
      continue;
    }
    logging::log()->trace(
        "IrisInspector:EvaluateCoverage: Evaluating coverage for newly added "
        "iris region {} against {} points",
        iris_region.index(), n_points);
    for (int i = 0; i < n_points; ++i) {
      if (coverage_result_types_.at(i) == CoverageResultType::kCoveredByVolume
          || coverage_result_types_.at(i)
                 == CoverageResultType::kViolatesConstraints) {
        continue;
      }
      const Eigen::VectorXd& config_point = config_points_.at(i);
      const bool check_visibility = coverage_result_types_.at(i)
                                    != CoverageResultType::kCoveredByVisibility;
      const IrisInspector::ConfigAgainstIrisRegionResult result =
          EvalConfigAgainstIrisRegion(config_point, iris_region, true,
                                      check_visibility);
      DRAKE_DEMAND(result.check_satisfied.has_value());
      if (result.inside) {
        coverage_result_types_.at(i) = CoverageResultType::kCoveredByVolume;
      } else if (result.connecting_set.has_value()) {
        coverage_result_types_.at(i) = CoverageResultType::kCoveredByVisibility;
      } else if (!result.check_satisfied.value()) {
        coverage_result_types_.at(i) = CoverageResultType::kViolatesConstraints;
      } else if (coverage_result_types_.at(i)
                 == CoverageResultType::kUncovered) {
        // uncovered remains uncovered
        coverage_result_types_.at(i) = CoverageResultType::kUncovered;
      }
    }
    evaluated_iris_regions_indices_.push_back(iris_region.index());
  }
  CoverageResult result = {0, 0, 0, 0};
  for (const auto& coverage_result_type : coverage_result_types_) {
    switch (coverage_result_type) {
      case CoverageResultType::kCoveredByVolume:
        result.volume_covered++;
        break;
      case CoverageResultType::kCoveredByVisibility:
        result.visibility_covered++;
        break;
      case CoverageResultType::kUncovered:
        result.uncovered++;
        break;
      case CoverageResultType::kViolatesConstraints:
        result.violates_constraints++;
        break;
    }
  }
  logging::log()->info(fmt::format(
      fg(FMT_MAGENTA),
      "IrisInspector::EvaluateCoverage result: volume_covered: {}, "
      "visibility_covered: {}, uncovered: {}, violates_constraints: {}",
      result.volume_covered, result.visibility_covered, result.uncovered,
      result.violates_constraints));
  return result;
}

IrisInspector::ConfigAgainstRegionResult IrisInspector::EvalConfigAgainstRegion(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::HPolyhedron& region,
    const bool check_satisfied, const bool check_visibility) const {
  ConfigAgainstRegionResult result {.inside = region.PointInSet(q),
                                    .check_satisfied = std::nullopt,
                                    .visible_point = std::nullopt};
  if (check_satisfied) {
    result.check_satisfied = robot_constraints_.CheckSatisfied(q, 0);
  }
  if (result.inside) {
    // if q is inside the region, then we don't need to check visibility
    return result;
  }
  if (check_visibility) {
    // let's solve the mathematical program of closest point
    auto prog = drake::solvers::MathematicalProgram();
    const int dim = region.ambient_dimension();
    auto x = prog.NewContinuousVariables(dim, "x");
    // region.AddPointInSetConstraints(&prog, x);
    const double eps = 1e-6;
    const auto eps_vec = Eigen::VectorXd::Constant(region.b().size(), eps);
    prog.AddLinearConstraint(
        region.A(),
        Eigen::VectorXd::Constant(region.b().size(),
                                  -std::numeric_limits<double>::infinity()),
        region.b() - eps_vec, x);
    // Add the distance of x and q as a cost
    prog.AddQuadraticCost(Eigen::MatrixXd::Identity(dim, dim), -q,
                          0.5 * q.squaredNorm(), x);
    // add the constraint that x to q on continuous revolute joints
    // does not exceed pi
    for (const auto& joint_index :
         robot_constraints_.robot_model().continuous_revolute_joint_indices()) {
      prog.AddLinearConstraint(x(joint_index) - q(joint_index) <= M_PI);
      prog.AddLinearConstraint(q(joint_index) - x(joint_index) <= M_PI);
    }
    drake::solvers::MathematicalProgramResult mp_result;
    // the order of preferred solvers: mosek, gurobi, clarabel, osqp, scs
    const std::vector<drake::solvers::SolverId> preferred_solvers {
        drake::solvers::MosekSolver::id(), drake::solvers::GurobiSolver::id(),
        drake::solvers::ClarabelSolver::id(), drake::solvers::OsqpSolver::id(),
        drake::solvers::ScsSolver::id()};
    auto solver = drake::solvers::MakeFirstAvailableSolver(preferred_solvers);
    solver->Solve(prog, {}, {}, &mp_result);
    if (!mp_result.is_success()) {
      logging::log()->error(
          "IrisInspector:EvalConfigAgainstRegion: failed to solve the closest "
          "point problem with solver: {}",
          solver->solver_id().name());
      return result;
    }
    const auto q_c {mp_result.GetSolution(x)};
    // now check the line of sight between q_c and q
    if (robot_constraints_.CheckSatisfiedEdge(q, q_c,
                                              check_satisfied_edge_step_)) {
      result.visible_point = q_c;
    }
  }
  return result;
}

namespace {
double CalcDistanceUpperBoundToRegion(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::HPolyhedron& region) {
  const Eigen::VectorXd error = region.A() * q - region.b();
  // Sadra: decide on the norm to use here.
  // double distance = error.lpNorm<Eigen::Infinity>();
  double distance = 0.0;
  for (int i = 0; i < std::ssize(error); ++i) {
    if (error(i) > 0) {
      // distance += error(i) / region.A().row(i).norm();
      distance = std::max(distance, error(i) / region.A().row(i).norm());
    }
  }
  return distance;
}
}  // namespace

IrisInspector::ConfigAgainstIrisRegionResult
IrisInspector::EvalConfigAgainstIrisRegion(
    const Eigen::VectorXd& q, const IrisRegionsAdapter::IrisRegion& iris_region,
    const bool check_satisfied, const bool check_visibility) const {
  logging::log()->trace(
      "IrisInspector:EvalConfigAgainstIrisRegion: evaluating config {} against "
      "iris region {} with check_satisfied = {}, check_visibility = {}",
      q.transpose(), iris_region.index(), check_satisfied, check_visibility);
  ConfigAgainstIrisRegionResult result;
  if (check_satisfied) {
    result.check_satisfied = robot_constraints_.CheckSatisfied(q, 0);
  }
  if (!iris_region.aabb().has_value()) {
    throw std::runtime_error(
        "IrisInspector:EvalConfigAgainstIrisRegion: aabb is not available. "
        "Consider migrating.");
  }
  // When we want to check the edge from a config to a iris region set in the
  // presence of continuous revolute joints, we need to consider the following
  // cases given the bounds of the set:
  // 0. The angle is within the bounds, no wrap is needed.
  // 1. The angle is outside the bounds, we need to wrap it such that it is
  // placed within pi distance from the lower bound.
  // 2. The angle is outside the bounds, we need to wrap it such that it is
  // placed within pi distance from the upper bound.
  // We use a huerestic to sort these distances. Typically, we want to
  // find the shortest distance to the bounds and run CheckSatisfied on that.
  // Given the two cases for each joint, we may need to construct a
  // combinatorial number of configurations to check the distance of the config
  // to the iris region set.
  const auto& aabb = iris_region.aabb().value();
  // If check visibility is false, then we give up on checking if the point is
  // not possible to be inside the region. This is because we don't need to
  // check the visibility of the point.
  const bool exit_if_inside_impossible = !check_visibility;
  // Find all possible wrap multiples for each joint (the number of times 2pi is
  // added to the angle to make it within pi distance from the bounds). The size
  // of the std::vector<int> in the following is at most 2, because we have two
  // cases for each joint.
  std::map<int, std::vector<int>> wrap_multiples_map =
      internal::CalcWrapMultiplesPerJoint(
          q, aabb,
          robot_constraints_.robot_model().continuous_revolute_joint_indices(),
          exit_if_inside_impossible);
  if (!check_visibility && wrap_multiples_map.empty()
      && !robot_constraints_.robot_model()
              .continuous_revolute_joint_indices()
              .empty()) {
    // then the point is not possible to be inside the region
    result.inside = false;
    logging::log()->debug(
        "IrisInspector:EvalConfigAgainstIrisRegion: point {} is not possible "
        "to "
        "be inside region {}",
        q.transpose(), iris_region.index());
    // let's add the distance
    result.distance_upper_bound =
        CalcDistanceUpperBoundToRegion(q, iris_region.set());
    return result;
  }
  // Generate all possible combinations of wrap multiples for each joint.
  const std::vector<std::map<int, int>> wrap_multiples_combinations =
      internal::CartesianProductOfWrapMultiples(wrap_multiples_map);
  // Calc all possible wrapped configurations sorted based on their manhattan
  // distance to the AABB.
  const std::vector<Eigen::VectorXd> wrapped_configs =
      internal::CalcSortedWrappedConfigs(q, aabb, wrap_multiples_combinations);
  const bool kCheckSatisfiedAgain = false;  // we already did this.
  // TODO(@sadraddini) refactor EvalConfigAgainstRegion to not call
  // CheckSatisfied again.
  for (const auto& q_wrapped : wrapped_configs) {
    const auto result_wrapped = EvalConfigAgainstRegion(
        q_wrapped, iris_region.set(), kCheckSatisfiedAgain, check_visibility);
    if (result_wrapped.inside) {
      result.inside = true;
      // If inside, only one wrap config is allowed
      DRAKE_DEMAND(wrapped_configs.size() == 1);
      // no need to check visibuility, we are already inside.
      return result;
    }
    if (result_wrapped.visible_point.has_value()) {
      const auto q_offset = q_wrapped - q;
      Eigen::VectorXd q_connected = result_wrapped.visible_point.value();
      Eigen::MatrixXd vertices = Eigen::MatrixXd::Zero(q.size(), 2);
      // we construct the V-polytope with two vertices in the unwrapped
      // (original) config. The GCS machinery will take care of the wrapping
      // under the hood in Drake.
      vertices.col(0) = q;
      vertices.col(1) = q_connected - q_offset;
      result.connecting_set =
          drake::geometry::optimization::VPolytope(vertices);
      // We are of course not inside by this point.
      DRAKE_DEMAND(!result.inside);
      return result;
    }
    if (!check_visibility) {
      // compute an upper bound on the distance to the region
      DRAKE_DEMAND(!result.distance_upper_bound.has_value());
      result.distance_upper_bound =
          CalcDistanceUpperBoundToRegion(q_wrapped, iris_region.set());
    }
  }
  // since are not inside, and we didn't find a visible point, we are not
  // inside.
  DRAKE_DEMAND(!result.inside);
  DRAKE_DEMAND(!result.connecting_set.has_value());
  return result;
}

IrisInspector::ConfigAgainstIrisRegionsResult
IrisInspector::EvalConfigAgainstIrisRegions(const Eigen::VectorXd& q) const {
  // First, let's evaluate with checking inside only
  ConfigAgainstIrisRegionsResult result;
  std::vector<double> distance_upper_bounds;
  for (const auto& iris_region : iris_regions_adapter_.regions_vec()) {
    const auto result_against_region =
        EvalConfigAgainstIrisRegion(q, iris_region, false, false);
    if (result_against_region.inside) {
      logging::log()->debug(
          "IrisInspector:EvalConfigAgainstIrisRegions: {} inside region {}",
          q.transpose(), iris_region.index());
      result.containing_regions_indices.push_back(iris_region.index());
    } else {
      DRAKE_DEMAND(result_against_region.distance_upper_bound.has_value());
      distance_upper_bounds.push_back(
          result_against_region.distance_upper_bound.value());
      logging::log()->debug(
          "IrisInspector:EvalConfigAgainstIrisRegions: not inside region {}, "
          "distance upper bound {:.3f}",
          iris_region.index(),
          result_against_region.distance_upper_bound.value());
    }
  }
  if (!result.containing_regions_indices.empty()) {
    return result;
  }
  // Sort region indices by their distance upper bounds
  std::vector<size_t> indices(distance_upper_bounds.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(),
            [&distance_upper_bounds](size_t i, size_t j) {
              return distance_upper_bounds.at(i) < distance_upper_bounds.at(j);
            });
  // Log the sorted distance upper bounds
  Eigen::VectorXd distance_upper_bounds_eigen(distance_upper_bounds.size());
  // And do the same for the indices
  Eigen::VectorXi indices_eigen(indices.size());
  for (size_t i = 0; i < indices.size(); ++i) {
    indices_eigen(i) = static_cast<int>(indices.at(i));
    distance_upper_bounds_eigen(i) = distance_upper_bounds.at(indices.at(i));
  }
  logging::log()->info(
      "IrisInspector:EvalConfigAgainstIrisRegions: Sorted distance upper "
      "bounds for {} regions: \n{}",
      distance_upper_bounds.size(), distance_upper_bounds_eigen.transpose());
  logging::log()->info(
      "IrisInspector:EvalConfigAgainstIrisRegions: Sorted indices for distance "
      "upper bounds: \n{}",
      indices_eigen.transpose());
  // Compute the visibility (expensive) in the order of the sorted
  // distance_upper_bounds This is likely to finish faster.
  logging::log()->info(
      "IrisInspector:EvalConfigAgainstIrisRegions: Checking visibility against "
      "{} IRIS regions for config {}",
      indices.size(), q.transpose());
  for (const auto& index : indices) {
    const auto& iris_region = iris_regions_adapter_.regions_vec().at(index);
    const auto result_against_region =
        EvalConfigAgainstIrisRegion(q, iris_region, false, true);
    if (result_against_region.connecting_set.has_value()) {
      const auto& vertices =
          result_against_region.connecting_set.value().vertices();
      result.visible_region_index = iris_region.index();
      result.visible_point = vertices.col(1);
      logging::log()->info(
          "IrisInspector:EvalConfigAgainstIrisRegions: visible from "
          "region {}. Visible point distance: {}",
          iris_region.index(), (result.visible_point.value() - q).transpose());
      return result;
    } else {
      logging::log()->debug(
          "IrisInspector:EvalConfigAgainstIrisRegions: not visible from "
          "region {}",
          iris_region.index());
    }
  }
  logging::log()->error(
      "IrisInspector:EvalConfigAgainstIrisRegions: No visible region found");
  return result;
}

bool IrisInspector::InsideAnyRegion(const Eigen::VectorXd& q) const {
  for (const auto& iris_region : iris_regions_adapter_.regions_vec()) {
    if (EvalConfigAgainstRegion(q, iris_region.set()).inside) {
      return true;
    }
  }
  return false;  // not inside any convex set
}

bool IrisInspector::IsEdgeCoveredByRegions(
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge) const {
  const auto& [q1, q2] = edge;
  double distance = (q2 - q1).norm();
  if (distance < check_satisfied_edge_step_) {
    // If the edge is too short, we consider it as not covered.
    return InsideAnyRegion(q1) && InsideAnyRegion(q2);
  }
  for (double step = 0; step <= distance; step += check_satisfied_edge_step_) {
    Eigen::VectorXd q = q1 + step / distance * (q2 - q1);
    if (!InsideAnyRegion(q)) {
      return false;  // if any point on the edge is not inside, then it's not
                     // covered
    }
  }
  return true;  // all points on the edge are inside the regions
}

std::vector<int> IrisInspector::GetRegionsContainingConfig(
    const Eigen::VectorXd& q) const {
  std::vector<int> regions_containing_point;
  for (const auto& iris_region : iris_regions_adapter_.regions_vec()) {
    if (EvalConfigAgainstRegion(q, iris_region.set()).inside) {
      regions_containing_point.push_back(iris_region.index());
    }
  }
  return regions_containing_point;
}

std::optional<std::pair<Eigen::VectorXd, double>>
IrisInspector::CalcClosestValidConfToRegion(
    const Eigen::VectorXd& q, const IrisRegionsAdapter::IrisRegion& iris_region,
    const std::vector<drake::multibody::ModelInstanceIndex>
        fixed_model_instances) const {
  auto prog = drake::solvers::MathematicalProgram();
  const auto& plant = robot_constraints_.robot_model().plant();
  auto q_var = prog.NewContinuousVariables(plant.num_positions(), "q");
  if (robot_constraints_.robot_model()
          .continuous_revolute_joint_indices()
          .size()
      > 0) {
    throw std::runtime_error(
        "IrisInspector:CalcClosestValidConfToRegion: not implemented for "
        "models "
        "with continuous revolute joints");
  }
  // Same for plants with mimic joints
  if (!robot_constraints_.robot_model().holonomic_mapping().is_identity()) {
    throw std::runtime_error(
        "IrisInspector:CalcClosestValidConfToRegion: not implemented yet for "
        "models with mimic joints");
  }
  const double complete_weight = 1;
  // Let's fix the fixed_model_instances of the configuration
  for (const auto& model_idx : fixed_model_instances) {
    int start_index =
        robot_constraints_.robot_model().GetModelStartIndex(model_idx);
    const auto q_model = q.segment(start_index, plant.num_positions(model_idx));
    auto Q = Eigen::MatrixXd::Identity(plant.num_positions(model_idx),
                                       plant.num_positions(model_idx));
    prog.AddQuadraticErrorCost(
        Q * complete_weight, q_model,
        q_var.segment(start_index, plant.num_positions(model_idx)));
  }
  auto Q_all =
      Eigen::MatrixXd::Identity(plant.num_positions(), plant.num_positions());
  prog.AddQuadraticErrorCost(Q_all, q, q_var);
  // Now add the constraint that the configuration is inside the iris region
  const auto& region = iris_region.set();
  region.AddPointInSetConstraints(&prog, q_var);
  // Solve the program
  drake::solvers::MathematicalProgramResult mp_result;
  // the order of preferred solvers: mosek, gurobi, clarabel, osqp, scs
  const std::vector<drake::solvers::SolverId> preferred_solvers {
      drake::solvers::MosekSolver::id(), drake::solvers::GurobiSolver::id(),
      drake::solvers::ClarabelSolver::id(), drake::solvers::OsqpSolver::id(),
      drake::solvers::ScsSolver::id()};
  auto solver = drake::solvers::MakeFirstAvailableSolver(preferred_solvers);
  solver->Solve(prog, {}, {}, &mp_result);
  if (!mp_result.is_success()) {
    logging::log()->error(
        "IrisInspector:FindClosestCompleteSysConf: failed to solve the closest "
        "point problem to region {} with solver: {}",
        iris_region.name(), solver->solver_id().name());
    return std::nullopt;
  }
  // Let's replace the part that is fixed
  Eigen::VectorXd q_sol = mp_result.GetSolution(q_var);
  for (const auto& model_instance : fixed_model_instances) {
    auto q_model =
        robot_constraints_.robot_model().plant().GetPositionsFromArray(
            model_instance, q);
    robot_constraints_.robot_model().plant().SetPositionsInArray(
        model_instance, q_model, &q_sol);
  }
  // Let's run check satisfied
  if (!robot_constraints_.CheckSatisfied(q_sol, 0)) {
    logging::log()->error(
        "IrisInspector:CalcClosestValidConfToRegion: solution found but not "
        "valid for region {}",
        iris_region.name());
    return std::nullopt;
  }
  logging::log()->debug(
      "IrisInspector:CalcClosestValidConfToRegion: found closest configuration "
      "to region {} at q_sol: {}",
      iris_region.name(), q_sol.transpose());
  return std::make_pair(q_sol, mp_result.get_optimal_cost());
}

std::optional<Eigen::VectorXd> IrisInspector::CalcClosestValidConfToRegions(
    const Eigen::VectorXd& q,
    const std::vector<drake::multibody::ModelInstanceIndex>
        fixed_model_instances) const {
  auto min_distance = std::numeric_limits<double>::infinity();
  std::optional<Eigen::VectorXd> q_opt;
  // First, let's order by simplified distances
  std::vector<const IrisRegionsAdapter::IrisRegion*> sorted_regions;
  for (const auto& iris_region : iris_regions_adapter_.regions_vec()) {
    sorted_regions.push_back(&iris_region);
  }
  std::sort(sorted_regions.begin(), sorted_regions.end(),
            [&q](const IrisRegionsAdapter::IrisRegion* a,
                 const IrisRegionsAdapter::IrisRegion* b) {
              const auto a_distance =
                  (a->set().A() * q - a->set().b()).array().maxCoeff();
              const auto b_distance =
                  (b->set().A() * q - b->set().b()).array().maxCoeff();
              return a_distance < b_distance;
            });
  for (const auto* iris_region : sorted_regions) {
    const auto result_opt =
        CalcClosestValidConfToRegion(q, *iris_region, fixed_model_instances);
    if (result_opt.has_value()) {
      const auto& [q, distance] = result_opt.value();
      if (distance < min_distance) {
        min_distance = distance;
        q_opt = q;
      }
    }
  }
  if (!q_opt.has_value()) {
    logging::log()->error(
        "IrisInspector:CalcClosestValidConfToRegions: failed to find "
        "closest configuration to any of the regions");
  }
  return q_opt;
}

std::optional<std::pair<int, Eigen::VectorXd>>
IrisInspector::MaybeCalcEdgeToRegions(const Eigen::VectorXd& q) const {
  // first, let's order by simplified distances
  ConfigAgainstIrisRegionsResult result = EvalConfigAgainstIrisRegions(q);
  if (result.containing_regions_indices.size() > 0) {
    logging::log()->info(
        "IrisInspector:MaybeCalcEdgeToRegions: found {} containing regions for "
        "config {}",
        result.containing_regions_indices.size(), q.transpose());
    // let's return the first region and the config itself
    return std::make_pair(result.containing_regions_indices.at(0), q);
  }
  if (result.visible_region_index.has_value()) {
    logging::log()->info(
        "IrisInspector:MaybeCalcEdgeToRegions: found visible region {} for "
        "config {}",
        result.visible_region_index.value(), q.transpose());
    // let's return the visible region and the visible point
    return std::make_pair(result.visible_region_index.value(),
                          result.visible_point.value());
  }
  logging::log()->error(
      "IrisInspector:MaybeCalcEdgeToRegions: no edge to regions found");
  return std::nullopt;
}

}  // namespace iris
}  // namespace motion
