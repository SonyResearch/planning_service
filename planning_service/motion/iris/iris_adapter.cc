
#include "iris_adapter.h"

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

namespace fs = std::filesystem;

namespace motion {
namespace iris {

IrisRegionsAdapter::IrisRegion::IrisRegion(
    drake::geometry::optimization::HPolyhedron set, int index, std::string name,
    size_t constraints_hash)
    : set_(set),
      index_(index),
      name_(name),
      constraints_hash_(constraints_hash),
      aabb_(drake::geometry::optimization::Hyperrectangle::
                MaybeCalcAxisAlignedBoundingBox(set)) {}

const IrisRegionsAdapter::IrisRegion* IrisRegionsAdapter::AddRegion(
    const drake::geometry::optimization::HPolyhedron& set,
    const std::string& name, const size_t constraints_hash,
    const std::vector<int>& continuous_revolute_joint_indices,
    int intersection_samples, int mixing_steps) {
  // populate intersections on add (happens offline) so that we only load them
  // on launch
  if (!intersections_vec_.has_value()) {
    logging::log()->info("IrisRegionsAdapter: Initializing intersections");
    intersections_vec_ = std::vector<IrisRegionsIntersection>();
  }
  auto convex_sets_so_far = GetConvexSets();
  std::vector<drake::geometry::optimization::Hyperrectangle> aabbs_so_far;
  for (const auto& region : regions_vec_) {
    DRAKE_DEMAND(region.aabb().has_value());
    aabbs_so_far.push_back(region.aabb().value());
  }
  const int index = std::ssize(regions_vec_);
  // Add the new region to the vector of regions
  regions_vec_.emplace_back(set, index, name, constraints_hash);
  if (regions_vec_.size() == 1) {
    return &regions_vec_.back();
  }
  const auto& last_region = regions_vec_.back();
  DRAKE_DEMAND(last_region.aabb().has_value());  // Because we just added it
  drake::geometry::optimization::ConvexSets convex_sets_last_region;
  convex_sets_last_region.push_back(
      drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
          last_region.set()));
  const auto& [new_intersections, new_offsets] =
      drake::geometry::optimization::ComputePairwiseIntersections(
          convex_sets_so_far, convex_sets_last_region,
          continuous_revolute_joint_indices, aabbs_so_far,
          {regions_vec_.back().aabb().value()});
  DRAKE_DEMAND(new_intersections.size() == new_offsets.size());
  for (int i = 0; i < std::ssize(new_intersections); ++i) {
    const auto& [index_existing, index_0] = new_intersections[i];
    DRAKE_DEMAND(index_existing < index);
    DRAKE_DEMAND(index_0 == 0);
    const auto& offset = new_offsets[i];
    // intersections are pairs of indices, so we need to check if the
    // intersection is with the last region
    const auto [_, samples] = CalcIntersectionSamples(
        regions_vec_.at(index_existing).set(), regions_vec_.back().set(),
        offset, intersection_samples, mixing_steps);
    intersections_vec_->emplace_back(index_existing, index, offset, samples);
  }
  return &regions_vec_.back();
}

drake::geometry::optimization::ConvexSets IrisRegionsAdapter::GetConvexSets()
    const {
  drake::geometry::optimization::ConvexSets convex_sets;
  for (const auto& region : regions_vec_) {
    convex_sets.push_back(
        drake::copyable_unique_ptr<drake::geometry::optimization::ConvexSet>(
            region.set()));
  }
  return convex_sets;
}

std::string IrisRegionsAdapter::CalcGraphVizString() const {
  std::string graphviz_string = "graph G {\n";
  for (int i = 0; i < std::ssize(regions_vec_); ++i) {
    graphviz_string += regions_vec_.at(i).name() + ";\n";
    for (const auto& intersection : intersections_vec_.value()) {
      if ((intersection.index_one() == i && intersection.index_two() > i)
          || (intersection.index_two() == i && intersection.index_one() > i)) {
        graphviz_string +=
            regions_vec_.at(intersection.index_one()).name() + " -- "
            + regions_vec_.at(intersection.index_two()).name() + ";\n";
      }
    }
  }
  graphviz_string += "}\n";
  return graphviz_string;
}

std::pair<drake::geometry::optimization::HPolyhedron,
          std::vector<Eigen::VectorXd>>
IrisRegionsAdapter::CalcIntersectionSamples(
    const drake::geometry::optimization::ConvexSet& a,
    const drake::geometry::optimization::ConvexSet& b,
    const Eigen::VectorXd& offset, const int sample_count,
    const int mixing_steps) const {
  const auto* a_polytope =
      dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(&a);
  const auto* b_polytope =
      dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(&b);
  DRAKE_THROW_UNLESS(a_polytope != nullptr);
  DRAKE_THROW_UNLESS(b_polytope != nullptr);
  const auto shifted_b_polytope = drake::geometry::optimization::HPolyhedron(
      b_polytope->A(), b_polytope->b() - b_polytope->A() * offset);
  const auto intersecting_polytope =
      a_polytope->Intersection(shifted_b_polytope);
  DRAKE_DEMAND(!intersecting_polytope.IsEmpty());
  std::vector<Eigen::VectorXd> samples = {
      intersecting_polytope.ChebyshevCenter()};
  drake::RandomGenerator gen {0};
  for (int i {0}; i < sample_count - 1; ++i) {
    for (int i {0}; i < mixing_steps; ++i) {
      intersecting_polytope.UniformSample(&gen, samples.back());
    }
    samples.push_back(
        intersecting_polytope.UniformSample(&gen, samples.back()));
  }
  return std::pair(intersecting_polytope, samples);
}

std::optional<std::pair<double, double>>
IrisRegionsAdapter::CalcEdgeIntersectionWithRegion(
    const std::pair<Eigen::VectorXd, Eigen::VectorXd>& edge,
    const drake::geometry::optimization::HPolyhedron& region) {
  // v1 and v2 are the vertices
  // Then v = (1-lambda) * v1 + lambda * v2, lambda \in [0,1]
  //
  // consider the optimization problem:
  //
  // min_{lambda} s.t. v \in region
  //
  // max_{lambda} s.t. v \in region
  //
  // return min_lambda, max_lambda
  //
  auto prog = drake::solvers::MathematicalProgram();
  auto lambda = prog.NewContinuousVariables(1, "lambda");
  // A * x < = b
  // x = (1-lambda) * v1 + lambda * v2
  // Replace
  // A * v1 * (1-lambda) + A * v2 * lambda < = b
  // Write all as a linear function of lambda
  // (A * v2 - A * v1) * lambda <= b - A*v1
  // -infinity <= lhs_matrix * lambda <= rhs_vector

  // lambda between 0 and 1
  const auto& [u, v] {edge};
  const Eigen::VectorXd lhs_matrix = region.A() * (v - u);
  const Eigen::VectorXd rhs_vector = region.b() - region.A() * u;
  const auto lower_bound = Eigen::VectorXd::Constant(
      region.b().rows(), -std::numeric_limits<double>::infinity());
  prog.AddLinearConstraint(lhs_matrix, lower_bound, rhs_vector, lambda);
  prog.AddBoundingBoxConstraint(0, 1, lambda(0));
  // cost in drake:: a * var + b, here a=1, b=0, var = lambda
  auto cost = prog.AddLinearCost(Eigen::VectorXd::Constant(1, 1.0), 0, lambda);
  // solve the program
  auto result = drake::solvers::Solve(prog);
  if (!result.is_success()) {
    logging::log()->trace(
        "Failed to solve the optimization, the whole line segment is outside "
        "the region");
    return std::nullopt;
  }
  const auto lambda_min = result.GetSolution(lambda)[0];
  cost.evaluator()->UpdateCoefficients(Eigen::VectorXd::Constant(1, -1.0), 0);
  result = drake::solvers::Solve(prog);
  const auto lambda_max = result.GetSolution(lambda)[0];
  logging::log()->trace("Returning lambda_min = {}, and lambda_max = {}",
                        lambda_min, lambda_max);
  return std::pair(lambda_min, lambda_max);
}

}  // namespace iris
}  // namespace motion
