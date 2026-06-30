#include "planning_service/motion/planning/internal/geodesic_math.h"

namespace motion {
namespace internal {

int CalcWrapMultiple(const double angle_query, const double angle_ref) {
  const double diff = angle_query - angle_ref;
  return -std::floor((diff + M_PI) / (2 * M_PI));
}

void WrapConfiguration(
    const Eigen::VectorXd& q_ref,
    const std::vector<int>& continuous_revolute_joint_indices,
    Eigen::VectorXd* q) {
  DRAKE_THROW_UNLESS(q->rows() == q_ref.rows());
  for (const auto& joint_index : continuous_revolute_joint_indices) {
    const int wrap_multiple =
        CalcWrapMultiple((*q)(joint_index), q_ref(joint_index));
    (*q)(joint_index) += wrap_multiple * 2 * M_PI;
  }
}

std::vector<std::map<int, int>> CartesianProductOfWrapMultiples(
    const std::map<int, std::vector<int>>& joint_to_offset_wraps) {
  std::vector<std::map<int, int>> result_prior_adding_depth = {{}};
  for (const auto& [joint_index, offset_wraps] : joint_to_offset_wraps) {
    DRAKE_THROW_UNLESS(offset_wraps.size() == 1 || offset_wraps.size() == 2);
    std::vector<std::map<int, int>> result_post_adding_depth;
    for (const auto& existing_map : result_prior_adding_depth) {
      for (const int offset_wrap : offset_wraps) {
        std::map<int, int> new_map = existing_map;
        new_map.emplace(joint_index, offset_wrap);
        result_post_adding_depth.push_back(new_map);
      }
    }
    result_prior_adding_depth = result_post_adding_depth;
  }
  return result_prior_adding_depth;
}

std::map<int, std::vector<int>> CalcWrapMultiplesPerJoint(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::Hyperrectangle& aabb,
    const std::vector<int>& continuous_revolute_joint_indices,
    const bool exit_if_inside_impossible) {
  std::map<int, std::vector<int>> joint_to_offset_wraps;
  for (const auto& joint_index : continuous_revolute_joint_indices) {
    std::vector<int> offset_wraps;
    // let's wrap for the lower bound
    const int wrap_multiple_lower_bound =
        CalcWrapMultiple(q(joint_index), aabb.lb()(joint_index));
    const int wrap_multiple_upper_bound =
        CalcWrapMultiple(q(joint_index), aabb.ub()(joint_index));
    if (wrap_multiple_lower_bound != wrap_multiple_upper_bound) {
      offset_wraps.push_back(wrap_multiple_lower_bound);
      offset_wraps.push_back(wrap_multiple_upper_bound);
      if (exit_if_inside_impossible) {
        // If the angle needs multiple wraps, it means that it one of the wraps
        // could not get it closer than pi distance from the bounds, so it is
        // not possible to be inside the region. Note: we enforce that the
        // length of every iris region projected on fully revoluted joints is
        // less than pi. Otherwise, those regions would fail
        // CheckIfSatisfiesConvexityRadius as described here:
        // https://drake.mit.edu/doxygen_cxx/namespacedrake_1_1geometry_1_1optimization.html#a05bd75c8b08787c334065792f8532797
        return {};
      }
    } else {
      offset_wraps.push_back(wrap_multiple_lower_bound);
    }
    joint_to_offset_wraps.emplace(joint_index, offset_wraps);
  }
  DRAKE_DEMAND(joint_to_offset_wraps.size()
               == continuous_revolute_joint_indices.size());
  return joint_to_offset_wraps;
}

bool MultipleWrapsPossible(
    const std::map<int, std::vector<int>>& joint_to_offset_wraps) {
  for (const auto& [joint_index, wrap_multiples] : joint_to_offset_wraps) {
    // Only two wrap multiples are possible for a joint. Otherwise it is a bug.
    DRAKE_THROW_UNLESS(wrap_multiples.size() == 1
                       || wrap_multiples.size() == 2);
    if (wrap_multiples.size() == 2) {
      return true;
    }
  }
  return false;
}

double CalcManhattanDistanceToAABB(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::Hyperrectangle& aabb) {
  DRAKE_THROW_UNLESS(q.rows() == aabb.ambient_dimension());
  double distance = 0;
  for (int i = 0; i < q.size(); ++i) {
    if (q(i) > aabb.lb()(i) && q(i) < aabb.ub()(i)) {
      // q(i) is inside the bounds
      continue;
    }
    const double lower_bound_distance = std::abs(q(i) - aabb.lb()(i));
    const double upper_bound_distance = std::abs(q(i) - aabb.ub()(i));
    distance += std::min(lower_bound_distance, upper_bound_distance);
  }
  return distance;
}

std::vector<Eigen::VectorXd> CalcSortedWrappedConfigs(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::Hyperrectangle& aabb,
    const std::vector<std::map<int, int>>& joint_to_offset_wraps) {
  std::vector<Eigen::VectorXd> wrapped_configs;
  std::vector<double> distance_to_bounds;
  for (const auto& wrap_map : joint_to_offset_wraps) {
    Eigen::VectorXd wrapped_q = q;
    for (const auto& [joint_index, wrap_multiple] : wrap_map) {
      wrapped_q(joint_index) += wrap_multiple * 2 * M_PI;
    }
    wrapped_configs.push_back(wrapped_q);
    distance_to_bounds.push_back(CalcManhattanDistanceToAABB(wrapped_q, aabb));
  }
  std::vector<int> sorted_indices(distance_to_bounds.size());
  std::iota(sorted_indices.begin(), sorted_indices.end(), 0);
  std::sort(sorted_indices.begin(), sorted_indices.end(),
            [&distance_to_bounds](int i, int j) {
              return distance_to_bounds[i] < distance_to_bounds[j];
            });
  std::vector<Eigen::VectorXd> sorted_wrapped_configs;
  for (const int index : sorted_indices) {
    sorted_wrapped_configs.push_back(wrapped_configs[index]);
  }
  return sorted_wrapped_configs;
}

}  // namespace internal
}  // namespace motion
