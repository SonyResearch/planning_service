#pragma once

#include <Eigen/Dense>
#include <drake/geometry/optimization/hyperrectangle.h>

#include <map>
#include <vector>

namespace motion {
namespace internal {
// Calculates the multiple of 2pi to add to the angle_query to make it within pi
// distance from the reference angle_ref
int CalcWrapMultiple(const double angle_query, const double angle_ref);

// Given a reference configuration q_ref, configuration q, and a vector of
// continuous revolute joint indices, modifies the configuration such that the
// continuous revolute joints are within pi distance from the reference
// configuration.
void WrapConfiguration(
    const Eigen::VectorXd& q_ref,
    const std::vector<int>& continuous_revolute_joint_indices,
    Eigen::VectorXd* q);

// Given a map from joint index to a vector of wrap multiples, returns a vector
// of maps from joint index to wrap multiple. Each map in the vector represents
// a possible combination of wrap multiples.
// For example, if we have three continuous joints such that
// joint 1 can be shifted by n_1 or n_2 multiple of 2pi and
// joint 2 can be shifted by m multiple of 2pi, and
// joint 3 can be shifted by k_1 or k_2 multiple of 2pi, then
// the result will be a vector of maps such that each map has
// joint index as key and the multiple of 2pi as value.
// For the above example, the result will be:
// [{1: n_1, 2: m, 3: k_1}, {1: n_1, 2: m, 3: k_2},
//  {1: n_2, 2: m, 3: k_1}, {1: n_2, 2: m, 3: k_2}]
std::vector<std::map<int, int>> CartesianProductOfWrapMultiples(
    const std::map<int, std::vector<int>>& joint_to_offset_wraps);

// Given  a configuration q, axis-aligned-bounding-box,
// continuous_revolute_joint_indices, and a flag to possibly exit if being
// inside the region, returns a map from joint index to a vector of wrap
// multiples.
std::map<int, std::vector<int>> CalcWrapMultiplesPerJoint(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::Hyperrectangle& aabb,
    const std::vector<int>& continuous_revolute_joint_indices,
    const bool exit_if_inside_impossible);

// Check if a multiple wrap is possible for any joint. Used for sanity checks.
bool MultipleWrapsPossible(
    const std::map<int, std::vector<int>>& joint_to_offset_wraps);

// Given a configuration q and an axis-aligned-bounding-box, returns the
// manhattan distance to the AABB. The reason for using Manhattan distance is
// that it is faster to compute than the Euclidean distance (which requires
// solving a QP problem to find the closest point on the AABB to the query point
// q)
double CalcManhattanDistanceToAABB(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::Hyperrectangle& aabb);

// Given a configuration q, an axis-aligned-bounding-box, and a vector of
// possible wrap multiples for each joint, returns a vector of wrapped
// configurations sorted by the manhattan distance to the AABB.
std::vector<Eigen::VectorXd> CalcSortedWrappedConfigs(
    const Eigen::VectorXd& q,
    const drake::geometry::optimization::Hyperrectangle& aabb,
    const std::vector<std::map<int, int>>& joint_to_offset_wraps);

}  // namespace internal
}  // namespace motion
