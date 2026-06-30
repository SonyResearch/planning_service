/// @file straight_path_planner.h

#pragma once

#include <drake/common/trajectories/piecewise_polynomial.h>

#include <expected>

#include "planning_service/motion/robot_constraints.h"

namespace motion {
namespace planning {

/** If the path between the start and end points is valid, returns the
path as a trajectory. Otherwise, returns nullopt.

@param robot_constraints The RobotConstraints object that defines the robot's
model and constraints.
@param start_point The start point of the path.
@param end_point The end point of the path.
@param step_size The step size to use when checking the path for validity.

note: Both start_point and end_point are expected to be in the same
configuration space. If some joints are continuous revolute joints, the
end_point will be wrapped to within π distance from start_point. The path is
checked for validity by calling robot_constraints.CheckSatisfiedEdge().
*/
std::optional<drake::trajectories::PiecewisePolynomial<double>>
MaybeValidStraightLinePath(const RobotConstraints& robot_constraints,
                           Eigen::VectorXd start_point,
                           Eigen::VectorXd end_point, double step_size = 0.01);

/**
* Compute a straight line trajectory that moves from the start point to the end
that
* does not violate the constraints.
@param robot_constraints The RobotConstraints object that defines the robot's
model and constraints.
@param start_violating_point The start point of the path.
See CalcClosestSatisfyingConfiguration for more details.
*/
std::expected<drake::trajectories::PiecewisePolynomial<double>, std::string>
MaybeOutOfViolationPath(const RobotConstraints& robot_constraints,
                        const Eigen::VectorXd& start_violating_point,
                        double collision_influence_distance = 0.1,
                        double joint_clearance = 0.01, int thread_num = 0,
                        std::optional<Eigen::VectorXd> gradient = std::nullopt);

}  // namespace planning
}  // namespace motion
