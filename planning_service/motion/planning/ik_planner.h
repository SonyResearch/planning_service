/*
 * Copyright © 2024 Dexai Robotics. All rights reserved.
 */

/// @file ik_planner.h

#pragma once
#include <drake/common/trajectories/piecewise_polynomial.h>
#include <drake/multibody/math/spatial_algebra.h>
#include <drake/solvers/constraint.h>
#include <drake/solvers/cost.h>
#include <drake/solvers/mathematical_program.h>

#include <future>
#include <mutex>
#include <thread>

#include <magic_enum/magic_enum.hpp>

#include "planning_service/motion/planning/ik_cache.h"
#include "planning_service/motion/planning/try_multiple_approaches.h"
#include "planning_service/motion/robot_constraints.h"

namespace motion {
namespace planning {

struct IkPlannerOptions {
  /** number of threads to use for IK planning.*/
  int num_threads {static_cast<int>(std::thread::hardware_concurrency())};

  /** Tolerance for the position of the frame in the IK problem.*/
  Eigen::Vector3d position_tolerance {0.0005, 0.0005, 0.0005};

  /** Tolerance for the orientation of the frame in the IK problem.*/
  double orientation_tolerance {0.0005};

  /** Minimum spacing between two consecutive poses in a linear Cartesian
   * plan.*/
  const double min_translation_spacing {0.01};  // 1 cm

  /** Minimum spacing between two consecutive rotations in a linear Cartesian
   * plan.*/
  const double min_rotation_spacing {0.02};  // ~ 1 degree

  /** If true, enforce a hard constarint on the optimization to fix the idle
   * robot joints from seed */
  bool fix_idle_joints {true};

  /** If true, the IK planner will impose collision-avoidance constraints, which
  are constructed from the seed configuration. If false, the planner will not
  impose collision-avoidance constraints at the first iteration. If the solution
  is not collision-free, the planner will try to solve the IK problem again with
  the collision-avoidance constraints.
  @note This option is only used if the constraints adapter has a collision
  checker. */
  bool make_collision_avoidance_constraint {false};

  /** If make_collision_avoidance_constraint is false, this option will be used
  to resolve the IK problem with collision avoidance constraints if the first
  pass fails CheckSatisfied. */
  bool resolve_with_collision_avoidance {true};

  /** If true, the IK planner will resolve the IK problem with collision
   * avoidance constraints if the first pass results in a self-collision.
   @note this will not get triggered if the IK problem is already
    * resolved with collision avoidance constraints, or
   resolve_with_collision_avoidance is true (it will already resolve the IK
   problem with collision avoidance)
    */
  bool self_collision_resolve_with_constraint {false};

  /** If the IK solution involves collisions that are ONLY from multiple arms,
   * then still return the solution without resolving the collision. This is
   * only useful if arms will be deconflicted later. */
  bool ignore_multi_arm_collision {false};

  /** If make_collision_avoidance_constraint is true, this is the influence
  distance for the collision avoidance constraints. The objects that are closer
  than this distance at the respective seed configuration will be considered in
  the IK problem.*/
  double collision_avoidance_influence_distance {0.05};

  /** Shrink the IK optimization joint limits by this amount to avoid
  hitting the joint limits due to hardware problems
  @note negative values will be ignored.
  */
  double joint_limits_safety_margin {0.02};

  /** The weight for the angle in the distance calculation. This is used to
   * weight the angle difference between the seed configuration and the
   * desired pose in the IK problem. */
  double angle_weight {1.0};

  /** The weight for the joint configuration in the distance calculation. */
  double q_weight {2.0};

  /** The number of seeds to use for the IK solver before giving up. If
   * negative, it will use all seeds from the cache. */
  int num_seeds {-1};

  /** If greater than 0, the global IK planner will use this number of random
   * seeds to try to find a solution in case all the used cached seeds fail to
   * find a solution.
   */
  int num_random_seeds {0};

  /** If the global IK planner uses random seeds, this option will
   * insert the random seed into the cache. This is useful for debugging and
   * testing purposes, where we want to see how the random seed performs
   * in the future. If false, the random seed will not be inserted into the
   * cache, and the IK planner will only use the cached seeds.
   * @note This option is only used if num_random_seeds is greater than 0.
   * If num_random_seeds is 0, this option will not be used.
   */
  bool insert_random_seed_into_cache {false};

  /** The random seed to use for the random number generator. This is used to
   * generate random seeds for the IK solver if num_random_seeds is greater
   * than 0.
   * @note This option is only used if num_random_seeds is greater than 0.
   * If num_random_seeds is 0, this option will not be used.
   */
  int random_seed {0};

  /** If true, a two-step approach is used to find the closest seeds. First, the
   * closest N configs are selected based on their distance in the task space.
   * Next, they are sorted by their distance to the reference configuration in
   * the configuration space. If false, a single-step approach is taken where
   * the seeds are sorted by a weighted combination of distances in the task and
   * configuration space.
   */
  bool select_seed_via_two_steps {false};

  /** If true, add a cost to the IK optimization to minimize the distance to
   * the seed configuration. If false, no such cost is added.
   */
  bool add_seed_distance_cost {true};
};

/** Implementation of the Inverse Kinematics (IK) planner for a robot model. The
IK planner uses mathematical programming to solve the IK problem for a specific
pose of frame B relative to frame A. The IK planner can also solve a plan for a
linear Cartesian trajectory between two poses. The constraints specified in the
constraints adapter are imposed on the IK problem therefore one can solve IK for
a specific pose while satisfying the constraints of the robot such as collision,
joint limits, etc.
@note The IK planner is thread-safe and can be used in a multi-threaded
environment. It owns its context pool.
*/
class IkPlanner {
 public:
  /** Constructor for the IK planner.
  @param robot_constraints The robot model with constraints.
  @param cache_configs The global configurations to use for the IK cache.
  @note The cache_configs are used to initialize the IK cache, which is used to
  speed up the IK solver by providing a set of seed configurations that are
  close to the desired pose. The cache_configs should be a set of global
  configurations that are representative of the robot's workspace. The IK
  planner will use these configurations to find the closest seed configuration
  to the desired pose. The IK planner will also use these configurations to
  initialize the IK cache, which is used to speed up the IK solver by providing
  a set of seed configurations that are close to the desired pose.
  */
  IkPlanner(const RobotConstraints& robot_constraints,
            const std::vector<Eigen::VectorXd>& cache_configs);

  class IkResult {
   public:
    bool optimization_success() const {
      return optimization_success_;
    }

    const Eigen::VectorXd& value() const {
      return q_;
    }

    bool multiarm_collision() const {
      return optimization_success_
             && collision_type_
                    == RobotConstraints::CollisionType::kAcrossArmsOnly;
    }

    bool self_collision() const {
      return optimization_success_
             && collision_type_
                    == RobotConstraints::CollisionType::kArmSelfOnly;
    }

    // ToDo(Sadra): Some certain env/robot collisions are actually sometimes
    // avoidable. Distinguish them after 447 gets merged later.
    bool unavoidable_collision() const {
      return optimization_success_
             && (collision_type_ == RobotConstraints::CollisionType::kMixed
                 || collision_type_
                        == RobotConstraints::CollisionType::kArmEnvOnly);
    }

    bool is_valid() const {
      return optimization_success_
             && collision_type_ == RobotConstraints::CollisionType::kNone;
    }

    struct FailureStatus {
      enum class FailureType {
        kUnknown,
        kOptimizationGeneral,
        kOptimizationNearSingularity,
        kOptimizationNearJointLimits,
        kOptimizationNearBothSingularityAndJointLimits,
        kOptimizationCollision,
        kNumerical,
        kCollision,
      } failure_type = FailureType::kUnknown;

      std::string message = "";

      FailureStatus() = default;
      FailureStatus(FailureType type, const std::string& msg)
          : failure_type(type), message(msg) {}
    };

    FailureStatus failure_status() const {
      DRAKE_THROW_UNLESS(failure_status_.has_value());
      return failure_status_.value();
    }

    std::string failure_status_message() const {
      DRAKE_THROW_UNLESS(failure_status_.has_value());
      return fmt::format("FailureType: {}, Message: {}",
                         magic_enum::enum_name(failure_status_->failure_type),
                         failure_status_->message);
    }

    IkResult() = default;

   private:
    bool optimization_success_ = false;

    /** Solution, or best effort solution if success is false. */
    Eigen::VectorXd q_ = Eigen::VectorXd::Zero(0);

    std::optional<FailureStatus> failure_status_;

    RobotConstraints::CollisionType collision_type_;

    friend class IkPlanner;
  };

  /** Solves the IK problem for a specific pose.
  @param pose The desired pose of the frame.
  @param q_seed The seed for the IK solver.
  @param thread_num The thread number to use for the IK solver.
  @param options Options for the planner.
  @return The configuration of the robot that achieves the desired pose, if
  found. Returns an empty optional if no solution is found. (Is this description
  wrong?))*/
  IkResult SolveIk(const drake::multibody::Frame<double>& frame_A,
                   const drake::multibody::Frame<double>& frame_B,
                   const drake::math::RigidTransformd& pose,
                   const Eigen::VectorXd& q_seed, const int thread_num = 0,
                   const IkPlannerOptions& options = IkPlannerOptions()) const;

  /** Solves the IK problem for a specific set of frame relative poses.
   * @param frame_relative_poses The desired frame relative poses.
   * @param q_seed The seed for the IK solver.
   * @param thread_num The thread number to use for the IK solver.
   * @param options Options for the planner.
   * @return The configuration of the robot that achieves the desired poses, if
   * found. Throws if not found. */
  IkResult SolveIk(const FrameRelativePoses& frame_relative_poses,
                   const Eigen::VectorXd& q_seed, const int thread_num = 0,
                   const IkPlannerOptions& options = IkPlannerOptions()) const;

  /** Solves the global IK problem.
  This method solves two Ik problems in parallel. The first problem is
  solved with the current configuration as the seed, and the second problem
  is solved with the closest seed from the cache. Whichever problem finishes
  first will return the solution. The other optimization will continue to
  run in the background until it finishes.
  @param pose The desired pose of the frame.
  @param q_current The current configuration, which is used as a seed along
  with the closest seed from the cache.
  @param thread_num The thread number to use for the IK solver.
  @param options Options for the planner.
  @return The configuration of the robot that achieves the desired pose, if
  found. Returns an empty optional if no solution is found.*/
  IkResult SolveGlobalIk(
      const FrameRelativePoses& frame_relative_poses,
      const Eigen::VectorXd& q_current,
      const IkPlannerOptions& options = IkPlannerOptions()) const;

  /** Getter for robot constraints.*/
  const RobotConstraints& robot_constraints() const {
    return robot_constraints_;
  }

  // ToDo(Sadra) Move this somewhere else
  static drake::math::RigidTransformd CalcInterpolatedPose(
      const drake::math::RigidTransformd& start_pose,
      const drake::math::RigidTransformd& end_pose, double t);

  /** Data structure for the error in the pose.*/
  struct PoseError {
    /** L2 norm of the position error in meters.*/
    double position_error;

    /** Angle between the two orientations in radians.*/
    double orientation_error;
  };

  /** Calculates the error in the linear Cartesian plan at a specific path
  parameter.
  @param start_pose The start pose of the trajectory.
  @param end_pose The end pose of the trajectory.
  @param plan The trajectory of the robot.
  @param t The path parameter between 0 (start) and 1 (end).
  @param thread_num The thread number to use for the IK solver.
  @return The PoseError for the position and orientation of the frame.*/
  PoseError CalcCartesianLinearPlanError(
      const drake::multibody::Frame<double>& frame_A,
      const drake::multibody::Frame<double>& frame_B,
      const drake::math::RigidTransformd& start_pose,
      const drake::math::RigidTransformd& end_pose,
      const drake::trajectories::PiecewisePolynomial<double>& plan, double t,
      int thread_num = 0);

  /** Checks a full config against joint limits and returns a string indicating
   * the closest joint limit and its distance.
   * @param q The configuration to check.
   */
  IkResult::FailureStatus EvalRobotLimits(
      const Eigen::VectorXd& q, const FrameRelativePoses& frps = {}) const;

  /** Calculates the configuration delta from a spatial delta for a specific
   * frame. This function is useful for calculating the configuration delta that
   * corresponds to a desired spatial delta in the task space. The spatial delta
   * is represented using drake::multibody::SpatialVelocity and consists of
   * angular (orientation) and translational (position) components of the error
   * of frame B relative to frame A. The configuration delta is defined as the
   * change in the robot's joint configuration that would achieve the
   * desired spatial delta. The spatial delta is evaluated in frame E.
   * @param q The current configuration of the robot.
   * @param frame_A The reference frame.
   * @param frame_B The target frame.
   * @param frame_E The frame in which the spatial delta is evaluated.
   * @param delta The desired spatial delta of frame B relative to frame A,
   * evaluated in frame E. Following SpatialVelocity semantics, the first 3
   * elements are the angular/orientation error (in radians), and the last 3
   * elements are the translational/position error (in meters).
   * @return The configuration delta that corresponds to the desired spatial
   * delta. The configuration delta is defined as the change in the robot's
   * joint configuration that would achieve the desired spatial delta. This
   * function uses the Jacobian of frame B relative to frame A, evaluated at the
   * current configuration q, to calculate the configuration delta. The
   * configuration delta is calculated as J^+ * delta, where J^+ is the
   * pseudo-inverse of the Jacobian. */
  Eigen::VectorXd CalcConfigDeltaFromSpatialDelta(
      const Eigen::VectorXd& q, const drake::multibody::Frame<double>& frame_A,
      const drake::multibody::Frame<double>& frame_B,
      const drake::multibody::Frame<double>& frame_E,
      const drake::multibody::SpatialVelocity<double>& delta) const;

  const IkCache& ik_cache() const {
    return *ik_cache_;
  }

 protected:
  // Solves the IK problem without a given seed configuration but uses the
  // closest seed from the cache.
  // @param pose The desired pose of the frame.
  // @param thread_num The thread number to use for the IK solver.
  // @param options Options for the planner.
  // @return The configuration of the robot that achieves the desired pose, if
  // found. Returns an empty optional if no solution is found.
  IkResult SolveGlobalIkUsingCache(
      const FrameRelativePoses& frame_relative_poses,
      const Eigen::VectorXd& q_ref, const int thread_num = 0,
      const IkPlannerOptions& options = IkPlannerOptions()) const;

  const RobotConstraints& robot_constraints_;
  const std::unique_ptr<IkCache> ik_cache_;
  std::vector<std::unique_ptr<std::mutex>> ik_solver_mutex_vec_;
  std::unique_ptr<ctpl::thread_pool> thread_pool_;
};

}  // namespace planning
}  // namespace motion
