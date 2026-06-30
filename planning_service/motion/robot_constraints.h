/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file robot_constraints.h

#pragma once
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/planning/collision_checker_context.h>
#include <drake/solvers/constraint.h>
#include <drake/solvers/cost.h>
#include <drake/solvers/mathematical_program.h>

#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "constraints_adapters.h"
#include "planning_service/common/override.h"
#include "robot_model.h"
namespace motion {

struct SampleOptions {
  /** The maximum number of samples to generate. */
  size_t max_num_samples {10000};
  /** If true, return exactly the specified number of samples. */
  bool truncate_samples {false};
  /** If true, then samples which do not meet constraints will be reprojected
   * to the closes satisfactory configuration.
   */
  bool use_projection {false};
  /** If true, return samples which do not satisfy constraints. */
  bool return_invalid {false};
  /** If true, generate samples in parallel. */
  bool parallel {true};
};

/**
 * Result of a CheckSatisfied call. Implicitly converts to bool for backward
 * compatibility with existing code that stores or checks the return value.
 * Optional fields are populated based on the CheckSatisfiedOptions provided.
 */
class CheckSatisfiedResult {
 public:
  CheckSatisfiedResult(
      bool satisfied, std::optional<double> penalty = std::nullopt,
      std::optional<std::vector<std::string>> failed_constraint_strings =
          std::nullopt,
      std::optional<std::vector<std::string>> offending_model_names =
          std::nullopt)
      : satisfied_(satisfied),
        penalty_(std::move(penalty)),
        failed_constraint_strings_(std::move(failed_constraint_strings)),
        offending_model_names_(std::move(offending_model_names)) {}

  /** Implicit bool conversion for backward compatibility. */
  operator bool() const {
    return satisfied_;
  }

  bool satisfied() const {
    return satisfied_;
  }

  /** The penalty value, populated when CheckSatisfiedOptions::calc_penalty is
   * true. */
  const std::optional<double>& penalty() const {
    return penalty_;
  }

  /** Descriptions of constraints that were violated, populated when
   * CheckSatisfiedOptions::collect_failed_constraint_strings is true. */
  const std::optional<std::vector<std::string>>& failed_constraint_strings()
      const {
    return failed_constraint_strings_;
  }

  /** Names of models involved in collision violations, populated when
   * CheckSatisfiedOptions::collect_offending_model_names is true. */
  const std::optional<std::vector<std::string>>& offending_model_names() const {
    return offending_model_names_;
  }

 private:
  bool satisfied_;
  std::optional<double> penalty_;
  std::optional<std::vector<std::string>> failed_constraint_strings_;
  std::optional<std::vector<std::string>> offending_model_names_;
};

struct CheckSatisfiedOptions {
  /** if true, then the constraints are checked in parallel. It is recommended
   to set this to true as about 2-2.5X speed gain is observed for >5 threads. */
  bool parallel {true};

  /** if true, then the failures of each constraint check will be logged. */
  bool verbose {false};

  /** The number of threads to use for checking the constraints. If not
  provided, then the number of threads will be set to the number of hardware
  threads */
  int num_threads {static_cast<int>(std::thread::hardware_concurrency())};

  // // Only for check satisfy of trajectories.
  // double trajectory_sampling_step {0.01};

  /** If true *and* if the meshcat session is available, the constraints will be
   * shown in meshcat. */
  bool color_collisions_meshcat {false};

  double tolerance {1e-3};

  /** If true, compute the penalty as part of CheckSatisfied. The result will
   * be stored in CheckSatisfiedResult::penalty(). This merges the functionality
   * of CalcPenalty into CheckSatisfied. */
  bool calc_penalty {false};

  /** If true, collect the names of models involved in collision violations.
   * These are stored in CheckSatisfiedResult::offending_model_names(). */
  bool collect_offending_model_names {false};

  /** If true, collect the descriptions of failed constraints. These are stored
   * in CheckSatisfiedResult::failed_constraint_strings(). */
  bool collect_failed_constraint_strings {false};

  CheckSatisfiedOptions() = default;
};

struct ConstraintMargins {
  double kDeltaCollision {0.0};
  double kDeltaPosition {0.0};
  double kDeltaAngle {0.0};

  /** serialization */
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(kDeltaCollision));
    a->Visit(DRAKE_NVP(kDeltaPosition));
    a->Visit(DRAKE_NVP(kDeltaAngle));
  }
};

enum PenaltyAggregationType { kSum, kMax };

/**
 * @brief Description of a Drake shape to be added to the world. Contains the
 * shape instance as well as transform and contextual information.
 *
 */
struct ShapeDescription {
  std::string name;
  std::string parent_frame;
  drake::math::RigidTransformd X_FG;
  std::shared_ptr<drake::geometry::Shape> shape;
};
/**
 RobotConstraints is a wrapper around the constraints for a homogenous set of
 constraints. It also provides methods to check if a configuration satisfies
 the constraints.
 */
class RobotConstraints {
  using ConstraintsThreadVec =
      std::vector<std::shared_ptr<drake::solvers::Constraint>>;

  using CostThreadVec = std::vector<std::shared_ptr<drake::solvers::Cost>>;

 public:
  /**
   * @param robot_model The robot model
   * @param constraints_adapter The constraints for a single plan
   * @param options The options for checking the constraints
   */
  RobotConstraints(const RobotModel& robot_model,
                   const ConstraintsAdapter& constraints_adapter,
                   const int num_threads =
                       static_cast<int>(std::thread::hardware_concurrency()));

  RobotConstraints(const RobotConstraints&) = delete;

  RobotConstraints(const RobotConstraints&&) = delete;

  std::unique_ptr<RobotConstraints> Clone(const int num_threads) const {
    return std::make_unique<RobotConstraints>(
        robot_model_, constraints_adapter_, num_threads);
  }

  /** Checks if a configuration satisfies the constraints.
  @param q The configuration to check
  @param thread_num The thread number to use for checking the constraints
  @param check_satisfied_options The options for checking the constraints. If
  not provided, then the default options will be used.
  @returns CheckSatisfiedResult which is implicitly convertible to bool. When
  CheckSatisfiedOptions::collect_failed_constraint_strings is true, also
  populates failed_constraint_strings. When calc_penalty is true, also populates
  penalty. When collect_offending_model_names is true, also populates
  offending_model_names.
  */
  CheckSatisfiedResult CheckSatisfied(
      const Eigen::VectorXd& q, const int thread_num = 0,
      const std::optional<CheckSatisfiedOptions>& check_satisfied_options =
          std::nullopt) const;

  /** Checks if a vector of configurations satisfies the constraints.
  It uses the thread pool to check the configurations in parallel.
  @param q_vec The vector of configurations to check
  @param check_satisfied_options The options for checking the constraints. If
  not provided, then the default options will be used.
  @returns CheckSatisfiedResult which is implicitly convertible to bool. */
  CheckSatisfiedResult CheckSatisfied(
      const std::vector<Eigen::VectorXd>& q_vec,
      const std::optional<CheckSatisfiedOptions>& check_satisfied_options =
          std::nullopt) const;

  /** Checks if an edge satisfies the constraints.
  @param q_1 The first configuration of the edge
  @param q_2 The second configuration of the edge
  @param step The step size to use for sampling the edge
  @param check_satisfied_options The options for checking the constraints. If
  not provided, then the default options will be used.
  @returns CheckSatisfiedResult which is implicitly convertible to bool. */
  CheckSatisfiedResult CheckSatisfiedEdge(
      const Eigen::VectorXd& q_1, const Eigen::VectorXd& q_2,
      const double step = 0.001,
      const std::optional<CheckSatisfiedOptions>& check_satisfied_options =
          std::nullopt) const;

  /** Checks if a trajectory satisfies the constraints.
  @param trajectory The trajectory to check
  @param step The step size to use for sampling the trajectory
  @param check_satisfied_options The options for checking the constraints. If
  not provided, then the default options will be used.
  @returns CheckSatisfiedResult which is implicitly convertible to bool. */
  CheckSatisfiedResult CheckSatisfiedTrajectory(
      const drake::trajectories::Trajectory<double>& trajectory,
      const double step = 0.01,
      const std::optional<CheckSatisfiedOptions>& check_satisfied_options =
          std::nullopt) const;

  /** Calculates the penalty for a configuration. The penalty is the summation
  of the penalties of each constraint. The penalty is 0 if all the constraints
  are satisfied beyond their influence distances.
  @param q The configuration to calculate the penalty for.
  @param thread_num The thread number to use for calculating the penalty.
  @returns a pair where the first element is the penalty, and the second
  element is true if the constraints are satisfied, and false otherwise.
  @warn: using this function only for checking if the constraints are satisfied
  is not recommended. Use CheckSatisfied instead. */
  std::pair<double, bool> CalcPenalty(const Eigen::VectorXd& q,
                                      const int thread_num = 0) const;

  /** Calculates the penalty for a vector of configurations and returns a vector
  that correspons to the penalties of the configurations in the vector, ordered
  appropriately.
   * It uses the thread pool to
  calculate the penalties in parallel. The penalty is 0 if all the constraints
  are satisfied beyond their influence distances.
  @param q_vec The vector of configurations to calculate the penalty for.
  @returns a pair where the first element is the penalty vector, and the second
  element is true if the constraints are satisfied, and false otherwise.
  @warn: using this function only for checking if the constraints are satisfied
  is not recommended. Use CheckSatisfied instead. */
  std::pair<std::vector<double>, bool> CalcPenaltyVec(
      const std::vector<Eigen::VectorXd>& q_vec) const;

  /**
   * @brief Returns an aggregate over the penalties of the configurations in the
  vector.
   * The aggregation method is specified by the agg_type parameter.
   *
   * @param q_vec vector of configurations to calculate the penalty for
   * @param agg_type the type of aggregation to use. Can be kSum or kMax.
  @returns a pair where the first element is the penalty aggregation, and the
  second element is true if the constraints are satisfied, and false otherwise.
   */
  std::optional<std::pair<double, bool>> CalcPenaltyVecAggregated(
      const std::vector<Eigen::VectorXd>& q_vec,
      const PenaltyAggregationType agg_type) const;

  /** Given a configuration, this function will project it to the closest
  configuration that satisfies the constraints excluding collision checking,
  which are more expensive. For considering collision constraints, use
  CalcClosestSatisfyingConfiguration.
  @param q The configuration to project
  @param thread_num The thread number to use for checking the constraints
  @param q_guess If provided, it is an initial guess for the projection
  @param distance_cost If provided, then the projection will minimize the
  distance to the q configuration. */
  std::optional<Eigen::VectorXd> ProjectConfToNonCollisionConstraints(
      const Eigen::VectorXd& q, const int thread_num = 0,
      const std::optional<Eigen::VectorXd>& q_guess = std::nullopt,
      const double distance_cost = 0) const;

  /** prints all the collision pairs to the console */
  void PrintAllCollisionPairs(
      double dist = std::numeric_limits<double>::infinity()) const;

  /** Generates configurations that satisfy the constraints
   @param generator The random generator seed
    @param num_desired_samples The desired number of samples to generate
    @param max_num_samples The maximum number of samples to inspect. If
   num_desired_samples is not reached before checking max_num_samples, then the
   function will return all the samples that were found.
    @param use_projection If true, then the samples will be projected to the
   closest configuration that satisfies the constraints. If false, then the
   samples will be generated uniformly and rejected if they do not satisfy the
   constraints.
    @param return_invalid If true, then the function will return invalid samples
   as opposed to valid samples. The samples will still satisfy robot joint
   limits, however.
    @warning the projection method is *much slower* than the rejection method.
   However, the rejection method may not be able to generate samples if the
   constraints are too strict.
    @note this function uses the thread pool to generate the samples in
   parallel.
    @returns a vector of configurations that satisfy the constraints (it may be
   empty if no samples were found) */
  std::vector<Eigen::VectorXd> GenerateSamples(
      drake::RandomGenerator* generator, const size_t num_desired_samples,
      const SampleOptions& options = {}) const;

  /** hasher for RobotConstraints. Refer to drake::hasher for more details.
  It only hashes the constraints adapter, and the collision geometries. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const RobotConstraints& robot_constraints) noexcept {
    hash_append(hasher, robot_constraints.robot_model());
    drake::DelegatingHasher delegating_hasher(
        [&hasher](const void* data, const size_t length) {
          // logging::log()->info("RobotConstraints: hasher value so far {}",
          // size_t(hasher));
          return hasher(data, length);
        });
    robot_constraints.HashCollisionGeos(&delegating_hasher);
    robot_constraints.HashConstraints(&delegating_hasher);
  }

  enum class CollisionType {
    kNone,
    kArmSelfOnly,
    kAcrossArmsOnly,
    kArmEnvOnly,
    kMixed
  };

  drake::planning::RobotClearance CalcRobotClearance(
      const Eigen::VectorXd& q, const int thread_num = 0) const;

  /** Classifies the collisions based on the given clearance object. */
  CollisionType ClassifyCollisions(
      const drake::planning::RobotClearance& clearance) const;

  /** Calculates the collisions and classifies the type of them. If no collision
   * is detected, then CollisionType::kNone is returned. If all collisions are
   * self-collisions, then CollisionType::kArmSelfOnly is returned. If all
   * collisions are across-arms collisions, then CollisionType::kAcrossArmsOnly
   * is returned. If all collisions are arm-environment collisions, then
   * CollisionType::kArmEnvOnly is returned. If there is a mix of collision
   * types, then CollisionType::kMixed is returned.
   * @param q The configuration to check
   * @param thread_num The thread number to use for checking the constraints
   * @returns the type of collisions detected
   */
  CollisionType CalcAndClassifyCollisions(const Eigen::VectorXd& q,
                                          const int thread_num = 0) const;

  bool DoArmsCollide(const Eigen::VectorXd& q, int thread_num = 0) const;

  /** Solves an optimization problem to find the closest configuration that
  satisfies the constraints.

  @param q The given configuration.
  @param fixed_model_instances The model instances that should be fixed in the
  returned configuration.
  @param collision_influence_distance geometry pairs that are within this
  distance will be considered for collision checking. If the distance is 0, then
  the collision constraints will be ignored for those pairs that are not
  colliding at the given configuration. Default is 0.1 (10 cm).
  @param thread_num The thread number to use for solving the optimization.

  @returns the closest configuration that satisfies the constraints. If no
  configuration is found, then std::nullopt is returned. */
  std::optional<Eigen::VectorXd> CalcClosestSatisfyingConfiguration(
      const Eigen::VectorXd& q, int thread_num = 0,
      const std::vector<drake::multibody::ModelInstanceIndex>&
          fixed_model_instances = {},
      double collision_influence_distance = 0.1) const;

  /**
   * @brief Find the configuration closest to an initial configuration q that
   * satisfies the given constraints by sampling along the edge between q and
   * some known valid configuration q_valid. In cases where a valid
   * configuration is readily available, this method is advantageous in that it
   * is faster and can be parallelized more easily than an optimization-based
   * approach.
   *
   * @param q The initial configuration
   * @param q_valid Some known valid configuration.
   * @param thread_num The thread number
   * @param step Step size to use for sampling between q and q_valid.
   * @param check_satisfied_options Options for constraint checking.
   * @return std::optional<Eigen::VectorXd>
   */
  std::optional<Eigen::VectorXd> CalcClosestSatisfyingConfigurationOnEdge(
      const Eigen::VectorXd& q, const Eigen::VectorXd& q_valid,
      int thread_num = 0, const double step = 0.001,
      const std::optional<CheckSatisfiedOptions>& check_satisfied_options =
          std::nullopt) const;

  /** Linearizes the constraints at the given configuration.
  @param q The configuration to linearize the constraints at.
  @param thread_num The thread number to use for linearizing the constraints.
  @returns a HPolyhedron(A,b) such that A * q <= b is a linear approximation
  of the constraints at the given configuration. */
  drake::geometry::optimization::HPolyhedron FindSatisfactionHPolyhedron(
      const Eigen::VectorXd& q, const int thread_num = 0) const;

  /** Calculates the distance from a point in frame A to a body.
   * This function computes the distance from a point in frame A to the closest
   * point on the body.
   * @param q The joint positions of the robot.
   * @param frame_A The frame in which the point is defined.
   * @param p_AQ The point in frame A.
   * @param body The body to which the distance is calculated.
   * @param influence_distance The distance within which the body is considered
   * to influence the point.
   * @param thread_num The thread number to use for the calculation.
   * @returns The distance from the point in frame A to the closest point on the
   * body's geometry, or std::nullopt if the point is outside the influence
   * or body does not have any collision geometry.
   * This function is useful for checking the distance from a point to a body
   * in the context of collision checking or constraint satisfaction.
   * @note This function will get more efficient once Drake addresses:
   * https://github.com/RobotLocomotion/drake/issues/23046 */
  std::optional<double> CalcPointDistanceToBody(
      const Eigen::VectorXd& q, const drake::multibody::Frame<double>& frame_A,
      const Eigen::Vector3d& p_AQ, const drake::multibody::Body<double>& body,
      double influence_distance, int thread_num = 0) const;

  /** Set filtering to true for a given pair of bodies or groups. */
  void SetFiltering(Eigen::MatrixXi& m, std::string_view body_or_group_1,
                    std::string_view body_or_group_2 = "") const {
    SetCollisionMatrix(m, 1, body_or_group_1, body_or_group_2, "SetFiltering");
  }

  /** Set padding distance in meters for a given pair of bodies or groups. */
  void SetPadding(Eigen::MatrixXd& m, double d,
                  std::string_view body_or_group_1,
                  std::string_view body_or_group_2 = "") const {
    SetCollisionMatrix(m, d, body_or_group_1, body_or_group_2, "SetPadding");
  }

  /**
   * @brief For a given collision matrix, set the value for a specific body or
   * group. A body will be tied to at most a single corresponding BodyIndex,
   * whereas a group, comprised of potentially many bodies, may be tied to
   * multiple BodyIndices.
   *
   * @tparam Scalar
   * @param m Matrix to modify
   * @param val Value to set
   * @param body_or_group_1 First body or group
   * @param body_or_group_2 Second body or group
   */
  template <typename Scalar>
  void SetCollisionMatrix(Eigen::Matrix<Scalar, -1, -1>& m, Scalar val,
                          std::string_view body_or_group_1,
                          std::string_view body_or_group_2,
                          const std::string caller = "") const {
    const auto is_group_1 {
        robot_model_.HasFilterGroupName(body_or_group_1.data())};
    const auto is_body_1 {
        robot_model_.HasScopedFrameNamed(body_or_group_1.data())};
    const auto is_group_2 {
        robot_model_.HasFilterGroupName(body_or_group_2.data())};
    const auto is_body_2 {
        robot_model_.HasScopedFrameNamed(body_or_group_2.data())};
    auto msg_prefix {caller.empty() ? "SetCollisionMatrix" : caller};
    if (!body_or_group_1.empty() && !(is_group_1 || is_body_1)) {
      throw std::runtime_error(fmt::format("{}: body or group 1 '{}' not found",
                                           msg_prefix, body_or_group_1));
    }
    if (!body_or_group_2.empty() && !(is_group_2 || is_body_2)) {
      throw std::runtime_error(fmt::format("{}: body or group 2 '{}' not found",
                                           msg_prefix, body_or_group_2));
    }
    // If both empty, warn that this will set the entire matrix to the value.
    if (body_or_group_1.empty() && body_or_group_2.empty()) {
      logging::log()->warn(
          "{}: Both elements are empty. Setting entire matrix to value {:.2f}. "
          "This may have unintended consequences.",
          msg_prefix, static_cast<double>(val));
    }
    logging::log()->debug(
        "RobotConstraints:{}: Setting value for {} '{}', {} '{}' to {:.2f}",
        msg_prefix, is_body_1 ? "body" : "group",
        body_or_group_1.empty() ? "ALL" : body_or_group_1,
        is_body_2 ? "body" : "group",
        body_or_group_2.empty() ? "ALL" : body_or_group_2,
        static_cast<double>(val));
    auto indices_1 {robot_model_.GetBodyIndices()};
    if (is_group_1) {
      indices_1 = robot_model_.GetBodyIndicesFromFilterGroupName(
          body_or_group_1.data());
    } else if (is_body_1) {
      indices_1 = {robot_model_.GetScopedFrameByName(body_or_group_1.data())
                       .body()
                       .index()};
    }
    // If no group/body B is provided, filter group/body A against
    // all other bodies in the model.
    auto indices_2 {robot_model_.GetBodyIndices()};
    if (is_group_2) {
      indices_2 = robot_model_.GetBodyIndicesFromFilterGroupName(
          body_or_group_2.data());
    } else if (is_body_2) {
      indices_2 = {robot_model_.GetScopedFrameByName(body_or_group_2.data())
                       .body()
                       .index()};
    }
    for (const auto& idx_1 : indices_1) {
      for (const auto& idx_2 : indices_2) {
        const auto i {static_cast<int>(idx_1)};
        const auto j {static_cast<int>(idx_2)};
        if (i == j) {
          continue;
        }
        m(i, j) = val;
        m(j, i) = val;
      }
    }
  }

  /**
   * Update the collision checker's padding matrix to the current matrix.
   * Callback for changes to the padding matrix.
   */
  void SetActivePaddingMatrix() const {
    DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
    collision_checker_->SetPaddingMatrix(*collision_padding_matrix_);
  }
  /**
   * Update the collision checker's filter matrix to the current matrix.
   * Callback for changes to the filter matrix.
   */
  void SetActiveCollisionFilterMatrix() const {
    DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
    collision_checker_->SetCollisionFilterMatrix(*collision_filter_matrix_);
  }

  /**
   * @brief Remove all collision shapes that were added to the collision
   * checker. Callback for restoration of collision shapes after a temporary
   * override.
   *
   */
  void RemoveAllAddedCollisionShapes() const;

  /**
   * @brief Adds all dynamic collision shapes to the collision checker,
   * and tracks their geometry IDs for management and cleanup. Callback for
   * override of collision shapes.
   *
   */
  void AddCollisionShapes() const;

  // ------------------------------- Getters -----------------------------------

  /** Read-only access to the @RobotModel object */
  const RobotModel& robot_model() const {
    return robot_model_;
  }

  /** Read-only access to the @constraints_adapter object */
  const ConstraintsAdapter& constraints_adapter() const {
    return constraints_adapter_;
  }

  /** Mutable access to the collision checker context */
  drake::planning::CollisionCheckerContext& mutable_collision_checker_context(
      const int thread_num = 0) const {
    return *contexts_.at(thread_num);
  }

  /** Read-only access to the plant contexts */
  drake::systems::Context<double>& mutable_plant_context(
      const int thread_num = 0) const {
    return contexts_.at(thread_num)->mutable_plant_context();
  }

  /** Returns the mathematical program for the given thread */
  drake::solvers::MathematicalProgram* iris_prog_no_collision_constraints(
      const int thread_num = 0) const {
    return iris_progs_no_collision_vec_.at(thread_num).get();
  }

  /** Returns if a collision checker exists */
  bool has_collision_checker() const {
    return collision_checker_ != nullptr;
  }

  /** Read-only access to the collision checker (read-only)*/
  const drake::planning::CollisionChecker& collision_checker() const {
    return *collision_checker_;
  }

  /** Read-only access to the drake constraints. */
  std::vector<std::shared_ptr<drake::solvers::Constraint>>
  get_non_collision_constraints(const int thread_num = 0) const {
    std::vector<std::shared_ptr<drake::solvers::Constraint>> result;
    for (const auto& constraint_thread_vec : constraints_no_collision_vec_) {
      result.push_back(constraint_thread_vec.at(thread_num));
    }
    return result;
  }

  /** Return the hash of the robot constraints. */
  size_t constraints_hash() const {
    return constraints_hash_;
  }

  int num_threads() const {
    return num_threads_;
  }

  auto& collision_padding_matrix() const {
    return collision_padding_matrix_;
  }

  auto& collision_filter_matrix() const {
    return collision_filter_matrix_;
  }

  auto& added_collision_shapes() const {
    return added_collision_shapes_;
  }

  std::string PrintRobotClearance(
      const drake::planning::RobotClearance& clearance,
      const std::optional<Eigen::VectorXd>& q_lifted = std::nullopt) const;

  static void RunMeshcatSlidersWithConstraints(
      const RobotConstraints& robot_constraints);

  void UpdateConstraintsOnMeshcat() const;

 private:
  std::vector<ConstraintsThreadVec> ConstructConstraintsNoCollision(
      const ConstraintsAdapter& constraints_info);

  std::unique_ptr<drake::planning::CollisionChecker> CreateCollisionChecker(
      const CollisionCheckerAdapter& collision_checker_adapter) const;

  void HashCollisionGeos(drake::DelegatingHasher* hasher) const;

  void HashConstraints(drake::DelegatingHasher* hasher) const;

  void HashPositionConstraints(drake::DelegatingHasher* hasher) const;

  void HashAngularConstraints(drake::DelegatingHasher* hasher) const;

  void HashJointPositionConstraints(drake::DelegatingHasher* hasher) const;

  // Displays the bounding box as a box on frame A in meshcat.
  // It also adds a sphere to position B on frame B. Visually,
  // The sphere must be in the box for the constraint to be satisfied.
  void DisplayPositionConstraintsInMeshcat() const;

  // Only show the frames participating in the angle constraints.
  std::vector<std::pair<std::string, drake::math::RigidTransformd>>
  DisplayAngleConstraintFramesInMeshcat() const;

  const RobotModel& robot_model_;
  const ConstraintsAdapter constraints_adapter_;
  const int num_threads_;
  std::unique_ptr<drake::planning::CollisionChecker> collision_checker_;
  std::vector<ConstraintsThreadVec> constraints_no_collision_vec_;
  std::vector<ConstraintsThreadVec> constraints_no_collision_reduced_vec_;
  // These need to be stored as shared_ptrs so that the relationship to the
  // CollisionChecker instance is maintained
  std::vector<std::shared_ptr<drake::planning::CollisionCheckerContext>>
      contexts_;
  std::vector<std::unique_ptr<drake::solvers::MathematicalProgram>>
      progs_no_collision_vec_, iris_progs_no_collision_vec_;
  size_t constraints_hash_;
  std::vector<std::unique_ptr<std::mutex>> check_satisfied_mutex_vec_;
  std::function<double(const double)> collision_penalty_func_;
  double collision_penalty_influence_distance_ = 0.0;
  std::vector<std::function<double(const Eigen::VectorXd&)>>
      non_collision_penalty_funcs_;
  mutable common::Overrideable<Eigen::MatrixXd> collision_padding_matrix_;
  mutable common::Overrideable<Eigen::MatrixXi> collision_filter_matrix_;
  mutable common::Overrideable<std::vector<ShapeDescription>>
      added_collision_shapes_;

  // Mapping of added shape names to their geometry IDs
  mutable std::unordered_map<drake::geometry::GeometryId, std::string>
      added_shape_geometry_ids_;
  std::vector<std::pair<std::string, drake::math::RigidTransformd>>
      angle_paths_;
};

}  // namespace motion

namespace std {
template <>
struct hash<motion::RobotConstraints> : public drake::DefaultHash {};
}  // namespace std

/** Formatter for CheckSatisfiedResult. Formats as "true" or "false". */
template <>
struct fmt::formatter<motion::CheckSatisfiedResult> : fmt::formatter<bool> {
  template <typename FormatContext>
  auto format(const motion::CheckSatisfiedResult& r, FormatContext& ctx) const {
    return fmt::formatter<bool>::format(r.satisfied(), ctx);
  }
};
