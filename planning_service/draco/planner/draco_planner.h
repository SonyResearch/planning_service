#pragma once

#include "planning_service/common/fmt.h"
#include "planning_service/draco/draco.h"
#include "planning_service/draco/visualizer/draco_visualizer.h"
#include "planning_service/motion/iris/iris_inspector.h"
#include "planning_service/motion/planning/artifact_builder.h"
#include "planning_service_client/check_satisfied_response.h"
#include "planning_service_client/planner/cartesian_linear_move_problem.h"
#include "planning_service_client/planner/fixed_frames_motion.h"
#include "planning_service_client/planner/general_multimodal_plan.h"
#include "planning_service_client/planner/global_ik_problem.h"
#include "planning_service_client/planner/max_cartesian_acceleration.h"
#include "planning_service_client/planner/motion_plan_result.h"
#include "planning_service_client/planner/multimodal_plan.h"
#include "planning_service_client/planner/out_of_violation.h"
#include "planning_service_client/planner/plan_options.h"
#include "planning_service_client/planner/planner_base.h"
#include "planning_service_client/planner/start_to_goal_plan.h"
#include "planning_service_client/planner/update_traj_toward_waypoints.h"

namespace draco {
namespace planner {

struct SolveGeneralizedMultiModalPlanResult {
  std::pair<drake::trajectories::PiecewisePolynomial<double>,
            drake::trajectories::PiecewisePolynomial<double>>
      trajectory;
  double transit_start_time;
  double transit_end_time;
};

using SolvePlanResult =
    std::expected<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                            drake::trajectories::PiecewisePolynomial<double>>,
                  std::string>;

class DracoPlanner : public Draco {
 public:
  DracoPlanner(const DracoAdapter& draco_adapter, bool is_builder = false);

  /**
   * @brief Check that the given vector of configurations satisfies the current
   * constraints.
   *
   * @param vq The vector of configurations to check.
   * @param check_satisfied_options Options passed to the constraints.
   * @param collision_options Options for collision checking.
   * @return true if constraints met, false otherwise
   *
   */
  planning_service_client::CheckSatisfiedResponse CheckSatisfied(
      const std::vector<psc::SystemConf>& system_conf_vec,
      const std::optional<psc::planner::CollisionOptions>& collision_options =
          std::nullopt) const;

  /** Solve a motion plan defined by the information provided in the problem
   * definition.
   * @param problem the problem definition.
   * @param start_sysconf the starting system configuration.
   * @param maybe_active_trajectory an optional active trajectory.
   * @param maybe_plan_options optional planning options.
   * @param label an optional label for the motion plan, used for
   * labeling the corresponding visualization tab in meshcat.
   * @return the result of the motion plan.
   */
  planning_service_client::planner::MotionPlanResult SolvePlan(
      const planning_service_client::planner::PlanningProblemBase& problem,
      const std::string& label = "",
      const std::optional<planning_service_client::planner::PlanOptions>&
          maybe_plan_options = std::nullopt,
      const std::optional<planning_service_client::SystemConf>& start_sysconf =
          std::nullopt,
      const std::optional<planning_service_client::SystemTimedTrajectory>&
          maybe_active_trajectory = std::nullopt) const;

  /**
   * @brief Get the sizes of planning artifacts (roadmap and IRIS regions).
   *
   * Returns a PlanningArtifactStatus struct populated with the number of
   * vertices and edges in the thunder planner roadmap, and the number of IRIS
   * regions in the iris inspector. Other fields are set to -1 indicating they
   * are not computed.
   *
   * @return motion::planning::PlanningArtifactStatus Status struct with
   * artifact sizes
   */
  motion::planning::PlanningArtifactStatus GetPlanningArtifactsSizes() const;

  class ResolveAnchorConfResult {
   public:
    // implicit conversion
    ResolveAnchorConfResult(const Eigen::VectorXd& valid_configuration)
        : valid_configuration_(valid_configuration) {}

    // implicit conversion
    ResolveAnchorConfResult(const std::string& error_message)
        : error_message_(error_message) {}

    // implicit conversion
    ResolveAnchorConfResult(
        const std::pair<Eigen::VectorXd,
                        std::vector<drake::multibody::ModelInstanceIndex>>&
            conflicted_conf_and_fixed_models)
        : conflicted_conf_and_fixed_models_(conflicted_conf_and_fixed_models) {}

    bool has_value() const {
      return valid_configuration_.has_value();
    }

    const Eigen::VectorXd& value() const {
      DRAKE_THROW_UNLESS(valid_configuration_.has_value());
      return valid_configuration_.value();
    }

    const std::string& error() const {
      DRAKE_THROW_UNLESS(error_message_.has_value());
      return error_message_.value();
    }

    bool is_conflicted() const {
      return conflicted_conf_and_fixed_models_.has_value();
    }

    const std::pair<Eigen::VectorXd,
                    std::vector<drake::multibody::ModelInstanceIndex>>&
    conflicted_conf_and_fixed_models() const {
      DRAKE_THROW_UNLESS(conflicted_conf_and_fixed_models_.has_value());
      return conflicted_conf_and_fixed_models_.value();
    }

    Eigen::VectorXd conflicted_value() const {
      DRAKE_THROW_UNLESS(conflicted_conf_and_fixed_models_.has_value());
      return conflicted_conf_and_fixed_models_->first;
    }

   private:
    std::optional<Eigen::VectorXd> valid_configuration_;
    std::optional<std::string> error_message_;
    std::optional<std::pair<Eigen::VectorXd,
                            std::vector<drake::multibody::ModelInstanceIndex>>>
        conflicted_conf_and_fixed_models_;
  };

  /** Compute a configuration that agrees with the contents of the anchor (e.g.,
   * sysconfs and poses).
   * @param anchor the anchor to resolve.
   * @param only_use_provided_seed if true, only the seed configuration provided
   * in the anchor will be used, and no seeds from the roadmap
   * @return AcncorConf containing the resolved configuration and the resolution
   * status.
   */
  ResolveAnchorConfResult ResolveAnchorConf(
      const planning_service_client::planner::Anchor& anchor,
      const std::optional<Eigen::VectorXd>& q_ref = std::nullopt,
      bool only_use_passed_seed = true, bool verbose = true) const;

  /**
   * @brief Solve a sequence of Anchors, using the previous solution as the seed
   * for the next.
   * @param anchors Vector of Anchor objects to solve.
   * @param q_ref The reference configuration to use as the initial seed.
   * @param only_use_ref_seed If true, only the passed q_ref seed will be
   * used for all anchors; otherwise, seeds from the IK cache will also be
   * ranked and attempted.
   * @return Vector of resolved configurations (Eigen::VectorXd).
   */
  std::expected<std::vector<Eigen::VectorXd>, std::string>
  SolveSequentialAnchors(
      const std::vector<planning_service_client::planner::Anchor>& anchors,
      const Eigen::VectorXd& q_ref, bool only_use_ref_seed = false,
      bool return_longest_incomplete_solution = false) const;

  planning_service_client::planner::GeneralizedMultimodalPlanningProblem
  ConstructGeneralizedProblemFromMultimodal(
      const planning_service_client::planner::MultimodalPlanningProblem&
          multimodal_problem,
      double max_translation_spacing, double max_rotation_spacing) const;

  std::expected<
      planning_service_client::planner::GeneralizedMultimodalPlanningProblem,
      std::string>
  ConstructGeneralizedProblemFromCartesianLinearProblem(
      const planning_service_client::planner::CartesianLinearMoveProblem&
          cartesian_problem,
      double max_translation_spacing, double max_rotation_spacing) const;

  std::vector<planning_service_client::planner::Anchor>
  ConstructWaypointAnchorsFromWayposesVec(
      const std::vector<planning_service_client::FrameRelativePosesVec>&
          wayposes,
      std::optional<planning_service_client::SystemConf> fixed_sysconf_opt =
          std::nullopt) const;

  std::vector<planning_service_client::planner::Anchor>
  ConstructWaypointAnchorsFromRelativeTransforms(
      const planning_service_client::planner::Anchor& start_anchor,
      const std::vector<planning_service_client::FrameRelativePose>&
          goal_offset_frps,
      const std::optional<Eigen::VectorXd>& q_ref_opt = std::nullopt,
      double max_translation_spacing = 1e-2,
      double max_rotation_spacing = 2e-2) const;

  std::expected<std::vector<planning_service_client::planner::Anchor>,
                std::string>
  ConstructWaypointAnchorsFromAbsoluteTransforms(
      const planning_service_client::planner::Anchor& start_anchor,
      const std::vector<planning_service_client::FrameRelativePose>& goal_frps,
      const std::optional<Eigen::VectorXd>& q_ref_opt = std::nullopt,
      double max_translation_spacing = 1e-2,
      double max_rotation_spacing = 2e-2) const;

  /** Completes a partial system configuration and returns the complete
   * configuration that is closest to the seed configuration.
   *
   * @param sys_conf The partial system configuration
   * @param q_seed The seed configuration that has to be made close to.
   * @return std::optional<Eigen::VectorXd> The closest valid complete
   * configuration to the seed configuration that completes the partial
   * configuration.
   */
  std::optional<Eigen::VectorXd> CalcClosestValidCompleteConf(
      const motion::system_conf_t& partial_sysconf,
      const Eigen::VectorXd q_seed) const;

  /**
  @brief Generate a cubic spline that interpolates the passed vector, and then
  run TOPPRA to get a time parametrization
  * @param path the vector of points to interpolate
  * @return a pair of PiecewisePolynomial representing the spline and the time
  parametrization
  * @note The path is assumed to be in the same order as the points in the
  vector
   */
  std::expected<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                          drake::trajectories::PiecewisePolynomial<double>>,
                std::string>
  SplineAndTimePath(std::vector<Eigen::VectorXd> path,
                    bool fast_estimate_cost = false) const;

  SolvePlanResult CalcTrajectoryBestAvailablePlanner(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      bool fast_estimate_cost = false) const;

  /**
   * @brief Compute the optimal trajectory using the GCS planner.
   *
   * @param q1 Start state as an array of joint positions
   * @param q2 Goal state as an array of joint positions
   * @return drake::trajectories::PiecewisePolynomial<double>
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>> CalcGcsPath(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const;

  /**
   * @brief Compute a time-optimal trajectory using the RRT planner.
   *
   * @param q1 Start state as an array of joint positions
   * @param q2 Goal state as an array of joint positions
   * @return drake::trajectories::PiecewisePolynomial<double>
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  CalcSampleBasedPath(const Eigen::VectorXd& q1,
                      const Eigen::VectorXd& q2) const;
  /**
   * @brief Compute a time-optimal trajectory by evaluating the straight-line
   * path in joint space.
   *
   * @param q1 Start state as an array of joint positions
   * @param q2 Goal state as an array of joint positions
   * @return drake::trajectories::PiecewisePolynomial<double>
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  CalcStraightLinePath(const Eigen::VectorXd& q1,
                       const Eigen::VectorXd& q2) const;

  // Returns the first sampled configuration along `trajectory` (sampling every
  // `step` seconds, and including the final state at trajectory.end_time())
  // that violates the robot constraints defined in this RobotConstraints
  // instance. If all sampled states are valid the function returns
  // std::nullopt.
  //
  // Parameters:
  // - trajectory: the trajectory to inspect
  // - step: sampling interval in seconds (must be > 0)
  // - check_satisfied_options: optional options forwarded to CheckSatisfied
  //
  // Returns:
  // - std::optional<Eigen::VectorXd> containing the first invalid
  // configuration,
  //   or std::nullopt if the trajectory is entirely valid.
  std::optional<Eigen::VectorXd> MaybeInvalidConf(
      const drake::trajectories::Trajectory<double>& trajectory, double step,
      const std::optional<motion::CheckSatisfiedOptions>&
          check_satisfied_options = std::nullopt) const;

  /**
   * @brief Calculates a fast estimate of the plan length.
   * ToDO(@sadraddini): This will move to the future planner interface.
   * @return If plan found, the estimated plan length, otherwise nullopt.
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  FastEstimatePlan(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) const;

  std::optional<double> TimeOfArmsCollision(
      const planning_service_client::SystemTimedTrajectory&
          sys_timed_trajectory,
      double step) const;

  std::optional<planning_service_client::SystemTimedTrajectory>
  BestApproachTrajectory(
      const planning_service_client::SystemTimedTrajectory&
          sys_timed_trajectory,
      const std::set<motion::ArmIndex>& movable_arms_indices, int num_slices,
      double step, std::optional<double> min_collision_time = std::nullopt,
      std::optional<double> max_collision_time = std::nullopt,
      double partial_solution_time_buffer = 0.3) const;

  void SaveConfigSpaceProblem(const Eigen::VectorXd& q_start,
                              const Eigen::VectorXd& q_goal) const;

  /**
   * @brief Sets the pose of the fixed offset frame from frp.
   * frame_A: parent frame (e.g., uncalibrated camera)
   * frame_B: child frame (e.g., calibrated camera)
   * X_AB: pose of frame B in frame A
   * @note frame_A is not used. Waiting for a drake feature to double check
   * that frame_B is indeed a child of frame_A.
   */
  void SetPoseInParentFrame(
      const planning_service_client::FrameRelativePose& frp) const;

  /** Get Ik Planner. */
  const motion::planning::IkPlanner& ik_planner() const {
    return *ik_planner_;
  }

  /** Get the time optimal spliner. */
  const motion::splining::TimeOptimalSpliner& time_optimal_spliner() const {
    return time_optimal_spliner_;
  }

  /** Read only getter for thunder planner */
  const motion::planning::ompl::ThunderPlanner& thunder_planner() const {
    return *thunder_planner_;
  }

  const motion::iris::IrisInspector& iris_inspector() const {
    return *iris_inspector_;
  }

  bool has_draco_visualizer() const {
    return draco_visualizer_ != nullptr;
  }

  const visualizer::DracoVisualizer& draco_visualizer() const {
    DRAKE_THROW_UNLESS(draco_visualizer_ != nullptr);
    return *draco_visualizer_;
  }

  visualizer::DracoVisualizer& mutable_draco_visualizer() {
    DRAKE_THROW_UNLESS(draco_visualizer_ != nullptr);
    return *draco_visualizer_;
  }

  // Artifact builder getter
  motion::planning::ArtifactBuilder& mutable_artifact_builder() {
    DRAKE_THROW_UNLESS(artifact_builder_ != nullptr);
    return *artifact_builder_;
  }

  // Single mode GCS planner getter
  motion::planning::SingleModeGcsPlanner& mutable_single_mode_gcs_planner()
      const {
    DRAKE_THROW_UNLESS(single_mode_gcs_planner_ != nullptr);
    return *single_mode_gcs_planner_;
  }

  std::expected<SolveGeneralizedMultiModalPlanResult, std::string>
  SolveGeneralizedMultiModalPlan(
      const planning_service_client::planner::
          GeneralizedMultimodalPlanningProblem& problem,
      const planning_service_client::SystemConf& start_sysconf,
      const std::optional<planning_service_client::SystemTimedTrajectory>&
          maybe_active_trajectory = std::nullopt) const;

 private:
  /**
   * @brief Apply the given dynamic limits in local scope. The original dynamic
   * limits settings will be restored when the returned ScopedOverride objects
   * are destroyed.
   *
   * @param dynamic_limits Limits controlling dynamic behavior.
   * @return tuple of ScopedOverride objects for cartesian dynamic limits and
   * splining parameters, respectively.
   */
  [[nodiscard]] std::tuple<
      common::ScopedOverride<motion::splining::cartesian_dynamic_limits_map_t>,
      common::ScopedOverride<motion::splining::TimeOptimalSplineParams>>
  ApplyScopedDynamicLimits(
      const std::optional<psc::planner::DynamicLimits>& dynamic_limits) const;

  planning_service_client::planner::MotionPlanResult SolvePlanImpl(
      const planning_service_client::planner::PlanningProblemBase& problem,
      const std::optional<planning_service_client::SystemConf>& start_sysconf =
          std::nullopt,
      const std::optional<planning_service_client::SystemTimedTrajectory>&
          maybe_active_trajectory = std::nullopt,
      const std::optional<
          planning_service_client::planner::TrajectoryUpdateOptions>&
          maybe_traj_update_options = std::nullopt) const;

  SolvePlanResult SolveOutOfViolationPlan(
      const planning_service_client::planner::OutOfViolation& problem,
      const planning_service_client::SystemConf& start_sysconf) const;

  SolvePlanResult SolveMaxCartesianAccelerationProblem(
      const planning_service_client::planner::MaxCartesianAcceleration& problem,
      const planning_service_client::SystemConf& start_sysconf) const;

  std::expected<drake::trajectories::PathParameterizedTrajectory<double>,
                std::string>
  SolveUpdateTrajTowardWaypointsPlan(
      const planning_service_client::planner::UpdateTrajTowardWaypointsProblem&
          problem,
      const std::optional<motion::ArmIndex> maybe_active_arm_index) const;

  SolvePlanResult SolveFixedFramesMotionPlan(
      const planning_service_client::planner::FixedFramesMotion& problem,
      const planning_service_client::SystemConf& start_sysconf) const;

  SolvePlanResult SolveGlobalIKPlan(
      const planning_service_client::planner::GlobalIKProblem& def) const;

  /**
   * @private
   * @brief Plan using fastest/best available planner. This method will return
   * a trajectory that is not time parameterized.
   * @param q1 Start state as an array of joint positions
   * @param q2 Goal state as an array of joint positions
   * @param result_ptr The motion plan result (as a pointer)
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  CalcPathBestAvailablePlanner(const Eigen::VectorXd& q1,
                               const Eigen::VectorXd& q2,
                               bool fast_estimate_plan) const;

  /** Find arms that their start and goal configurations are the same, and
   * if exists any, fix them in the path by turning their polynomials into
   * constants. If the fixed path is valid, return it. Otherwise, return an
   * empty optional.
   * @param path The path to be fixed.
   * @return A PiecewisePolynomial that has arms fixed in the path, or an
   * empty optional if no arms were found.
   */
  std::optional<drake::trajectories::PiecewisePolynomial<double>>
  MaybeFixArmsInPath(
      const drake::trajectories::PiecewisePolynomial<double>& path) const;

  /** @brief Enqueue a visualizable into the DracoVisualizer, bundled with a
   * snapshot of the currently active collision matrices so the visualizer
   * reproduces the same collision state as the planner had at the time of
   * the call. No-op if no visualizer is attached.
   *
   * All call sites that previously called `draco_visualizer_->Add(...)` should
   * use this helper instead so the snapshot is always captured automatically
   * from the live (possibly overridden) RobotConstraints.
   */
  template <typename T>
  void AddToVisualizer(const T& visualizable, const std::string& label) const {
    if (!has_draco_visualizer()) return;
    draco::CollisionOptionsSnapshot snapshot {
        robot_constraints_->collision_padding_matrix().value(),
        robot_constraints_->collision_filter_matrix().value(),
        robot_constraints_->added_collision_shapes().value()};
    draco_visualizer_->Add(visualizable, label, snapshot);
  }

  void SetActiveTimeOptimalSplineParams() const {
    const auto& params {*time_optimal_spline_params_};
    time_optimal_spliner_.SetTimeOptimalSplineParams(params);
    for (auto& [_, spliner] : arms_time_optimal_spliners_) {
      spliner->SetTimeOptimalSplineParams(params);
    }
  };

  void SetActiveCartesianDynamicLimits() const {
    const auto& limits {*cartesian_dynamic_limits_map_};
    time_optimal_spliner_.SetCartesianDynamicLimits(limits);
    for (auto& [_, spliner] : arms_time_optimal_spliners_) {
      spliner->SetCartesianDynamicLimits(limits);
    }
  };

  const motion::iris::IrisRegionsAdapter iris_regions_adapter_;
  mutable common::Overrideable<motion::splining::cartesian_dynamic_limits_map_t>
      cartesian_dynamic_limits_map_;
  mutable common::Overrideable<motion::splining::TimeOptimalSplineParams>
      time_optimal_spline_params_;
  std::map<motion::ArmIndex,
           std::unique_ptr<motion::splining::TimeOptimalSpliner>>
      arms_time_optimal_spliners_;
  // spliner for GCS trajectories
  motion::splining::TimeOptimalSpliner time_optimal_spliner_;
  // cubic spliner
  const motion::splining::CubicSpliner cubic_spliner_;
  // Iris inspector
  std::unique_ptr<motion::iris::IrisInspector> iris_inspector_;
  // Thunder planner
  std::unique_ptr<motion::planning::ompl::ThunderPlanner> thunder_planner_;
  // IK planner
  std::unique_ptr<motion::planning::IkPlanner> ik_planner_;
  // GCS planner
  std::unique_ptr<motion::planning::SingleModeGcsPlanner>
      single_mode_gcs_planner_;
  // Draco visualizer
  std::unique_ptr<visualizer::DracoVisualizer> draco_visualizer_;
  // Problems directory
  const fs::path problems_dir_;
  // Artifact builder (only constructed when draco is a builder)
  std::unique_ptr<motion::planning::ArtifactBuilder> artifact_builder_;

  friend class TestDracoPlanner_PrivateMaybeFixArmsInPath_Test;
  friend class GeneralizedMultimodalPlan_BestApproachTrajectory_Valid_Test;
  friend class GeneralizedMultimodalPlan_BestApproachTrajectory_Invalid_Test;
  friend class GeneralizedMultimodalPlan_BestApproachTrajectory_Invalid2_Test;
};

}  // namespace planner
}  // namespace draco
