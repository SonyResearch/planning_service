#include "ik_planner.h"

#include <Eigen/Geometry>
#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h>
#include <drake/multibody/inverse_kinematics/orientation_constraint.h>
#include <drake/multibody/inverse_kinematics/position_constraint.h>
#include <drake/solvers/ipopt_solver.h>
#include <drake/solvers/solve.h>
#include <drake/solvers/solver_interface.h>

#include <future>
#include <thread>

namespace motion {
namespace planning {

using FailureType = IkPlanner::IkResult::FailureStatus::FailureType;

namespace {
std::string FkErrorString(const RobotModel& robot_model,
                          const FrameRelativePoses& frame_relative_poses,
                          const Eigen::VectorXd& q) {
  std::string msg = "";
  for (const auto& frame_relative_pose : frame_relative_poses) {
    const auto& [frame_A, frame_B, pose] {frame_relative_pose};
    auto pose_fk = robot_model.CalcRelativeTransform(q, *frame_A, *frame_B);
    const auto delta_pose = pose.inverse() * pose_fk;
    msg +=
        fmt::format("Frame pair {} -> {}:", frame_A->scoped_name().get_full(),
                    frame_B->scoped_name().get_full());
    msg += fmt::format(
        FMT_BOLD | fg(FMT_RED),
        "translation error = {:.3f}[mm], rotation error = {:.3f}[deg]",
        delta_pose.translation().norm() * 1000,
        delta_pose.rotation().ToAngleAxis().angle() * 180 / M_PI);
  }
  return msg;
}
}  // namespace

IkPlanner::IkPlanner(const RobotConstraints& robot_constraints,
                     const std::vector<Eigen::VectorXd>& cache_configs)
    : robot_constraints_(robot_constraints),
      ik_cache_ {std::make_unique<IkCache>(robot_constraints_.robot_model(),
                                           cache_configs)},
      ik_solver_mutex_vec_ {std::vector<std::unique_ptr<std::mutex>>(
          robot_constraints_.num_threads())},
      // We only use this for global Ik with two approaches, so 2 threads are
      // enough
      thread_pool_ {std::make_unique<ctpl::thread_pool>(2)} {
  for (int i = 0; i < robot_constraints_.num_threads(); ++i) {
    ik_solver_mutex_vec_[i] = std::make_unique<std::mutex>();
  }
  logging::log()->info(
      "IkPlanner:Ctor: Made IK planner with {} cached "
      "configurations",
      cache_configs.size());
}

IkPlanner::IkResult IkPlanner::SolveIk(
    const FrameRelativePoses& frame_relative_poses,
    const Eigen::VectorXd& q_seed, const int thread_num,
    const IkPlannerOptions& options) const {
  IkPlanner::IkResult result;
  DRAKE_THROW_UNLESS(thread_num >= 0
                     && thread_num < robot_constraints_.num_threads());
  const auto& plant = robot_constraints_.robot_model().plant();
  const auto& hm = robot_constraints_.robot_model().holonomic_mapping();
  // Lock the mutex for the thread.
  // Why not scoped_lock? Because we want to unlock if a recursion is needed.
  std::unique_lock<std::mutex> lock(*ik_solver_mutex_vec_.at(thread_num));
  auto& plant_context = robot_constraints_.mutable_plant_context(thread_num);
  auto q_seed_lifted = hm.Lift(q_seed);
  plant.SetPositions(&plant_context, q_seed_lifted);
  auto prog = std::make_unique<drake::solvers::MathematicalProgram>();
  auto q_var = prog->NewContinuousVariables(plant.num_positions());
  prog->SetInitialGuess(q_var, q_seed_lifted);
  // get the active model instances from the target frame relative poses
  std::set<drake::multibody::ModelInstanceIndex> active_model_instances;
  for (const auto& [frame_A, frame_B, pose] : frame_relative_poses) {
    auto model_instance_A = frame_A->model_instance();
    auto model_instance_B = frame_B->model_instance();
    active_model_instances.insert(model_instance_A);
    active_model_instances.insert(model_instance_B);
  }
  if (options.fix_idle_joints) {
    // get all the fixed model instances
    std::vector<drake::multibody::ModelInstanceIndex> fixed_model_instances;
    for (int i = 0; i < plant.num_model_instances(); ++i) {
      auto model_instance = drake::multibody::ModelInstanceIndex(i);
      if (active_model_instances.find(model_instance)
          == active_model_instances.end()) {
        fixed_model_instances.push_back(model_instance);
      }
    }
    // add the fixed model instances to the program as linear equality
    // constraints
    for (const auto& model_instance : fixed_model_instances) {
      if (plant.num_positions(model_instance) == 0) {
        continue;
      }
      const int start_idx =
          robot_constraints_.robot_model().GetModelStartIndex(model_instance);
      Eigen::SparseMatrix<double> A(plant.num_positions(model_instance),
                                    plant.num_positions());
      for (int i {0}; i < plant.num_positions(model_instance); ++i) {
        A.insert(i, start_idx + i) = 1;
      }
      prog->AddLinearEqualityConstraint(
          A, q_seed_lifted.segment(start_idx, A.rows()), q_var);
    }
  }
  for (const auto& constraint :
       robot_constraints_.get_non_collision_constraints(thread_num)) {
    prog->AddConstraint(constraint, q_var);
  }
  if (options.add_seed_distance_cost) {
    // Add cost to minimize distance to the seed
    auto cost = prog->AddQuadraticErrorCost(1.0, q_seed_lifted, q_var);
  }
  // Add conservative joint limits. Note: joint limit already exists in
  // robot_constraints_->get_non_collision_constraints(thread_num). But we
  // don't want to shrink that constraint because it would effect elsewhere and
  // CheckSatisfied. It's better to add a new constraint.
  if (options.joint_limits_safety_margin > 0.0) {
    const auto& lb = plant.GetPositionLowerLimits();
    const auto& ub = plant.GetPositionUpperLimits();
    // Only add the margin for active model instances
    Eigen::VectorXd margin_vector = Eigen::VectorXd::Zero(lb.size());
    // Go through the active model instances, make the margin vector
    for (const auto& model_instance : active_model_instances) {
      int num_model_positions = plant.num_positions(model_instance);
      if (num_model_positions == 0) {
        // Not a mobile model instance
        continue;
      }
      logging::log()->debug(
          "IkPlanner:SolveIk: Adding joint limit safety "
          "margin {} for model instance {}",
          options.joint_limits_safety_margin,
          plant.GetModelInstanceName(model_instance));
      auto model_margin = Eigen::VectorXd::Ones(num_model_positions)
                          * options.joint_limits_safety_margin;
      robot_constraints_.robot_model()
          .instance_dof_masks()
          .at(model_instance)
          .SetInArray(model_margin, &margin_vector);
    }
    auto con = prog->AddBoundingBoxConstraint(lb + margin_vector,
                                              ub - margin_vector, q_var);
    con.evaluator()->set_description("Joint limits with safety margin");
  }
  std::string request_msg = "";
  for (int i = 0; i < std::ssize(frame_relative_poses); ++i) {
    const auto& [frame_A, frame_B, pose] = frame_relative_poses[i];
    // get scoped names for frame_A and frame_B and log them
    const auto scoped_frame_A = frame_A->scoped_name().to_string();
    const auto scoped_frame_B = frame_B->scoped_name().to_string();
    // Add the constraint for position of the frame
    auto position_constraint =
        std::make_shared<drake::multibody::PositionConstraint>(
            &plant, *frame_A, pose.translation() - options.position_tolerance,
            pose.translation() + options.position_tolerance, *frame_B,
            Eigen::Vector3d::Zero(), &plant_context);
    position_constraint->set_description(
        "IK position constraint for "
        "frame pair "
        + scoped_frame_A + " -> " + scoped_frame_B);
    prog->AddConstraint(position_constraint, q_var);
    // Add the orientation constraint
    auto orientation_constraint =
        std::make_shared<drake::multibody::OrientationConstraint>(
            &plant, *frame_A, pose.rotation(), *frame_B,
            drake::math::RotationMatrixd::Identity(),
            options.orientation_tolerance, &plant_context);
    orientation_constraint->set_description(
        "IK orientation constraint for frame pair " + scoped_frame_A + " -> "
        + scoped_frame_B);
    prog->AddConstraint(orientation_constraint, q_var);
    request_msg += fmt::format(
        "{}/{}: frame pair {} -> {} with pose: translation "
        "[xyz]{}[m], rotation[rpy] {}[rad]",
        i + 1, std::ssize(frame_relative_poses),
        frame_A->scoped_name().get_full(), frame_B->scoped_name().get_full(),
        pose.translation().transpose(),
        pose.rotation().ToRollPitchYaw().vector().transpose());
  }
  // Add coupling constraints between mimic joints
  if (!hm.is_identity()) {
    for (int i = 0; i < plant.num_positions(); ++i) {
      if (hm.IsMimickingJoint(i)) {
        auto [j, gear_ratio, offset] = hm.MimickingJointInfo(i);
        // q[i] = gear_ratio * q[j] + offset
        Eigen::SparseMatrix<double> A(1, plant.num_positions());
        A.insert(0, i) = 1;
        A.insert(0, j) = -gear_ratio;
        auto con = prog->AddLinearEqualityConstraint(
            A, Eigen::VectorXd::Constant(1, offset), q_var);
        con.evaluator()->set_description(fmt::format(
            "Coupling constraint for mimicking joint {} and {}", i, j));
      }
    }
  }
  // If asked for, add collision constraint
  if (options.make_collision_avoidance_constraint
      && robot_constraints_.has_collision_checker()) {
    auto& collision_checker_context =
        robot_constraints_.mutable_collision_checker_context(thread_num);
    const auto minimum_distance_constraint =
        std::make_shared<drake::multibody::MinimumDistanceLowerBoundConstraint>(
            &robot_constraints_.collision_checker(),
            options.collision_avoidance_influence_distance,
            &collision_checker_context,
            drake::solvers::QuadraticallySmoothedHingeLoss, 0.1);
    minimum_distance_constraint->set_description(
        "Minimum distance constraint for IK optimization");
    prog->AddConstraint(minimum_distance_constraint, q_var);
  }
  // Solve the program
  const std::string failure_red =
      fmt::format(FMT_BOLD | fg(FMT_RED), "FAILURE");
  const std::string success_green =
      fmt::format(FMT_BOLD | fg(FMT_GREEN), "SUCCESS");
  const std::string success_but_yellow =
      fmt::format(FMT_BOLD | fg(FMT_YELLOW), "SUCCESS, BUT");
  const auto mp_result = drake::solvers::Solve(*prog, q_seed_lifted);
  auto q_sol = mp_result.GetSolution(q_var);
  auto q_sol_reduced = hm.Reduce(q_sol);
  result.q_ = q_sol_reduced;
  if (!mp_result.is_success()) {
    std::string failed_constraints;
    bool includes_collision = false;
    for (const auto& binding : mp_result.GetInfeasibleConstraintNames(*prog)) {
      failed_constraints += "\n" + binding;
      if (binding.find("Minimum distance constraint") != std::string::npos) {
        includes_collision = true;
      }
    }
    // If there are no failed constraints, it means it is a numerical issue.
    if (failed_constraints.empty()) {
      auto msg = fmt::format(
          "IK request {} on thread {} resulted in {} but no "
          "failed constraints found. This is likely due to numerical issues. "
          "Was seed cost used? {}.",
          request_msg, thread_num, failure_red, options.add_seed_distance_cost);
      if (options.add_seed_distance_cost) {
        auto options_copy = options;
        options_copy.add_seed_distance_cost = false;
        logging::log()->info(
            "IkPlanner:SolveIk: {} Retrying without seed distance cost.", msg);
        lock.unlock();
        return SolveIk(frame_relative_poses, q_seed, thread_num, options_copy);
      }
      result.failure_status_ = IkPlanner::IkResult::FailureStatus {
          FailureType::kNumerical, std::move(msg)};
      return result;
    }
    logging::log()->debug(
        "IK Request {} (thread {}) resulted in {} with {}: {}", request_msg,
        thread_num, failure_red, mp_result.get_solver_id().name(),
        fmt::format(fg(FMT_RED), failed_constraints));
    result.optimization_success_ = false;
    auto limits_failure = EvalRobotLimits(result.q_, frame_relative_poses);
    if (includes_collision) {
      // If the failure includes collision, we classify it as collision
      limits_failure.failure_type = FailureType::kOptimizationCollision;
    }
    // Log what the best effort solution vs seed could achieve
    auto result_msg = fmt::format(
        "IK request {}. Failed constraints: {}."
        "\nSeed FK performance:        {}"
        "\nBest effort FK performance: {}"
        "\nCheck limits: {}",
        request_msg, failed_constraints,
        FkErrorString(robot_constraints_.robot_model(), frame_relative_poses,
                      q_seed),
        FkErrorString(robot_constraints_.robot_model(), frame_relative_poses,
                      result.q_),
        limits_failure.message);
    result.failure_status_ = IkPlanner::IkResult::FailureStatus(
        limits_failure.failure_type, std::move(result_msg));
    return result;
  }
  result.optimization_success_ = true;
  auto clearance =
      robot_constraints().CalcRobotClearance(q_sol_reduced, thread_num);
  auto collision_type = robot_constraints().ClassifyCollisions(clearance);
  result.collision_type_ = collision_type;
  // If collision detected
  if (collision_type != RobotConstraints::CollisionType::kNone) {
    if (options.make_collision_avoidance_constraint) {
      auto msg = fmt::format(
          "IK request {} on thread {} "
          "resulted in {} a post-optimization CheckSatisfied failed for q= "
          "[{}]. This is likely due to solver issues or numerical errors",
          request_msg, thread_num, failure_red, q_sol_reduced.transpose());
      logging::log()->error("IkPlanner:SolveIk: {}", msg);
      // We change our mind, this is an actual optimization failure
      result.optimization_success_ = false;
      result.failure_status_ = IkPlanner::IkResult::FailureStatus {
          FailureType::kNumerical, std::move(msg)};
      return result;
    }
    if (options.ignore_multi_arm_collision
        && collision_type == RobotConstraints::CollisionType::kAcrossArmsOnly) {
      auto log_msg = fmt::format(
          "IK request {} on thread {}: {}. Post-optimization "
          "CheckSatisfied failed due to collisions across arms, but since "
          "ignore_multi_arm_collision is set to true. Returning the solution "
          "anyways. The arms must be de-conflicted in a separate algorithm.",
          request_msg, thread_num, success_but_yellow);
      logging::log()->info("IkPlanner:SolveIk: {}", log_msg);
      result.failure_status_ = IkPlanner::IkResult::FailureStatus {
          FailureType::kCollision, std::move(log_msg)};
      return result;
    }
    if (options.resolve_with_collision_avoidance
        || (collision_type == RobotConstraints::CollisionType::kArmSelfOnly
            && options.self_collision_resolve_with_constraint)) {
      auto log_msg = fmt::format(
          "IK request {} on thread {} resulted in {} a post-optimization "
          "CheckSatisfied while not enforcing collision avoidance constraints. "
          "Adding them and resolving IK. Was it self-collision? {}. ",
          request_msg, thread_num, success_but_yellow,
          collision_type == RobotConstraints::CollisionType::kArmSelfOnly);
      logging::log()->info("IkPlanner:SolveIk: {}", log_msg);
      auto new_options = options;
      DRAKE_DEMAND(!new_options.make_collision_avoidance_constraint);
      new_options.make_collision_avoidance_constraint = true;
      // Unlock the mutex before calling SolveIk again.
      lock.unlock();
      return SolveIk(frame_relative_poses, q_sol_reduced, thread_num,
                     new_options);
    }
    // Get collision string
    auto collision_string = robot_constraints_.PrintRobotClearance(clearance);
    auto log_msg = fmt::format(
        "IK request {} on thread {}: {}. Post-optimization CheckSatisfied "
        "failed. multi-arm collision ignored? {} collision_str = {}",
        request_msg, thread_num, failure_red,
        options.ignore_multi_arm_collision, collision_string);
    logging::log()->debug("IkPlanner:SolveIk: {}", log_msg);
    result.failure_status_ = IkPlanner::IkResult::FailureStatus {
        FailureType::kCollision, std::move(log_msg)};
    return result;
  }
  // If got here, the optimization was successful and the solution is
  // collision-free.
  DRAKE_DEMAND(result.is_valid());
  return result;
}

IkPlanner::IkResult IkPlanner::SolveIk(
    const drake::multibody::Frame<double>& frame_A,
    const drake::multibody::Frame<double>& frame_B,
    const drake::math::RigidTransformd& pose, const Eigen::VectorXd& q_seed,
    const int thread_num, const IkPlannerOptions& options) const {
  FrameRelativePoses frame_relative_poses;
  frame_relative_poses.emplace_back(
      &frame_A, &frame_B, pose);  // Add the pose for the frame B relative to A
  return SolveIk(frame_relative_poses, q_seed, thread_num, options);
}

IkPlanner::PoseError IkPlanner::CalcCartesianLinearPlanError(
    const drake::multibody::Frame<double>& frame_A,
    const drake::multibody::Frame<double>& frame_B,
    const drake::math::RigidTransformd& start_pose,
    const drake::math::RigidTransformd& end_pose,
    const drake::trajectories::PiecewisePolynomial<double>& plan, double t,
    int thread_num) {
  const auto interpolated_pose = CalcInterpolatedPose(start_pose, end_pose, t);
  const auto q = plan.value(t);
  auto& plant_context = robot_constraints_.mutable_plant_context(thread_num);
  robot_constraints_.robot_model().plant().SetPositions(&plant_context, q);
  drake::math::RigidTransformd actual_pose =
      robot_constraints_.robot_model().plant().CalcRelativeTransform(
          plant_context, frame_A, frame_B);
  const auto delta_pose = interpolated_pose.inverse() * actual_pose;
  const double delta_translation = delta_pose.translation().norm();
  const double delta_rotation = delta_pose.rotation().ToAngleAxis().angle();
  return IkPlanner::PoseError {.position_error = delta_translation,
                               .orientation_error = delta_rotation};
}

drake::math::RigidTransformd IkPlanner::CalcInterpolatedPose(
    const drake::math::RigidTransformd& start_pose,
    const drake::math::RigidTransformd& end_pose, double t) {
  DRAKE_THROW_UNLESS(t >= 0 && t <= 1.0);
  const auto delta_pose = start_pose.inverse() * end_pose;
  const auto translation =
      start_pose.translation() * (1 - t) + end_pose.translation() * t;
  const auto delta_axis_angle = delta_pose.rotation().ToAngleAxis();
  const auto interpolated_axis_angle =
      Eigen::AngleAxisd(delta_axis_angle.angle() * t, delta_axis_angle.axis());
  const auto delta_rotation =
      drake::math::RotationMatrixd(interpolated_axis_angle);
  const auto rotation = start_pose.rotation() * delta_rotation;
  return drake::math::RigidTransformd(rotation, translation);
}

IkPlanner::IkResult IkPlanner::SolveGlobalIkUsingCache(
    const FrameRelativePoses& frame_relative_poses,
    const Eigen::VectorXd& q_ref, const int thread_num,
    const IkPlannerOptions& options) const {
  auto best_seeds = ik_cache_->CalcClosestSeed(
      frame_relative_poses, q_ref, options.angle_weight, options.q_weight,
      options.num_seeds, options.select_seed_via_two_steps);
  IkPlanner::IkResult ik_result, first_seed_ik_result;
  for (int i = 0; i < std::ssize(best_seeds); ++i) {
    std::string order_str;
    if (i == 0) {
      order_str = "st";
    } else if (i == 1) {
      order_str = "nd";
    } else if (i == 2) {
      order_str = "rd";
    } else {
      order_str = "th";
    }
    logging::log()->debug(
        "IkPlanner:SolveGlobalIk: Trying {}{} closest out of {} seeds", i + 1,
        order_str, best_seeds.size());
    ik_result =
        SolveIk(frame_relative_poses, best_seeds[i], thread_num, options);
    // optimization fails: don't return, try the next seed (if exists)
    // is_valid: return (happy)
    // unavoidable_collision: return (sad, but no point trying other seeds)
    // multiarm_collision: return (hopeful, maybe deconfliction will help)
    // self_collision: don't return, other seeds might be better
    if (ik_result.is_valid() || ik_result.unavoidable_collision()
        || ik_result.multiarm_collision()) {
      logging::log()->info(
          "IkPlanner:SolveGlobalIk: {}{} seed succeeded in optimization and it "
          "is not a self-collision",
          i + 1, order_str);
      return ik_result;
    } else if (i == 0) {
      logging::log()->info("IkPlanner:SolveGlobalIk: Best seed failed");
      first_seed_ik_result = ik_result;
    }
  }
  auto msg = fmt::format(
      "IK request on thread {} failed to find a solution after trying {} "
      "seeds. ",
      thread_num, best_seeds.size());
  logging::log()->error("IkPlanner:SolveGlobalIk: {}", msg);
  // We won't give up, let's try with random seeds if requested.
  if (options.num_random_seeds > 0) {
    // Find active model instances
    std::set<drake::multibody::ModelInstanceIndex> active_model_instances;
    for (const auto& [frame_A, frame_B, _] : frame_relative_poses) {
      // Add to the active model instances (set automatically handles
      // uniqueness)
      active_model_instances.insert(frame_A->model_instance());
      active_model_instances.insert(frame_B->model_instance());
    }
    const auto& lb =
        robot_constraints_.robot_model().plant().GetPositionLowerLimits();
    const auto& ub =
        robot_constraints_.robot_model().plant().GetPositionUpperLimits();
    const auto rec = drake::geometry::optimization::Hyperrectangle(lb, ub);
    drake::RandomGenerator gen(options.random_seed);
    int i = 0;
    int max_samples_to_try = options.num_random_seeds * 10;  // ToDo: fix this?
    for (int samples_to_try = 0;
         samples_to_try < max_samples_to_try && i < options.num_random_seeds;
         ++samples_to_try) {
      // Generate a random seed
      auto q_random = rec.UniformSample(&gen);
      // Set idle arms to the reference configuration
      robot_constraints_.robot_model().SetIdleModelsConfigToRef(
          &q_random, q_ref, active_model_instances);
      // Pass if it fails check satisfied
      if (!robot_constraints_.CheckSatisfied(q_random, thread_num)) {
        logging::log()->info(
            "IkPlanner:SolveGlobalIk: random seed {}/{} failed CheckSatisfied",
            samples_to_try + 1, max_samples_to_try);
        continue;
      }
      i++;
      ik_result = SolveIk(frame_relative_poses, q_random, thread_num, options);
      if (ik_result.is_valid() || ik_result.unavoidable_collision()
          || ik_result.multiarm_collision()) {
        // Let's add the random seed to the ik cache
        if (options.insert_random_seed_into_cache) {
          ik_cache_->AddCacheConfig(ik_result.value());
        }
        return ik_result;
      }
    }
    msg += fmt::format(
        " Tried {} random samples, but only {} passed "
        "CheckSatisfied. ",
        max_samples_to_try, i);
    logging::log()->error("IkPlanner:SolveGlobalIk: {}", msg);
  }
  DRAKE_DEMAND(!ik_result.optimization_success() || ik_result.self_collision());
  DRAKE_DEMAND(first_seed_ik_result.failure_status_.has_value());
  ik_result.failure_status_ = first_seed_ik_result.failure_status();
  ik_result.failure_status_->message = fmt::format(
      "{}\nFirst seed failure: {}", msg, ik_result.failure_status_->message);
  return ik_result;
}

IkPlanner::IkResult IkPlanner::SolveGlobalIk(
    const FrameRelativePoses& frps, const Eigen::VectorXd& q_current,
    const IkPlannerOptions& options) const {
  // Ensure that we have more than 2 threads.
  DRAKE_THROW_UNLESS(robot_constraints_.num_threads() >= 2);
  // Todo(@sadraddini): solve the following using a thread pool
  return SolveGlobalIkUsingCache(frps, q_current, 0, options);
}

IkPlanner::IkResult::FailureStatus IkPlanner::EvalRobotLimits(
    const Eigen::VectorXd& q, const FrameRelativePoses& frps) const {
  // ToDo(@sadraddini): Set these thresholds from options
  double condition_number_threshold = 1000;
  double joint_limit_threshold = 0.05;
  std::string msg = "";
  bool jacobian_issue = false;
  bool joint_limit_issue = false;
  const auto& plant = robot_constraints_.robot_model().plant();
  std::set<drake::multibody::ModelInstanceIndex> active_mobile_model_instances;
  auto q_lifted = robot_constraints_.robot_model().holonomic_mapping().Lift(q);
  if (!frps.empty()) {
    auto* calc_pose_context =
        robot_constraints_.robot_model().calc_pose_context_ptr();
    auto* plant_context =
        &(plant.GetMyMutableContextFromRoot(calc_pose_context));
    plant.SetPositions(plant_context, q_lifted);
    for (const auto& frp : frps) {
      Eigen::MatrixXd jacobian(6, plant.num_positions());
      const auto& frame_A = *std::get<0>(frp);
      const auto& frame_B = *std::get<1>(frp);
      const auto arm_A_index = robot_constraints_.robot_model().get_arm_index(
          frame_A.model_instance());
      const auto arm_B_index = robot_constraints_.robot_model().get_arm_index(
          frame_B.model_instance());
      if (arm_A_index.is_valid()) {
        const auto& arm_A =
            robot_constraints_.robot_model().GetArm(arm_A_index);
        // Insert all the model instances of the arm_A to
        // active_mobile_model_instances
        for (const auto& model_instance : arm_A.model_instances()) {
          if (plant.num_positions(model_instance) > 0) {
            active_mobile_model_instances.insert(model_instance);
          }
        }
      }
      if (arm_B_index.is_valid()) {
        const auto& arm_B =
            robot_constraints_.robot_model().GetArm(arm_B_index);
        // Insert all the model instances of the arm_B to
        // active_mobile_model_instances
        for (const auto& model_instance : arm_B.model_instances()) {
          if (plant.num_positions(model_instance) > 0) {
            active_mobile_model_instances.insert(model_instance);
          }
        }
      }
      // If active_mobile_model_instances is empty, it means both frames are on
      // fixed model instances. In that case, throw an error.
      if (active_mobile_model_instances.empty()) {
        auto msg = fmt::format(
            "IkPlanner:EvalRobotLimits: Both frames ({} and {}) in the frame "
            "relative pose are on fixed model instances. This failed IK "
            "evaluation.",
            frame_A.name(), frame_B.name());
        logging::log()->error("IkPlanner:EvalRobotLimits: {}", msg);
        throw std::runtime_error(msg);
      }
      plant.CalcJacobianSpatialVelocity(
          *plant_context, drake::multibody::JacobianWrtVariable::kQDot, frame_B,
          Eigen::Vector3d::Zero(), frame_A, frame_A, &jacobian);
      // Find the corresponding arm.
      auto arm_A = robot_constraints_.robot_model().get_arm_index(
          frame_A.model_instance());
      auto arm_B = robot_constraints_.robot_model().get_arm_index(
          frame_B.model_instance());
      if (!arm_A.is_valid() && arm_B.is_valid()) {
        jacobian = robot_constraints_.robot_model()
                       .GetArm(arm_B)
                       .dof_mask()
                       .GetColumnsFromMatrix(jacobian);
      }
      // Log the condition number of the jacobian
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian);
      double cond_number =
          svd.singularValues()(0)
          / svd.singularValues()(svd.singularValues().size() - 1);
      auto frp_jacobian_str = fmt::format(
          "Jacobian condition number for frame pair ({}->{}) is: {:.0f}",
          frame_A.name(), frame_B.name(), cond_number);
      if (cond_number > condition_number_threshold) {
        msg += fmt::format(fg(FMT_RED), "close to singularity: {}\n",
                           frp_jacobian_str);
        jacobian_issue = true;
        logging::log()->warn(
            "IkPlanner:EvalRobotLimits: Condition number of the jacobian for "
            "frame pair ({}->{}) is high: {}",
            frame_A.name(), frame_B.name(), cond_number);
      } else {
        logging::log()->debug(
            "IkPlanner:EvalRobotLimits: Condition number of the jacobian for "
            "frame pair ({}->{}) is normal: {}",
            frame_A.name(), frame_B.name(), cond_number);
      }
    }
  }
  DRAKE_DEMAND(active_mobile_model_instances.size() > 0 || frps.empty());
  const auto& lower_limits = plant.GetPositionLowerLimits();
  const auto& upper_limits = plant.GetPositionUpperLimits();
  double min_distance = std::numeric_limits<double>::max();
  int closest_joint_index = -1;
  enum class ClosestLimit {
    kLower,
    kUpper,
    kNotSet
  } closest_limit = ClosestLimit::kNotSet;
  for (int i = 0; i < q_lifted.size(); ++i) {
    // only check for active joints
    bool is_joint_i_active = false;
    for (const auto& model_instance : active_mobile_model_instances) {
      if (robot_constraints_.robot_model().instance_dof_masks().at(
              model_instance)[i]) {
        is_joint_i_active = true;
        break;
      }
    }
    // Only skip a joint if some active mobile model instance exists.
    if (!is_joint_i_active && !frps.empty()) {
      continue;
    }
    double distance_to_lower = q_lifted(i) - lower_limits(i);
    double distance_to_upper = upper_limits(i) - q_lifted(i);
    double distance = std::min(distance_to_lower, distance_to_upper);
    if (distance < min_distance) {
      min_distance = distance;
      closest_joint_index = i;
      closest_limit = (distance_to_lower < distance_to_upper)
                          ? ClosestLimit::kLower
                          : ClosestLimit::kUpper;
    }
  }
  DRAKE_DEMAND(closest_joint_index != -1);
  DRAKE_DEMAND(closest_limit != ClosestLimit::kNotSet);
  // If joint limit issue, get the joint name from the curresponding model
  // instance
  if (min_distance < joint_limit_threshold) {
    joint_limit_issue = true;
    for (const auto& [model_idx, dof_mask] :
         robot_constraints_.robot_model().instance_dof_masks()) {
      if (dof_mask[closest_joint_index]) {
        auto model_name = plant.GetModelInstanceName(model_idx);
        // Find the first joint
        for (int j = 0; j < dof_mask.size(); ++j) {
          if (dof_mask[j]) {
            auto joint_index = closest_joint_index - j;
            std::string limit_str =
                (closest_limit == ClosestLimit::kLower) ? "lower" : "upper";
            msg += fmt::format(
                fg(FMT_YELLOW), "Joint {}[{}] is {:.6f} away from its {} limit",
                model_name, joint_index, min_distance, limit_str);
            break;
          }
        }
        break;
      }
    }
  } else {
    logging::log()->info(
        "IkPlanner:EvalRobotLimits: No joint limit issue found. Closest joint "
        "is {} away from its limit.",
        min_distance);
  }
  auto failure_type = FailureType::kOptimizationGeneral;
  if (jacobian_issue && joint_limit_issue) {
    failure_type = FailureType::kOptimizationNearBothSingularityAndJointLimits;
  } else if (jacobian_issue) {
    failure_type = FailureType::kOptimizationNearSingularity;
  } else if (joint_limit_issue) {
    failure_type = FailureType::kOptimizationNearJointLimits;
  }
  return IkPlanner::IkResult::FailureStatus {failure_type, msg};
}

Eigen::VectorXd IkPlanner::CalcConfigDeltaFromSpatialDelta(
    const Eigen::VectorXd& q, const drake::multibody::Frame<double>& frame_A,
    const drake::multibody::Frame<double>& frame_B,
    const drake::multibody::Frame<double>& frame_E,
    const drake::multibody::SpatialVelocity<double>& delta) const {
  const auto& plant = robot_constraints_.robot_model().plant();
  Eigen::MatrixXd J(6, plant.num_positions());
  auto& plant_context = plant.GetMyMutableContextFromRoot(
      robot_constraints_.robot_model().calc_pose_context_ptr());
  const Eigen::VectorXd q_lifted =
      robot_constraints_.robot_model().holonomic_mapping().Lift(q);
  plant.SetPositions(&plant_context, q_lifted);
  plant.CalcJacobianSpatialVelocity(
      plant_context, drake::multibody::JacobianWrtVariable::kQDot, frame_B,
      Eigen::Vector3d::Zero(), frame_A, frame_E, &J);
  // Now solve the least square problem J * delta_q = delta
  Eigen::VectorXd delta_vec = delta.get_coeffs();
  Eigen::VectorXd delta_q_lifted =
      J.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(delta_vec);
  std::set<drake::multibody::ModelInstanceIndex> active_model_instances;
  for (const auto& model_instance :
       {frame_A.model_instance(), frame_B.model_instance()}) {
    const auto arm_index =
        robot_constraints_.robot_model().get_arm_index(model_instance);
    if (arm_index.is_valid()) {
      for (const auto& mi : robot_constraints_.robot_model()
                                .GetArm(arm_index)
                                .model_instances()) {
        if (plant.num_positions(mi) > 0) {
          active_model_instances.insert(mi);
        }
      }
    }
  }
  Eigen::VectorXd delta_q_reduced =
      robot_constraints_.robot_model().holonomic_mapping().Reduce(
          delta_q_lifted);
  delta_q_reduced = robot_constraints_.robot_model().SetIdleModelsConfigToRef(
      delta_q_reduced,
      Eigen::VectorXd::Zero(
          robot_constraints_.robot_model().holonomic_mapping().minimal_dim()),
      active_model_instances);
  return delta_q_reduced;
}

}  // namespace planning
}  // namespace motion
