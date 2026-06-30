/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include "robot_constraints.h"

#include <drake/common/autodiff.h>
#include <drake/geometry/optimization/hyperrectangle.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rigid_transform.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/inverse_kinematics/angle_between_vectors_constraint.h>
#include <drake/multibody/inverse_kinematics/minimum_distance_lower_bound_constraint.h>
#include <drake/multibody/inverse_kinematics/position_constraint.h>
#include <drake/multibody/inverse_kinematics/position_cost.h>
#include <drake/multibody/parsing/scoped_names.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>
#include <fmt/color.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <mutex>

#include <omp.h>

#include "planning_service/common/misc_utils.h"

namespace drake {
namespace geometry {
class ShapeHasher final : public ShapeReifier {
 public:
  ShapeHasher(drake::DelegatingHasher* hasher) : hasher_(hasher) {}

  void HashAppend(const Shape& shape) {
    shape.Reify(this, hasher_);
    const std::string shape_name {shape.type_name()};
    hash_append(*hasher_, shape_name);
  }

  void ImplementGeometry(const Box& box, void* data) override {
    auto* hasher {reinterpret_cast<drake::DelegatingHasher*>(data)};
    hash_append(*hasher, box.width());
    hash_append(*hasher, box.depth());
    hash_append(*hasher, box.height());
  }

  void ImplementGeometry(const Capsule&, void*) override {
    ThrowUnsupportedGeometry("Capsule");
  }

  void ImplementGeometry(const Convex&, void*) override {
    ThrowUnsupportedGeometry("Convex");
  }

  void ImplementGeometry(const Cylinder& cylinder, void* data) override {
    auto* hasher {reinterpret_cast<drake::DelegatingHasher*>(data)};
    hash_append(*hasher, cylinder.radius());
    hash_append(*hasher, cylinder.length());
  }

  void ImplementGeometry(const HalfSpace&, void*) override {
    ThrowUnsupportedGeometry("HalfSpace");
  }

  void ImplementGeometry(const Mesh& mesh, void* data) override {
    auto* hasher {reinterpret_cast<drake::DelegatingHasher*>(data)};
    const auto& hull {mesh.GetConvexHull()};
    hash_append(*hasher, hull.num_vertices());
    hash_append(*hasher, hull.num_faces());
    const auto* start {hull.centroid().data()};
    const auto* end {start + hull.centroid().size()};
    hash_append_range(*hasher, start, end);
  }

  void ImplementGeometry(const MeshcatCone&, void*) override {
    ThrowUnsupportedGeometry("MeshcatCone");
  }

  void ImplementGeometry(const Sphere& sphere, void* data) override {
    auto* hasher {reinterpret_cast<drake::DelegatingHasher*>(data)};
    hash_append(*hasher, sphere.radius());
  }

 private:
  drake::DelegatingHasher* hasher_;
};

}  // namespace geometry
}  // namespace drake

namespace motion {

using drake::multibody::AngleBetweenVectorsConstraint;
using drake::multibody::PositionConstraint;

namespace {
class ReducedPositionConstraint : public drake::solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ReducedPositionConstraint)

  ReducedPositionConstraint(
      const drake::multibody::MultibodyPlant<double>* plant,
      const drake::multibody::Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower,
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper,
      const drake::multibody::Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& p_BQ,
      drake::systems::Context<double>* plant_context,
      const HolonomicMapping& holonomic_mapping)
      : drake::solvers::Constraint(3, holonomic_mapping.minimal_dim(),
                                   p_AQ_lower, p_AQ_upper),
        position_constraint_ {PositionConstraint(plant, frameA, p_AQ_lower,
                                                 p_AQ_upper, frameB, p_BQ,
                                                 plant_context)},
        holonomic_mapping_ {holonomic_mapping} {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    auto x_lifted = holonomic_mapping_.Lift(x);
    position_constraint_.Eval(x_lifted, y);
  }

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override {
    auto x_lifted = holonomic_mapping_.LiftAutoDiff(x);
    position_constraint_.Eval(x_lifted, y);
  }

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
      drake::VectorX<drake::symbolic::Expression>* y) const override {
    (void)(x);  // unused
    (void)(y);  // unused
    throw std::runtime_error(
        "ReducedPositionConstraint::DoEval for symbolic variables is not "
        "implemented.");
  }

  const PositionConstraint position_constraint_;
  const HolonomicMapping holonomic_mapping_;
};

class ReducedAngleBetweenVectorsConstraint : public drake::solvers::Constraint {
 public:
  DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(ReducedAngleBetweenVectorsConstraint)

  ReducedAngleBetweenVectorsConstraint(
      const drake::multibody::MultibodyPlant<double>* plant,
      const drake::multibody::Frame<double>& frameA,
      const Eigen::Ref<const Eigen::Vector3d>& v_A,
      const drake::multibody::Frame<double>& frameB,
      const Eigen::Ref<const Eigen::Vector3d>& v_B, double angle_lower,
      double angle_upper, drake::systems::Context<double>* plant_context,
      const HolonomicMapping& holonomic_mapping)
      : drake::solvers::Constraint(1, holonomic_mapping.minimal_dim(),
                                   drake::Vector1d(std::cos(angle_upper)),
                                   drake::Vector1d(std::cos(angle_lower))),
        angle_between_vectors_constraint_ {AngleBetweenVectorsConstraint(
            plant, frameA, v_A, frameB, v_B, angle_lower, angle_upper,
            plant_context)},
        holonomic_mapping_ {holonomic_mapping} {}

 private:
  void DoEval(const Eigen::Ref<const Eigen::VectorXd>& x,
              Eigen::VectorXd* y) const override {
    auto x_lifted = holonomic_mapping_.Lift(x);
    angle_between_vectors_constraint_.Eval(x_lifted, y);
  }

  void DoEval(const Eigen::Ref<const drake::AutoDiffVecXd>& x,
              drake::AutoDiffVecXd* y) const override {
    auto x_lifted = holonomic_mapping_.LiftAutoDiff(x);
    angle_between_vectors_constraint_.Eval(x_lifted, y);
  }

  void DoEval(
      const Eigen::Ref<const drake::VectorX<drake::symbolic::Variable>>& x,
      drake::VectorX<drake::symbolic::Expression>* y) const override {
    (void)(x);  // unused
    (void)(y);  // unused
    throw std::runtime_error(
        "ReducedAngleBetweenVectorsConstraint::DoEval for symbolic variables "
        "is not implemented.");
  }

  const AngleBetweenVectorsConstraint angle_between_vectors_constraint_;
  const HolonomicMapping holonomic_mapping_;
};

std::function<double(const double)> MakePenaltyFunc(
    const MinimumValuePenaltyParams& penalty_params) {
  return [penalty_params](const double x) {
    if (x > penalty_params.x0) {
      return 0.0;
    }
    return penalty_params.m
           * (std::exp(-penalty_params.gamma * x)
              - std::exp(-penalty_params.gamma * penalty_params.x0))
           / (1 - std::exp(-penalty_params.gamma * penalty_params.x0));
  };
}

std::function<double(const Eigen::VectorXd&)> MakePositionPenaltyFunc(
    const MinimumValuePenaltyParams& penalty_params,
    const Eigen::VectorXd& lower_bound, const Eigen::VectorXd& upper_bound) {
  DRAKE_DEMAND(lower_bound.size() == upper_bound.size());
  auto penalty_func = MakePenaltyFunc(penalty_params);
  return
      [penalty_func, lower_bound, upper_bound](const Eigen::VectorXd& output) {
        DRAKE_DEMAND(output.size() == lower_bound.size());
        double penalty {0.0};
        for (int i {0}; i < output.size(); ++i) {
          penalty += penalty_func(output(i) - lower_bound(i));
          penalty += penalty_func(upper_bound(i) - output(i));
        }
        return penalty;
      };
}

std::function<double(const Eigen::VectorXd&)> MakeAnglePenaltyFunc(
    const MinimumValuePenaltyParams& penalty_params, const double lower_bound,
    const double upper_bound) {
  auto penalty_func = MakePenaltyFunc(penalty_params);
  return
      [penalty_func, lower_bound, upper_bound](const Eigen::VectorXd& output) {
        DRAKE_DEMAND(output.size() == 1);
        return penalty_func(std::acos(output(0)) - std::acos(upper_bound))
               + penalty_func(std::acos(lower_bound) - std::acos(output(0)));
      };
}

}  // namespace

RobotConstraints::RobotConstraints(
    const RobotModel& robot_model,
    const ConstraintsAdapter& constraints_adapter, const int num_threads)
    : robot_model_ {robot_model},
      constraints_adapter_ {
          AddJointLimitToConstraintsAdapter(robot_model, constraints_adapter)},
      num_threads_ {num_threads} {
  const auto start {std::chrono::high_resolution_clock::now()};
  if (constraints_adapter_.collision_checker.has_value()) {
    collision_checker_ =
        CreateCollisionChecker(constraints_adapter_.collision_checker.value());
    // Set default matrices
    collision_padding_matrix_.initialize(
        collision_checker_->GetPaddingMatrix());
    collision_filter_matrix_.initialize(
        collision_checker_->GetNominalFilteredCollisionMatrix());
    added_collision_shapes_.initialize(std::vector<ShapeDescription> {});
  }  // for non-collision constraints,
  for (int i {0}; i < num_threads_; ++i) {
    progs_no_collision_vec_.push_back(
        std::make_unique<drake::solvers::MathematicalProgram>());
    iris_progs_no_collision_vec_.push_back(
        std::make_unique<drake::solvers::MathematicalProgram>());
    if (collision_checker_ != nullptr) {
      contexts_.push_back(collision_checker_->MakeStandaloneModelContext());
    } else {
      contexts_.push_back(robot_model_.default_collision_checker()
                              .MakeStandaloneModelContext());
    }
  }
  logging::log()->debug("RobotConstraints:RobotConstraints: cloned {} contexts",
                        contexts_.size());
  // position constraints
  if (constraints_adapter_.position_constraints) {
    logging::log()->debug(
        "RobotConstraints:RobotConstraints: Parsing position constraints");
    for (const auto& position_constraint :
         *constraints_adapter_.position_constraints) {
      ConstraintsThreadVec position_constraints_thread_array(num_threads_);
      ConstraintsThreadVec position_constraints_reduced_thread_array(
          num_threads_);
      CostThreadVec cost_thread_array(num_threads_);
      const auto description {drake::yaml::SaveYamlString(position_constraint)};
      const auto& frame_A {
          robot_model_.GetScopedFrameByName(position_constraint.frame_A)};
      const auto& frame_B {
          robot_model_.GetScopedFrameByName(position_constraint.frame_B)};
      for (int i {0}; i < num_threads_; ++i) {
        position_constraints_thread_array.at(i) =
            std::make_shared<PositionConstraint>(
                &robot_model_.plant(), frame_A,
                position_constraint.position_AQ_lower,
                position_constraint.position_AQ_upper, frame_B,
                position_constraint.position_BQ,
                &(contexts_[i]->mutable_plant_context()));
        position_constraints_reduced_thread_array.at(i) =
            std::make_shared<ReducedPositionConstraint>(
                &robot_model_.plant(), frame_A,
                position_constraint.position_AQ_lower,
                position_constraint.position_AQ_upper, frame_B,
                position_constraint.position_BQ,
                &(contexts_[i]->mutable_plant_context()),
                robot_model_.holonomic_mapping());
        position_constraints_thread_array.at(i)->set_description(description);
        position_constraints_reduced_thread_array.at(i)->set_description(
            description + " (reduced)");
      }
      constraints_no_collision_vec_.push_back(
          position_constraints_thread_array);
      constraints_no_collision_reduced_vec_.push_back(
          position_constraints_reduced_thread_array);
      // make a penalty function
      const auto penalty_params {
          position_constraint.minimum_value_penalty_params.value_or(
              MinimumValuePenaltyParams())};
      non_collision_penalty_funcs_.push_back(MakePositionPenaltyFunc(
          penalty_params, position_constraint.position_AQ_lower,
          position_constraint.position_AQ_upper));
    }
  }
  // angle constraints
  if (constraints_adapter_.angle_constraints) {
    logging::log()->debug(
        "RobotConstraints:RobotConstraints: Parsing angle constraints");
    for (const auto& angle_constraint :
         *constraints_adapter_.angle_constraints) {
      std::vector<std::shared_ptr<drake::solvers::Constraint>>
          angle_constraints_thread_array(num_threads_);
      std::vector<std::shared_ptr<drake::solvers::Constraint>>
          angle_constraints_reduced_thread_array(num_threads_);
      const auto description {drake::yaml::SaveYamlString(angle_constraint)};
      const auto& frame_A {
          angle_constraint.frame_A == "world"
              ? robot_model_.plant().world_frame()
              : robot_model_.GetScopedFrameByName(angle_constraint.frame_A)};
      const auto& frame_B {
          robot_model_.GetScopedFrameByName(angle_constraint.frame_B)};
      for (int i {0}; i < num_threads_; ++i) {
        angle_constraints_thread_array.at(i) =
            std::make_shared<AngleBetweenVectorsConstraint>(
                &robot_model_.plant(), frame_A, angle_constraint.a_A, frame_B,
                angle_constraint.b_B, angle_constraint.angle_lower,
                angle_constraint.angle_upper,
                &(contexts_[i]->mutable_plant_context()));
        angle_constraints_reduced_thread_array.at(i) =
            std::make_shared<ReducedAngleBetweenVectorsConstraint>(
                &robot_model_.plant(), frame_A, angle_constraint.a_A, frame_B,
                angle_constraint.b_B, angle_constraint.angle_lower,
                angle_constraint.angle_upper,
                &(contexts_[i]->mutable_plant_context()),
                robot_model_.holonomic_mapping());
        angle_constraints_thread_array.at(i)->set_description(description);
        angle_constraints_reduced_thread_array.at(i)->set_description(
            description + " (reduced)");
      }
      constraints_no_collision_vec_.push_back(angle_constraints_thread_array);
      constraints_no_collision_reduced_vec_.push_back(
          angle_constraints_reduced_thread_array);
      // make a penalty function
      const auto penalty_params {
          angle_constraint.minimum_value_penalty_params.value_or(
              MinimumValuePenaltyParams())};
      non_collision_penalty_funcs_.push_back(
          MakeAnglePenaltyFunc(penalty_params, angle_constraint.angle_lower,
                               angle_constraint.angle_upper));
    }
  }
  // joint_position_box_constraints
  if (constraints_adapter_.joint_position_box_constraints) {
    logging::log()->debug(
        "RobotConstraints:RobotConstraints: parsing "
        "joint_position_box_constraints");
    for (const auto& joint_position_box_constraints :
         *constraints_adapter_.joint_position_box_constraints) {
      std::vector<std::shared_ptr<drake::solvers::Constraint>>
          joint_position_box_constraints_thread_array(num_threads_);
      const auto description {fmt::format(
          "joint_limits: {}",
          drake::yaml::SaveYamlString(joint_position_box_constraints))};
      // setup the A matrix
      const auto model_idx {robot_model_.plant().GetModelInstanceByName(
          joint_position_box_constraints.multibody_entity_name)};
      const int start_idx = robot_model_.GetModelStartIndex(model_idx);
      logging::log()->debug(
          "RobotConstraints:RobotConstraints: joint_position_box_constraints "
          "for model {} start_idx {}",
          joint_position_box_constraints.multibody_entity_name, start_idx);
      Eigen::SparseMatrix<double> A(
          robot_model_.plant().num_positions(model_idx),
          robot_model_.plant().num_positions());
      for (int i {0}; i < robot_model_.plant().num_positions(model_idx); ++i) {
        A.insert(i, start_idx + i) = 1;
      }
      for (int i {0}; i < num_threads_; ++i) {
        joint_position_box_constraints_thread_array.at(i) =
            std::make_shared<drake::solvers::LinearConstraint>(
                A, joint_position_box_constraints.lower_bounds,
                joint_position_box_constraints.upper_bounds);
        joint_position_box_constraints_thread_array.at(i)->set_description(
            description);
      }
      constraints_no_collision_vec_.push_back(
          joint_position_box_constraints_thread_array);
      // make a penalty function
      const auto penalty_params {
          joint_position_box_constraints.minimum_value_penalty_params.value_or(
              MinimumValuePenaltyParams())};
      non_collision_penalty_funcs_.push_back(MakePositionPenaltyFunc(
          penalty_params, joint_position_box_constraints.lower_bounds,
          joint_position_box_constraints.upper_bounds));
    }
  }
  // setup the mathematical program constraints
  for (int i {0}; i < num_threads_; ++i) {
    auto q_var = progs_no_collision_vec_.at(i)->NewContinuousVariables(
        robot_model_.plant().num_positions(), "q");
    auto q_reduced_var =
        iris_progs_no_collision_vec_.at(i)->NewContinuousVariables(
            robot_model_.holonomic_mapping().minimal_dim(), "q_reduced");
    for (const auto& constraint : constraints_no_collision_vec_) {
      progs_no_collision_vec_.at(i)->AddConstraint(constraint.at(i), q_var);
      logging::log()->trace(
          "RobotConstraints:RobotConstraints: added constraint {} to "
          "RobotConstraints mathematical program at thread {}",
          constraint.at(i)->get_description(), i);
    }
    for (const auto& constraint : constraints_no_collision_reduced_vec_) {
      iris_progs_no_collision_vec_.at(i)->AddConstraint(constraint.at(i),
                                                        q_reduced_var);
      logging::log()->trace(
          "RobotConstraints:RobotConstraints: added constraint {} to "
          "RobotConstraints IRIS mathematical program at thread {}",
          constraint.at(i)->get_description(), i);
    }
  }
  // Make penalty function for the collision
  if (collision_checker_ != nullptr) {
    // Of course we have a collision checker
    DRAKE_DEMAND(constraints_adapter_.collision_checker.has_value());
    const auto collision_penalty_params {
        constraints_adapter_.collision_checker.value()
            .minimum_value_penalty_params.value_or(
                MinimumValuePenaltyParams())};
    collision_penalty_func_ = MakePenaltyFunc(collision_penalty_params);
    collision_penalty_influence_distance_ = collision_penalty_params.x0;
  }
  const drake::DefaultHash hash_func;
  constraints_hash_ = hash_func(*this);
  if (robot_model_.meshcat() != nullptr) {
    DisplayPositionConstraintsInMeshcat();
    angle_paths_ = DisplayAngleConstraintFramesInMeshcat();
  }  // Setup the mutex instances for check_satisfied
  for (int i {0}; i < num_threads_; ++i) {
    check_satisfied_mutex_vec_.push_back(std::make_unique<std::mutex>());
  }
  const auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start)};
  logging::log()->debug(
      "RobotConstraints:RobotConstraints: Constructed constraints in {} ms",
      duration.count());
}

CheckSatisfiedResult RobotConstraints::CheckSatisfied(
    const Eigen::VectorXd& q, const int thread_num,
    const std::optional<CheckSatisfiedOptions>& check_satisfied_options_opt)
    const {
  const auto& opts {
      check_satisfied_options_opt.value_or(CheckSatisfiedOptions())};
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto& hm = robot_model_.holonomic_mapping();
  DRAKE_THROW_UNLESS(q.rows() == hm.minimal_dim());
  auto q_lifted = hm.Lift(q);
  bool valid {true};
  const auto& plant {robot_model_.plant()};
  // Initialize optional result fields
  std::optional<double> penalty =
      opts.calc_penalty ? std::optional<double>(0.0) : std::nullopt;
  std::optional<std::vector<std::string>> failed_strings;
  if (opts.collect_failed_constraint_strings) {
    failed_strings = std::vector<std::string>();
  }
  std::optional<std::set<std::string>> offending_model_names;
  if (opts.collect_offending_model_names) {
    offending_model_names = std::set<std::string>();
  }
  // If we need to collect any data, we should not fail fast.
  bool fail_fast = !(opts.collect_offending_model_names
                     || opts.collect_failed_constraint_strings || opts.verbose
                     || opts.calc_penalty);
  // Lock the mutex
  std::scoped_lock lock(*(check_satisfied_mutex_vec_.at(thread_num)));
  // Check non-collision constraints
  for (int i {0}; i < std::ssize(constraints_no_collision_vec_); ++i) {
    const auto& constraint {constraints_no_collision_vec_.at(i)};
    const auto* constraint_this_thread {constraint.at(thread_num).get()};
    plant.SetPositions(&(contexts_[thread_num]->mutable_plant_context()),
                       q_lifted);
    const bool satisfied {
        constraint_this_thread->CheckSatisfied(q_lifted, opts.tolerance)};
    valid = valid && satisfied;
    // Evaluate output when needed for penalty or verbose logging
    const bool need_output {penalty.has_value()
                            || (!satisfied && opts.verbose)};
    Eigen::VectorXd output;
    if (need_output) {
      constraint_this_thread->Eval(q_lifted, &output);
    }
    if (penalty.has_value()) {
      *penalty += non_collision_penalty_funcs_.at(i)(output);
    }
    if (!satisfied) {
      if (opts.verbose) {
        const auto& lower_bound {constraint_this_thread->lower_bound()};
        const auto& upper_bound {constraint_this_thread->upper_bound()};
        logging::log()->error(
            "\033[35m"
            "RobotConstraints:CheckSatisfied: constraint violated:"
            "\ndescription: [{}]"
            "\nlower_bound: [{}]"
            "\noutput:      [{}]"
            "\nupper_bound: [{}]"
            "\033[0m",
            constraint_this_thread->get_description(), lower_bound.transpose(),
            output.transpose(), upper_bound.transpose());
      }
      if (failed_strings.has_value()) {
        failed_strings->push_back(constraint_this_thread->get_description());
      }
      if (fail_fast) {
        return CheckSatisfiedResult(false);
      }
      if (offending_model_names.has_value()) {
        // For offending names, we collect the strings.
        const auto str = constraint_this_thread->get_description();
        // Check model names to see if they are in the description string, this
        // is a bit hacky but we don't have a better way to get offending model
        // names for linear constraints like joint position box constraints
        for (int i {0}; i < plant.num_model_instances(); ++i) {
          auto model_idx = drake::multibody::ModelInstanceIndex(i);
          if (plant.num_positions(model_idx) == 0) {
            // Skip models with no position dof, since they won't be in the
            // constraint description and can't be offending
            continue;
          }
          const auto model_name {plant.GetModelInstanceName(model_idx)};
          if (str.find(model_name) != std::string::npos) {
            offending_model_names->insert(model_name);
          }
        }
      }
    }
  }
  // check collision constraints, Why we don't use MinimumDistanceConstraint?
  // Because this is faster!
  if (collision_checker_ != nullptr) {
    const bool collision_free {
        collision_checker_->CheckContextConfigCollisionFree(
            contexts_.at(thread_num).get(), q_lifted)};
    valid = valid && collision_free;
    // Determine if we need to compute clearance for detailed info
    const bool need_clearance_detail {
        !collision_free
        && (opts.verbose || opts.color_collisions_meshcat
            || failed_strings.has_value()
            || offending_model_names.has_value())};
    if (need_clearance_detail || opts.calc_penalty) {
      // Use the penalty influence distance when computing penalty (to capture
      // near-miss pairs), otherwise use 0 (to capture only colliding pairs).
      const double clearance_dist {
          opts.calc_penalty ? collision_penalty_influence_distance_ : 0};
      const auto clearance {collision_checker_->CalcContextRobotClearance(
          contexts_.at(thread_num).get(), q_lifted, clearance_dist)};
      if (!collision_free) {
        if (opts.verbose) {
          logging::log()->error(
              "RobotConstraints:CheckSatisfied: collision constraint "
              "violated:\nq= [{}],\nclearance = \n{}",
              q_lifted.transpose(), PrintRobotClearance(clearance, q_lifted));
        }
        if (robot_model_.meshcat() && opts.color_collisions_meshcat) {
          logging::log()->debug(
              "RobotConstraints:CheckSatisfied: coloring colliding bodies in "
              "meshcat");
          robot_model_.ColorCollidingBodiesInMeshcat(clearance);
        }
        if (failed_strings.has_value()) {
          failed_strings->push_back(
              fmt::format("Collision constraint violated:,\nclearance = \n{}",
                          PrintRobotClearance(clearance)));
        }
        if (offending_model_names.has_value()) {
          const auto& plant {robot_model_.plant()};
          for (int i {0}; i < clearance.size(); ++i) {
            if (clearance.distances()(i) < 0) {
              const auto& body_A =
                  plant.get_body(clearance.robot_indices().at(i));
              const auto& body_B =
                  plant.get_body(clearance.other_indices().at(i));
              if (plant.num_positions(body_A.model_instance()) > 0) {
                offending_model_names->insert(
                    plant.GetModelInstanceName(body_A.model_instance()));
              }
              if (plant.num_positions(body_B.model_instance()) > 0) {
                offending_model_names->insert(
                    plant.GetModelInstanceName(body_B.model_instance()));
              }
            }
          }
        }
      } else if (robot_model_.meshcat() && opts.color_collisions_meshcat) {
        // make an empty clearance to bring back the original colors
        const auto empty_clearance {drake::planning::RobotClearance(
            robot_model_.plant().num_positions())};
        robot_model_.ColorCollidingBodiesInMeshcat(empty_clearance);
      }
      if (opts.calc_penalty) {
        for (const auto& distance : clearance.distances()) {
          *penalty += collision_penalty_func_(distance);
        }
      }
    } else if (robot_model_.meshcat() && opts.color_collisions_meshcat) {
      // collision_free && no penalty needed: just clear meshcat colors
      const auto empty_clearance {drake::planning::RobotClearance(
          robot_model_.plant().num_positions())};
      robot_model_.ColorCollidingBodiesInMeshcat(empty_clearance);
    }
    if (!collision_free && fail_fast) {
      return CheckSatisfiedResult(false);
    }
  }
  std::optional<std::vector<std::string>> offending_model_names_vec;
  if (offending_model_names.has_value()) {
    offending_model_names_vec = std::vector<std::string>(
        offending_model_names->begin(), offending_model_names->end());
  }
  return CheckSatisfiedResult(valid, penalty, failed_strings,
                              offending_model_names_vec);
}

CheckSatisfiedResult RobotConstraints::CheckSatisfied(
    const std::vector<Eigen::VectorXd>& q_vec,
    const std::optional<CheckSatisfiedOptions>& opts) const {
  const auto& check_satisfied_options = opts.value_or(CheckSatisfiedOptions());
  const auto start_time = std::chrono::high_resolution_clock::now();
  volatile bool valid = true;
  bool fail_fast =
      !(check_satisfied_options.collect_offending_model_names
        || check_satisfied_options.collect_failed_constraint_strings
        || check_satisfied_options.verbose
        || check_satisfied_options.calc_penalty);
  std::optional<double> penalty;
  std::optional<std::vector<std::string>> failed_strings;
  std::optional<std::vector<std::string>> offending_model_names_vec;

  if (check_satisfied_options.parallel && fail_fast) {
    omp_set_num_threads(
        std::min(check_satisfied_options.num_threads, num_threads_));
#pragma omp parallel for shared(valid)
    for (size_t i = 0; i < q_vec.size(); ++i) {
      if (!valid && fail_fast) {
        continue;
      }
      const auto& q = q_vec.at(i);
      auto result = CheckSatisfied(q, omp_get_thread_num(), opts);
      if (!result.satisfied()) {
        valid = false;
      }
    }
    const auto end = std::chrono::high_resolution_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);
    logging::log()->debug(
        "RobotConstraints:CheckSatisfied: hash = {}, n_confs = {}, parallel: "
        "{}, num_threads: {}, verbose: {}, duration = {} ms, valid = {}",
        constraints_hash_, q_vec.size(), check_satisfied_options.parallel,
        check_satisfied_options.num_threads, check_satisfied_options.verbose,
        duration.count(), valid);
  } else {
    // Non-parallel to collect detailed info
    if (check_satisfied_options.calc_penalty) {
      penalty = 0.0;
    }
    if (check_satisfied_options.collect_failed_constraint_strings) {
      failed_strings = std::vector<std::string>();
    }
    std::optional<std::set<std::string>> offending_model_names;
    if (check_satisfied_options.collect_offending_model_names) {
      offending_model_names = std::set<std::string>();
    }
    for (const auto& q : q_vec) {
      const auto result = CheckSatisfied(q, 0, opts);
      valid = valid && result.satisfied();
      if (result.failed_constraint_strings().has_value()
          && !result.failed_constraint_strings()->empty()) {
        failed_strings->insert(failed_strings->end(),
                               result.failed_constraint_strings()->begin(),
                               result.failed_constraint_strings()->end());
      }
      if (result.offending_model_names().has_value()
          && !result.offending_model_names()->empty()) {
        offending_model_names->insert(result.offending_model_names()->begin(),
                                      result.offending_model_names()->end());
      }
      if (result.penalty().has_value()) {
        *penalty += result.penalty().value();
      }
      if (!valid && fail_fast) {
        break;
      }
    }
    if (offending_model_names.has_value()) {
      offending_model_names_vec = std::vector<std::string>(
          offending_model_names->begin(), offending_model_names->end());
    }
  }
  const auto end = std::chrono::high_resolution_clock::now();
  const auto duration =
      std::chrono::duration_cast<std::chrono::milliseconds>(end - start_time);
  logging::log()->debug(
      "RobotConstraints:CheckSatisfied: hash = {}, n_confs = {}, parallel: "
      "{}, num_threads: {}, verbose: {}, duration = {} ms, valid = {}",
      constraints_hash_, q_vec.size(), check_satisfied_options.parallel,
      check_satisfied_options.num_threads, check_satisfied_options.verbose,
      duration.count(), valid);
  return CheckSatisfiedResult(valid, penalty, failed_strings,
                              offending_model_names_vec);
}

CheckSatisfiedResult RobotConstraints::CheckSatisfiedEdge(
    const Eigen::VectorXd& q1, const Eigen::VectorXd& q2, const double step,
    const std::optional<CheckSatisfiedOptions>& check_satisfied_options_opt)
    const {
  DRAKE_DEMAND(step > 0);
  const double distance {(q1 - q2).norm()};
  if (distance < step) {
    std::vector<Eigen::VectorXd> q_vec {q1, q2};
    return CheckSatisfied(q_vec, check_satisfied_options_opt);
  }
  const int num_samples {static_cast<int>(std::ceil(distance / step)) + 1};
  std::vector<Eigen::VectorXd> q_vec;
  for (int i {0}; i < num_samples; ++i) {
    // check convex combination factor
    const double c = static_cast<double>(i) / (num_samples - 1);
    q_vec.push_back(q1 * (1 - c) + q2 * c);
  }
  return CheckSatisfied(q_vec, check_satisfied_options_opt);
}

CheckSatisfiedResult RobotConstraints::CheckSatisfiedTrajectory(
    const drake::trajectories::Trajectory<double>& trajectory,
    const double step,
    const std::optional<CheckSatisfiedOptions>& check_satisfied_options) const {
  DRAKE_DEMAND(step > 0);
  std::vector<Eigen::VectorXd> q_vec;
  for (double t {0}; t < trajectory.end_time(); t += step) {
    q_vec.push_back(trajectory.value(t));
  }
  return CheckSatisfied(q_vec, check_satisfied_options);
}

std::pair<double, bool> RobotConstraints::CalcPenalty(
    const Eigen::VectorXd& q, const int thread_num) const {
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto& hm = robot_model_.holonomic_mapping();
  DRAKE_THROW_UNLESS(q.rows() == hm.minimal_dim());
  auto q_lifted = hm.Lift(q);
  // Lock the mutex
  std::scoped_lock lock(*(check_satisfied_mutex_vec_.at(thread_num)));
  double penalty {0.0};
  bool valid {true};
  for (int i {0}; i < std::ssize(constraints_no_collision_vec_); ++i) {
    const auto& constraint {constraints_no_collision_vec_.at(i)};
    const auto* constraint_this_thread {constraint.at(thread_num).get()};
    robot_model_.plant().SetPositions(
        &(contexts_[thread_num]->mutable_plant_context()), q_lifted);
    Eigen::VectorXd output;
    constraint_this_thread->Eval(q_lifted, &output);
    const auto& lower_bound {constraint_this_thread->lower_bound()};
    const auto& upper_bound {constraint_this_thread->upper_bound()};
    const auto& penalty_func {non_collision_penalty_funcs_.at(i)};
    penalty += penalty_func(output);
    if ((output.array() < lower_bound.array()).any()
        || (output.array() > upper_bound.array()).any()) {
      valid = false;
    }
  }
  // check collisions
  if (collision_checker_ != nullptr) {
    const auto clearance {collision_checker_->CalcContextRobotClearance(
        contexts_.at(thread_num).get(), q_lifted,
        collision_penalty_influence_distance_)};
    for (const auto& distance : clearance.distances()) {
      penalty += collision_penalty_func_(distance);
      if (distance < 0) {
        valid = false;
      }
    }
  }
  return {penalty, valid};
}

std::pair<std::vector<double>, bool> RobotConstraints::CalcPenaltyVec(
    const std::vector<Eigen::VectorXd>& q_vec) const {
  std::vector<double> penalties(q_vec.size(), 0.0);
  bool valid {true};
  omp_set_num_threads(num_threads_);
#pragma omp parallel for shared(valid)
  for (size_t i = 0; i < q_vec.size(); ++i) {
    const auto& q {q_vec.at(i)};
    const auto thread_num {omp_get_thread_num()};
    const auto [penalty, valid_this_thread] {CalcPenalty(q, thread_num)};
    penalties.at(i) = penalty;
    if (!valid_this_thread) {
      valid = false;
    }
  }
  return std::pair(penalties, valid);
}

std::optional<std::pair<double, bool>>
RobotConstraints::CalcPenaltyVecAggregated(
    const std::vector<Eigen::VectorXd>& q_vec,
    const PenaltyAggregationType agg_type) const {
  const auto [penalties, valid] {CalcPenaltyVec(q_vec)};
  if (penalties.empty()) {
    return {std::make_pair(0.0, valid)};
  }
  if (agg_type == PenaltyAggregationType::kSum) {
    return {std::make_pair(
        std::accumulate(penalties.begin(), penalties.end(), 0.0), valid)};
  } else if (agg_type == PenaltyAggregationType::kMax) {
    return {std::make_pair(
        *std::max_element(penalties.begin(), penalties.end()), valid)};
  }
  return std::nullopt;
}

std::optional<Eigen::VectorXd>
RobotConstraints::ProjectConfToNonCollisionConstraints(
    const Eigen::VectorXd& q, const int thread_num,
    const std::optional<Eigen::VectorXd>& q_guess,
    const double distance_cost) const {
  // setup a mathematical program with the constraints
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto& hm = robot_model_.holonomic_mapping();
  auto q_lifted = hm.Lift(q);
  drake::solvers::MathematicalProgram prog;
  const auto& q_var {
      prog.NewContinuousVariables(robot_model_.plant().num_positions(), "q")};
  // Add the position constraint
  for (const auto& constraint : constraints_no_collision_vec_) {
    robot_model_.plant().SetPositions(
        &(contexts_[thread_num]->mutable_plant_context()), q_lifted);
    prog.AddConstraint(constraint.at(thread_num), q_var);
  }
  // Add a cost between the current q and the projected q
  prog.AddQuadraticCost(distance_cost
                        * (q_var - q_lifted).dot(q_var - q_lifted));
  if (q_guess.has_value()) {
    auto q_guess_lifted = hm.Lift(q_guess.value());
    prog.SetInitialGuess(q_var, q_guess_lifted);
  } else {
    prog.SetInitialGuess(q_var, q_lifted);
  }
  // solve the program
  const auto result {drake::solvers::Solve(prog)};
  if (!result.is_success()) {
    logging::log()->error(
        "RobotConstraints:ProjectConfToNonCollisionConstraints: failed to "
        "solve the optimization problem");
    return std::nullopt;
  }
  auto q_sol = result.GetSolution(q_var);
  return hm.Reduce(q_sol);
}

std::vector<Eigen::VectorXd> RobotConstraints::GenerateSamples(
    drake::RandomGenerator* gen, const size_t desired_num_samples,
    const SampleOptions& options) const {
  const auto now {std::chrono::high_resolution_clock::now()};
  const auto& hm = robot_model_.holonomic_mapping();
  const auto q_min {robot_model_.plant().GetPositionLowerLimits()};
  const auto q_max {robot_model_.plant().GetPositionUpperLimits()};
  const auto q_min_reduced {hm.Reduce(q_min)};
  const auto q_max_reduced {hm.Reduce(q_max)};
  drake::geometry::optimization::Hyperrectangle config_space {q_min_reduced,
                                                              q_max_reduced};
  std::vector<Eigen::VectorXd> q_vec_samples;
  for (size_t i {0}; i < options.max_num_samples; ++i) {
    const auto q {config_space.UniformSample(gen)};
    q_vec_samples.push_back(q);
  }
  std::mutex valid_samples_mutex;
  std::vector<Eigen::VectorXd> valid_samples;
  omp_set_num_threads(options.parallel ? num_threads_ : 1);
  CheckSatisfiedOptions check_satified_options;
  check_satified_options.verbose = true;
#pragma omp parallel for shared(valid_samples)
  for (size_t i = 0; i < q_vec_samples.size(); ++i) {
    if (valid_samples.size() >= desired_num_samples) {
      continue;
    }
    const auto thread_num = omp_get_thread_num();
    const auto& q {q_vec_samples.at(i)};
    bool add_sample =
        bool(CheckSatisfied(q, thread_num)) == !options.return_invalid;  // XOR
    if (add_sample) {
      std::lock_guard<std::mutex> lock(valid_samples_mutex);
      valid_samples.push_back(q);
    } else if (options.use_projection) {
      // let's do projection
      if (valid_samples.size() == 0) {
        continue;
      }
      const auto guess_index {i % valid_samples.size()};
      const auto& q_guess {valid_samples.at(guess_index)};
      const auto q_projected {
          ProjectConfToNonCollisionConstraints(q, thread_num, q_guess, 10)};
      if (q_projected.has_value()) {
        // run check satisfied again
        if (CheckSatisfied(q_projected.value(), thread_num,
                           check_satified_options)) {
          std::lock_guard<std::mutex> lock(valid_samples_mutex);
          valid_samples.push_back(q_projected.value());
          logging::log()->info(
              "RobotConstraints:GenerateSamples: projected q {}/{} to "
              "constraints",
              i, q_vec_samples.size());
        } else {
          logging::log()->error(
              "RobotConstraints:GenerateSamples: failed to project q {}/{} to "
              "constraints",
              i, q_vec_samples.size());
        }
      }
    }
  }
  const auto time_elapsed {std::chrono::high_resolution_clock::now() - now};
  logging::log()->debug(
      "RobotConstraints:GenerateSamples: took {} ms to generate {} valid "
      "samples out of {} samples with use_projection = {}",
      std::chrono::duration_cast<std::chrono::milliseconds>(time_elapsed)
          .count(),
      valid_samples.size(), options.max_num_samples, options.use_projection);
  if (options.truncate_samples) {
    valid_samples.resize(std::min(valid_samples.size(), desired_num_samples));
  }
  return valid_samples;
}

drake::planning::RobotClearance RobotConstraints::CalcRobotClearance(
    const Eigen::VectorXd& q, const int thread_num) const {
  if (collision_checker_ == nullptr) {
    return drake::planning::RobotClearance(
        robot_model_.plant().num_positions());
  }
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto& hm = robot_model_.holonomic_mapping();
  DRAKE_THROW_UNLESS(q.rows() == hm.minimal_dim());
  auto q_lifted = hm.Lift(q);
  return collision_checker_->CalcContextRobotClearance(
      contexts_.at(thread_num).get(), q_lifted, 0);
}

RobotConstraints::CollisionType RobotConstraints::ClassifyCollisions(
    const drake::planning::RobotClearance& clearance) const {
  if (clearance.size() == 0) {
    return CollisionType::kNone;
  }
  const auto& plant = robot_model_.plant();
  std::optional<CollisionType> collision_type_so_far;
  for (int i {0}; i < clearance.size(); ++i) {
    std::optional<CollisionType> collision_type;
    auto model_index_1 =
        plant.get_body(clearance.robot_indices().at(i)).model_instance();
    auto arm_index_1 = robot_model_.get_arm_index(model_index_1);
    auto model_index_2 =
        plant.get_body(clearance.other_indices().at(i)).model_instance();
    auto arm_index_2 = robot_model_.get_arm_index(model_index_2);
    if ((arm_index_1.is_valid() && !arm_index_2.is_valid())
        || (!arm_index_1.is_valid() && arm_index_2.is_valid())) {
      // arm hitting a non-arm body.
      collision_type = CollisionType::kArmEnvOnly;
    } else if (arm_index_1.is_valid() && arm_index_2.is_valid()) {
      if (arm_index_1 == arm_index_2) {
        // self-arm collision
        collision_type = CollisionType::kArmSelfOnly;
      } else {
        // across-arm collision
        collision_type = CollisionType::kAcrossArmsOnly;
      }
    } else {
      // non-arm body collision
      throw std::runtime_error(
          "RobotConstraints:ClassifyCollisions: collision between two "
          "non-arm bodies detected, which should not be possible. This is a "
          "bug!");
    }
    if (collision_type_so_far.has_value()
        && collision_type_so_far.value() != collision_type) {
      return CollisionType::kMixed;
    }
    collision_type_so_far = collision_type;
  }
  DRAKE_DEMAND(collision_type_so_far.has_value());
  return collision_type_so_far.value();
}

RobotConstraints::CollisionType RobotConstraints::CalcAndClassifyCollisions(
    const Eigen::VectorXd& q, const int thread_num) const {
  if (collision_checker_ == nullptr) {
    return CollisionType::kNone;
  }
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto clearance = CalcRobotClearance(q, thread_num);
  return ClassifyCollisions(clearance);
}

bool RobotConstraints::DoArmsCollide(const Eigen::VectorXd& q,
                                     int thread_num) const {
  DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
  const auto clearance = CalcRobotClearance(q, thread_num);
  const auto& plant = robot_model_.plant();
  for (int i {0}; i < clearance.size(); ++i) {
    auto model_index_1 =
        plant.get_body(clearance.robot_indices().at(i)).model_instance();
    auto arm_index_1 = robot_model_.get_arm_index(model_index_1);
    auto model_index_2 =
        plant.get_body(clearance.other_indices().at(i)).model_instance();
    auto arm_index_2 = robot_model_.get_arm_index(model_index_2);
    if (arm_index_1.is_valid() && arm_index_2.is_valid()
        && arm_index_1 != arm_index_2) {
      return true;
    }
  }
  return false;
}

std::optional<Eigen::VectorXd>
RobotConstraints::CalcClosestSatisfyingConfiguration(
    const Eigen::VectorXd& q, int thread_num,
    const std::vector<drake::multibody::ModelInstanceIndex>& fixed_models,
    double collision_influence_distance) const {
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto& hm = robot_model_.holonomic_mapping();
  if (!hm.is_identity()) {
    throw std::runtime_error(
        "RobotConstraints:CalcClosestSatisfyingConfiguration: only supports "
        "identity holonomic mapping right now");
  }
  drake::solvers::MathematicalProgram prog;
  const auto& q_var {
      prog.NewContinuousVariables(robot_model_.plant().num_positions(), "q")};
  for (const auto& model_index : fixed_models) {
    if (robot_model_.plant().num_positions(model_index) == 0) {
      continue;
    }
    const int start_idx = robot_model_.GetModelStartIndex(model_index);
    Eigen::SparseMatrix<double> A(
        robot_model_.plant().num_positions(model_index),
        robot_model_.plant().num_positions());
    for (int i {0}; i < robot_model_.plant().num_positions(model_index); ++i) {
      A.insert(i, start_idx + i) = 1;
    }
    logging::log()->info(
        "RobotConstraints:CalcClosestSatisfyingConfiguration: fixed model "
        "{} start_idx {}, \n Dense A=\n{}, q.segment(start_idx, A.rows()) = {}",
        robot_model_.plant().GetModelInstanceName(model_index), start_idx,
        A.toDense(), q.segment(start_idx, A.rows()).transpose());
    prog.AddLinearEqualityConstraint(A, q.segment(start_idx, A.rows()), q_var);
  }
  for (const auto& constraint : constraints_no_collision_vec_) {
    prog.AddConstraint(constraint.at(thread_num), q_var);
    robot_model_.plant().SetPositions(
        &(contexts_.at(thread_num)->mutable_plant_context()), q);
    logging::log()->trace(
        "RobotConstraints:RobotConstraints: added constraint {} to "
        "RobotConstraints mathematical program at thread {}",
        constraint.at(thread_num)->get_description(), thread_num);
  }
  // Add if a collision checker exists
  if (collision_checker_ != nullptr) {
    const double kDelta {0.005};
    // The influence distance of minimum distance constraint is the distance
    // up to which the collision constraint is enforced.
    const auto minimum_distance_constraint =
        std::make_shared<drake::multibody::MinimumDistanceLowerBoundConstraint>(
            collision_checker_.get(), kDelta, contexts_.at(thread_num).get(),
            drake::solvers::QuadraticallySmoothedHingeLoss,
            collision_influence_distance);
    prog.AddConstraint(minimum_distance_constraint, q_var);
  }
  prog.SetInitialGuess(q_var, q);
  prog.AddQuadraticCost((q_var - q).dot(q_var - q));
  const auto result {drake::solvers::Solve(prog)};
  if (!result.is_success()) {
    logging::log()->error(
        "RobotConstraints:CalcClosestSatisfyingConfiguration: failed to "
        "solve the optimization problem");
    return std::nullopt;
  }
  logging::log()->debug(
      "RobotConstraints:CalcClosestSatisfyingConfiguration: solved the "
      "optimization problem");
  return result.GetSolution(q_var);
}

std::optional<Eigen::VectorXd>
RobotConstraints::CalcClosestSatisfyingConfigurationOnEdge(
    const Eigen::VectorXd& q, const Eigen::VectorXd& q_valid, int thread_num,
    const double step,
    const std::optional<CheckSatisfiedOptions>& check_satisfied_options) const {
  DRAKE_DEMAND(step > 0);
  if (!CheckSatisfied(q_valid, thread_num, check_satisfied_options)) {
    throw std::runtime_error(
        "CalcClosestSatisfyingConfigurationOnEdge: q_valid is not valid!");
  }
  const auto distance {(q_valid - q).norm()};
  if (distance < step) {
    if (CheckSatisfied(q, thread_num, check_satisfied_options)) {
      logging::log()->info(
          "CalcClosestSatisfyingConfigurationOnEdge: q is already valid");
      return q;
    }
    return std::nullopt;
  }
  const auto direction {(q_valid - q).normalized()};
  auto q_candidate {q};
  int i {0};
  auto edge_check_options {
      check_satisfied_options.value_or(CheckSatisfiedOptions())};
  edge_check_options.verbose = false;
  edge_check_options.color_collisions_meshcat = false;
  while ((q_candidate - q).norm() < distance) {
    if (CheckSatisfied(q_candidate, thread_num, edge_check_options)) {
      logging::log()->info(
          "CalcClosestSatisfyingConfigurationOnEdge: thread {} found valid "
          "config at "
          "step {} along the edge",
          thread_num, i);
      return q_candidate;
    }
    q_candidate += direction * step;
    ++i;
  }
  logging::log()->warn(
      "CalcClosestSatisfyingConfigurationOnEdge: no valid config found along "
      "the edge");
  return std::nullopt;
}

drake::geometry::optimization::HPolyhedron
RobotConstraints::FindSatisfactionHPolyhedron(const Eigen::VectorXd& q,
                                              const int thread_num) const {
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto& hm = robot_model_.holonomic_mapping();
  if (!hm.is_identity()) {
    throw std::runtime_error(
        "RobotConstraints:FindSatisfactionHPolyhedron: only supports "
        "identity holonomic mapping right now");
  }
  Eigen::MatrixXd A =
      Eigen::MatrixXd::Zero(0, robot_model_.plant().num_positions());
  Eigen::VectorXd b = Eigen::VectorXd::Zero(0);
  // let's solve for collision only
  const double kDelta {0.05};
  const auto clearance {collision_checker_->CalcContextRobotClearance(
      contexts_.at(thread_num).get(), q, kDelta)};
  const auto& distances {clearance.distances()};
  const auto& jacobians {clearance.jacobians()};
  const auto& robot_indices {clearance.robot_indices()};
  const auto& other_indices {clearance.other_indices()};
  if (distances.size() == 0) {
    logging::log()->info(
        "RobotConstraints:FindSatisfactionHPolyhedron: no collision "
        "found");
  } else {
    // let's move the conf in the direction of the Jacobian
    const int num_distances {clearance.size()};
    logging::log()->info(
        "RobotConstraints:FindSatisfactionHPolyhedron: found {} "
        "distances",
        num_distances);
    for (int i {0}; i < num_distances; ++i) {
      const double distance {distances(i)};
      const auto& jacobian {jacobians.row(i)};
      const std::string robot_body_name {
          collision_checker_->get_body(robot_indices.at(i)).name()};
      const std::string other_body_name {
          collision_checker_->get_body(other_indices.at(i)).name()};
      logging::log()->debug(
          "RobotConstraints:FindSatisfactionHPolyhedron: distance "
          "between {} and {} is {} with \n Jacobian {}",
          robot_body_name, other_body_name, distance, jacobian.transpose());
    }
    Eigen::MatrixXd A_collision {-jacobians};
    Eigen::VectorXd b_collision {distances
                                 - Eigen::VectorXd::Ones(num_distances) * kDelta
                                 - jacobians * q};
    A = A_collision;
    b = b_collision;
  }
  const auto q_ad = drake::math::InitializeAutoDiff(
      q, Eigen::MatrixXd::Identity(q.size(), q.size()));
  std::vector<Eigen::VectorXd> a_vec;
  std::vector<double> b_vec;
  const double kTol {-1e-3};
  for (const auto& constraint : constraints_no_collision_vec_) {
    drake::AutoDiffVecXd output;
    constraint.at(thread_num)->Eval(q_ad, &output);
    const auto value = drake::math::ExtractValue(output);
    const auto gradient = drake::math::ExtractGradient(output);
    for (int i {0}; i < output.rows(); ++i) {
      if (value(i) - constraint.at(thread_num)->lower_bound()(i) < kTol) {
        logging::log()->info(
            "RobotConstraints:FindSatisfactionHPolyhedron: constraint "
            "{} violated at {} with value {} and lower bound {}",
            constraint.at(thread_num)->get_description(), i, value(i),
            constraint.at(thread_num)->lower_bound()(i));
        a_vec.push_back(-gradient.row(i));
        b_vec.push_back(value(i) - constraint.at(thread_num)->lower_bound()(i)
                        - kTol - gradient.row(i).dot(q));
      }
      if (constraint.at(thread_num)->upper_bound()(i) - value(i) < kTol) {
        logging::log()->info(
            "RobotConstraints:FindSatisfactionHPolyhedron: constraint "
            "{} violated at {} with value {} and upper bound {}",
            constraint.at(thread_num)->get_description(), i, value(i),
            constraint.at(thread_num)->upper_bound()(i));
        a_vec.push_back(gradient.row(i));
        b_vec.push_back(constraint.at(thread_num)->upper_bound()(i) - value(i)
                        - kTol + gradient.row(i).dot(q));
      }
    }
    logging::log()->trace(
        "RobotConstraints:FindSatisfactionHPolyhedron: constraint {} "
        "output = \n {}",
        constraint.at(thread_num)->get_description(), value.transpose());
    logging::log()->trace(
        "RobotConstraints:FindSatisfactionHPolyhedron: constraint {} "
        "gradient = \n {}",
        constraint.at(thread_num)->get_description(), gradient);
  }
  // let's make the HPolyhedron
  for (size_t i {0}; i < a_vec.size(); ++i) {
    A.conservativeResize(A.rows() + 1, A.cols());
    A.row(A.rows() - 1) = a_vec.at(i);
    b.conservativeResize(b.size() + 1);
    b(b.size() - 1) = b_vec.at(i);
  }
  return drake::geometry::optimization::HPolyhedron(A, b);
}

std::optional<double> RobotConstraints::CalcPointDistanceToBody(
    const Eigen::VectorXd& q, const drake::multibody::Frame<double>& frame_A,
    const Eigen::Vector3d& p_AQ, const drake::multibody::Body<double>& body,
    double influence_distance, int thread_num) const {
  DRAKE_THROW_UNLESS(thread_num < num_threads_);
  const auto& hm = robot_model_.holonomic_mapping();
  DRAKE_THROW_UNLESS(q.rows() == hm.minimal_dim());
  auto q_lifted = hm.Lift(q);
  DRAKE_THROW_UNLESS(influence_distance >= 0);
  const auto& plant = robot_model_.plant();
  plant.SetPositions(&(contexts_.at(thread_num)->mutable_plant_context()),
                     q_lifted);
  const auto& sg_query_port =
      robot_model_.scene_graph().get_query_output_port();
  const auto& sg_query =
      sg_query_port.Eval<drake::geometry::QueryObject<double>>(
          contexts_.at(thread_num)->scene_graph_context());
  const auto X_W_frame_A = plant.CalcRelativeTransform(
      contexts_.at(thread_num)->plant_context(), plant.world_frame(), frame_A);
  const Eigen::VectorXd p_WQ = X_W_frame_A * p_AQ;
  auto signed_distance_to_points =
      sg_query.ComputeSignedDistanceToPoint(p_WQ, influence_distance);
  auto frame_id = plant.GetBodyFrameIdOrThrow(body.index());
  const auto& sg_inspector = robot_model_.scene_graph().model_inspector();
  auto geos =
      sg_inspector.GetGeometries(frame_id, drake::geometry::Role::kProximity);
  std::optional<double> min_distance;
  for (const auto& sdp : signed_distance_to_points) {
    if (!common::utils::contains(geos, sdp.id_G)) {
      continue;  // skip geometries not in the body
    }
    if (min_distance.has_value()) {
      min_distance = std::min(min_distance.value(), sdp.distance);
    } else {
      min_distance = sdp.distance;
    }
  }
  return min_distance;
}

void RobotConstraints::PrintAllCollisionPairs(double dist) const {
  DRAKE_DEMAND(collision_checker_ != nullptr);
  // calc clearance with maximum distance
  Eigen::VectorXd q {
      Eigen::VectorXd::Zero(robot_model_.plant().num_positions())};
  const auto clearance {collision_checker_->CalcRobotClearance(q, dist)};
  logging::log()->info(
      "RobotConstraints:PrintAllCollisionPairs: collision constraint from "
      "collision checker {}",
      drake::yaml::SaveYamlString(
          constraints_adapter_.collision_checker.value()));
  logging::log()->info(
      "RobotConstraints:PrintAllCollisionPairs: clearance = \n{}  ",
      PrintRobotClearance(clearance, robot_model_.holonomic_mapping().Lift(q)));
}

std::unique_ptr<drake::planning::CollisionChecker>
RobotConstraints::CreateCollisionChecker(
    const CollisionCheckerAdapter& collision_checker_adapter) const {
  auto start {std::chrono::steady_clock::now()};
  auto collision_checker {robot_model_.default_collision_checker().Clone()};
  // parse collision checkers
  Eigen::MatrixXi new_filter {collision_checker->GetFilteredCollisionMatrix()};
  Eigen::MatrixXd new_padding_matrix {collision_checker->GetPaddingMatrix()};
  // filtering groups
  if (collision_checker_adapter.filtered_groups.has_value()) {
    for (const auto& group_name :
         collision_checker_adapter.filtered_groups.value()) {
      SetFiltering(new_filter, group_name);
    }
  }
  // filtering pairs
  if (collision_checker_adapter.filtered_pairs.has_value()) {
    for (const auto& group_name_pair :
         collision_checker_adapter.filtered_pairs.value()) {
      const auto& group_name_1 {group_name_pair.at(0)};
      const auto& group_name_2 {group_name_pair.at(1)};
      SetFiltering(new_filter, group_name_1, group_name_2);
    }
  }
  collision_checker->SetCollisionFilterMatrix(new_filter);
  // padding
  if (collision_checker_adapter.paddings.has_value()) {
    for (const auto& padding_adapter :
         collision_checker_adapter.paddings.value()) {
      const auto& group_name_1 {padding_adapter.pair.at(0)};
      const auto& group_name_2 {padding_adapter.pair.at(1)};
      SetPadding(new_padding_matrix, padding_adapter.distance, group_name_1,
                 group_name_2);
    }
  }
  collision_checker->SetPaddingMatrix(new_padding_matrix);
  return collision_checker;
}

std::string RobotConstraints::PrintRobotClearance(
    const drake::planning::RobotClearance& clearance,
    const std::optional<Eigen::VectorXd>& q_lifted) const {
  if (clearance.size() == 0) {
    const auto msg {
        fmt::format(fg(fmt::rgb(0, 255, 0)), "\t\t{}\t\t", "No collision!")};
    logging::log()->info(msg);
    return msg;
  }
  std::string msg_all {"\n"};
  auto header {fmt::format(fg(fmt::rgb(0, 255, 255)), "{} \t {:30} {:30} {:20}",
                           "#", "robot_body", "other_body", "distance (mm)")};
  msg_all.append(header);
  const auto& added_geo_idx = robot_model_.added_geometry_body_index();
  const auto& plant = collision_checker_->plant();
  const bool can_query_shapes {q_lifted.has_value()
                               && !added_shape_geometry_ids_.empty()};
  if (can_query_shapes) {
    collision_checker_->UpdateContextPositions(contexts_.at(0).get(),
                                               q_lifted.value());
  }
  // This cache stores the closest added geometry label to each robot body index
  // to avoid repeat queries. We don't need to report the name of each added
  // geometry in collision with a given body; it's good enough to know the
  // closest.
  std::unordered_map<drake::multibody::BodyIndex, std::string>
      added_geo_label_cache;
  // For rows where idx == added_geo_idx, query only the (robot_geo, added_geo)
  // pairs we actually care about instead of computing all scene pairs.
  auto label_for_added_geo =
      [&](drake::multibody::BodyIndex robot_body_idx) -> std::string {
    auto it = added_geo_label_cache.find(robot_body_idx);
    if (it != added_geo_label_cache.end()) return it->second;
    std::string best_name;
    if (can_query_shapes) {
      const auto& query_object = contexts_.at(0)->GetQueryObject();
      const auto& inspector = query_object.inspector();
      const auto& robot_geos =
          plant.GetCollisionGeometriesForBody(plant.get_body(robot_body_idx));
      double best_dist = std::numeric_limits<double>::infinity();
      std::string_view best_raw_name;
      for (const auto& robot_geo_id : robot_geos) {
        for (const auto& [added_geo_id, added_shape_name] :
             added_shape_geometry_ids_) {
          if (inspector.CollisionFiltered(robot_geo_id, added_geo_id)) continue;
          const auto pair = query_object.ComputeSignedDistancePairClosestPoints(
              robot_geo_id, added_geo_id);
          if (pair.distance < best_dist) {
            best_dist = pair.distance;
            best_name = added_shape_name;
          }
        }
      }
    }
    if (best_name.empty()) {
      best_name = "added_geometry::Unnamed_Geometry";
    } else {
      best_name = fmt::format("added_geometry::{}", best_name);
    }
    added_geo_label_cache[robot_body_idx] = best_name;
    return best_name;
  };
  for (size_t i {0}; i < static_cast<size_t>(clearance.size()); ++i) {
    const auto r {clearance.distances()(i) > 0 ? 0 : 255};
    const auto g {clearance.distances()(i) > 0 ? 255 : 0};
    const auto b {150 + ((i + 1) % 2) * 155};
    const auto distance_mm {clearance.distances()(i) * 1000};
    const auto robot_idx {clearance.robot_indices().at(i)};
    const auto other_idx {clearance.other_indices().at(i)};
    const auto padding_mm {
        collision_checker_->GetPaddingMatrix()(static_cast<int>(robot_idx),
                                               static_cast<int>(other_idx))
        * 1000};
    // Label added geo with the special lambda defined above.
    auto body_label =
        [&](drake::multibody::BodyIndex idx,
            drake::multibody::BodyIndex counterpart) -> std::string {
      if (idx == added_geo_idx) {
        return label_for_added_geo(counterpart);
      }
      return robot_model_.default_collision_checker()
          .get_body(idx)
          .scoped_name()
          .to_string();
    };
    const auto msg {fmt::format(
        fg(fmt::rgb(r, g, b)), "{} \t {:30}  {:30} {:.2f} {} {:.2f}", i,
        body_label(robot_idx, other_idx), body_label(other_idx, robot_idx),
        distance_mm + padding_mm, distance_mm <= 0 ? "<=" : ">", padding_mm)};
    msg_all.append("\n");
    msg_all.append(msg);
  }
  return msg_all;
}

void RobotConstraints::HashCollisionGeos(
    drake::DelegatingHasher* hasher) const {
  logging::log()->debug(
      "RobotConstraints:HashCollisionGeos: Hashing collision geos");
  const auto start {std::chrono::high_resolution_clock::now()};
  drake::geometry::ShapeHasher shape_hasher {hasher};
  const auto& plant {robot_model_.plant()};
  if (!collision_checker_.get()) {
    logging::log()->debug(
        "RobotConstraints:HashCollisionGeos: collision_checker_ is null");
    return;
  }
  const auto& filter_matrix {collision_checker_->GetFilteredCollisionMatrix()};
  const auto& padding_matrix {collision_checker_->GetPaddingMatrix()};
  std::unordered_set<drake::multibody::BodyIndex> hashed_body_indices;
  for (int i {0}; i < filter_matrix.rows(); ++i) {
    for (int j {i + 1}; j < filter_matrix.cols(); ++j) {
      hash_append(*hasher, filter_matrix(i, j));
      hash_append(*hasher, padding_matrix(i, j));
      if (filter_matrix(i, j) == -1) {
        continue;
      }
      if (hashed_body_indices.contains(drake::multibody::BodyIndex(i))) {
        continue;
      }
      const auto& body {plant.get_body(drake::multibody::BodyIndex(i))};
      const auto& inspector {robot_model_.scene_graph().model_inspector()};
      for (const auto& geometry_id :
           plant.GetCollisionGeometriesForBody(body)) {
        shape_hasher.HashAppend(inspector.GetShape(geometry_id));
      }
    }
  }
  const auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start)};
  logging::log()->debug(
      "RobotConstraints:HashCollisionGeos: Hashed collision geometry in {} ms",
      duration.count());
}

void RobotConstraints::HashConstraints(drake::DelegatingHasher* hasher) const {
  // position constraints
  const auto start {std::chrono::high_resolution_clock::now()};
  if (constraints_adapter_.position_constraints) {
    HashPositionConstraints(hasher);
  }
  if (constraints_adapter_.angle_constraints) {
    HashAngularConstraints(hasher);
  }
  if (constraints_adapter_.joint_position_box_constraints) {
    HashJointPositionConstraints(hasher);
  }
  // TODO: hash other robot constraints
  const auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start)};
  logging::log()->debug(
      "RobotConstraints:HashConstraints: Hashed all constraints in {} ms",
      duration.count());
}

void RobotConstraints::HashPositionConstraints(
    drake::DelegatingHasher* hasher) const {
  for (const auto& position_box_constraint :
       *constraints_adapter_.position_constraints) {
    const auto& frame_A {
        robot_model_.GetScopedFrameByName(position_box_constraint.frame_A)};
    const auto& frame_B {
        robot_model_.GetScopedFrameByName(position_box_constraint.frame_B)};
    hash_append(*hasher, frame_A.index());
    hash_append(*hasher, frame_B.index());
    {
      const double* start = position_box_constraint.position_AQ_lower.data();
      const double* end =
          start + position_box_constraint.position_AQ_lower.size();
      drake::hash_append_range(*hasher, start, end);
    }
    {
      const double* start = position_box_constraint.position_AQ_upper.data();
      const double* end =
          start + position_box_constraint.position_AQ_upper.size();
      drake::hash_append_range(*hasher, start, end);
    }
    {
      const double* start = position_box_constraint.position_BQ.data();
      const double* end = start + position_box_constraint.position_BQ.size();
      drake::hash_append_range(*hasher, start, end);
    }
  }
}

void RobotConstraints::HashAngularConstraints(
    drake::DelegatingHasher* hasher) const {
  for (const auto& angle_constraint : *constraints_adapter_.angle_constraints) {
    const auto& frame_A {
        angle_constraint.frame_A == "world"
            ? robot_model_.plant().world_frame()
            : robot_model_.GetScopedFrameByName(angle_constraint.frame_A)};
    const auto& frame_B {
        robot_model_.GetScopedFrameByName(angle_constraint.frame_B)};
    hash_append(*hasher, frame_A.index());
    hash_append(*hasher, frame_B.index());
    {
      const double* start = angle_constraint.a_A.data();
      const double* end = start + angle_constraint.a_A.size();
      drake::hash_append_range(*hasher, start, end);
    }
    {
      const double* start = angle_constraint.b_B.data();
      const double* end = start + angle_constraint.b_B.size();
      drake::hash_append_range(*hasher, start, end);
    }
    drake::hash_append(*hasher, angle_constraint.angle_lower);
    drake::hash_append(*hasher, angle_constraint.angle_upper);
  }
}

void RobotConstraints::HashJointPositionConstraints(
    drake::DelegatingHasher* hasher) const {
  logging::log()->debug(
      "RobotConstraints:HashJointPositionConstraints: hashing "
      "joint_position_box_constraints");
  for (const auto& joint_position_box_constraint :
       *constraints_adapter_.joint_position_box_constraints) {
    hash_append(*hasher, joint_position_box_constraint.multibody_entity_name);
    for (const auto& bound : std::vector<Eigen::VectorXd> {
             joint_position_box_constraint.lower_bounds,
             joint_position_box_constraint.upper_bounds}) {
      const double* start = bound.data();
      const double* end = start + bound.size();
      drake::hash_append_range(*hasher, start, end);
    }
  }
}

void RobotConstraints::UpdateConstraintsOnMeshcat() const {
  DRAKE_THROW_UNLESS(robot_model_.meshcat() != nullptr);
  const auto q {robot_model_.GetMeshcatPositions()};
  // Render added collision shapes and color them per-geometry using a query
  if (added_collision_shapes_) {
    // First, place them in the scene
    for (int i = 0; i < std::ssize(*added_collision_shapes_); ++i) {
      const auto& shape_description = (*added_collision_shapes_)[i];
      const auto& frame_name = shape_description.parent_frame;
      const auto& X_FG = shape_description.X_FG;
      const auto& shape = shape_description.shape;
      const auto& frame {frame_name == "world"
                             ? robot_model_.plant().world_frame()
                             : robot_model_.GetScopedFrameByName(frame_name)};
      const auto X_WF = robot_model_.CalcRelativeTransform(
          q, robot_model_.plant().world_frame(), frame);
      const auto path = fmt::format("/drake/collision/added_geometry/{}",
                                    shape_description.name);
      robot_model_.meshcat()->SetObject(
          path, *shape, drake::geometry::Rgba(1.0, 0.5, 0.0, 0.5));
      robot_model_.meshcat()->SetTransform(path, X_WF * X_FG);
    }
    const auto q_lifted = robot_model_.holonomic_mapping().Lift(q);
    collision_checker_->UpdateContextPositions(contexts_.at(0).get(), q_lifted);
    const auto& query_object = contexts_.at(0)->GetQueryObject();
    const auto pairs =
        query_object.ComputeSignedDistancePairwiseClosestPoints(0.0);
    // Track minimum distance between each added geometry and robot.
    std::unordered_map<drake::geometry::GeometryId, double> min_dist;
    for (const auto& [geo_id, _] : added_shape_geometry_ids_) {
      min_dist[geo_id] = std::numeric_limits<double>::infinity();
    }
    for (const auto& pair : pairs) {
      const bool a_is_added {min_dist.contains(pair.id_A)};
      const bool b_is_added {min_dist.contains(pair.id_B)};
      // Ignore collisions without added geometries
      if (!a_is_added && !b_is_added) continue;
      if (a_is_added && pair.distance < min_dist[pair.id_A])
        min_dist[pair.id_A] = pair.distance;
      if (b_is_added && pair.distance < min_dist[pair.id_B])
        min_dist[pair.id_B] = pair.distance;
    }
    for (const auto& [geo_id, shape_name] : added_shape_geometry_ids_) {
      const auto path {
          fmt::format("/drake/collision/added_geometry/{}", shape_name)};
      const auto color = (min_dist.at(geo_id) <= 0.0)
                             ? std::vector<double> {1.0, 0.0, 0.0, 0.5}
                             : std::vector<double> {1.0, 0.5, 0.0, 0.5};
      robot_model_.meshcat()->SetProperty(path, "color", color);
    }
  }
  if (constraints_adapter_.angle_constraints.has_value()) {
    for (int i = 0; i < std::ssize(*constraints_adapter_.angle_constraints);
         ++i) {
      const auto& angle_constraint {
          constraints_adapter_.angle_constraints.value().at(i)};
      const auto& frame_A {
          angle_constraint.frame_A == "world"
              ? robot_model_.plant().world_frame()
              : robot_model_.GetScopedFrameByName(angle_constraint.frame_A)};
      const auto& frame_B {
          robot_model_.GetScopedFrameByName(angle_constraint.frame_B)};
      // Get the relative transform between frame_A and frame_B in meshcat
      auto X_AB = robot_model_.CalcRelativeTransform(q, frame_A, frame_B);
      const auto& cone_A_path = angle_paths_.at(i).first;
      const auto& X_A_cone_A = angle_paths_.at(i).second;
      auto X_B_cone_A = X_AB * X_A_cone_A;
      // Only want the translation of it, not the rotation.
      auto X_B_cone_A_on_B = drake::math::RigidTransformd(
          X_A_cone_A.rotation(), X_B_cone_A.translation());
      robot_model_.meshcat()->SetTransform(cone_A_path, X_B_cone_A_on_B);
    }
  }
}

void RobotConstraints::DisplayPositionConstraintsInMeshcat() const {
  DRAKE_THROW_UNLESS(robot_model_.meshcat() != nullptr);
  if (constraints_adapter_.position_constraints.has_value()) {
    for (const auto& position_constraint :
         constraints_adapter_.position_constraints.value()) {
      const auto& plant {robot_model_.plant()};
      const auto& frame_A {
          robot_model_.GetScopedFrameByName(position_constraint.frame_A)};
      const auto& frame_B {
          robot_model_.GetScopedFrameByName(position_constraint.frame_B)};
      const auto& body_A {frame_A.body()};
      const auto& body_B {frame_B.body()};
      const auto& model_A_name {
          plant.GetModelInstanceName(body_A.model_instance())};
      const auto& model_B_name {
          plant.GetModelInstanceName(body_B.model_instance())};
      const auto box {
          drake::geometry::Box {position_constraint.position_AQ_upper
                                - position_constraint.position_AQ_lower}};
      const auto sphere {drake::geometry::Sphere {0.01}};
      // TODO(sadra): role_to_augment can be visual if robot_meshcat_params
      // has visual role set to true. Give user the option to set the role.
      const std::string role_to_augment = "collision";
      const auto path_A {fmt::format("/drake/{}/{}/{}/box_constraint",
                                     role_to_augment, model_A_name,
                                     body_A.name())};
      logging::log()->info("path_A: {}", path_A);
      const auto color_box {position_constraint.color.value_or(
          drake::geometry::Rgba(0, 0.5, 0, 0.5))};
      robot_model_.meshcat()->SetObject(path_A, box, color_box);
      const drake::math::RigidTransformd X_frame_box {
          0.5
          * (position_constraint.position_AQ_upper
             + position_constraint.position_AQ_lower)};
      robot_model_.meshcat()->SetTransform(path_A, X_frame_box);
      const auto path_B {fmt::format("/drake/{}/{}/{}/point_Q", role_to_augment,
                                     model_B_name, body_B.name())};
      logging::log()->info("path_B: {}", path_B);
      robot_model_.meshcat()->SetObject(path_B, sphere,
                                        drake::geometry::Rgba(0, 0, 0, 0.5));
      const drake::math::RigidTransformd X_frame_sphere {
          position_constraint.position_BQ};
      robot_model_.meshcat()->SetTransform(path_B, X_frame_sphere);
    }
  }
}

std::vector<std::pair<std::string, drake::math::RigidTransformd>>
RobotConstraints::DisplayAngleConstraintFramesInMeshcat() const {
  DRAKE_THROW_UNLESS(robot_model_.meshcat() != nullptr);
  auto meshcat = robot_model_.meshcat();
  const auto& meshcat_diagram_context = robot_model_.meshcat_diagram_context();
  const auto& plant_context =
      robot_model_.plant().GetMyContextFromRoot(meshcat_diagram_context);
  std::vector<std::pair<std::string, drake::math::RigidTransformd>> cone_paths;
  if (constraints_adapter_.angle_constraints.has_value()) {
    for (const auto& angle_constraint :
         constraints_adapter_.angle_constraints.value()) {
      const auto& plant {robot_model_.plant()};
      const auto& frame_A {
          angle_constraint.frame_A == "world"
              ? plant.world_frame()
              : robot_model_.GetScopedFrameByName(angle_constraint.frame_A)};
      const auto& frame_B {
          robot_model_.GetScopedFrameByName(angle_constraint.frame_B)};
      robot_model_.AddFrameAxesToMeshcat(frame_A);
      robot_model_.AddFrameAxesToMeshcat(frame_B);
      auto vector_A = angle_constraint.a_A;
      auto vector_B = angle_constraint.b_B;
      // Let's add a cylinder object to vector A and vector B
      const double length_A {1.0};
      const double length_B {0.2};
      const double radius {0.005};
      const auto cylinder_A {drake::geometry::Cylinder {radius, length_A}};
      const auto cylinder_B {drake::geometry::Cylinder {radius, length_B}};
      const auto& visual_body_B =
          robot_model_.GetVisualBodyInTheSameMobileGroup(frame_B);
      const auto& X_visual_body_B_to_B = plant.CalcRelativeTransform(
          plant_context, visual_body_B.body_frame(), frame_B);
      auto path_to_vector_B = fmt::format(
          "/drake/visual/{}/{}/frame_B_angle_constraint",
          plant.GetModelInstanceName(visual_body_B.model_instance()),
          visual_body_B.name());
      // color for cone A
      auto color_A_cone = drake::geometry::Rgba(0.2, 0.1, 0.4, 0.2);
      // color for vector B
      auto color_B = drake::geometry::Rgba(0.2, 0.1, 0.4, 0.5);
      // cone vs vector for A
      double cone_scale = 0.7;
      meshcat->SetObject(path_to_vector_B, cylinder_B, color_B);
      drake::math::RigidTransformd X_A;
      X_A.set_rotation(
          drake::math::RotationMatrixd::MakeFromOneVector(vector_A, 2));
      X_A.set_translation(0.5 * length_A * vector_A.normalized());
      drake::math::RigidTransformd X_B;
      X_B.set_rotation(
          drake::math::RotationMatrixd::MakeFromOneVector(vector_B, 2));
      X_B.set_translation(0.5 * length_B * vector_B.normalized());
      meshcat->SetTransform(path_to_vector_B, X_visual_body_B_to_B * X_B);
      // Also add a meshcat cone with upper bound of angle constraint, only if
      // theta_lower = 0 and theta_upper < 90 degrees
      if (angle_constraint.angle_lower > 1e-3
          || angle_constraint.angle_upper > M_PI / 2 - 1e-3) {
        continue;
      }
      auto cone_radius = length_A * std::tan(angle_constraint.angle_upper);
      auto shape_meshcat_cone = drake::geometry::MeshcatCone {
          length_A * cone_scale, cone_radius * cone_scale,
          cone_radius * cone_scale};
      auto path_to_cone = fmt::format(
          "/drake/visual/{}/{}/angle_constraint_cone",
          plant.GetModelInstanceName(frame_A.body().model_instance()),
          frame_A.body().name());
      meshcat->SetObject(path_to_cone, shape_meshcat_cone, color_A_cone);
      drake::math::RigidTransformd X_A_cone;
      X_A_cone.set_rotation(
          drake::math::RotationMatrixd::MakeFromOneVector(vector_A, 2));
      meshcat->SetTransform(path_to_cone, X_A_cone);
      cone_paths.push_back(std::make_pair(path_to_cone, X_A_cone));
      // log vector A and vector B
      logging::log()->info(
          "RobotConstraints:DisplayAngleConstraintFramesInMeshcat: angle "
          "constraint between frame {} and frame {} with vector A {} and "
          "vector B {}, angle_lower (deg) = {}, angle_upper (deg) = {}",
          frame_A.name(), frame_B.name(), vector_A.transpose(),
          vector_B.transpose(), angle_constraint.angle_lower * 180 / M_PI,
          angle_constraint.angle_upper * 180 / M_PI);
    }
  }
  return cone_paths;
}

void RobotConstraints::RunMeshcatSlidersWithConstraints(
    const RobotConstraints& robot_constraints) {
  DRAKE_THROW_UNLESS(robot_constraints.robot_model().meshcat() != nullptr);
  auto meshcat = robot_constraints.robot_model().meshcat();
  logging::log()->info("MeshcatViz: Running Joint Sliders");
  meshcat->AddButton("Stop Sliders");
  const auto& plant {robot_constraints.robot_model().plant()};
  const auto q_up {plant.GetPositionUpperLimits()};
  const auto q_low {plant.GetPositionLowerLimits()};
  std::map<int, std::string> q_map;
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    const auto idx {drake::multibody::ModelInstanceIndex(i)};
    for (const auto& joint_index : plant.GetJointIndices(idx)) {
      const auto& joint {plant.get_joint(joint_index)};
      for (int j = 0; j < joint.num_positions(); ++j) {
        const auto joint_name =
            fmt::format("{}_{}", plant.GetModelInstanceName(idx), joint.name());
        logging::log()->info("MeshcatViz: Adding slider for joint {}, j:{}",
                             joint_name, joint.position_start());
        meshcat->AddSlider(joint_name, q_low(joint.position_start()),
                           q_up(joint.position_start()), 0.001, 0.0);
        q_map.emplace(std::make_pair(joint.position_start(), joint_name));
      }
    }
  }
  bool button_exists {false};
  while (meshcat->GetButtonClicks("Stop Sliders") == 0) {
    Eigen::VectorXd q(plant.num_positions());
    for (const auto& [q_index, joint_name] : q_map) {
      const auto q_val {meshcat->GetSliderValue(joint_name)};
      q(q_index) = q_val;
    }
    const auto& hm = robot_constraints.robot_model().holonomic_mapping();
    // Reduce it and the lift it
    auto q_reduced = hm.Reduce(q);
    q = hm.Lift(q_reduced);
    // Set the slider values to the lifted values
    for (const auto& [q_index, joint_name] : q_map) {
      meshcat->SetSliderValue(joint_name, q(q_index));
    }
    const std::string violation_msg {"Constraints Violated. Click to print"};
    robot_constraints.robot_model().SetMeshcatPositions(q_reduced);
    // make options such that we color collisions in realtime
    CheckSatisfiedOptions check_satisfied_options_meshcat;
    check_satisfied_options_meshcat.collect_offending_model_names = true;
    check_satisfied_options_meshcat.color_collisions_meshcat = true;
    if (!robot_constraints.CheckSatisfied(q_reduced, 0,
                                          check_satisfied_options_meshcat)) {
      if (!button_exists) {
        meshcat->AddButton(violation_msg);
        button_exists = true;
      }
    } else {
      if (button_exists) {
        meshcat->DeleteButton(violation_msg);
        button_exists = false;
      }
    }
    if (button_exists) {
      // now the generic one
      if (meshcat->GetButtonClicks(violation_msg) > 0) {
        logging::log()->error("Constraints Violated at {}", q.transpose());
        CheckSatisfiedOptions check_satisfied_options;
        check_satisfied_options.verbose = true;
        check_satisfied_options.color_collisions_meshcat = true;
        robot_constraints.CheckSatisfied(q, 0, check_satisfied_options);
        meshcat->DeleteButton(violation_msg);
        button_exists = false;
      }
    }
    robot_constraints.robot_model().PublishMeshcatContext();
    robot_constraints.UpdateConstraintsOnMeshcat();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void RobotConstraints::RemoveAllAddedCollisionShapes() const {
  DRAKE_THROW_UNLESS(collision_checker_ != nullptr);

  for (const auto& [_, name] : added_shape_geometry_ids_) {
    collision_checker_->RemoveAllAddedCollisionShapes(name);
  }
  added_shape_geometry_ids_.clear();
  if (robot_model_.meshcat() != nullptr) {
    robot_model_.meshcat()->Delete("/drake/collision/added_geometry");
  }
}

void RobotConstraints::AddCollisionShapes() const {
  DRAKE_THROW_UNLESS(collision_checker_ != nullptr);
  if (!added_collision_shapes_) return;
  const auto& plant = collision_checker_->plant();
  const auto& world = plant.world_frame();
  auto& plant_context = contexts_.at(0)->mutable_plant_context();
  const auto& added_geo_body_idx {robot_model_.added_geometry_body_index()};
  const auto& added_geo_frame = plant.get_body(added_geo_body_idx).body_frame();
  const auto added_geo_frame_id =
      plant.GetBodyFrameIdOrThrow(added_geo_body_idx);
  for (const auto& shape_description : *added_collision_shapes_) {
    const bool is_world {shape_description.parent_frame == "world"};
    const auto& source_frame {
        is_world ? world
                 : drake::multibody::parsing::GetScopedFrameByName(
                       plant, shape_description.parent_frame)};
    // Get the transform of the new shape in the frame of the special
    // "added_geometry" body
    const auto X_AG {plant.CalcRelativeTransform(plant_context, added_geo_frame,
                                                 source_frame)
                     * shape_description.X_FG};
    // Get the newly-added geo ID by comparing before and after adding the
    // shape.
    const auto geos_before {
        contexts_.at(0)->GetQueryObject().inspector().GetGeometries(
            added_geo_frame_id, drake::geometry::Role::kProximity)};
    if (!collision_checker_->AddCollisionShapeToFrame(
            shape_description.name, added_geo_frame, *shape_description.shape,
            X_AG)) {
      throw std::runtime_error(
          fmt::format("Failed to add shape '{}' to frame '{}'.",
                      shape_description.name, shape_description.parent_frame));
    }
    const auto geos_after {
        contexts_.at(0)->GetQueryObject().inspector().GetGeometries(
            added_geo_frame_id, drake::geometry::Role::kProximity)};
    bool added {false};
    for (const auto& geo_id : geos_after) {
      if (std::find(geos_before.begin(), geos_before.end(), geo_id)
          == geos_before.end()) {
        DRAKE_DEMAND(!added_shape_geometry_ids_.contains(geo_id));
        added_shape_geometry_ids_.emplace(geo_id, shape_description.name);
        added = true;
        break;
      }
    }
    if (!added) {
      throw std::runtime_error(fmt::format(
          "Failed to find geometry ID for added shape '{}' on frame '{}'.",
          shape_description.name, shape_description.parent_frame));
    }
    logging::log()->debug(
        "RobotConstraints:AddCollisionShapes: Added '{}' at world pose {} "
        "(from frame '{}')",
        shape_description.name, X_AG,
        is_world ? "world" : shape_description.parent_frame);
  }
}

}  // namespace motion
