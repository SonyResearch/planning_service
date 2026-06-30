/*
 * Copyright © 2024 Sony Research.  All rights reserved.
 */

/// @file ik_planner.h

#pragma once

#include <drake/common/name_value.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/math/rigid_transform.h>

#include "planning_service/motion/robot_model.h"

namespace motion {
namespace planning {

using FrameRelativePoses =
    std::vector<std::tuple<const drake::multibody::Frame<double>*,
                           const drake::multibody::Frame<double>*,
                           const drake::math::RigidTransformd>>;

class IkCache {
 public:
  IkCache(const motion::RobotModel& robot_model,
          const std::vector<Eigen::VectorXd>& global_configs);

  void AddFrames(const drake::multibody::Frame<double>& frame_A,
                 const drake::multibody::Frame<double>& frame_B) const;

  void AddCacheConfig(const Eigen::VectorXd& q);

  int size() const {
    return static_cast<int>(cache_configs_.size());
  }

  /** Searches the IK cache for the closest seeds to a target pose and reference
   configuration. Two different algorithms are available and can be selected
   using the flag `select_seed_via_two_steps`.
   @param frame_relative_poses A vector of tuples containing pairs of frames and
   their relative poses.
   @param q_reference A reference configuration that determines idle arms as
   well as plays a role in the cost function.
   @param angle_weight A weight for the angle difference in the cost function.
   The weight for translation is 1.
   @param q_weight A weight for the configuration difference from the reference
   configuration.
   @param num_seeds The number of seeds to return. If -1, returns all seeds.
   @param select_seed_via_two_steps If true, uses a two-step approach to find
   the closest seeds.

   @return A vector of configurations, sorted by their cost function value.
   */
  std::vector<Eigen::VectorXd> CalcClosestSeed(
      const FrameRelativePoses& frame_relative_poses,
      const Eigen::VectorXd& q_reference, double angle_weight = 1.0,
      double q_weight = 0.0, int num_seeds = -1,
      bool select_seed_via_two_steps = false) const;

  const motion::RobotModel& robot_model_;
  std::vector<Eigen::VectorXd> cache_configs_;
  mutable std::map<std::pair<const drake::multibody::Frame<double>*,
                             const drake::multibody::Frame<double>*>,
                   std::vector<drake::math::RigidTransformd>>
      cached_frps_;

 private:
  /** Searches the IK cache for a set of seeds that have a minimum cost, defined
    as a combination of:
    - a distance from the target pose in frame_relative_poses and the
    corresponding pose for the seed
      - itself a combination of translation distance and rotation distance
    weighted by angle_weight
    - a distance from the reference configuration and the seed's configuration,
    weighted by q_weight
    @note This is the first algorithm used historically in this project.
    @param frame_relative_poses A vector of tuples containing pairs of frames
    and their relative poses.
    @param q_reference A reference configuration that determines idle arms as
    well as plays a role in the cost function.
    @param angle_weight A weight for the angle difference in the cost function.
    The weight for translation is 1.
    @param q_weight A weight for the configuration difference from the
    reference configuration.
    @param num_seeds The number of seeds to return. If -1, returns all seeds.
   */
  std::vector<Eigen::VectorXd> CalcClosestSeed1Step(
      const FrameRelativePoses& frame_relative_poses,
      const Eigen::VectorXd& q_reference, double angle_weight = 1.0,
      double q_weight = 0.0, int num_seeds = -1) const;

  /** Searches the IK cache for a set of seeds that are the closest to the
    target pose and reference joint configuration in 2 steps:
    - first step: find the subset of seeds with the minimal distance from the
    target pose in frame_relative_poses and the corresponding pose for the seed
      - this distance is itself a combination of translation distance and
    rotation distance, weighted by angle_weight
    - second step: order this subset based on the distance from the reference
    configuration and the seed's configuration (increasing order)
    @param frame_relative_poses A vector of tuples containing pairs of frames
    and their relative poses.
    @param q_reference A reference configuration that determines idle arms as
    well as plays a role in the cost function.
    @param angle_weight A weight for the angle difference in the cost function.
    The weight for translation is 1.
    @param num_seeds The number of seeds to return. If -1, returns all seeds.
   */
  std::vector<Eigen::VectorXd> CalcClosestSeed2Steps(
      const FrameRelativePoses& frame_relative_poses,
      const Eigen::VectorXd& q_reference, double angle_weight = 1.0,
      int num_seeds = -1) const;
};

}  // namespace planning
}  // namespace motion
