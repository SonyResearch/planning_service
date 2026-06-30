#include "planning_service/motion/planning/ik_cache.h"

#include <drake/common/name_value.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/math/rigid_transform.h>

#include <algorithm>
#include <chrono>
#include <numeric>
#include <set>
#include <vector>

namespace motion {
namespace planning {

namespace {
double Distance(const drake::math::RigidTransformd& pose_A,
                const drake::math::RigidTransformd& pose_B,
                double angle_weight = 1.0) {
  const auto delta = pose_A.inverse() * pose_B;
  const double distance_translation = delta.translation().norm();
  const double distance_angle = delta.rotation().ToAngleAxis().angle();
  return distance_translation + angle_weight * std::abs(distance_angle);
}
}  // namespace

IkCache::IkCache(const motion::RobotModel& robot_model,
                 const std::vector<Eigen::VectorXd>& global_configs)
    : robot_model_(robot_model), cache_configs_(global_configs) {
  logging::log()->debug(
      "IkCache:Ctor: Made IK cache from {} global configurations and global "
      "frame relative poses",
      global_configs.size());
}

void IkCache::AddFrames(const drake::multibody::Frame<double>& frame_A,
                        const drake::multibody::Frame<double>& frame_B) const {
  auto now = std::chrono::steady_clock::now();
  auto key = std::make_pair(&frame_A, &frame_B);
  // Check if the frame pair already exists
  if (cached_frps_.find(key) != cached_frps_.end()) {
    logging::log()->warn(
        "IkCache:AddFrames: Frame pair {} and {} already exists in the cache. "
        "Re-evaluating ",
        frame_A.scoped_name().to_string(), frame_B.scoped_name().to_string());
    cached_frps_[key].clear();
  } else {
    cached_frps_.emplace(key, std::vector<drake::math::RigidTransformd>());
  }
  for (const auto& conf : cache_configs_) {
    // Add the frame pair to the global frame relative poses
    auto pose = robot_model_.CalcRelativeTransform(conf, frame_A, frame_B);
    cached_frps_[key].push_back(pose);
  }
  logging::log()->debug(
      "IkCache:AddFrames: Added {} {}/{} transforms in {} microseconds",
      cache_configs_.size(), frame_A.scoped_name().to_string(),
      frame_B.scoped_name().to_string(),
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - now)
          .count());
}

void IkCache::AddCacheConfig(const Eigen::VectorXd& q) {
  cache_configs_.push_back(q);
  for (auto& [key, q_vec] : cached_frps_) {
    const auto& [frame_A, frame_B] = key;
    auto pose = robot_model_.CalcRelativeTransform(q, *frame_A, *frame_B);
    q_vec.push_back(pose);
  }
  logging::log()->info(
      "IkCache:AddCacheConfig: Added new configuration to the cache. "
      "Total configurations: {}",
      cache_configs_.size());
}

std::vector<Eigen::VectorXd> IkCache::CalcClosestSeed(
    const FrameRelativePoses& frame_relative_poses,
    const Eigen::VectorXd& q_reference, double angle_weight, double q_weight,
    int num_seeds, bool select_seed_via_two_steps) const {
  if (select_seed_via_two_steps) {
    return CalcClosestSeed2Steps(frame_relative_poses, q_reference,
                                 angle_weight, num_seeds);
  } else {
    return CalcClosestSeed1Step(frame_relative_poses, q_reference, angle_weight,
                                q_weight, num_seeds);
  }
}

std::vector<Eigen::VectorXd> IkCache::CalcClosestSeed1Step(
    const FrameRelativePoses& frame_relative_poses,
    const Eigen::VectorXd& q_reference, double angle_weight, double q_weight,
    int num_seeds) const {
  DRAKE_THROW_UNLESS(num_seeds >= 1 || num_seeds == -1);
  DRAKE_THROW_UNLESS(frame_relative_poses.size() > 0);
  auto now = std::chrono::steady_clock::now();
  // Identify the active model instances from the frame pairs
  std::set<drake::multibody::ModelInstanceIndex> active_model_instances;
  for (const auto& [frame_A, frame_B, _] : frame_relative_poses) {
    // Add to the active model instances (set automatically handles
    // uniqueness)
    active_model_instances.insert(frame_A->model_instance());
    active_model_instances.insert(frame_B->model_instance());
  }
  int size_cache = std::ssize(cache_configs_);
  std::vector<double> distances;
  distances.reserve(size_cache + 1);  // +1 for the reference configuration
  std::vector<Eigen::VectorXd> seeds;
  // The reference configuration is itself a seed! Treat it with i == 0.
  for (int i = 0; i < size_cache + 1; ++i) {
    auto conf = i == 0 ? q_reference : cache_configs_[i - 1];
    robot_model_.SetIdleModelsConfigToRef(&conf, q_reference,
                                          active_model_instances);
    double distance_pose = 0;
    for (const auto& [frame_A_ptr, frame_B_ptr, pose] : frame_relative_poses) {
      const auto& key = std::make_pair(frame_A_ptr, frame_B_ptr);
      if (!cached_frps_.contains(key)) {
        // Need to add it!
        AddFrames(*frame_A_ptr, *frame_B_ptr);
        return CalcClosestSeed(frame_relative_poses, q_reference, angle_weight,
                               q_weight, num_seeds);
      }
      if (i == 0) {
        // Reference configuration: compute directly
        auto ref_pose = robot_model_.CalcRelativeTransform(conf, *frame_A_ptr,
                                                           *frame_B_ptr);
        distance_pose += Distance(pose, ref_pose, angle_weight);
      } else {
        // Read from cache
        const auto& cached_pose = cached_frps_.at(key).at(i - 1);
        distance_pose += Distance(cached_pose, pose, angle_weight);
      }
    }
    double distance_q = (conf - q_reference).norm();
    double distance =
        distance_pose + q_weight * distance_q;  // combine pose and config
    distances.push_back(distance);
    seeds.push_back(conf);
  }
  // Sort by distances, from closest to farthest
  std::vector<int> indices(seeds.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&distances](int a, int b) {
    return distances[a] < distances[b];
  });
  // return num_seeds closest seeds, or all if num_seeds is -1
  int num_seeds_to_return = num_seeds;
  if (num_seeds < 0 || num_seeds > std::ssize(seeds)) {
    num_seeds_to_return = std::ssize(seeds);
  }
  std::vector<Eigen::VectorXd> closest_seeds;
  closest_seeds.reserve(num_seeds_to_return);
  int i = 0;
  while (std::ssize(closest_seeds) < num_seeds_to_return
         && i < std::ssize(indices)) {
    const auto& candidate_seed = seeds[indices[i]];
    // avoid repeating the same seed. It happens a lot in PRM sampling.
    if (i > 0) {
      if (candidate_seed.isApprox(closest_seeds.back())) {
        ++i;
        continue;
      }
    }
    closest_seeds.push_back(candidate_seed);
    ++i;
  }
  logging::log()->debug(
      "IkCache:CalcClosestSeed: Sorted {} closest seeds out of {} candidates "
      "in {} microseconds",
      closest_seeds.size(), size_cache,
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - now)
          .count());
  return closest_seeds;
}

std::vector<Eigen::VectorXd> IkCache::CalcClosestSeed2Steps(
    const FrameRelativePoses& frame_relative_poses,
    const Eigen::VectorXd& q_reference, const double angle_weight,
    int num_seeds) const {
  DRAKE_THROW_UNLESS(num_seeds != 0);
  DRAKE_THROW_UNLESS(frame_relative_poses.size() > 0);
  auto now = std::chrono::steady_clock::now();
  // Identify the active model instances from the frame pairs
  std::set<drake::multibody::ModelInstanceIndex> active_model_instances;
  for (const auto& [frame_A, frame_B, _] : frame_relative_poses) {
    // Add to the active model instances (set automatically handles
    // uniqueness)
    active_model_instances.insert(frame_A->model_instance());
    active_model_instances.insert(frame_B->model_instance());
  }
  // Compute the distance from the seed's pose to the target pose (summed over
  // desired frames, if multiple exist)
  std::vector<Eigen::VectorXd> seeds;
  std::vector<double> pose_distances;
  seeds.reserve(cache_configs_.size() + 1);           // +1 for q_reference
  pose_distances.reserve(cache_configs_.size() + 1);  // +1 for q_reference
  for (std::size_t i = 0; i < cache_configs_.size() + 1; ++i) {
    // The reference configuration is itself a seed! Treat it with i == 0.
    auto q = i == 0 ? q_reference : cache_configs_[i - 1];
    robot_model_.SetIdleModelsConfigToRef(&q, q_reference,
                                          active_model_instances);
    double d = 0;
    for (const auto& [frame_A_ptr, frame_B_ptr, pose] : frame_relative_poses) {
      const auto& key = std::make_pair(frame_A_ptr, frame_B_ptr);
      // We add it to the cache to save time in case of future use
      if (!cached_frps_.contains(key)) AddFrames(*frame_A_ptr, *frame_B_ptr);
      // We compute the seed pose
      drake::math::RigidTransformd seed_pose =
          i == 0 ? robot_model_.CalcRelativeTransform(q, *frame_A_ptr,
                                                      *frame_B_ptr)
                 : cached_frps_.at(key).at(i - 1);
      d += Distance(pose, seed_pose, angle_weight);
    }
    pose_distances.push_back(d);
    seeds.push_back(q);
  }
  // Sort seed indices by distances, from closest to farthest
  std::vector<int> indices(seeds.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(), [&pose_distances](int a, int b) {
    return pose_distances[a] < pose_distances[b];
  });
  // Handle negative num_seeds values
  int num_seeds_final = num_seeds < 0 ? std::ssize(seeds) : num_seeds;
  // Handle too large num_seeds
  if (num_seeds_final > std::ssize(seeds)) {
    num_seeds_final = std::ssize(seeds);
    logging::log()->warn(
        "IkCache:CalcClosestSeed: Requested number of seeds ({}) is larger "
        "than the number of available seeds ({}). Capping to number of "
        "seeds.",
        num_seeds, seeds.size());
  }
  // Find the closest seeds (while checking for duplicates)
  std::vector<Eigen::VectorXd> closest_seeds;
  closest_seeds.reserve(num_seeds_final);
  int i = 0;
  while (std::ssize(closest_seeds) < num_seeds_final
         && i < std::ssize(indices)) {
    const Eigen::VectorXd& candidate_seed = seeds[indices[i]];
    // @Alban: this whole test is only necessary because the PRM-based IK
    // cache can contain a lot of duplicates when looking only at a subset
    // of the full joint configuration (ex: for one arm only)
    if (i == 0 || !candidate_seed.isApprox(closest_seeds.back()))
      closest_seeds.push_back(candidate_seed);
    ++i;
  }
  // We now re-order the closest seeds by their distance to q_reference
  std::sort(closest_seeds.begin(), closest_seeds.end(),
            [&q_reference](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
              return (a - q_reference).norm() < (b - q_reference).norm();
            });
  logging::log()->debug(
      "IkCache:CalcClosestSeed: Sorted {} closest seeds out of {} candidates "
      "in {} microseconds",
      closest_seeds.size(), seeds.size(),
      std::chrono::duration_cast<std::chrono::microseconds>(
          std::chrono::steady_clock::now() - now)
          .count());
  return closest_seeds;
}

}  // namespace planning
}  // namespace motion
