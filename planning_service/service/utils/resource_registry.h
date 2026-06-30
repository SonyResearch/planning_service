/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file resource_registry.h

#pragma once

#include <stdlib.h>

#include <expected>

#include "planning_service/common/file_utils.h"
#include "planning_service/common/time_utils.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/service/types/error.h"
#include "utils.h"

namespace fs = std::filesystem;

namespace service {
namespace utils {

/**
 * @brief Class which manages the main suite of compute resources (i.e., the
 * "Draco") for planning and other algorithms.
 */
class ResourceRegistry {
 public:
  /**
   * @brief Constructor. Create the Draco instances at the data path.
   * @param system_name Enabled system name
   * @param options Options handling the loading of resources from disk
   */
  ResourceRegistry(const std::string& system_name,
                   const ResourceOptions& options);

  // At most a single manager is scoped to an active service
  ResourceRegistry(ResourceRegistry& other) = delete;
  ResourceRegistry(ResourceRegistry&& other) = delete;

  ~ResourceRegistry() {
    html_stop_requested_.store(true);
    html_cv_.notify_all();
    if (html_thread_.joinable()) {
      html_thread_.join();
    }
  }

  /** Remove a Draco instance at the given ID. */
  void RemoveDraco(const draco::PlanContextId& id);

  /** Check if a Draco instance exists for the given ID.  */
  bool HasDraco(const draco::PlanContextId& id) const {
    return draco_map_.count(id.value);
  }

  /** Read-only Draco access. */
  const std::unique_ptr<draco::planner::DracoPlanner>& GetDraco(
      const draco::PlanContextId& id) const {
    if (!draco_map_.count(id.value)) {
      throw std::runtime_error(
          fmt::format("Draco {} was requested, but does not exist!", id));
    }
    return draco_map_.at(id.value);
  }

  /** Mutable Draco access. */
  const std::unique_ptr<draco::planner::DracoPlanner>& GetMutableDraco(
      const draco::PlanContextId& id) {
    if (!draco_map_.count(id.value)) {
      throw std::runtime_error(
          fmt::format("Draco {} was requested, but does not exist!", id));
    }
    return draco_map_.at(id.value);
  }

  /**
   * @brief Register a given plan context and initialize its corresponding Draco
   * instance.
   *
   * @param context
   * @return std::expected<draco::PlanContextId, ServiceError>
   */
  std::expected<draco::PlanContextId, ServiceError> RegisterPlanContext(
      const std::string& system, PlanContext context);

  /**
   * @brief Remove a plan context and its corresponding Draco instance.
   *
   * If passed `erase`, then remove the context from disk afterward.
   *
   * @param system
   * @param context_id
   * @param erase
   * @return std::expected<draco::PlanContextId, ServiceError>
   */
  std::expected<draco::PlanContextId, ServiceError> RemovePlanContext(
      const std::string& system, const draco::PlanContextId& context_id,
      bool erase = false);
  /**
   * @brief Migrate planning artifacts from one context to another.
   * @param from_context_id the context to migrate from
   * @param to_context_id the context to migrate to
   * @param num_samples the number of samples to generate
   */
  std::expected<void, std::string> MigratePlanningArtifacts(
      const draco::PlanContextId& from_context_id,
      const draco::PlanContextId& to_context_id, const int num_samples,
      const bool repair_artifacts = true);

  /** Read-only getter for context_base_path. */
  const fs::path context_base_path() const {
    return context_base_path_;
  }
  /** Read-only getter for data_path. */
  const fs::path data_path() const {
    return data_path_;
  }
  /** Read-only getter for log_path. */
  const fs::path log_path() const {
    return log_path_;
  }

  const ResourceOptions& options() const {
    return options_;
  }

  const std::string& version() const {
    return version_;
  }

  /**
   * @brief Create a new problems directory for a given context.
   *
   * @param context_id ID
   * @param plan_id Optional plan ID to use in the directory name.
   * @return const fs::path
   */
  fs::path NewProblemDirectory(const draco::PlanContextId& context_id,
                               const std::string& plan_id = "") const;

  const std::map<uint64_t, PlanContext>& active_contexts_map() const {
    return active_contexts_map_;
  }

  const std::map<uint64_t, std::unique_ptr<draco::planner::DracoPlanner>>&
  draco_map() const {
    return draco_map_;
  }

 protected:
  /**
   * @brief Internal. Add a Draco instance at the assigned ID constructed from
   * the adapter. Includes a validation check that the assigned ID matches the
   * computed ID of the actual Draco instance.
   *
   * @param id Assigned ID
   * @param adapter target Draco adapter
   */
  void AddDraco(const draco::PlanContextId& id,
                const service::PlanContext& context);

 private:
  void LaunchOrResetResourcesHtmlThread();

  static const fs::path kDefaultLogPathRoot;
  static const fs::path kDefaultDataPathRoot;

  // map to draco instances
  const std::string version_;
  const std::string system_name_;
  const ResourceOptions options_;
  const fs::path data_path_root_;
  // path to all data
  const fs::path data_path_;
  // path to all logs
  const fs::path log_path_;
  // path to all contexts
  const fs::path context_base_path_;
  // map of Draco instances by ID
  std::map<uint64_t, std::unique_ptr<draco::planner::DracoPlanner>> draco_map_;
  // map of active contexts by ID
  std::map<uint64_t, PlanContext> active_contexts_map_;
  mutable std::mutex draco_map_mutex_;
  // Related to the HTML server for meshcat ports
  std::optional<int> maybe_html_port_;
  std::thread html_thread_;
  std::mutex html_mutex_;
  std::condition_variable html_cv_;
  std::atomic<bool> html_stop_requested_ {false};
};

}  // namespace utils
}  // namespace service
