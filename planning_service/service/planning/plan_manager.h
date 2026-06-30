/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file plan_manager.h

#pragma once

#include <memory>
#include <string>

#include "planning_service/draco/draco.h"
#include "planning_service/service/types/types.h"
#include "planning_service/service/utils/resource_manager.h"
#include "planning_service/service/utils/utils.h"

namespace fs = std::filesystem;
namespace psc = planning_service_client;

namespace service {
namespace planning {

/**
 * @brief Class owning all threads of execution for solving motion plans.
 * Derived from ResourceManager, this class consumes PlanRequestAdapters,
 * computes their respective solutions on assigned threads, and stores the
 * results under the request ID.
 */
class MotionPlanManager {
 public:
  /**
   * @brief Constructor. Initializes the underlying queue.
   * @param system_name Enabled system name
   * @param options Options handling the loading of resources from disk
   */
  MotionPlanManager(const std::string& system_name,
                    const utils::ResourceOptions& options)
      : registry_ {std::make_shared<service::utils::ResourceRegistry>(
            system_name, options)} {}

 public:
  const std::shared_ptr<service::utils::ResourceRegistry>& registry() const {
    return registry_;
  }

  /**
   * @brief Given a planning context and a joint configuration, calculate the
   * pose of a frame B relative to a frame A.
   *
   * @param context_id ID for target context.
   * @param system_conf Joints as system configuration.
   * @param frame_B Target frame.
   * @param frame_A Relative frame.
   * @return const drake::math::RigidTransformd
   */
  const drake::math::RigidTransformd CalcRelativePose(
      const draco::PlanContextId& context_id,
      const psc::SystemConf& system_conf, const std::string_view frame_B,
      const std::string_view frame_A = "world") const;

 protected:
  std::string version_;
  std::shared_ptr<service::utils::ResourceRegistry> registry_;
};

}  // namespace planning
}  // namespace service
