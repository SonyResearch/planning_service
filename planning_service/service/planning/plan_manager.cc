/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file plan_manager.cc

#include "plan_manager.h"

#include <magic_enum/magic_enum.hpp>

#include "planning_service/draco/client_conversions.h"
namespace service {
namespace planning {

using PlanContextId = draco::PlanContextId;

const drake::math::RigidTransformd MotionPlanManager::CalcRelativePose(
    const PlanContextId& context_id, const psc::SystemConf& system_conf,
    const std::string_view frame_B, const std::string_view frame_A) const {
  const auto& pdraco {registry_->GetDraco(context_id)};
  const auto q {draco::conversions::ToGeneralizedPosition(pdraco->robot_model(),
                                                          system_conf)};
  return pdraco->CalcRelativePose(q, frame_B.data(), frame_A.data());
}

}  // namespace planning
}  // namespace service
