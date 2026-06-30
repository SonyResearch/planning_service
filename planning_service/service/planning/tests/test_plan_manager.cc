/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file test_plan_manager.cc

#include <gtest/gtest.h>

#include "planning_service/service/planning/plan_manager.h"
namespace service {
namespace planning {
/** Stub class exposing protected methods to the test fixture. */

using PlanContextId = draco::PlanContextId;

class MotionPlanManagerStub : public MotionPlanManager {
 public:
  MotionPlanManagerStub(const std::string_view system,
                        const utils::ResourceOptions& options)
      : MotionPlanManager(system.data(), options) {}
};
/** Test fixture for the plan manager. */
class MotionPlanManagerTest : public ::testing::Test {
 protected:
  void SetUp() override {
    const fs::path iiwa_data_dir {"planning_service/test_data/iiwa/"};
    temp_dir = common::utils::temp_dir();
    const auto temp_iiwa_data_dir {temp_dir / "iiwa"};
    fs::create_directories(temp_iiwa_data_dir);
    fs::copy(iiwa_data_dir, temp_iiwa_data_dir,
             fs::copy_options::recursive | fs::copy_options::update_existing);
    utils::ResourceOptions options;
    options.data_path_root_override = temp_dir;
    options.make_draco_visualizers = false;
    options.make_meshcat_ports_html_thread = false;
    mgr = std::make_shared<MotionPlanManagerStub>("iiwa", options);
  }
  void TearDown() override {
    fs::remove_all(temp_dir);
  }

 private:
  const uint64_t kIiwaHash {8878108380082535913U};
  const PlanContextId iiwa_id {kIiwaHash};
  fs::path temp_dir {};
  std::shared_ptr<MotionPlanManagerStub> mgr {nullptr};
};

}  // namespace planning
}  // namespace service
