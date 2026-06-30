
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file test_resource_manager.cc

#include <gtest/gtest.h>

#include "planning_service/service/utils/resource_registry.h"

namespace service {
namespace utils {

class ResourceRegistryTest : public ::testing::Test {
  class ResourceRegistryStub : public ResourceRegistry {
   public:
    ResourceRegistryStub(const std::string_view system,
                         const ResourceOptions& options)
        : ResourceRegistry(system.data(), options) {}
    FRIEND_TEST(ResourceRegistryTest, AddDraco);
  };
  /**
   * @brief Copy the `iiwa` data into a temp dir to load into the TestManager,
   * and create a new manager instance.
   */
  void SetUp() override {
    const fs::path iiwa_data_dir {"planning_service/test_data/iiwa/"};
    const fs::path temp_dir {common::utils::temp_dir()};
    temp_iiwa_data_dir = temp_dir / "iiwa";
    fs::create_directories(temp_iiwa_data_dir);
    fs::copy(iiwa_data_dir, temp_iiwa_data_dir,
             fs::copy_options::recursive | fs::copy_options::update_existing);
    ResourceOptions options;
    options.data_path_root_override = temp_dir;
    options.make_draco_visualizers = false;
    options.make_meshcat_ports_html_thread = false;
    registry = std::make_shared<ResourceRegistryStub>("iiwa", options);
    // test confs
    q1 = Eigen::VectorXd::Zero(7);
    q2 = Eigen::VectorXd::Zero(7);
    // small move from view conf
    q1 << -1.47974, 0.161801, -0.472364, -1.66149, 0.45, 1.80, 0.6;
    q2 << -1.47974, 0.161801, -0.472364, -1.66149, 0.403434, 1.62826, 0.389758;
  }
  void TearDown() override {
    fs::remove_all(temp_iiwa_data_dir);
  }

 protected:
  fs::path temp_iiwa_data_dir {};
  const uint64_t kIiwaHash {8878108380082535913U};
  const uint64_t kIiwaBoxesHash {10878899324708315470U};
  PlanContext context {kIiwaHash};
  std::shared_ptr<ResourceRegistryStub> registry {nullptr};
  Eigen::VectorXd q1, q2;
};

TEST_F(ResourceRegistryTest, Constructor) {
  EXPECT_TRUE(registry->HasDraco(kIiwaHash));
  EXPECT_TRUE(registry->HasDraco(kIiwaBoxesHash));
  EXPECT_NO_THROW(registry->RemoveDraco(kIiwaHash));
  EXPECT_NO_THROW(registry->RemoveDraco(kIiwaBoxesHash));
  EXPECT_FALSE(registry->HasDraco(kIiwaHash));
  EXPECT_FALSE(registry->HasDraco(kIiwaBoxesHash));
}

TEST_F(ResourceRegistryTest, AddDraco) {
  registry->RemoveDraco(kIiwaHash);
  EXPECT_TRUE(utils::LoadContext(context, registry->context_base_path()));
  EXPECT_THROW(registry->AddDraco(100000, context), std::runtime_error);
  EXPECT_NO_THROW(registry->AddDraco(kIiwaHash, context));
  EXPECT_TRUE(registry->HasDraco(kIiwaHash));
  EXPECT_EQ(
      kIiwaHash,
      registry->GetDraco(kIiwaHash)->robot_constraints().constraints_hash());
}

TEST_F(ResourceRegistryTest, RegisterPlanContext) {
  registry->RemoveDraco(kIiwaHash);
  EXPECT_TRUE(utils::LoadContext(context, registry->context_base_path()));
  const auto registration_result {
      registry->RegisterPlanContext("iiwa", context)};
  EXPECT_TRUE(registration_result.has_value());
}

TEST_F(ResourceRegistryTest, RemovePlanContext) {
  EXPECT_TRUE(registry->HasDraco(kIiwaHash));
  EXPECT_TRUE(fs::is_directory(registry->context_base_path()
                               / std::to_string(kIiwaHash)));
  const auto removal_result {registry->RemovePlanContext(
      "iiwa", draco::PlanContextId(kIiwaHash), true)};
  EXPECT_TRUE(removal_result.has_value());
  EXPECT_FALSE(registry->HasDraco(kIiwaHash));
  EXPECT_FALSE(fs::is_directory(registry->context_base_path()
                                / std::to_string(kIiwaHash)));
}
}  // namespace utils
}  // namespace service
