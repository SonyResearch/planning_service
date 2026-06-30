
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file test_iris_manager.cc

#include <gtest/gtest.h>

#include "planning_service/service/iris/iris_manager.h"

namespace service {
namespace iris {
using IrisRegionsAdapter = motion::iris::IrisRegionsAdapter;

/** Stub class exposing protected methods to the test fixture. */
class IrisBuildManagerStub : public IrisBuildManager {
 public:
  IrisBuildManagerStub(const std::string_view system,
                       const utils::ResourceOptions& options)
      : IrisBuildManager(system.data(), options) {}
  FRIEND_TEST(IrisManagerTest, BuildFromSystemConfs);
  FRIEND_TEST(IrisManagerTest, BuildFromEdges);
  FRIEND_TEST(IrisManagerTest, BuildFromRoadmap);
};
/** Test fixture for the IRIS manager. */
class IrisManagerTest : public ::testing::Test {
 protected:
  /**
   * @brief Copy the `iiwa` data into a temp dir to load into the TestManager,
   * and create a new manager instance.
   */
  void SetUp() override {
    const fs::path iiwa_data_dir {"planning_service/test_data/iiwa/"};
    temp_dir = common::utils::temp_dir();
    const auto temp_iiwa_data_dir {temp_dir / "iiwa"};
    fs::create_directories(temp_iiwa_data_dir);
    fs::copy(iiwa_data_dir, temp_iiwa_data_dir,
             fs::copy_options::recursive | fs::copy_options::update_existing);
    // remove extant IRIS regions, we don't need them
    fs::remove(temp_iiwa_data_dir / "contexts" / std::to_string(kIiwaHash)
               / "iris_regions.yaml");
    utils::ResourceOptions options;
    options.data_path_root_override = temp_dir;
    options.make_draco_visualizers = false;
    options.make_meshcat_ports_html_thread = false;
    mgr = std::make_shared<IrisBuildManagerStub>("iiwa", options);

    sys_conf_0 = {{"iiwa", Eigen::VectorXd::Zero(7)}};
    sys_conf_1 = {{"iiwa", Eigen::VectorXd::Constant(7, 0.05)}};
  }
  void TearDown() override {
    fs::remove_all(temp_dir);
  }
  const IrisRegionsAdapter LoadIrisRegions() const {
    const auto iris_regions_path {temp_dir / "iiwa" / "contexts"
                                  / std::to_string(kIiwaHash)
                                  / "iris_regions.yaml"};
    return drake::yaml::LoadYamlFile<IrisRegionsAdapter>(iris_regions_path);
  }
  system_conf_t sys_conf_0, sys_conf_1;
  const uint64_t kIiwaHash {8878108380082535913U};
  const draco::PlanContextId iiwa_id {kIiwaHash};
  fs::path temp_dir {};
  std::shared_ptr<IrisBuildManagerStub> mgr {nullptr};
};

TEST_F(IrisManagerTest, BuildFromSystemConfs) {
  const auto result {mgr->BuildRegionsFromSystemConfs(
      iiwa_id, std::vector<system_conf_t>({sys_conf_0}))};
  EXPECT_EQ(result.context_id.value, kIiwaHash);
  EXPECT_EQ(result.type, IrisBuildJobType::IRIS_FROM_CONFIGS);
  const auto regions {LoadIrisRegions()};
  EXPECT_EQ(regions.regions_vec().size(), 1);
  EXPECT_EQ(regions.regions_vec().front().name(), "config_0");
  EXPECT_EQ(regions.regions_vec().front().constraints_hash(), kIiwaHash);
}

TEST_F(IrisManagerTest, BuildFromEdges) {
  const auto result {mgr->BuildRegionsFromEdges(
      iiwa_id, std::vector<system_conf_edge_t>(
                   {std::make_pair(sys_conf_0, sys_conf_1)}))};
  EXPECT_EQ(result.context_id.value, kIiwaHash);
  EXPECT_EQ(result.type, IrisBuildJobType::IRIS_FROM_EDGES);
  const auto regions {LoadIrisRegions()};
  EXPECT_EQ(regions.regions_vec().size(), 1);
  EXPECT_EQ(regions.regions_vec().front().name(), "edge_0");
  EXPECT_EQ(regions.regions_vec().front().constraints_hash(), kIiwaHash);
}

}  // namespace iris
}  // namespace service
