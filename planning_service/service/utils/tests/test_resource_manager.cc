
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file test_resource_manager.cc

#include <gtest/gtest.h>

#include "planning_service/service/utils/resource_manager.h"

namespace service {
namespace utils {

/** Trivial request type. */
struct TestRequest : public RequestAdapter {
  const double data;
  TestRequest(const request_id_t& id, const double data)
      : RequestAdapter(id), data {data} {}
};
/** Trivial result type. */
struct TestResult {
  const double value;
  TestResult(const double value) : value {value} {}
};
/** Simple manager around test request/result types. */
class TestManager : public ResourceManager<TestRequest, TestResult> {
 public:
  // Constructor
  TestManager(const std::string_view system, const ResourceOptions& options)
      : ResourceManager<TestRequest, TestResult>(system.data(), options) {}

  /**
   * @brief Start job implementation. Create a trivial job (wait for some 1
   * second before returning a result) and initiate it.
   */
  const std::expected<bool, ServiceError> StartJobImpl(
      const TestRequest& request) {
    const auto func {[request]() {
      const int delay_ms {1000};
      std::this_thread::sleep_for(chrono_ms(delay_ms));
      return TestResult(request.data);
    }};
    job_runner_->InsertNewJob(request.id, func);
    return true;
  }
};

class ResourceManagerTest : public ::testing::Test {
 protected:
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
    mgr = std::make_shared<TestManager>("iiwa", options);
  }
  void TearDown() override {
    fs::remove_all(temp_iiwa_data_dir);
  }

  fs::path temp_iiwa_data_dir {};
  std::shared_ptr<TestManager> mgr {nullptr};
};

TEST_F(ResourceManagerTest, QueueRequest) {
  EXPECT_NO_THROW(mgr->QueueRequest(TestRequest("1", 1.0)))
      << "Queueing initial request should be allowed";
  const auto over_capacity_result {mgr->QueueRequest(TestRequest("2", 2.0))};
  EXPECT_FALSE(over_capacity_result.has_value())
      << "Request in excess of capacity should return an error";
  EXPECT_EQ(over_capacity_result.error().code,
            ServiceErrorCode::QUEUE_CAPACITY_REACHED)
      << "Incorrect error code";
  const auto dupe_result {mgr->QueueRequest(TestRequest("1", 1.0))};
  EXPECT_TRUE(dupe_result.has_value())
      << "Duplicate request should not return an error";
  EXPECT_TRUE(dupe_result.value()) << "Should always return true";
  EXPECT_TRUE(mgr->HasJob("1")) << "Job not present as expected";
  mgr->ClearQueue();
  EXPECT_FALSE(mgr->HasJob("1")) << "Job should no longer be present";
}

TEST_F(ResourceManagerTest, RunOnce) {
  auto start_result {mgr->RunOnce()};
  EXPECT_FALSE(start_result.has_value())
      << "Starting an empty queue is not permitted";
  EXPECT_EQ(start_result.error().code, ServiceErrorCode::QUEUE_EMPTY)
      << "Incorrect error code for empty queue";
  mgr->QueueRequest(TestRequest("1", 1.0));
  start_result = mgr->RunOnce();
  EXPECT_TRUE(start_result.has_value()) << "Job should have been started";
  EXPECT_EQ(start_result.value(), true) << "Success should return true";
}
}  // namespace utils
}  // namespace service
