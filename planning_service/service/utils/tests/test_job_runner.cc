
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file safe_job_runner.cc

#include <gtest/gtest.h>

#include "planning_service/service/utils/job_runner.h"

namespace service {
namespace utils {

class JobRunnerTest : public ::testing::Test {
 protected:
  struct TestResult {
    double value {1.0};
  };
  using TestJobRunner = JobRunner<TestResult>;
  void SetUp() override {
    const size_t capacity {2};
    runner = std::make_shared<TestJobRunner>(capacity);
  }

  std::function<TestResult()> safe_job {std::function([]() {
    std::this_thread::sleep_for(chrono_ms(1000));
    return TestResult();
  })};

  std::function<TestResult()> exception_job {std::function([]() {
    throw std::runtime_error("This job throws!");
    return TestResult();
  })};

  std::shared_ptr<TestJobRunner> runner {nullptr};
};

TEST_F(JobRunnerTest, Constructor) {
  EXPECT_THROW(TestJobRunner(0), std::runtime_error);
  EXPECT_THROW(TestJobRunner(-1), std::runtime_error);
  EXPECT_NO_THROW(runner = std::make_shared<TestJobRunner>(10));
  EXPECT_EQ(runner->Capacity(), 10);
  EXPECT_TRUE(runner->Empty());
  EXPECT_EQ(runner->Size(), 0);
}

TEST_F(JobRunnerTest, InsertNewJob) {
  runner->InsertNewJob("1", safe_job);
  EXPECT_FALSE(runner->Empty());
  EXPECT_EQ(runner->NumJobsInProgress(), 1);
  EXPECT_TRUE(runner->Contains("1"));
  EXPECT_FALSE(runner->ResultIsReady("1"));
  EXPECT_TRUE(runner->GetCompletedJobIds().empty());
  runner->WaitForResult("1");
  EXPECT_EQ(runner->NumJobsInProgress(), 0);
  EXPECT_TRUE(runner->ResultIsReady("1"));
  EXPECT_EQ(runner->GetCompletedJobIds().size(), 1);
  EXPECT_EQ(runner->GetCompletedJobIds().count("1"), 1);

  EXPECT_THROW(runner->InsertNewJob("1", safe_job), std::runtime_error);
  EXPECT_NO_THROW(runner->InsertNewJob("2", safe_job));
  EXPECT_THROW(runner->InsertNewJob("3", safe_job), std::runtime_error);
}

TEST_F(JobRunnerTest, WaitForResult) {
  EXPECT_THROW(runner->WaitForResult("1"), std::runtime_error)
      << "Cannot wait for result which does not exist";
  runner->InsertNewJob("1", safe_job);
  EXPECT_NO_THROW(runner->WaitForResult("1", 100))
      << "Waiting for active job should not throw";
  EXPECT_FALSE(runner->ResultIsReady("1"))
      << "Job running for 1 second should not be ready yet";
  EXPECT_NO_THROW(runner->WaitForResult("1"))
      << "Waiting for active job should not throw";
  EXPECT_TRUE(runner->ResultIsReady("1"))
      << "Job should be ready when blocking WaitForResult call terminates";
}

TEST_F(JobRunnerTest, RetrieveResult) {
  EXPECT_THROW(runner->RetrieveResult("1"), std::runtime_error);
  runner->InsertNewJob("1", safe_job);
  EXPECT_THROW(runner->RetrieveResult("1"), std::runtime_error);
  runner->WaitForResult("1");
  const auto result {runner->RetrieveResult("1")};
  EXPECT_TRUE(result.has_value());
  EXPECT_EQ(result->value, 1.0);
  EXPECT_THROW(runner->RetrieveResult("1"), std::runtime_error);
}

TEST_F(JobRunnerTest, RetrieveException) {
  runner->InsertNewJob("1", exception_job);
  runner->WaitForResult("1");
  std::expected<TestResult, ServiceError> result;
  EXPECT_NO_THROW(result = runner->RetrieveResult("1"))
      << "Job which failed due to internal exception should not rethrow on "
         "retrieval";
  EXPECT_FALSE(result.has_value()) << "Result should be nullified on exception";
}

TEST_F(JobRunnerTest, Clear) {
  runner->InsertNewJob("1", safe_job);
  runner->Clear();
  EXPECT_TRUE(runner->Empty());
  EXPECT_FALSE(runner->Contains("1"));
  EXPECT_THROW(runner->ResultIsReady("1"), std::runtime_error);
  EXPECT_EQ(runner->GetCompletedJobIds().size(), 0);
  EXPECT_EQ(runner->GetCompletedJobIds().count("1"), 0);
}

}  // namespace utils
}  // namespace service
