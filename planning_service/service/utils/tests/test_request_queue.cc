
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file test_request_queue.cc

#include <gtest/gtest.h>

#include "planning_service/service/utils/request_queue.h"

namespace service {
namespace utils {
class RequestQueueTest : public ::testing::Test {
 protected:
  struct TestRequest : public RequestAdapter {
    const double data;
    TestRequest(const request_id_t& id, const double data)
        : RequestAdapter(id), data {data} {}
  };
  using TestQueue = RequestQueue<TestRequest>;

  void SetUp() override {
    const size_t max_length {2};
    queue = std::make_shared<TestQueue>(max_length);
  }

  std::shared_ptr<TestQueue> queue {nullptr};
};

TEST_F(RequestQueueTest, Constructor) {
  EXPECT_THROW(TestQueue(0), std::runtime_error);
  EXPECT_THROW(TestQueue(-1), std::runtime_error);
  EXPECT_NO_THROW(queue = std::make_shared<TestQueue>(10));
  EXPECT_EQ(queue->Capacity(), 10);
  EXPECT_TRUE(queue->Empty());
}

TEST_F(RequestQueueTest, Enqueue) {
  EXPECT_NO_THROW(queue->Enqueue(TestRequest("1", 100)));
  EXPECT_TRUE(queue->Contains("1"));
  EXPECT_NO_THROW(queue->Enqueue(TestRequest("2", 100)));
  EXPECT_TRUE(queue->Contains("2"));
  EXPECT_THROW(queue->Enqueue(TestRequest("3", 100)), queue_at_capacity_error);
  EXPECT_FALSE(queue->Contains("3"));
}

TEST_F(RequestQueueTest, PeekPop) {
  queue->Enqueue(TestRequest("1", 100));
  const auto& peeked {queue->PeekNextRequest()};
  EXPECT_EQ(peeked.id, "1");
  EXPECT_EQ(peeked.data, 100);
  EXPECT_EQ(queue->Size(), 1);
  EXPECT_TRUE(queue->Contains("1"));
  const auto popped {queue->PopNextRequest()};
  EXPECT_EQ(popped.id, "1");
  EXPECT_EQ(popped.data, 100);
  EXPECT_TRUE(queue->Empty());
  EXPECT_FALSE(queue->Contains("1"));
  EXPECT_THROW(queue->PeekNextRequest(), queue_empty_error);
  EXPECT_THROW(queue->PopNextRequest(), queue_empty_error);
}

TEST_F(RequestQueueTest, Clear) {
  queue->Enqueue(TestRequest("1", 100));
  queue->Enqueue(TestRequest("2", 100));
  queue->Clear();
  EXPECT_TRUE(queue->Empty());
  EXPECT_FALSE(queue->Contains("1"));
  EXPECT_THROW(queue->PeekNextRequest(), queue_empty_error);
}
}  // namespace utils
}  // namespace service
