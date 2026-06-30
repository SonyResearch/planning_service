
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <regex>
#include <thread>

#include "planning_service/common/logging.h"

TEST(Logger, Logger) {
  EXPECT_TRUE(logging::is_log_null()) << "logger should not be initialized";
  EXPECT_TRUE(logging::create_log()) << "logger creation has failed";
  EXPECT_FALSE(logging::is_log_null()) << "logger should no longer be null";
  logging::log()->set_level(spdlog::level::info);
  EXPECT_EQ(logging::log()->level(), spdlog::level::info)
      << "logger level should be info";
  logging::log()->set_level(spdlog::level::debug);
  EXPECT_EQ(logging::log()->level(), spdlog::level::debug)
      << "logger level should be debug";
  // map of verbosities to spdlog levels
  std::map<int, spdlog::level::level_enum> levels {
      {0, spdlog::level::critical}, {1, spdlog::level::err},
      {2, spdlog::level::warn},     {3, spdlog::level::info},
      {4, spdlog::level::debug},    {5, spdlog::level::trace}};
  for (const auto& [verbosity, level] : levels) {
    logging::set_verbosity(verbosity);
    EXPECT_EQ(logging::log()->level(), level)
        << "logger level should be " << level;
  }
}

/// Matches a UUID v4 string: xxxxxxxx-xxxx-4xxx-[89ab]xxx-xxxxxxxxxxxx
static bool IsValidUuid(const std::string& s) {
  static const std::regex kUuidRegex {
      "[0-9a-f]{8}-[0-9a-f]{4}-4[0-9a-f]{3}-[89ab][0-9a-f]{3}-[0-9a-f]{12}"};
  return std::regex_match(s, kUuidRegex);
}

TEST(TransactionId, GenerateIsUniqueUuid) {
  const auto id1 = logging::generate_transaction_id();
  const auto id2 = logging::generate_transaction_id();
  EXPECT_TRUE(IsValidUuid(id1)) << "id1 is not a valid UUID v4: " << id1;
  EXPECT_TRUE(IsValidUuid(id2)) << "id2 is not a valid UUID v4: " << id2;
  EXPECT_NE(id1, id2) << "Two generated UUIDs should not be equal";
}

TEST(TransactionId, SetAndGet) {
  const std::string custom_id {"deadbeef-1234-4abc-89ab-0123456789ab"};
  logging::set_transaction_id(custom_id);
  EXPECT_EQ(logging::get_transaction_id(), custom_id);
  EXPECT_EQ(logging::get_full_transaction_id(), custom_id);
  EXPECT_EQ(Logger::get_full_transaction_id(), custom_id);
}

TEST(TransactionId, SetEmptyGeneratesUuid) {
  logging::set_transaction_id("");  // empty → auto-generate
  const auto id = logging::get_transaction_id();
  EXPECT_FALSE(id.empty());
  EXPECT_TRUE(IsValidUuid(id))
      << "auto-generated id is not a valid UUID: " << id;
}

TEST(TransactionId, DefaultAutoGenerates) {
  // Force the thread-local to be empty by setting a blank UUID first then
  // clearing; we call set_transaction_id() without argument to auto-generate.
  logging::set_transaction_id();
  const auto id = logging::get_transaction_id();
  EXPECT_FALSE(id.empty());
  EXPECT_TRUE(IsValidUuid(id)) << "default id is not a valid UUID: " << id;
}

TEST(TransactionId, ThreadLocalIsolation) {
  // Each thread should have its own independent transaction ID.
  const std::string main_id {"aaaaaaaa-aaaa-4aaa-8aaa-aaaaaaaaaaaa"};
  logging::set_transaction_id(main_id);

  std::string thread_id;
  std::thread t([&thread_id] {
    // Spawn a fresh thread; it starts with an empty ID that auto-generates.
    thread_id = logging::get_transaction_id();
  });
  t.join();

  EXPECT_EQ(logging::get_transaction_id(), main_id)
      << "Main thread ID should be unchanged";
  EXPECT_NE(thread_id, main_id)
      << "Child thread should have a different ID than the main thread";
  EXPECT_FALSE(thread_id.empty()) << "Child thread ID should not be empty";
  EXPECT_TRUE(IsValidUuid(thread_id))
      << "Child thread ID is not a valid UUID: " << thread_id;
}

TEST(TransactionId, PropagateToChildThread) {
  // If the caller explicitly passes the ID into a spawned thread, both should
  // report the same ID.
  const std::string shared_id {"bbbbbbbb-bbbb-4bbb-8bbb-bbbbbbbbbbbb"};
  logging::set_transaction_id(shared_id);

  std::string child_id;
  std::thread t([&child_id, &shared_id] {
    logging::set_transaction_id(shared_id);
    child_id = logging::get_transaction_id();
  });
  t.join();

  EXPECT_EQ(child_id, shared_id)
      << "Child thread should report the same ID when explicitly set";
}
