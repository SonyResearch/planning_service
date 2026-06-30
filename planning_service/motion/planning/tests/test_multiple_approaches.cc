// test_multiple_approaches.cpp

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <numeric>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#include "planning_service/motion/planning/try_multiple_approaches.h"

namespace internal {

using return_t = std::expected<int, std::string>;

// Function that returns quickly
return_t FastFunction() {
  return 42;
}

return_t SlowFunction() {
  // Simulate delay before returning a valid result
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return 42;
}

// Function that is very slow
return_t VerySlowFunction() {
  std::this_thread::sleep_for(std::chrono::seconds(1));
  return 123;
}

std::string fast_error = "FastExceptionFunction failed";
std::string slow_error = "SlowExceptionFunction failed";
// Function that throws an error quickly
return_t FastExceptionFunction() {
  return std::unexpected(fast_error);
}
// Function that throws an error slowly
return_t SlowExceptionFunction() {
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  return std::unexpected(slow_error);
}

// Base function template
return_t BaseFunction(const std::string& function_name,
                      std::chrono::microseconds sleep_duration, int arg1,
                      int arg2, int arg3, std::atomic<int>& call_count,
                      bool do_print = false) {
  int current_call = ++call_count;

  // Randomly throw an exception approximately every 10 times
  if (current_call % 50 == 0) {
    std::stringstream ss;
    ss << function_name << " with args (" << arg1 << ", " << arg2 << ", "
       << arg3 << ")";
    return std::unexpected(ss.str());
  }

  std::this_thread::sleep_for(sleep_duration);
  if (do_print) {
    std::cout << function_name << " with args (" << arg1 << ", " << arg2 << ", "
              << arg3 << ") completed\n";
  }

  return arg1 + arg2 + arg3;
}

// Specializations of BaseFunction
return_t FastFunctionArgs(int arg1, int arg2, int arg3) {
  static std::atomic<int> call_count {0};
  return BaseFunction("FastFunction", std::chrono::microseconds(10), arg1, arg2,
                      arg3, call_count);
}

return_t MediumFunctionArgs(int arg1, int arg2, int arg3) {
  static std::atomic<int> call_count {0};
  return BaseFunction("MediumFunction", std::chrono::milliseconds(1), arg1,
                      arg2, arg3, call_count);
}

return_t SlowFunctionArgs(int arg1, int arg2, int arg3) {
  static std::atomic<int> call_count {0};
  return BaseFunction("SlowFunction", std::chrono::seconds(1), arg1, arg2, arg3,
                      call_count, true);
}

TEST(TryMultipleApproachesTest, FastFunctionReturnsCorrectly) {
  ctpl::thread_pool pool(4);
  std::vector<std::function<return_t()>> approaches = {FastFunction};
  int result;
  EXPECT_NO_THROW(result =
                      TryMultipleApproaches<int>(pool, approaches).value());
  EXPECT_EQ(result, 42);
}

TEST(TryMultipleApproachesTest, ExceptionFunctionThrows) {
  ctpl::thread_pool pool(4);
  std::vector<std::function<return_t()>> approaches = {FastExceptionFunction};
  EXPECT_TRUE(
      TryMultipleApproaches<int>(pool, approaches).error().find(fast_error)
      != std::string::npos);
}

TEST(TryMultipleApproachesTest, SlowExceptionFunctionThrowsLast) {
  ctpl::thread_pool pool(4);
  std::vector<std::function<return_t()>> approaches = {FastExceptionFunction,
                                                       SlowExceptionFunction};
  EXPECT_TRUE(
      TryMultipleApproaches<int>(pool, approaches).error().find(slow_error)
      != std::string::npos);
}

TEST(TryMultipleApproachesTest, FastAndExceptionFunction) {
  ctpl::thread_pool pool(4);
  std::vector<std::function<return_t()>> approaches = {FastExceptionFunction,
                                                       FastFunction};
  int result;
  EXPECT_NO_THROW(result =
                      TryMultipleApproaches<int>(pool, approaches).value());
  EXPECT_EQ(result, 42);
}

TEST(TryMultipleApproachesTest, SlowAndExceptionFunctions) {
  ctpl::thread_pool pool(4);
  std::vector<std::function<return_t()>> approaches = {
      FastExceptionFunction, SlowExceptionFunction, VerySlowFunction};
  int result;
  EXPECT_NO_THROW(result =
                      TryMultipleApproaches<int>(pool, approaches).value());
  EXPECT_EQ(result, 123);
}

TEST(TryMultipleApproachesTest, VerySlowFunction) {
  ctpl::thread_pool pool(4);
  std::vector<std::function<return_t()>> approaches = {VerySlowFunction,
                                                       FastFunction};
  int result;
  EXPECT_NO_THROW(result =
                      TryMultipleApproaches<int>(pool, approaches).value());
  EXPECT_EQ(result, 42);
}

TEST(TryMultipleApproachesTest, FastMediumSlowFunctions) {
  // Create a thread pool with at least 4 threads
  ctpl::thread_pool thread_pool(8);

  // Define the approaches
  std::vector<std::function<return_t(int, int, int)>> approaches = {
      FastFunctionArgs, MediumFunctionArgs, SlowFunctionArgs};

  int arg1 = 1;
  int arg2 = 2;
  int arg3 = 3;

  // Run multiple iterations to gather statistics
  const int num_trials = 100;
  std::vector<double> solve_times;

  for (int iter = 0; iter < num_trials; ++iter) {
    auto start_time = std::chrono::steady_clock::now();
    arg1 = iter;
    try {
      auto result = TryMultipleApproaches<int, int, int, int>(
                        thread_pool, approaches, arg1, arg2, arg3)
                        .value();
      EXPECT_EQ(result, arg1 + arg2 + arg3);
    } catch (const std::exception& e) {
      // Handle exception if needed
      std::cout << "Caught exception: " << e.what() << std::endl;
    }

    auto end_time = std::chrono::steady_clock::now();

    // Calculate time taken in microseconds
    double solve_time = std::chrono::duration_cast<std::chrono::microseconds>(
                            end_time - start_time)
                            .count();

    // Add to solve times vector
    solve_times.push_back(solve_time);
  }

  // Calculate average and standard deviation
  double sum = std::accumulate(solve_times.begin(), solve_times.end(), 0.0);
  double avg_solve_time = sum / solve_times.size();

  double sq_sum = std::inner_product(solve_times.begin(), solve_times.end(),
                                     solve_times.begin(), 0.0);
  double std_solve_time =
      std::sqrt(sq_sum / solve_times.size() - avg_solve_time * avg_solve_time);

  // Print the statistics
  std::cout << "Average solve time: " << avg_solve_time << " microseconds\n";
  std::cout << "Standard deviation of solve time: " << std_solve_time
            << " microseconds\n";
}

TEST(TryMultipleApproachesTest, WaitsForValidResultWhenOthersThrow) {
  ctpl::thread_pool pool(4);

  std::vector<std::function<return_t()>> approaches = {
      FastExceptionFunction, FastExceptionFunction, SlowExceptionFunction,
      SlowFunction};

  auto start_time = std::chrono::steady_clock::now();

  int result;
  EXPECT_NO_THROW(result =
                      TryMultipleApproaches<int>(pool, approaches).value());

  auto end_time = std::chrono::steady_clock::now();
  auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                        end_time - start_time)
                        .count();

  // Verify that we received the valid result
  EXPECT_EQ(result, 42);

  // Verify that the elapsed time is at least 100 ms (since SlowFunction
  // sleeps for 100 ms)
  EXPECT_GE(elapsed_ms, 100);
  // Allow some buffer for timing variations
  EXPECT_LT(elapsed_ms, 200);
}

}  // namespace internal
