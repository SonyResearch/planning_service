#ifndef TRY_MULTIPLE_APPROACHES_H
#define TRY_MULTIPLE_APPROACHES_H

#include <fmt/format.h>

#include <atomic>
#include <functional>
#include <future>
#include <mutex>
#include <sstream>
#include <vector>

#include <expected>

#include "ctpl_stl.h"

namespace internal {

template <typename R, typename... Args>
std::expected<R, std::string> TryMultipleApproaches(
    ctpl::thread_pool& thread_pool,
    const std::vector<std::function<std::expected<R, std::string>(Args...)>>&
        approaches,
    Args... args) {
  using Expected = std::expected<R, std::string>;

  auto result_promise = std::make_shared<std::promise<Expected>>();
  auto result_future = result_promise->get_future();

  auto has_value = std::make_shared<std::atomic<bool>>(false);
  auto approaches_remaining =
      std::make_shared<std::atomic<int>>(approaches.size());

  auto error_mutex = std::make_shared<std::mutex>();
  auto error_messages = std::make_shared<std::vector<std::string>>();

  for (int index = 0; index < static_cast<int>(approaches.size()); ++index) {
    auto approach = approaches[index];

    thread_pool.push([approach, index, has_value, approaches_remaining,
                      result_promise, error_messages, error_mutex,
                      args...](int id) mutable {
      (void)id;

      try {
        if (has_value->load()) return;

        auto result = approach(args...);

        if (result.has_value()) {
          if (!has_value->exchange(true)) {
            result_promise->set_value(std::move(result));
          }
        } else {
          {
            std::lock_guard<std::mutex> lock(*error_mutex);
            error_messages->emplace_back(
                fmt::format("Approach {} failed: {}", index, result.error()));
          }

          if (approaches_remaining->fetch_sub(1) == 1) {
            // Last one, and none succeeded
            if (!has_value->exchange(true)) {
              std::ostringstream oss;
              for (const auto& msg : *error_messages) {
                oss << msg << "\n";
              }
              result_promise->set_value(std::unexpected(oss.str()));
            }
          }
        }
      } catch (const std::exception& e) {
        {
          std::lock_guard<std::mutex> lock(*error_mutex);
          error_messages->emplace_back(
              fmt::format("Approach {} threw exception: {}", index, e.what()));
        }

        if (approaches_remaining->fetch_sub(1) == 1) {
          if (!has_value->exchange(true)) {
            std::ostringstream oss;
            for (const auto& msg : *error_messages) {
              oss << msg << "\n";
            }
            result_promise->set_value(std::unexpected(oss.str()));
          }
        }
      }
    });
  }

  return result_future.get();  // returns immediately once first result is set
}

}  // namespace internal

#endif  // TRY_MULTIPLE_APPROACHES_H
