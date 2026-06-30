/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file common/logging.h
#pragma once
/// NEW GOALS:
/// In general, do not shadow drake::log(), because that makes the original
/// drake logger inaccessible wherever it is shadowed.  In that case Drake
/// libraries will themselves still call drake::log() with the [console] label
/// writing to stderr, and our code will not be able to change its settings.
/// So define OVERRIDE_DRAKE_LOG 0

#include <pwd.h>
#include <time.h>

#include <drake/common/text_logging.h>
#include <fmt/color.h>
#include <spdlog/pattern_formatter.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <filesystem>
#include <iomanip>
#include <sstream>
#include <string>

#include "planning_service/common/fmt.h"
#include "planning_service/common/fmt_eigen.h"
#include "planning_service/common/fmt_json.h"

#define OVERRIDE_DRAKE_LOG 0

/// Declare a logger based on spdlog (or nothing).  We can use it regardless.
class Logger : public spdlog::logger {
 public:
  template <class It>
  Logger(const std::string& name, const It& begin, const It& end)
      : spdlog::logger {name, begin, end} {}

  /// Returns the current thread's transaction ID (UUID).
  static std::string get_full_transaction_id();

 private:
  unsigned log_id_error_size_ =
      42;  // based on "[2022-10-27 14:14:48.870] [dexai] [error] "
};

/// Goals:
/// * Leave drake's "console" stderr logger visible by default, or, optionally,
/// * overload and hide drake::log() by header declaration; replace it by
/// linkage.
/// * If specified, add a file to the Dexai (replacement) logger's sinks;
///   - Use a default file if the specified file is not found.

namespace logging {

/// Returns true IFF the logger object is NULL,
/// meaning that either is has not yet been created, or it was nullified.
bool is_log_null();

/// Expose the logger, whether drake::log is shadowed or not.
spdlog::logger* log(int notify_if_null = 1);

/// * Leave drake's "console" stderr logger visible by default, or, optionally,
/// * overload and hide drake::log() by header declaration; replace it by
/// linkage.
/// * If specified, add a file to the Dexai (replacement) logger's sinks;
///   - Use a default file if the specified file is not found.
/// @param program Name of program, used as directory name for logs
/// the user's HOME/log_robot directory
/// @param prefix For use in format: [timestamp] [prefix] [level] message
bool create_log(const std::string& program = "planning_service",
                const std::string& prefix = "", bool use_stdout = false,
                bool create_once = true);

/// Generates a new random UUID v4 string (e.g.
/// "b5f1eaf7-ea01-40bb-bd3f-cfce8c8ee571").
std::string generate_transaction_id();

/// Sets the transaction ID for the current thread. If @p id is empty, a new
/// UUID is generated automatically.
void set_transaction_id(const std::string& id = "");

/// Returns the transaction ID for the current thread.
const std::string& get_transaction_id();

/// Returns the full transaction ID for the current thread (same as
/// get_transaction_id()). Provided for use by gRPC clients:
///   context.AddMetadata("transaction_id", Logger::get_full_transaction_id());
std::string get_full_transaction_id();

/// Returns: const pointer to array of 7 string views glossing spdlog level
/// names. Example: const auto log_level_names = SpdLogLevelNames();
///          const std::string& warn_view = log_level_names[pdlog::level::warn];
// const std::vector<std::string>& SpdLogLevelNames();

// Drake redirects python logging which can segfault if importing
// drake into python interface again, causing frequent TC failure.
// Disabling drake logging is a temporary hack. There are better workarounds
// as outlined in the drake slack support thread.
inline void disable_drake_logging() {
  drake::logging::set_log_level("off");
}

/**
 * @brief Set the verbosity of the logger given an integer from 0 to 5.
 *
 * spdlog levels go from 0 to 5, but in the inverse order of our
 * desired use case; i.e., in spdlog, 0 is "trace", and 5 is "critical". We
 * prefer the opposite ordering, such that the larger values correspond to
 * "higher" verbosity.
 *
 * @param level Integer verbosity level.
 */
inline void set_verbosity(int level) {
  level = 5 - std::clamp(level, 0, 5);
  log()->set_level(static_cast<spdlog::level::level_enum>(level));
}

}  // namespace logging

#if OVERRIDE_DRAKE_LOG
namespace drake {
// Shadow drake::log() with an overload.  NOTE: s_logger must be initialized
// elsewhere.
logging::logger* log();

logging::logger* original_drake_log();
}  // namespace drake
#endif  //  OVERRIDE_DRAKE_LOG
