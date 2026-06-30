/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

// @file logger.cc
#include "planning_service/common/logging.h"

#include <filesystem>
#include <iostream>
#include <random>

namespace fs = std::filesystem;
namespace {
static std::shared_ptr<spdlog::logger> s_logger {};

/// Thread-local transaction ID.  Each thread starts with an empty string;
/// the first call to get_transaction_id() (or set_transaction_id("")) will
/// auto-generate a UUID so that every thread always has a non-empty ID.
thread_local std::string tl_transaction_id {};

/// Custom spdlog flag formatter that injects the current thread's
/// transaction ID into every log message via the %* flag.
class TransactionIdFormatter : public spdlog::custom_flag_formatter {
 public:
  void format(const spdlog::details::log_msg&, const std::tm&,
              spdlog::memory_buf_t& dest) override {
    const std::string& id = logging::get_transaction_id();
    dest.append(id.data(), id.data() + id.size());
  }

  std::unique_ptr<spdlog::custom_flag_formatter> clone() const override {
    return std::make_unique<TransactionIdFormatter>();
  }
};
}  // namespace

namespace logging {

// Returns true IFF the logger object is NULL,
// meaning that either is has not yet been created, or it was nullified.
bool is_log_null() {
  return s_logger == nullptr;
}

spdlog::logger* log(int notify_if_null) {
  if (is_log_null()) {
    if (notify_if_null) {
      std::cout << "[logging]: log() called without having initialized logger"
                << std::endl;
      std::cout << "[logging]: creating new logger with defaults in "
                   "logging::logging::logger log()"
                << std::endl;
    }
    create_log();
  }
  assert(
      s_logger
      && "FATAL: s_logger still not set after calling logging::create_log()");
  return s_logger.get();
}

std::string generate_transaction_id() {
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_int_distribution<uint32_t> dis;

  const uint32_t a = dis(gen);
  const uint32_t b = (dis(gen) & 0xFFFF0FFFu) | 0x00004000u;  // version 4
  const uint32_t c = (dis(gen) & 0x3FFFFFFFu) | 0x80000000u;  // variant 10xx
  const uint32_t e_hi = dis(gen) & 0xFFFFu;
  const uint32_t e_lo = dis(gen);

  return fmt::format("{:08x}-{:04x}-{:04x}-{:04x}-{:04x}{:08x}", a,
                     (b >> 16) & 0xFFFFu, b & 0xFFFFu, (c >> 16) & 0xFFFFu,
                     e_hi, e_lo);
}

void set_transaction_id(const std::string& id) {
  tl_transaction_id = id.empty() ? generate_transaction_id() : id;
}

const std::string& get_transaction_id() {
  if (tl_transaction_id.empty()) {
    tl_transaction_id = generate_transaction_id();
  }
  return tl_transaction_id;
}

std::string get_full_transaction_id() {
  return get_transaction_id();
}

bool create_log(const std::string& program_in, const std::string& prefix_in,
                bool use_stdout, bool create_once) {
  if (s_logger != nullptr && create_once) {
    s_logger->warn("logging:create_log: s_logger already created!");
    return false;
  }
  std::string program = program_in;
  if (program.empty()) {
    program = "planning_service";
  }

  std::string prefix = prefix_in;
  if (prefix.empty()) {
    prefix = program;
  }

  std::vector<spdlog::sink_ptr> sinks;  // the first is always terminal
  if (use_stdout) {
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_color_sink_mt>());
  } else {
    sinks.push_back(std::make_shared<spdlog::sinks::stderr_color_sink_mt>());
  }

  s_logger = std::make_shared<Logger>(prefix, sinks.begin(), sinks.end());

  // Apply a custom pattern that includes the per-thread transaction ID via %*.
  auto formatter = std::make_unique<spdlog::pattern_formatter>();
  formatter->add_flag<TransactionIdFormatter>('*').set_pattern(
      "[%Y-%m-%d %T.%e] [%n] [%^%l%$] [%*] %v");
  s_logger->set_formatter(std::move(formatter));

  // register it if you need to access it globally
  spdlog::register_logger(s_logger);
  s_logger->set_level(spdlog::level::info);
  // Honour VERBOSITY env var if set (e.g. passed via Bazel --test_env).
  if (const char* env_val = std::getenv("VERBOSITY")) {
    try {
      set_verbosity(std::stoi(env_val));
    } catch (const std::exception&) {
      s_logger->warn("logging:create_log: invalid VERBOSITY='{}'", env_val);
    }
  }
  return true;
}

}  // namespace logging

// static member delegation
std::string Logger::get_full_transaction_id() {
  return logging::get_full_transaction_id();
}

#if OVERRIDE_DRAKE_LOG

namespace drake {
logging::logger* log() {
#ifdef EXPECT_TRUE  // To be compiled only in a test context.
  std::cerr << "\t<<<< Call to drake::log() got the override log() >>>>"
            << std::endl
            << std::endl;
#endif

  // Would prefer to assert rather than conpensate for failing
  // to initialize the logger, but then every executable that includes
  // logging.h with OVERRIDE_DRAKE_LOG defined as truthy must either call
  // logging::create_log or work around it.
  // assert(s_logger && "s_logger not set; call logging::create_log() first!");
  if (logging::is_log_null()) {
    std::cerr << "WARNING: Call to uninitialized spdlog::logger log()"
              << std::endl;
    logging::create_log();
    std::cerr << "WARNING: Used all defaults for spdlog::logger log()"
              << std::endl;
  }
  assert(
      s_logger
      && "FATAL: s_logger still not set after calling logging::create_log()");
  return s_logger.get();
}
}  // namespace drake

// TODO(@anyone): If possible and worthwhile, provide access to the original
// result of drake::log() that we overrode.
spdlog::logger* original_drake_log() {
  if (d_logger == nullptr) {
    d_logger = drake::log();
  }
  return d_logger;
}

#endif  //  OVERRIDE_DRAKE_LOG
