/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file error.h

#pragma once

#include <fmt/format.h>

#include <magic_enum/magic_enum.hpp>

namespace service {

enum ServiceErrorCode {
  NONE = 0,
  UNKNOWN = 1,
  QUEUE_EMPTY = 2,
  QUEUE_CAPACITY_REACHED = 3,
  DUPLICATE_ID = 4,
  JOB_CAPACITY_REACHED = 5,
  JOB_START_FAILED = 6,
  CONTEXT_NOT_FOUND = 7,
  CONTEXT_INVALID = 8,
  RESULT_NOT_READY = 9,
  RESULT_NOT_FOUND = 10,
  IRIS_GENERATION_FAILED = 11,
  UNKNOWN_PLANNING_FAILURE = 12,
  START_IN_VIOLATION = 13,
  GOAL_IN_VIOLATION = 14,
  JOB_THREAD_INVALIDATED = 15,
  JOB_THREAD_EXCEPTION = 16,
  JOB_TYPE_UNSUPPORTED = 17,
  DEPRECATED_API = 18,
};

struct ServiceError {
  ServiceErrorCode code;
  std::string msg;
  ServiceError(const ServiceErrorCode code, const std::string_view msg)
      : code {code}, msg {msg} {}
};

}  // namespace service

/// \cond DO_NOT_DOCUMENT
template <>
struct fmt::formatter<service::ServiceError> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(service::ServiceError const& err, FormatContext& ctx) const {
    return fmt::format_to(ctx.out(), "ServiceError: Code: {}, Message: {}",
                          magic_enum::enum_name(err.code), err.msg);
  }
};
/// \endcond
