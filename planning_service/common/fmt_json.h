/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file fmt.h
#pragma once
#include <fmt/format.h>

#include <nlohmann/json.hpp>

/** Json formatter. */
template <>
struct fmt::formatter<nlohmann::json> : fmt::formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const nlohmann::json& j, FormatContext& ctx) const {
    return fmt::formatter<std::string_view>::format(j.dump(), ctx);
  }
};
