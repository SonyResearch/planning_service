/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file fmt.h
#pragma once

#include <fmt/color.h>
#include <fmt/format.h>

#include <filesystem>
#include <map>
#include <vector>

constexpr auto FMT_RED = fmt::terminal_color::bright_red;          // Red
constexpr auto FMT_GREEN = fmt::terminal_color::bright_green;      // Green
constexpr auto FMT_YELLOW = fmt::terminal_color::bright_yellow;    // Yellow
constexpr auto FMT_BLUE = fmt::terminal_color::bright_blue;        // Blue
constexpr auto FMT_MAGENTA = fmt::terminal_color::bright_magenta;  // Magenta
constexpr auto FMT_CYAN = fmt::terminal_color::bright_cyan;        // Cyan

constexpr auto FMT_BOLD = fmt::emphasis::bold;            // Bold
constexpr auto FMT_FAINT = fmt::emphasis::faint;          // Faint
constexpr auto FMT_ITALIC = fmt::emphasis::italic;        // Italic
constexpr auto FMT_UNDERLINE = fmt::emphasis::underline;  // Underline

/**
 * @brief
 *
 */

namespace fmt {
/** Formatter for fs::path */
template <>
struct formatter<std::filesystem::path> : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const std::filesystem::path& path, FormatContext& ctx) const {
    return formatter<std::string_view>::format(path.string(), ctx);
  }
};

/** Formatter for maps. */
template <typename T, typename U>
struct formatter<std::map<T, U>> : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const std::map<T, U>& m, FormatContext& ctx) const {
    if (m.empty()) {
      return formatter<std::string_view>::format("{{ }}", ctx);
    }
    std::vector<std::string> v;
    v.reserve(m.size());
    for (const auto& [k, val] : m) {
      v.push_back(fmt::format("{}: {}", k, val));
    }
    return formatter<std::string_view>::format(
        fmt::format("{{ {} }}", fmt::join(v, ", ")), ctx);
  }
};

/** Formatter for std::pair. */
template <typename T, typename U>
struct formatter<std::pair<T, U>> : formatter<std::string_view> {
  template <typename FormatContext>
  auto format(const std::pair<T, U>& p, FormatContext& ctx) const {
    return formatter<std::string_view>::format(
        fmt::format("({}, {})", p.first, p.second), ctx);
  }
};

}  // namespace fmt
