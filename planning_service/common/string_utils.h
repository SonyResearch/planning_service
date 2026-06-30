/// @file string_utils.h
#pragma once

#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace common {
namespace utils {

/** Return a lowercase copy of the given string. */
std::string to_lower(const std::string_view str);

/** Return an uppercase copy of the given string. */
std::string to_upper(const std::string_view str);

/** Join a set of strings with the given separator. */
std::string join_strings(const std::set<std::string>& strs,
                         const std::string_view sep = ", ");

/** Join a vector of strings with the given separator. */
std::string join_strings(const std::vector<std::string>& strs,
                         const std::string_view sep = ", ");

/** Split a string by the given delimiter character. */
std::vector<std::string> split_string(const std::string& str,
                                      const char delimiter = ',');

bool string_includes(const std::string& str, const std::string& substr);

}  // namespace utils
}  // namespace common
