/// @file string_utils.cc

#include "string_utils.h"

#include <algorithm>
namespace common {
namespace utils {

std::string to_lower(const std::string_view str) {
  std::string s_cpy {str};
  std::transform(s_cpy.begin(), s_cpy.end(), s_cpy.begin(),
                 [](unsigned char c) {
                   return std::tolower(c);
                 });
  return s_cpy;
}

std::string to_upper(const std::string_view str) {
  std::string s_cpy {str};
  std::transform(s_cpy.begin(), s_cpy.end(), s_cpy.begin(),
                 [](unsigned char c) {
                   return std::toupper(c);
                 });
  return s_cpy;
}

std::string join_strings(const std::set<std::string>& strs,
                         const std::string_view sep) {
  // Do not need to respect order
  std::vector<std::string> str_vec(strs.begin(), strs.end());
  return join_strings(str_vec, sep);
}

std::string join_strings(const std::vector<std::string>& strs,
                         const std::string_view sep) {
  // Need to respect order
  std::ostringstream oss;
  bool first {true};
  for (const auto& str : strs) {
    if (!first) {
      oss << sep.data();
    }
    first = false;
    oss << str;
  }
  return oss.str();
}

std::vector<std::string> split_string(const std::string& str,
                                      const char delimiter) {
  std::vector<std::string> result;
  std::istringstream ss(str);
  std::string token;
  while (std::getline(ss, token, delimiter)) {
    result.push_back(token);
  }
  return result;
}

bool string_includes(const std::string& str, const std::string& substr) {
  return str.find(substr) != std::string::npos;
}

}  // namespace utils
}  // namespace common
