/// @file time_utils.cc

#include "time_utils.h"

#include <iomanip>
#include <sstream>

namespace common {
namespace utils {

const std::string DATE_FMT {"%Y%m%d"};
const std::string DATETIME_FMT {"%Y%m%dT%H%M%S"};

const std::string datetime_str(const std::string_view fmt) {
  time_t time = std::time(nullptr);
  std::tm* stm = std::localtime(&time);
  std::ostringstream iss;
  iss << std::put_time(stm, fmt.data());
  return iss.str();
}

}  // namespace utils
}  // namespace common
