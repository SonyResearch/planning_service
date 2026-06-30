#include "planning_service_client/internal/client_throw.h"

#include <string>
namespace planning_service_client {
namespace internal {

std::string GenerateErrorMessage(const std::string& msg, const char* file,
                                 int line, const char* function) {
  return std::string(file) + ":" + std::to_string(line) + " in " + function
         + ": " + msg;
}

}  // namespace internal
}  // namespace planning_service_client
