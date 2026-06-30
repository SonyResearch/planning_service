#pragma once

#include <stdexcept>

namespace planning_service_client {
namespace internal {

std::string GenerateErrorMessage(const std::string& msg, const char* file,
                                 int line, const char* function);

}  // namespace internal
}  // namespace planning_service_client

#define CLIENT_THROW_UNLESS(condition)                           \
  if (!(condition)) {                                            \
    throw std::runtime_error(                                    \
        planning_service_client::internal::GenerateErrorMessage( \
            #condition, __FILE__, __LINE__, __func__));          \
  }

#define CLIENT_THROW_INVALID_ARGUMENT_UNLESS(condition)          \
  if (!(condition)) {                                            \
    throw std::invalid_argument(                                 \
        planning_service_client::internal::GenerateErrorMessage( \
            #condition, __FILE__, __LINE__, __func__));          \
  }
