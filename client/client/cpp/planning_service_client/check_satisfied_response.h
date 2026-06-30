#pragma once

#include <string>
#include <vector>

#include "planning_service_client/internal/proto_base.h"

namespace planning_service_client {

class CheckSatisfiedResponse
    : public internal::ProtoBase<proto::CheckSatisfiedResponse> {
 public:
  CheckSatisfiedResponse() = default;

  CheckSatisfiedResponse(
      bool satisfied, const std::vector<std::string>& failed_constraint_strings,
      const std::vector<std::string>& offending_resource_names);

  bool satisfied() const {
    return satisfied_;
  }

  const std::vector<std::string>& failed_constraint_strings() const {
    return failed_constraint_strings_;
  }

  const std::vector<std::string>& offending_resource_names() const {
    return offending_resource_names_;
  }

 private:
  proto::CheckSatisfiedResponse ToProtoImpl() const override;

  void FromProtoImpl(const proto::CheckSatisfiedResponse& msg) override;

  bool satisfied_ {false};
  std::vector<std::string> failed_constraint_strings_;
  std::vector<std::string> offending_resource_names_;
};

}  // namespace planning_service_client
