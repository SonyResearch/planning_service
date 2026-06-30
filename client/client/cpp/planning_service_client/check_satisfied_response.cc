#include "planning_service_client/check_satisfied_response.h"

namespace planning_service_client {

CheckSatisfiedResponse::CheckSatisfiedResponse(
    bool satisfied, const std::vector<std::string>& failed_constraint_strings,
    const std::vector<std::string>& offending_resource_names)
    : satisfied_(satisfied),
      failed_constraint_strings_(failed_constraint_strings),
      offending_resource_names_(offending_resource_names) {}

proto::CheckSatisfiedResponse CheckSatisfiedResponse::ToProtoImpl() const {
  proto::CheckSatisfiedResponse msg;
  msg.set_satisfied(satisfied_);
  for (const auto& constraint : failed_constraint_strings_) {
    msg.add_unsatisfied_constraints(constraint);
  }
  for (const auto& name : offending_resource_names_) {
    msg.add_offending_resource_names(name);
  }
  return msg;
}

void CheckSatisfiedResponse::FromProtoImpl(
    const proto::CheckSatisfiedResponse& msg) {
  satisfied_ = msg.satisfied();
  failed_constraint_strings_.assign(msg.unsatisfied_constraints().begin(),
                                    msg.unsatisfied_constraints().end());
  offending_resource_names_.assign(msg.offending_resource_names().begin(),
                                   msg.offending_resource_names().end());
}

}  // namespace planning_service_client
