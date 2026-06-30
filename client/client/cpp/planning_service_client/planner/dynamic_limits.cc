
#include "dynamic_limits.h"
namespace planning_service_client {
namespace planner {

proto::DynamicLimits DynamicLimits::ToProtoImpl() const {
  proto::DynamicLimits msg;
  msg.set_safety_factor_velocity(safety_factor_velocity);
  msg.set_safety_factor_acceleration(safety_factor_acceleration);
  msg.set_safety_factor_torque(safety_factor_torque);
  for (const auto& [key, val] : cartesian_velocity_limits) {
    (*msg.mutable_cartesian_velocity_limits())[key] = val;
  }
  return msg;
}

void DynamicLimits::FromProtoImpl(const proto::DynamicLimits& msg) {
  safety_factor_velocity = msg.safety_factor_velocity();
  safety_factor_acceleration = msg.safety_factor_acceleration();
  safety_factor_torque = msg.safety_factor_torque();
  for (const auto& [key, val] : msg.cartesian_velocity_limits()) {
    cartesian_velocity_limits[key] = val;
  }
  valid_or_throw();
}
}  // namespace planner
}  // namespace planning_service_client
