#pragma once

#include "planning_service_client/conf.h"
#include "proto/basic_types.pb.h"
namespace planning_service_client {
namespace planner {
/**
 * @brief The set of dynamic limits which must be satisfied by the solution of a
 * given motion planning problem. These limits can be used to set a maximum
 * safety factor for velocity, acceleration, and torque, as well as a maximum
 * velocity for Cartesian motion
 *
 */
class DynamicLimits : public internal::ProtoBase<proto::DynamicLimits> {
 public:
  DynamicLimits() = default;
  DynamicLimits(double safety_factor_velocity,
                double safety_factor_acceleration, double safety_factor_torque,
                const std::map<std::string, double>& cartesian_velocity_limits)
      : safety_factor_velocity(safety_factor_velocity),
        safety_factor_acceleration(safety_factor_acceleration),
        safety_factor_torque(safety_factor_torque),
        cartesian_velocity_limits(cartesian_velocity_limits) {
    valid_or_throw();
  }
  /** Public members. */
  double safety_factor_velocity {0.0};
  double safety_factor_acceleration {0.0};
  double safety_factor_torque {0.0};
  std::map<std::string, double> cartesian_velocity_limits;

 private:
  /** Return true if limits are valid. All safety factors must lie within the
   * range [0, 1], and cartesian_velocity_limits entries must be positive.
   * Safety factors set to 0 will be treated as unset, equivalent to the
   * Protobuf standard.
   */
  void valid_or_throw() const {
    const std::vector<double> safety_factors {safety_factor_velocity,
                                              safety_factor_acceleration,
                                              safety_factor_torque};
    auto safety_factors_valid =
        std::all_of(safety_factors.cbegin(), safety_factors.cend(), [](auto f) {
          return f >= 0.0 && f <= 1.0;
        });

    auto max_frame_cartesian_velocities_valid =
        std::all_of(cartesian_velocity_limits.cbegin(),
                    cartesian_velocity_limits.cend(), [](const auto& pair) {
                      return pair.second > 0.0;
                    });
    if (!safety_factors_valid || !max_frame_cartesian_velocities_valid) {
      throw std::runtime_error(
          "Invalid dynamic limits: safety factors must be in (0, 1] and "
          "cartesian_velocity_limits entries must be positive.");
    }
  }
  proto::DynamicLimits ToProtoImpl() const;

  void FromProtoImpl(const proto::DynamicLimits& msg);
};
}  // namespace planner
}  // namespace planning_service_client
