#include <gtest/gtest.h>

#include <google/protobuf/util/message_differencer.h>

#include "planning_service_client/planner/dynamic_limits.h"
#include "test_utils.h"
namespace planning_service_client {
namespace planner {

namespace pb = google::protobuf;

TEST(DynamicLimitsTest, ToProto) {
  std::map<std::string, double> cartesian_velocity_limits {{"a", 1.0},
                                                           {"b", 2.0}};
  DynamicLimits obj {1.0, 0.9, 0.8, cartesian_velocity_limits};

  const auto msg {ToProto(obj)};

  EXPECT_EQ(msg.safety_factor_velocity(), 1.0);
  EXPECT_EQ(msg.safety_factor_acceleration(), 0.9);
  EXPECT_EQ(msg.safety_factor_torque(), 0.8);
  EXPECT_EQ(msg.cartesian_velocity_limits().size(), 2);
  EXPECT_EQ(msg.cartesian_velocity_limits().at("a"), 1.0);
  EXPECT_EQ(msg.cartesian_velocity_limits().at("b"), 2.0);
}

TEST(DynamicLimitsTest, FromProto) {
  proto::DynamicLimits msg;
  msg.set_safety_factor_velocity(1.0);
  msg.set_safety_factor_acceleration(0.9);
  msg.set_safety_factor_torque(0.8);
  (*msg.mutable_cartesian_velocity_limits())["a"] = 1.0;
  (*msg.mutable_cartesian_velocity_limits())["b"] = 2.0;

  const auto obj {FromProto<DynamicLimits>(msg)};

  EXPECT_EQ(obj.safety_factor_velocity, 1.0);
  EXPECT_EQ(obj.safety_factor_acceleration, 0.9);
  EXPECT_EQ(obj.safety_factor_torque, 0.8);
  EXPECT_EQ(obj.cartesian_velocity_limits.size(), 2);
  EXPECT_EQ(obj.cartesian_velocity_limits.at("a"), 1.0);
  EXPECT_EQ(obj.cartesian_velocity_limits.at("b"), 2.0);
}

TEST(DynamicLimitsTest, Valid) {
  EXPECT_THROW(DynamicLimits(-0.1, 0.9, 0.8, {{"a", 1.0}}), std::runtime_error)
      << "Velocity safety factor of -0.1 should throw";
  EXPECT_THROW(DynamicLimits(1.1, 0.9, 0.8, {{"a", 1.0}}), std::runtime_error)
      << "Velocity safety factor of 1.1 < 1.0 should throw";
  EXPECT_THROW(DynamicLimits(1.0, 1.0, 1.0, {{"a", -1.0}}), std::runtime_error)
      << "Negative cartesian_velocity_limits should throw";
}
}  // namespace planner
}  // namespace planning_service_client
