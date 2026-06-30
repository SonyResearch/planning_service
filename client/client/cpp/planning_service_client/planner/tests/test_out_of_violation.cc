#include <gtest/gtest.h>

#include "planning_service_client/planner/out_of_violation.h"
#include "test_utils.h"

namespace planning_service_client {
namespace planner {

TEST(OutOfViolation, ToProtoFromProto) {
  double configuration_clearance_norm = 0.05;
  double influence_distance = 0.05;
  OutOfViolation out_of_violation(configuration_clearance_norm,
                                  influence_distance);
  auto proto = ToProto(out_of_violation);
  auto out_of_violation_from_proto = FromProto<OutOfViolation>(proto);
  EXPECT_DOUBLE_EQ(out_of_violation_from_proto.configuration_clearance_norm(),
                   configuration_clearance_norm);
  EXPECT_DOUBLE_EQ(out_of_violation_from_proto.influence_distance(),
                   influence_distance);
  // Now get the proto back from the serialized string
  proto::OutOfViolationProblem proto_from_serialized;
  auto serialized = proto.SerializeAsString();
  EXPECT_TRUE(proto_from_serialized.ParseFromString(serialized));
  EXPECT_DOUBLE_EQ(proto_from_serialized.configuration_clearance_norm(),
                   configuration_clearance_norm);
  EXPECT_DOUBLE_EQ(proto_from_serialized.influence_distance(),
                   influence_distance);
}

TEST(OutOfViolation, UseGradientDefaultTrue) {
  OutOfViolation out_of_violation;
  EXPECT_TRUE(out_of_violation.use_gradient());
  EXPECT_FALSE(out_of_violation.twist().has_value());
}

TEST(OutOfViolation, UseGradientForcesNulloptTwist) {
  Twist twist("world", "eef", "world", Eigen::Vector3d::Zero(),
              Eigen::Vector3d::Zero());
  // When use_gradient=true, twist must be nullopt even if provided.
  OutOfViolation out_of_violation(0.05, 0.05, true, twist);
  EXPECT_TRUE(out_of_violation.use_gradient());
  EXPECT_FALSE(out_of_violation.twist().has_value());
}

TEST(OutOfViolation, TwistSetWhenUseGradientFalse) {
  Twist twist("world", "eef", "world", Eigen::Vector3d(1.0, 2.0, 3.0),
              Eigen::Vector3d(0.1, 0.2, 0.3));
  OutOfViolation out_of_violation(0.05, 0.05, false, twist);
  EXPECT_FALSE(out_of_violation.use_gradient());
  ASSERT_TRUE(out_of_violation.twist().has_value());
  EXPECT_EQ(out_of_violation.twist()->frame_A(), "world");
  EXPECT_EQ(out_of_violation.twist()->frame_B(), "eef");
  EXPECT_EQ(out_of_violation.twist()->frame_E(), "world");
  EXPECT_TRUE(out_of_violation.twist()->delta_xyz().isApprox(
      Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(out_of_violation.twist()->delta_rpy().isApprox(
      Eigen::Vector3d(0.1, 0.2, 0.3)));
}

TEST(OutOfViolation, UseGradientFalseNoTwist) {
  OutOfViolation out_of_violation(0.05, 0.05, false);
  EXPECT_FALSE(out_of_violation.use_gradient());
  EXPECT_FALSE(out_of_violation.twist().has_value());
}

TEST(OutOfViolation, ToProtoFromProtoWithTwist) {
  Twist twist("world", "eef", "world", Eigen::Vector3d(1.0, 2.0, 3.0),
              Eigen::Vector3d(0.1, 0.2, 0.3));
  OutOfViolation out_of_violation(0.05, 0.05, false, twist);
  auto proto = ToProto(out_of_violation);
  EXPECT_TRUE(proto.has_twist());
  EXPECT_FALSE(proto.use_gradient());

  auto from_proto = FromProto<OutOfViolation>(proto);
  EXPECT_FALSE(from_proto.use_gradient());
  ASSERT_TRUE(from_proto.twist().has_value());
  EXPECT_EQ(from_proto.twist()->frame_A(), "world");
  EXPECT_EQ(from_proto.twist()->frame_B(), "eef");
  EXPECT_EQ(from_proto.twist()->frame_E(), "world");
  EXPECT_TRUE(
      from_proto.twist()->delta_xyz().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
  EXPECT_TRUE(
      from_proto.twist()->delta_rpy().isApprox(Eigen::Vector3d(0.1, 0.2, 0.3)));
}

TEST(OutOfViolation, ToProtoFromProtoUseGradientTrue) {
  OutOfViolation out_of_violation(0.05, 0.05, true);
  auto proto = ToProto(out_of_violation);
  EXPECT_TRUE(proto.use_gradient());
  EXPECT_FALSE(proto.has_twist());

  auto from_proto = FromProto<OutOfViolation>(proto);
  EXPECT_TRUE(from_proto.use_gradient());
  EXPECT_FALSE(from_proto.twist().has_value());
}

TEST(OutOfViolation, FromProtoDefaultsUseGradientToTrue) {
  // When deserializing a proto without use_gradient set, it should default to
  // true.
  proto::OutOfViolationProblem msg;
  EXPECT_FALSE(msg.has_use_gradient());
  auto from_proto = FromProto<OutOfViolation>(msg);
  EXPECT_TRUE(from_proto.use_gradient());
  EXPECT_FALSE(from_proto.twist().has_value());
}

}  // namespace planner
}  // namespace planning_service_client
