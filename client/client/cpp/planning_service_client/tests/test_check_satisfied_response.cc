#include <gtest/gtest.h>

#include "planning_service_client/check_satisfied_response.h"

namespace planning_service_client {

TEST(CheckSatisfiedResponse, DefaultConstructor) {
  CheckSatisfiedResponse response;
  EXPECT_FALSE(response.satisfied());
  EXPECT_TRUE(response.failed_constraint_strings().empty());
  EXPECT_TRUE(response.offending_resource_names().empty());
}

TEST(CheckSatisfiedResponse, ConstructorSatisfied) {
  CheckSatisfiedResponse response(true, {}, {});
  EXPECT_TRUE(response.satisfied());
  EXPECT_TRUE(response.failed_constraint_strings().empty());
  EXPECT_TRUE(response.offending_resource_names().empty());
}

TEST(CheckSatisfiedResponse, ConstructorNotSatisfied) {
  std::vector<std::string> constraints = {"constraint_A", "constraint_B"};
  std::vector<std::string> model_names = {"model_1", "model_2"};
  CheckSatisfiedResponse response(false, constraints, model_names);
  EXPECT_FALSE(response.satisfied());
  EXPECT_EQ(response.failed_constraint_strings(), constraints);
  EXPECT_EQ(response.offending_resource_names(), model_names);
}

TEST(CheckSatisfiedResponse, ToProtoFromProtoSatisfied) {
  CheckSatisfiedResponse response(true, {}, {});
  proto::CheckSatisfiedResponse pb = ToProto(response);
  EXPECT_TRUE(pb.satisfied());
  EXPECT_EQ(pb.unsatisfied_constraints_size(), 0);
  EXPECT_EQ(pb.offending_resource_names_size(), 0);

  CheckSatisfiedResponse from_pb = FromProto<CheckSatisfiedResponse>(pb);
  EXPECT_TRUE(from_pb.satisfied());
  EXPECT_TRUE(from_pb.failed_constraint_strings().empty());
  EXPECT_TRUE(from_pb.offending_resource_names().empty());
}

TEST(CheckSatisfiedResponse, ToProtoFromProtoNotSatisfied) {
  std::vector<std::string> constraints = {"constraint_A", "constraint_B"};
  std::vector<std::string> model_names = {"model_1", "model_2"};
  CheckSatisfiedResponse response(false, constraints, model_names);

  proto::CheckSatisfiedResponse pb = ToProto(response);
  EXPECT_FALSE(pb.satisfied());
  ASSERT_EQ(pb.unsatisfied_constraints_size(), 2);
  EXPECT_EQ(pb.unsatisfied_constraints(0), "constraint_A");
  EXPECT_EQ(pb.unsatisfied_constraints(1), "constraint_B");
  ASSERT_EQ(pb.offending_resource_names_size(), 2);
  EXPECT_EQ(pb.offending_resource_names(0), "model_1");
  EXPECT_EQ(pb.offending_resource_names(1), "model_2");

  CheckSatisfiedResponse from_pb = FromProto<CheckSatisfiedResponse>(pb);
  EXPECT_FALSE(from_pb.satisfied());
  EXPECT_EQ(from_pb.failed_constraint_strings(), constraints);
  EXPECT_EQ(from_pb.offending_resource_names(), model_names);
}

TEST(CheckSatisfiedResponse, Equality) {
  std::vector<std::string> constraints = {"constraint_A"};
  std::vector<std::string> model_names = {"model_1"};
  CheckSatisfiedResponse r1(false, constraints, model_names);
  CheckSatisfiedResponse r2(false, constraints, model_names);
  EXPECT_EQ(r1, r2);

  CheckSatisfiedResponse r3(true, {}, {});
  EXPECT_NE(r1, r3);
}

}  // namespace planning_service_client
