#include <gtest/gtest.h>

#include <google/protobuf/util/message_differencer.h>

#include "planning_service_client/planner/constraints.h"

namespace planning_service_client {
namespace planner {

TEST(PositionConstraint, ToProtoFromProto) {
  std::string frame_A = "world";
  std::string frame_B = "robot";
  Eigen::Vector3d p_AQ_lower(0.1, 0.2, 0.3);
  Eigen::Vector3d p_AQ_upper(1.0, 1.1, 1.2);
  Eigen::Vector3d p_BQ(0.5, 0.6, 0.7);
  PositionConstraint obj(frame_A, frame_B, p_AQ_lower, p_AQ_upper, p_BQ);
  auto msg = ToProto(obj);
  // Verify back from proto
  auto obj_from_proto = FromProto<PositionConstraint>(msg);
  EXPECT_EQ(obj_from_proto.frame_A(), frame_A);
  EXPECT_EQ(obj_from_proto.frame_B(), frame_B);
  EXPECT_TRUE(obj_from_proto.p_AQ_lower().isApprox(p_AQ_lower));
  EXPECT_TRUE(obj_from_proto.p_AQ_upper().isApprox(p_AQ_upper));
  EXPECT_TRUE(obj_from_proto.p_BQ().isApprox(p_BQ));
}

TEST(AngleBetweenVectorsConstraint, ToProtoFromProto) {
  std::string frame_A = "world";
  std::string frame_B = "robot";
  Eigen::Vector3d a_A(1.0, 0.0, 0.0);
  Eigen::Vector3d b_B(0.0, 1.0, 0.0);
  double angle_lower = 0.0;
  double angle_upper = 1.57;  // ~90 degrees
  AngleBetweenVectorsConstraint obj(frame_A, frame_B, a_A, b_B, angle_lower,
                                    angle_upper);
  auto msg = ToProto(obj);
  // Verify back from proto
  auto obj_from_proto = FromProto<AngleBetweenVectorsConstraint>(msg);
  EXPECT_EQ(obj_from_proto.frame_A(), frame_A);
  EXPECT_EQ(obj_from_proto.frame_B(), frame_B);
  EXPECT_TRUE(obj_from_proto.a_A().isApprox(a_A));
  EXPECT_TRUE(obj_from_proto.b_B().isApprox(b_B));
  EXPECT_DOUBLE_EQ(obj_from_proto.angle_lower(), angle_lower);
  EXPECT_DOUBLE_EQ(obj_from_proto.angle_upper(), angle_upper);
}

TEST(GeometricConstraints, ToProtoFromProto) {
  GeometricConstraints geo_constraints;
  // Add PositionConstraint
  std::string frame_A1 = "world";
  std::string frame_B1 = "robot";
  Eigen::Vector3d p_AQ_lower(0.1, 0.2, 0.3);
  Eigen::Vector3d p_AQ_upper(1.0, 1.1, 1.2);
  Eigen::Vector3d p_BQ(0.5, 0.6, 0.7);
  PositionConstraint pos_constraint(frame_A1, frame_B1, p_AQ_lower, p_AQ_upper,
                                    p_BQ);
  geo_constraints.Add(pos_constraint);
  // Add AngleBetweenVectorsConstraint
  std::string frame_A2 = "world";
  std::string frame_B2 = "robot";
  Eigen::Vector3d a_A(1.0, 0.0, 0.0);
  Eigen::Vector3d b_B(0.0, 1.0, 0.0);
  double angle_lower = 0.0;
  double angle_upper = 1.57;  // ~90 degrees
  AngleBetweenVectorsConstraint angle_constraint(frame_A2, frame_B2, a_A, b_B,
                                                 angle_lower, angle_upper);
  geo_constraints.Add(angle_constraint);
  // Convert to proto
  auto msg = ToProto(geo_constraints);
  // Verify back from proto
  auto geo_constraints_from_proto = FromProto<GeometricConstraints>(msg);
  ASSERT_EQ(geo_constraints_from_proto.position_constraints().size(), 1);
  EXPECT_EQ(geo_constraints_from_proto.position_constraints()[0].frame_A(),
            frame_A1);
  EXPECT_EQ(geo_constraints_from_proto.position_constraints()[0].frame_B(),
            frame_B1);
  EXPECT_TRUE(geo_constraints_from_proto.position_constraints()[0]
                  .p_AQ_lower()
                  .isApprox(p_AQ_lower));
  EXPECT_TRUE(geo_constraints_from_proto.position_constraints()[0]
                  .p_AQ_upper()
                  .isApprox(p_AQ_upper));
  EXPECT_TRUE(
      geo_constraints_from_proto.position_constraints()[0].p_BQ().isApprox(
          p_BQ));
  ASSERT_EQ(
      geo_constraints_from_proto.angle_between_vectors_constraints().size(), 1);
  EXPECT_EQ(geo_constraints_from_proto.angle_between_vectors_constraints()[0]
                .frame_A(),
            frame_A2);
  EXPECT_EQ(geo_constraints_from_proto.angle_between_vectors_constraints()[0]
                .frame_B(),
            frame_B2);
}

TEST(Comparison, PositionConstraint) {
  PositionConstraint pc1("A", "B", Eigen::Vector3d(0, 0, 0),
                         Eigen::Vector3d(1, 1, 1),
                         Eigen::Vector3d(0.5, 0.5, 0.5));
  PositionConstraint pc2("A", "B", Eigen::Vector3d(-1, -1, -1),
                         Eigen::Vector3d(2, 2, 2),
                         Eigen::Vector3d(0.5, 0.5, 0.5));
  PositionConstraint pc3("C", "D", Eigen::Vector3d(-1, -1, -1),
                         Eigen::Vector3d(2, 2, 2),
                         Eigen::Vector3d(0.1, 0.1, 0.1));
  PositionConstraint pc4("A", "B", Eigen::Vector3d(0.2, 0.2, 0.2),
                         Eigen::Vector3d(0.8, 0.8, 0.8),
                         Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_TRUE(pc1 <= pc2);
  EXPECT_FALSE(pc2 <= pc1);
  EXPECT_FALSE(pc1 <= pc3);  // Different frames
  EXPECT_FALSE(pc1 <= pc4);  // pc4 has different p_BQ
}

TEST(Comparison, AngleBetweenVectorsConstraint) {
  AngleBetweenVectorsConstraint ac1("A", "B", Eigen::Vector3d(1, 0, 0),
                                    Eigen::Vector3d(0, 1, 0), 0.0, 1.0);
  AngleBetweenVectorsConstraint ac2("A", "B", Eigen::Vector3d(1, 0, 0),
                                    Eigen::Vector3d(0, 1, 0), -0.5, 1.5);
  AngleBetweenVectorsConstraint ac3("C", "D", Eigen::Vector3d(1, 0, 0),
                                    Eigen::Vector3d(0, 1, 0), -0.5, 1.5);
  AngleBetweenVectorsConstraint ac4("A", "B", Eigen::Vector3d(1, 0, 0),
                                    Eigen::Vector3d(1, 1, 0), 0.2, 0.8);
  EXPECT_TRUE(ac1 <= ac2);
  EXPECT_FALSE(ac2 <= ac1);
  EXPECT_FALSE(ac1 <= ac3);  // Different frames
  EXPECT_FALSE(ac1 <= ac4);  // Different vectors
}

TEST(Comparison, GeometricConstraints) {
  // Let's create two GeometricConstraints objects
  GeometricConstraints gc1, gc2;
  // Add PositionConstraints
  gc1.Add(PositionConstraint("A", "B", Eigen::Vector3d(0, 0, 0),
                             Eigen::Vector3d(1, 1, 1),
                             Eigen::Vector3d(0.5, 0.5, 0.5)));
  gc2.Add(PositionConstraint("A", "B", Eigen::Vector3d(-1, -1, -1),
                             Eigen::Vector3d(2, 2, 2),
                             Eigen::Vector3d(0.5, 0.5, 0.5)));
  // Add AngleBetweenVectorsConstraints
  gc1.Add(AngleBetweenVectorsConstraint("A", "B", Eigen::Vector3d(1, 0, 0),
                                        Eigen::Vector3d(0, 1, 0), 0.0, 1.0));
  gc2.Add(AngleBetweenVectorsConstraint("A", "B", Eigen::Vector3d(1, 0, 0),
                                        Eigen::Vector3d(0, 1, 0), 0.0, 2.0));
  EXPECT_TRUE(gc1 <= gc2);
  // Add some more constraints to gc2, but actually they are even looser
  gc2.Add(PositionConstraint("A", "B", Eigen::Vector3d(-2, -2, -2),
                             Eigen::Vector3d(2, 2, 2),
                             Eigen::Vector3d(0.5, 0.5, 0.5)));
  // Still gc1 is tighter than gc2, even with the extra loose constraint in gc2
  EXPECT_TRUE(gc1 <= gc2);
  // Now add a different constraint to gc2 that makes it impossible for gc1 to
  // be tighter
  gc2.Add(AngleBetweenVectorsConstraint("C", "D", Eigen::Vector3d(1, 0, 0),
                                        Eigen::Vector3d(0, 1, 0), 0.0, 2.0));
  EXPECT_FALSE(gc1 <= gc2);
}

}  // namespace planner
}  // namespace planning_service_client
