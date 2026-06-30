#include <gtest/gtest.h>

#include "planning_service_client/shape.h"
namespace planning_service_client {
TEST(ShapeTest, BoxCtor) {
  EXPECT_NO_THROW(Box());
  EXPECT_NO_THROW(Box(1.0, 2.0, 3.0));
  EXPECT_NO_THROW(Box(Eigen::Vector3d(1.0, 2.0, 3.0)));
  const Box box(1.0, 2.0, 3.0);
  EXPECT_EQ(box.type(), "Box");
}

TEST(ShapeTest, BoxDims) {
  const Box box(1.0, 2.0, 3.0);
  EXPECT_EQ(box.width(), 1.0);
  EXPECT_EQ(box.depth(), 2.0);
  EXPECT_EQ(box.height(), 3.0);
  Eigen::VectorXd dims = box.dims();
  EXPECT_EQ(dims.size(), 3);
  EXPECT_TRUE(dims.isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(ShapeTest, BoxClone) {
  const Box box(1.0, 2.0, 3.0);
  std::unique_ptr<Shape> box_clone = box.Clone();
  EXPECT_EQ(box_clone->type(), "Box");
  EXPECT_TRUE(box_clone->dims().isApprox(Eigen::Vector3d(1.0, 2.0, 3.0)));
}

TEST(ShapeTest, BoxRoundTrip) {
  const Box box(1.0, 2.0, 3.0);
  proto::Shape proto = ToProto(box);
  EXPECT_EQ(proto.shape_case(), proto::Shape::ShapeCase::kBox);
  EXPECT_EQ(proto.box().width(), box.width());
  EXPECT_EQ(proto.box().depth(), box.depth());
  EXPECT_EQ(proto.box().height(), box.height());
  Box box_from_proto;
  box_from_proto = FromProto<Box>(proto);
  EXPECT_EQ(box_from_proto.type(), "Box");
  EXPECT_TRUE(box_from_proto.dims().isApprox(box.dims()));
}

TEST(ShapeTest, BoxCube) {
  const double side_length = 2.0;
  Box cube = Box::Cube(side_length);
  EXPECT_EQ(cube.type(), "Box");
  EXPECT_TRUE(cube.dims().isApprox(
      Eigen::Vector3d(side_length, side_length, side_length)));
}

TEST(ShapeTest, SphereCtor) {
  EXPECT_NO_THROW(Sphere());
  EXPECT_NO_THROW(Sphere(0.5));
  const Sphere sphere(0.5);
  EXPECT_EQ(sphere.type(), "Sphere");
}

TEST(ShapeTest, SphereDims) {
  const Sphere sphere(0.5);
  EXPECT_EQ(sphere.radius(), 0.5);
  Eigen::VectorXd dims = sphere.dims();
  EXPECT_EQ(dims.size(), 1);
  EXPECT_TRUE(dims.isApprox(Eigen::VectorXd::Ones(1) * 0.5));
}

TEST(ShapeTest, SphereClone) {
  const Sphere sphere(0.5);
  std::unique_ptr<Shape> sphere_clone = sphere.Clone();
  EXPECT_EQ(sphere_clone->type(), "Sphere");
  EXPECT_TRUE(sphere_clone->dims().isApprox(Eigen::VectorXd::Ones(1) * 0.5));
}

TEST(ShapeTest, SphereRoundTrip) {
  const Sphere sphere(0.5);
  proto::Shape proto = ToProto(sphere);
  EXPECT_EQ(proto.shape_case(), proto::Shape::ShapeCase::kSphere);
  EXPECT_EQ(proto.sphere().radius(), sphere.radius());
  Sphere sphere_from_proto;
  sphere_from_proto = FromProto<Sphere>(proto);
  EXPECT_EQ(sphere_from_proto.type(), "Sphere");
  EXPECT_TRUE(sphere_from_proto.dims().isApprox(sphere.dims()));
}

TEST(ShapeTest, CylinderCtor) {
  EXPECT_NO_THROW(Cylinder());
  EXPECT_NO_THROW(Cylinder(0.5, 2.0));
  const Cylinder cylinder(0.5, 2.0);
  EXPECT_EQ(cylinder.type(), "Cylinder");
}

TEST(ShapeTest, CylinderDims) {
  const Cylinder cylinder(0.5, 2.0);
  EXPECT_EQ(cylinder.radius(), 0.5);
  EXPECT_EQ(cylinder.height(), 2.0);
  Eigen::VectorXd dims = cylinder.dims();
  EXPECT_EQ(dims.size(), 2);
  EXPECT_TRUE(dims.isApprox(Eigen::Vector2d(0.5, 2.0)));
}

TEST(ShapeTest, CylinderClone) {
  const Cylinder cylinder(0.5, 2.0);
  std::unique_ptr<Shape> cylinder_clone = cylinder.Clone();
  EXPECT_EQ(cylinder_clone->type(), "Cylinder");
  EXPECT_TRUE(cylinder_clone->dims().isApprox(Eigen::Vector2d(0.5, 2.0)));
}

TEST(ShapeTest, CylinderRoundTrip) {
  const Cylinder cylinder(0.5, 2.0);
  proto::Shape proto = ToProto(cylinder);
  EXPECT_EQ(proto.shape_case(), proto::Shape::ShapeCase::kCylinder);
  EXPECT_EQ(proto.cylinder().radius(), cylinder.radius());
  EXPECT_EQ(proto.cylinder().height(), cylinder.height());
  Cylinder cylinder_from_proto;
  cylinder_from_proto = FromProto<Cylinder>(proto);
  EXPECT_EQ(cylinder_from_proto.type(), "Cylinder");
  EXPECT_TRUE(cylinder_from_proto.dims().isApprox(cylinder.dims()));
}

TEST(ShapeTest, CapsuleCtor) {
  EXPECT_NO_THROW(Capsule());
  EXPECT_NO_THROW(Capsule(0.5, 2.0));
  const Capsule capsule(0.5, 2.0);
  EXPECT_EQ(capsule.type(), "Capsule");
}

TEST(ShapeTest, CapsuleDims) {
  const Capsule capsule(0.5, 2.0);
  EXPECT_EQ(capsule.radius(), 0.5);
  EXPECT_EQ(capsule.height(), 2.0);
  Eigen::VectorXd dims = capsule.dims();
  EXPECT_EQ(dims.size(), 2);
  EXPECT_TRUE(dims.isApprox(Eigen::Vector2d(0.5, 2.0)));
}

TEST(ShapeTest, CapsuleClone) {
  const Capsule capsule(0.5, 2.0);
  std::unique_ptr<Shape> capsule_clone = capsule.Clone();
  EXPECT_EQ(capsule_clone->type(), "Capsule");
  EXPECT_TRUE(capsule_clone->dims().isApprox(Eigen::Vector2d(0.5, 2.0)));
}

TEST(ShapeTest, CapsuleRoundTrip) {
  const Capsule capsule(0.5, 2.0);
  proto::Shape proto = ToProto(capsule);
  EXPECT_EQ(proto.shape_case(), proto::Shape::ShapeCase::kCapsule);
  EXPECT_EQ(proto.capsule().radius(), capsule.radius());
  EXPECT_EQ(proto.capsule().height(), capsule.height());
  Capsule capsule_from_proto;
  capsule_from_proto = FromProto<Capsule>(proto);
  EXPECT_EQ(capsule_from_proto.type(), "Capsule");
  EXPECT_TRUE(capsule_from_proto.dims().isApprox(capsule.dims()));
}

}  // namespace planning_service_client
