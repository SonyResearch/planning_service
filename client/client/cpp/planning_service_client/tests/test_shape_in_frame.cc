#include <gtest/gtest.h>

#include "planning_service_client/shape.h"

namespace planning_service_client {

TEST(ShapeInFrameTest, FrameAndPose) {
  ShapeInFrame sif;
  sif.set_frame("world");
  sif.set_translation(Eigen::Vector3d(1.0, 2.0, 3.0));
  sif.set_quaternion(Eigen::Quaterniond(0.707, 0.0, 0.0, 0.707));

  EXPECT_EQ(sif.frame(), "world");
  EXPECT_DOUBLE_EQ(sif.translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(sif.translation().y(), 2.0);
  EXPECT_DOUBLE_EQ(sif.translation().z(), 3.0);
  EXPECT_DOUBLE_EQ(sif.quaternion().w(), 0.707);
  EXPECT_DOUBLE_EQ(sif.quaternion().z(), 0.707);
}

TEST(ShapeInFrameTest, BoxRoundTrip) {
  ShapeInFrame sif;
  sif.set_frame("base_link");
  sif.set_translation(Eigen::Vector3d(0.5, 1.0, 1.5));
  sif.set_quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  sif.set_shape(Box(2.0, 3.0, 4.0));

  EXPECT_TRUE(sif.Is<Box>());
  EXPECT_FALSE(sif.Is<Sphere>());
  EXPECT_FALSE(sif.Is<Cylinder>());
  EXPECT_FALSE(sif.Is<Capsule>());

  const auto& box = sif.box();
  EXPECT_DOUBLE_EQ(box.width(), 2.0);
  EXPECT_DOUBLE_EQ(box.depth(), 3.0);
  EXPECT_DOUBLE_EQ(box.height(), 4.0);

  const auto msg = ToProto(sif);
  EXPECT_EQ(msg.frame(), "base_link");
  EXPECT_EQ(msg.shape().shape_case(), proto::Shape::kBox);

  const auto sif2 = FromProto<ShapeInFrame>(msg);
  EXPECT_EQ(sif2.frame(), "base_link");
  EXPECT_TRUE(sif2.Is<Box>());
  EXPECT_DOUBLE_EQ(sif2.box().width(), 2.0);
  EXPECT_DOUBLE_EQ(sif2.box().depth(), 3.0);
  EXPECT_DOUBLE_EQ(sif2.box().height(), 4.0);
  EXPECT_DOUBLE_EQ(sif2.translation().x(), 0.5);
  EXPECT_DOUBLE_EQ(sif2.translation().y(), 1.0);
  EXPECT_DOUBLE_EQ(sif2.translation().z(), 1.5);
}

TEST(ShapeInFrameTest, SphereRoundTrip) {
  ShapeInFrame sif;
  sif.set_frame("end_effector");
  sif.set_translation(Eigen::Vector3d(0.0, 0.0, 0.1));
  sif.set_quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  sif.set_shape(Sphere(0.05));

  EXPECT_TRUE(sif.Is<Sphere>());
  EXPECT_FALSE(sif.Is<Box>());

  const auto& sphere = sif.sphere();
  EXPECT_DOUBLE_EQ(sphere.radius(), 0.05);

  const auto msg = ToProto(sif);
  EXPECT_EQ(msg.shape().shape_case(), proto::Shape::kSphere);

  const auto sif2 = FromProto<ShapeInFrame>(msg);
  EXPECT_TRUE(sif2.Is<Sphere>());
  EXPECT_DOUBLE_EQ(sif2.sphere().radius(), 0.05);
}

TEST(ShapeInFrameTest, CylinderRoundTrip) {
  ShapeInFrame sif;
  sif.set_frame("link1");
  sif.set_translation(Eigen::Vector3d(1.0, 0.0, 0.0));
  sif.set_quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  sif.set_shape(Cylinder(0.1, 0.5));

  EXPECT_TRUE(sif.Is<Cylinder>());
  EXPECT_FALSE(sif.Is<Capsule>());

  const auto& cyl = sif.cylinder();
  EXPECT_DOUBLE_EQ(cyl.radius(), 0.1);
  EXPECT_DOUBLE_EQ(cyl.height(), 0.5);

  const auto msg = ToProto(sif);
  EXPECT_EQ(msg.shape().shape_case(), proto::Shape::kCylinder);

  const auto sif2 = FromProto<ShapeInFrame>(msg);
  EXPECT_TRUE(sif2.Is<Cylinder>());
  EXPECT_DOUBLE_EQ(sif2.cylinder().radius(), 0.1);
  EXPECT_DOUBLE_EQ(sif2.cylinder().height(), 0.5);
}

TEST(ShapeInFrameTest, CapsuleRoundTrip) {
  ShapeInFrame sif;
  sif.set_frame("link2");
  sif.set_translation(Eigen::Vector3d(0.0, 1.0, 0.0));
  sif.set_quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  sif.set_shape(Capsule(0.075, 0.3));

  EXPECT_TRUE(sif.Is<Capsule>());
  EXPECT_FALSE(sif.Is<Cylinder>());

  const auto& cap = sif.capsule();
  EXPECT_DOUBLE_EQ(cap.radius(), 0.075);
  EXPECT_DOUBLE_EQ(cap.height(), 0.3);

  const auto msg = ToProto(sif);
  EXPECT_EQ(msg.shape().shape_case(), proto::Shape::kCapsule);

  const auto sif2 = FromProto<ShapeInFrame>(msg);
  EXPECT_TRUE(sif2.Is<Capsule>());
  EXPECT_DOUBLE_EQ(sif2.capsule().radius(), 0.075);
  EXPECT_DOUBLE_EQ(sif2.capsule().height(), 0.3);
}

TEST(ShapeInFrameTest, CopyConstruction) {
  ShapeInFrame sif1;
  sif1.set_frame("original");
  sif1.set_translation(Eigen::Vector3d(1.0, 2.0, 3.0));
  sif1.set_shape(Box(1.0, 1.0, 1.0));

  ShapeInFrame sif2(sif1);
  EXPECT_EQ(sif2.frame(), "original");
  EXPECT_TRUE(sif2.Is<Box>());
  EXPECT_DOUBLE_EQ(sif2.translation().x(), 1.0);
}

TEST(ShapeInFrameTest, CopyAssignment) {
  ShapeInFrame sif1;
  sif1.set_frame("source");
  sif1.set_translation(Eigen::Vector3d(4.0, 5.0, 6.0));
  sif1.set_shape(Sphere(2.0));

  ShapeInFrame sif2;
  sif2 = sif1;
  EXPECT_EQ(sif2.frame(), "source");
  EXPECT_TRUE(sif2.Is<Sphere>());
  EXPECT_DOUBLE_EQ(sif2.sphere().radius(), 2.0);
}

TEST(ShapeInFrameTest, PolymorphicAccess) {
  ShapeInFrame sif;
  sif.set_frame("test_frame");
  sif.set_shape(Cylinder(0.5, 1.0));

  const Shape& shape = sif.shape();
  EXPECT_EQ(shape.type(), "Cylinder");
  EXPECT_DOUBLE_EQ(CalcVolume(shape), M_PI * 0.5 * 0.5 * 1.0);

  const auto dims = shape.dims();
  EXPECT_EQ(dims.size(), 2);
  EXPECT_DOUBLE_EQ(dims(0), 0.5);
  EXPECT_DOUBLE_EQ(dims(1), 1.0);
}

TEST(ShapeInFrameTest, VectorOfShapesInFrame) {
  std::vector<ShapeInFrame> shapes;

  ShapeInFrame sif1;
  sif1.set_frame("frame1");
  sif1.set_shape(Box(1.0, 1.0, 1.0));
  shapes.push_back(sif1);

  ShapeInFrame sif2;
  sif2.set_frame("frame2");
  sif2.set_shape(Sphere(0.5));
  shapes.push_back(sif2);

  ShapeInFrame sif3;
  sif3.set_frame("frame3");
  sif3.set_shape(Capsule(0.2, 0.8));
  shapes.push_back(sif3);

  EXPECT_EQ(shapes.size(), 3);
  EXPECT_TRUE(shapes[0].Is<Box>());
  EXPECT_TRUE(shapes[1].Is<Sphere>());
  EXPECT_TRUE(shapes[2].Is<Capsule>());
}

TEST(ShapeInFrameTest, WrongTypeAccessThrows) {
  ShapeInFrame sif;
  sif.set_shape(Box(1.0, 1.0, 1.0));

  EXPECT_THROW(sif.sphere(), std::runtime_error);
  EXPECT_THROW(sif.cylinder(), std::runtime_error);
  EXPECT_THROW(sif.capsule(), std::runtime_error);
}

TEST(ShapeTest, VisitWithAutoDeduction) {
  Cylinder cyl(0.5, 1.0);

  // Visit with automatic return type deduction
  std::string result = cyl.Visit([](const auto& shape) {
    return shape.type();
  });

  EXPECT_EQ(result, "Cylinder");
}

TEST(ShapeTest, VisitWithExplicitReturnType) {
  Box box(1.0, 2.0, 3.0);

  // Visit with explicit return type
  double result = box.Visit<double>([](const auto& shape) {
    return CalcVolume(shape);
  });

  EXPECT_DOUBLE_EQ(result, 6.0);
}

TEST(ShapeTest, VisitDispatchesToConcreteType) {
  Cylinder cyl(0.3, 0.8);

  // Visit can distinguish concrete types at compile time
  cyl.Visit([](const auto& shape) {
    using ShapeType = std::decay_t<decltype(shape)>;
    if constexpr (std::is_same_v<ShapeType, Cylinder>) {
      EXPECT_DOUBLE_EQ(shape.radius(), 0.3);
      EXPECT_DOUBLE_EQ(shape.height(), 0.8);
    } else {
      FAIL() << "Wrong shape type dispatched";
    }
  });
}

TEST(ShapeTest, VisitWorksWithAllShapeTypes) {
  std::vector<std::unique_ptr<Shape>> shapes;
  shapes.push_back(std::make_unique<Box>(1.0, 1.0, 1.0));
  shapes.push_back(std::make_unique<Sphere>(0.5));
  shapes.push_back(std::make_unique<Cylinder>(0.25, 1.0));
  shapes.push_back(std::make_unique<Capsule>(0.15, 0.6));

  std::vector<std::string> types;
  for (const auto& shape : shapes) {
    shape->Visit([&types](const auto& s) {
      types.push_back(s.type());
    });
  }

  ASSERT_EQ(types.size(), 4);
  EXPECT_EQ(types[0], "Box");
  EXPECT_EQ(types[1], "Sphere");
  EXPECT_EQ(types[2], "Cylinder");
  EXPECT_EQ(types[3], "Capsule");
}

TEST(ShapeInFrameTest, VisitShapeInFrame) {
  ShapeInFrame sif;
  sif.set_frame("test_frame");
  sif.set_shape(Capsule(0.1, 0.5));

  // Can visit the underlying shape
  double vol = sif.shape().Visit<double>([](const auto& s) {
    return CalcVolume(s);
  });

  EXPECT_NEAR(vol, M_PI * 0.1 * 0.1 * ((4.0 / 3.0) * 0.1 + 0.5), 1e-10);
}

TEST(ShapeTest, CalcVolumeFromShape) {
  Box box(1.0, 2.0, 3.0);
  Sphere sphere(0.5);
  Cylinder cylinder(0.2, 1.5);
  Capsule capsule(0.1, 0.8);

  EXPECT_DOUBLE_EQ(CalcVolume(box), 6.0);
  EXPECT_NEAR(CalcVolume(sphere), (4.0 / 3.0) * M_PI * std::pow(0.5, 3), 1e-12);
  EXPECT_NEAR(CalcVolume(cylinder), M_PI * std::pow(0.2, 2) * 1.5, 1e-12);
  EXPECT_NEAR(CalcVolume(capsule),
              M_PI * std::pow(0.1, 2) * ((4.0 / 3.0) * 0.1 + 0.8), 1e-12);
}

TEST(ShapeInFrameTest, ColorAndName) {
  ShapeInFrame sif;
  sif.set_shape(Box(1.0, 2.0, 3.0));
  sif.set_frame("world");
  sif.set_color(Rgba(1.0, 0.0, 0.0, 0.5));
  sif.set_name("my_box");

  EXPECT_DOUBLE_EQ(sif.color().r(), 1.0);
  EXPECT_DOUBLE_EQ(sif.color().g(), 0.0);
  EXPECT_DOUBLE_EQ(sif.color().b(), 0.0);
  EXPECT_DOUBLE_EQ(sif.color().a(), 0.5);
  EXPECT_EQ(sif.name(), "my_box");
}

TEST(ShapeInFrameTest, ColorAndNameDefaultValues) {
  ShapeInFrame sif;
  sif.set_shape(Sphere(0.1));

  // Default color should be (0, 0, 0, 1) per Rgba default constructor
  EXPECT_DOUBLE_EQ(sif.color().r(), 0.0);
  EXPECT_DOUBLE_EQ(sif.color().g(), 0.0);
  EXPECT_DOUBLE_EQ(sif.color().b(), 0.0);
  EXPECT_DOUBLE_EQ(sif.color().a(), 1.0);
  // Default name should be empty
  EXPECT_EQ(sif.name(), "");
}

TEST(ShapeInFrameTest, ColorAndNameRoundTrip) {
  ShapeInFrame sif;
  sif.set_frame("base");
  sif.set_shape(Cylinder(0.1, 0.5));
  sif.set_color(Rgba(0.0, 1.0, 0.0, 0.8));
  sif.set_name("green_cylinder");

  const auto msg = ToProto(sif);
  EXPECT_DOUBLE_EQ(msg.color().g(), 1.0);
  EXPECT_EQ(msg.name(), "green_cylinder");

  const auto sif2 = FromProto<ShapeInFrame>(msg);
  EXPECT_DOUBLE_EQ(sif2.color().r(), 0.0);
  EXPECT_DOUBLE_EQ(sif2.color().g(), 1.0);
  EXPECT_DOUBLE_EQ(sif2.color().b(), 0.0);
  EXPECT_DOUBLE_EQ(sif2.color().a(), 0.8);
  EXPECT_EQ(sif2.name(), "green_cylinder");
}

TEST(ShapeInFrameTest, ColorAndNameCopied) {
  ShapeInFrame sif1;
  sif1.set_shape(Box(1.0, 1.0, 1.0));
  sif1.set_color(Rgba(0.5, 0.5, 0.5, 1.0));
  sif1.set_name("grey_box");

  ShapeInFrame sif2(sif1);
  EXPECT_EQ(sif2.color(), sif1.color());
  EXPECT_EQ(sif2.name(), "grey_box");

  ShapeInFrame sif3;
  sif3 = sif1;
  EXPECT_EQ(sif3.color(), sif1.color());
  EXPECT_EQ(sif3.name(), "grey_box");
}

TEST(ShapeInFrameTest, CalcVolumeFromShapeInFrame) {
  ShapeInFrame sif;
  sif.set_frame("tool");
  sif.set_shape(Cylinder(0.3, 0.4));

  EXPECT_NEAR(CalcVolume(sif), M_PI * std::pow(0.3, 2) * 0.4, 1e-12);
}

}  // namespace planning_service_client
