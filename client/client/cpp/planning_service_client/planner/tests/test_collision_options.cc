#include <gtest/gtest.h>

#include <google/protobuf/util/message_differencer.h>

#include "planning_service_client/planner/collision_options.h"
#include "planning_service_client/shape.h"
#include "test_utils.h"
namespace planning_service_client {
namespace planner {

namespace pb = google::protobuf;

TEST(CollisionPairTest, Proto) {
  CollisionPair obj("body_A", "body_B");
  auto msg = ToProto(obj);
  EXPECT_EQ(obj.body_1(), msg.body_1());
  EXPECT_EQ(obj.body_2(), msg.body_2());

  const auto new_obj {FromProto<CollisionPair>(msg)};

  EXPECT_EQ(obj.body_1(), new_obj.body_1());
  EXPECT_EQ(obj.body_2(), new_obj.body_2());
}

TEST(CollisionPaddingTest, Proto) {
  CollisionPair pair("body_A", "body_B");
  CollisionPadding obj(pair, 0.1);
  auto msg = ToProto(obj);
  EXPECT_EQ(obj.pair().body_1(), msg.pair().body_1());
  EXPECT_EQ(obj.pair().body_2(), msg.pair().body_2());
  EXPECT_EQ(obj.padding(), msg.padding());

  const auto new_obj {FromProto<CollisionPadding>(msg)};

  EXPECT_EQ(obj.pair().body_1(), new_obj.pair().body_1());
  EXPECT_EQ(obj.pair().body_2(), new_obj.pair().body_2());
  EXPECT_EQ(obj.padding(), new_obj.padding());
}

// Helper: build a ShapeInFrame with a Sphere.
ShapeInFrame MakeSphereInFrame(const std::string& frame, double radius,
                               const Eigen::Vector3d& t) {
  ShapeInFrame sif;
  sif.set_frame(frame);
  sif.set_translation(t);
  sif.set_quaternion(Eigen::Quaterniond::Identity());
  sif.set_shape(Sphere(radius));
  return sif;
}

// Helper: build a ShapeInFrame with a Box.
ShapeInFrame MakeBoxInFrame(const std::string& frame, double w, double d,
                            double h, const Eigen::Vector3d& t) {
  ShapeInFrame sif;
  sif.set_frame(frame);
  sif.set_translation(t);
  sif.set_quaternion(Eigen::Quaterniond::Identity());
  sif.set_shape(Box(w, d, h));
  return sif;
}

TEST(CollisionOptionsTest, ToProto) {
  CollisionPadding padding1(CollisionPair("body_A", "body_B"), 0.1);
  CollisionPadding padding2(CollisionPair("body_C", "body_D"), 0.2);
  CollisionPair filtered_pair1("body_E", "body_F");
  CollisionPair filtered_pair2("body_G", "body_H");
  ShapeInFrame shape1 =
      MakeSphereInFrame("world", 0.05, Eigen::Vector3d(0.0, 0.0, 0.5));
  ShapeInFrame shape2 =
      MakeBoxInFrame("base", 0.1, 0.2, 0.3, Eigen::Vector3d(1.0, 0.0, 0.0));

  CollisionOptions obj;
  obj.paddings.push_back(padding1);
  obj.paddings.push_back(padding2);
  obj.filtered_pairs.push_back(filtered_pair1);
  obj.filtered_pairs.push_back(filtered_pair2);
  obj.shapes.push_back(shape1);
  obj.shapes.push_back(shape2);

  auto msg = ToProto(obj);
  EXPECT_EQ(msg.paddings_size(), 2);
  EXPECT_EQ(msg.filtered_pairs_size(), 2);
  EXPECT_EQ(msg.shapes_size(), 2);
  EXPECT_TRUE(
      pb::util::MessageDifferencer::Equals(msg.paddings(0), ToProto(padding1)));
  EXPECT_TRUE(
      pb::util::MessageDifferencer::Equals(msg.paddings(1), ToProto(padding2)));
  EXPECT_TRUE(pb::util::MessageDifferencer::Equals(msg.filtered_pairs(0),
                                                   ToProto(filtered_pair1)));
  EXPECT_TRUE(pb::util::MessageDifferencer::Equals(msg.filtered_pairs(1),
                                                   ToProto(filtered_pair2)));
  EXPECT_TRUE(
      pb::util::MessageDifferencer::Equals(msg.shapes(0), ToProto(shape1)));
  EXPECT_TRUE(
      pb::util::MessageDifferencer::Equals(msg.shapes(1), ToProto(shape2)));

  // Verify shape types are encoded correctly.
  EXPECT_EQ(msg.shapes(0).shape().shape_case(), proto::Shape::kSphere);
  EXPECT_EQ(msg.shapes(1).shape().shape_case(), proto::Shape::kBox);
}

TEST(CollisionOptionsTest, FromProto) {
  proto::CollisionOptions msg;
  auto padding = msg.add_paddings();
  padding->mutable_pair()->set_body_1("body_A");
  padding->mutable_pair()->set_body_2("body_B");
  padding->set_padding(0.1);
  auto filtered_pair = msg.add_filtered_pairs();
  filtered_pair->set_body_1("body_C");
  filtered_pair->set_body_2("body_D");
  // Add a sphere shape.
  auto shape_proto = msg.add_shapes();
  shape_proto->set_frame("tool");
  shape_proto->mutable_pose_in_frame()->mutable_translation()->set_x(0.1);
  shape_proto->mutable_pose_in_frame()->mutable_translation()->set_y(0.2);
  shape_proto->mutable_pose_in_frame()->mutable_translation()->set_z(0.3);
  shape_proto->mutable_shape()->mutable_sphere()->set_radius(0.05);

  const auto obj = FromProto<CollisionOptions>(msg);
  ASSERT_EQ(obj.paddings.size(), 1);
  EXPECT_EQ(obj.paddings[0].pair().body_1(), "body_A");
  EXPECT_EQ(obj.paddings[0].pair().body_2(), "body_B");
  EXPECT_EQ(obj.paddings[0].padding(), 0.1);
  ASSERT_EQ(obj.filtered_pairs.size(), 1);
  EXPECT_EQ(obj.filtered_pairs[0].body_1(), "body_C");
  EXPECT_EQ(obj.filtered_pairs[0].body_2(), "body_D");
  ASSERT_EQ(obj.shapes.size(), 1);
  EXPECT_EQ(obj.shapes[0].frame(), "tool");
  EXPECT_TRUE(obj.shapes[0].Is<Sphere>());
  EXPECT_DOUBLE_EQ(obj.shapes[0].sphere().radius(), 0.05);
}

TEST(CollisionOptionsTest, ShapesRoundTrip) {
  ShapeInFrame sphere_sif =
      MakeSphereInFrame("link_1", 0.03, Eigen::Vector3d(0.0, 0.1, 0.2));
  ShapeInFrame box_sif =
      MakeBoxInFrame("link_2", 0.4, 0.5, 0.6, Eigen::Vector3d(0.7, 0.8, 0.9));

  CollisionOptions obj;
  obj.shapes.push_back(sphere_sif);
  obj.shapes.push_back(box_sif);

  const auto roundtripped = FromProto<CollisionOptions>(ToProto(obj));

  ASSERT_EQ(roundtripped.shapes.size(), 2);

  // Sphere
  EXPECT_EQ(roundtripped.shapes[0].frame(), "link_1");
  EXPECT_TRUE(roundtripped.shapes[0].Is<Sphere>());
  EXPECT_DOUBLE_EQ(roundtripped.shapes[0].sphere().radius(), 0.03);
  EXPECT_TRUE(roundtripped.shapes[0].translation().isApprox(
      Eigen::Vector3d(0.0, 0.1, 0.2)));

  // Box
  EXPECT_EQ(roundtripped.shapes[1].frame(), "link_2");
  EXPECT_TRUE(roundtripped.shapes[1].Is<Box>());
  EXPECT_DOUBLE_EQ(roundtripped.shapes[1].box().width(), 0.4);
  EXPECT_DOUBLE_EQ(roundtripped.shapes[1].box().depth(), 0.5);
  EXPECT_DOUBLE_EQ(roundtripped.shapes[1].box().height(), 0.6);
  EXPECT_TRUE(roundtripped.shapes[1].translation().isApprox(
      Eigen::Vector3d(0.7, 0.8, 0.9)));
}

}  // namespace planner
}  // namespace planning_service_client
