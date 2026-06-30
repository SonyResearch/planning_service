#include "shape.h"

#include "planning_service_client/internal/overloaded.h"
namespace planning_service_client {

proto::Shape Box::ToProtoImpl() const {
  proto::Shape msg;
  auto* box_msg = msg.mutable_box();
  box_msg->set_width(dims_(0));
  box_msg->set_depth(dims_(1));
  box_msg->set_height(dims_(2));
  return msg;
}

void Box::FromProtoImpl(const proto::Shape& msg) {
  if (msg.shape_case() != proto::Shape::ShapeCase::kBox) {
    throw std::runtime_error(
        "Box::FromProtoImpl: Proto message is not of type Box.");
  }
  const auto& box_msg = msg.box();
  dims_ = Eigen::Vector3d(box_msg.width(), box_msg.depth(), box_msg.height());
}

proto::Shape Cylinder::ToProtoImpl() const {
  proto::Shape msg;
  auto* cyl_msg = msg.mutable_cylinder();
  cyl_msg->set_radius(dims_(0));
  cyl_msg->set_height(dims_(1));
  return msg;
}

void Cylinder::FromProtoImpl(const proto::Shape& msg) {
  if (msg.shape_case() != proto::Shape::ShapeCase::kCylinder) {
    throw std::runtime_error(
        "Cylinder::FromProtoImpl: Proto message is not of type Cylinder.");
  }
  const auto& cyl_msg = msg.cylinder();
  dims_ = Eigen::Vector2d(cyl_msg.radius(), cyl_msg.height());
}

proto::Shape Capsule::ToProtoImpl() const {
  proto::Shape msg;
  auto* capsule_msg = msg.mutable_capsule();
  capsule_msg->set_radius(dims_(0));
  capsule_msg->set_height(dims_(1));
  return msg;
}

void Capsule::FromProtoImpl(const proto::Shape& msg) {
  if (msg.shape_case() != proto::Shape::ShapeCase::kCapsule) {
    throw std::runtime_error(
        "Capsule::FromProtoImpl: Proto message is not of type Capsule.");
  }
  const auto& capsule_msg = msg.capsule();
  dims_ = Eigen::Vector2d(capsule_msg.radius(), capsule_msg.height());
}

proto::Shape Sphere::ToProtoImpl() const {
  proto::Shape msg;
  msg.mutable_sphere()->set_radius(dims_(0));
  return msg;
}
void Sphere::FromProtoImpl(const proto::Shape& msg) {
  if (msg.shape_case() != proto::Shape::ShapeCase::kSphere) {
    throw std::runtime_error(
        "Sphere::FromProtoImpl: Proto message is not of type Sphere.");
  }
  dims_(0) = msg.sphere().radius();
}

proto::ShapeInFrame ShapeInFrame::ToProtoImpl() const {
  proto::ShapeInFrame msg;
  msg.set_frame(frame_);

  auto* translation_pb = msg.mutable_pose_in_frame()->mutable_translation();
  translation_pb->set_x(translation_.x());
  translation_pb->set_y(translation_.y());
  translation_pb->set_z(translation_.z());

  auto* rotation_pb = msg.mutable_pose_in_frame()->mutable_quat();
  rotation_pb->set_w(quaternion_.w());
  rotation_pb->set_x(quaternion_.x());
  rotation_pb->set_y(quaternion_.y());
  rotation_pb->set_z(quaternion_.z());

  if (!shape_) {
    throw std::runtime_error("ShapeInFrame::ToProtoImpl: Shape is not set!");
  }
  msg.mutable_shape()->CopyFrom(ToProto(*shape_));

  msg.mutable_color()->CopyFrom(ToProto(color_));
  msg.set_name(name_);

  return msg;
}

void ShapeInFrame::FromProtoImpl(const proto::ShapeInFrame& msg) {
  frame_ = msg.frame();
  const proto::Vector3& translation_pb = msg.pose_in_frame().translation();
  translation_ = Eigen::Vector3d(translation_pb.x(), translation_pb.y(),
                                 translation_pb.z());

  const proto::Quaternion& rotation_pb = msg.pose_in_frame().quat();
  quaternion_ = Eigen::Quaterniond(rotation_pb.w(), rotation_pb.x(),
                                   rotation_pb.y(), rotation_pb.z());

  const auto& shape_pb = msg.shape();
  switch (shape_pb.shape_case()) {
    case proto::Shape::kCylinder:
      shape_ = std::make_unique<planning_service_client::Cylinder>(
          FromProto<planning_service_client::Cylinder>(shape_pb));
      break;
    case proto::Shape::kSphere:
      shape_ = std::make_unique<planning_service_client::Sphere>(
          FromProto<planning_service_client::Sphere>(shape_pb));
      break;
    case proto::Shape::kBox:
      shape_ = std::make_unique<planning_service_client::Box>(
          FromProto<planning_service_client::Box>(shape_pb));
      break;
    case proto::Shape::kCapsule:
      shape_ = std::make_unique<planning_service_client::Capsule>(
          FromProto<planning_service_client::Capsule>(shape_pb));
      break;
    case proto::Shape::SHAPE_NOT_SET:
      throw std::runtime_error(
          "ShapeInFrame::FromProtoImpl: Shape not set in proto message!");
  }

  color_ = FromProto<Rgba>(msg.color());
  name_ = msg.name();
}

double CalcVolume(const Shape& shape) {
  return shape.Visit<double>(overloaded {
      [](const Cylinder& cylinder) {
        return M_PI * std::pow(cylinder.radius(), 2) * cylinder.height();
      },
      [](const Capsule& capsule) {
        return M_PI * std::pow(capsule.radius(), 2)
               * ((4.0 / 3.0) * capsule.radius() + capsule.height());
      },
      [](const Sphere& sphere) {
        return (4.0 / 3.0) * M_PI * std::pow(sphere.radius(), 3);
      },
      [](const Box& box) {
        return box.width() * box.depth() * box.height();
      },
  });
}

double CalcVolume(const ShapeInFrame& shape_in_frame) {
  return CalcVolume(shape_in_frame.shape());
}

}  // namespace planning_service_client
