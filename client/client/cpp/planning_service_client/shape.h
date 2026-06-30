#pragma once

#include <Eigen/Dense>

#include <memory>
#include <variant>

#include <concepts>

#include "planning_service_client/internal/proto_base.h"
#include "planning_service_client/rgba.h"
namespace planning_service_client {

// Forward declarations
class Cylinder;
class Capsule;
class Sphere;
class Box;

/**
 * @brief Abstract Shape class for representing geometric shapes to the planner
 * and visualizer services.
 *
 */
class Shape : public internal::ProtoBase<proto::Shape> {
 public:
  virtual ~Shape() = default;
  /** Dimensions as a vector. */
  virtual Eigen::VectorXd dims() const = 0;
  /** Type of the shape as a string. */
  virtual std::string type() const = 0;
  /** Variant pointer for downcasting. */
  using ShapeVariantConstPtr =
      std::variant<const Cylinder*, const Capsule*, const Sphere*, const Box*>;

  virtual ShapeVariantConstPtr variant_ptr() const = 0;
  virtual std::unique_ptr<Shape> Clone() const = 0;

  /**
   * Downcasting visit wrapper borrowed from Drake. See
   * https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_shape.html#ad60fe701b675aace3d2ab5b6f0b51871
   * for more.
   */
  template <typename ReturnType = void, typename Visitor>
  decltype(auto) Visit(Visitor&& visitor) const {
    if constexpr (std::is_same_v<ReturnType, void>) {
      return std::visit(
          [&visitor](auto* shape) {
            return visitor(*shape);
          },
          variant_ptr());
    } else {
      return std::visit(
          [&visitor](auto* shape) -> ReturnType {
            return visitor(*shape);
          },
          variant_ptr());
    }
  }
};

class Sphere final : public Shape {
 public:
  Sphere() = default;
  explicit Sphere(double radius)
      : dims_(Eigen::Matrix<double, 1, 1>::Constant(radius)) {}

  const double& radius() const {
    return dims_(0);
  }
  Eigen::VectorXd dims() const override {
    return dims_;
  }
  std::string type() const override {
    return "Sphere";
  }
  ShapeVariantConstPtr variant_ptr() const override {
    return ShapeVariantConstPtr {this};
  }
  std::unique_ptr<Shape> Clone() const override {
    return std::make_unique<Sphere>(*this);
  }

 private:
  Eigen::Matrix<double, 1, 1> dims_ {Eigen::Matrix<double, 1, 1>::Zero()};
  proto::Shape ToProtoImpl() const override;
  void FromProtoImpl(const proto::Shape& msg) override;
};

class Cylinder final : public Shape {
 public:
  Cylinder() = default;
  Cylinder(double radius, double height) : dims_(radius, height) {}
  Cylinder(Eigen::Vector2d dims) : dims_(dims) {}

  const double& radius() const {
    return dims_(0);
  }
  const double& height() const {
    return dims_(1);
  }
  Eigen::VectorXd dims() const override {
    return dims_;
  }
  std::string type() const override {
    return "Cylinder";
  }
  ShapeVariantConstPtr variant_ptr() const override {
    return ShapeVariantConstPtr {this};
  }
  std::unique_ptr<Shape> Clone() const override {
    return std::make_unique<Cylinder>(*this);
  }

 private:
  Eigen::Vector2d dims_ {0.0, 0.0};
  proto::Shape ToProtoImpl() const override;
  void FromProtoImpl(const proto::Shape& msg) override;
};

class Capsule : public Shape {
 public:
  Capsule() = default;
  Capsule(double radius, double height) : dims_(radius, height) {}
  Capsule(Eigen::Vector2d dims) : dims_(dims) {}

  const double& radius() const {
    return dims_(0);
  }
  const double& height() const {
    return dims_(1);
  }
  Eigen::VectorXd dims() const override {
    return dims_;
  }
  std::string type() const override {
    return "Capsule";
  }
  ShapeVariantConstPtr variant_ptr() const override {
    return ShapeVariantConstPtr {this};
  }
  std::unique_ptr<Shape> Clone() const override {
    return std::make_unique<Capsule>(*this);
  }

 private:
  Eigen::Vector2d dims_ {0.0, 0.0};
  proto::Shape ToProtoImpl() const override;
  void FromProtoImpl(const proto::Shape& msg) override;
};

class Box final : public Shape {
 public:
  Box() = default;
  Box(double width, double depth, double height)
      : dims_(width, depth, height) {}
  Box(Eigen::Vector3d dims) : dims_(dims) {}

  const double& width() const {
    return dims_(0);
  }
  const double& depth() const {
    return dims_(1);
  }
  const double& height() const {
    return dims_(2);
  }
  Eigen::VectorXd dims() const override {
    return dims_;
  }
  std::string type() const override {
    return "Box";
  }
  static Box Cube(double side_length) {
    return Box(side_length, side_length, side_length);
  }

  ShapeVariantConstPtr variant_ptr() const override {
    return ShapeVariantConstPtr {this};
  }
  std::unique_ptr<Shape> Clone() const override {
    return std::make_unique<Box>(*this);
  }

 private:
  Eigen::Vector3d dims_ {0.0, 0.0, 0.0};
  proto::Shape ToProtoImpl() const override;
  void FromProtoImpl(const proto::Shape& msg) override;
};

/**
 * @brief A Shape with a pose in a specified reference frame.
 *
 */
class ShapeInFrame : public internal::ProtoBase<proto::ShapeInFrame> {
 public:
  ShapeInFrame() = default;
  ShapeInFrame(const ShapeInFrame& other)
      : frame_(other.frame_),
        translation_(other.translation_),
        quaternion_(other.quaternion_),
        color_(other.color_),
        name_(other.name_) {
    if (other.shape_) {
      shape_ = other.shape_->Clone();
    }
  }
  ShapeInFrame(const Shape& shape) : shape_(shape.Clone()) {}

  ShapeInFrame& operator=(const ShapeInFrame& other) {
    if (this == &other) {
      return *this;
    }
    frame_ = other.frame_;
    translation_ = other.translation_;
    quaternion_ = other.quaternion_;
    color_ = other.color_;
    name_ = other.name_;
    shape_ = other.shape_ ? other.shape_->Clone() : nullptr;
    return *this;
  }
  ShapeInFrame(ShapeInFrame&&) noexcept = default;
  ShapeInFrame& operator=(ShapeInFrame&&) noexcept = default;

  const std::string& frame() const {
    return frame_;
  }
  const Eigen::Vector3d& translation() const {
    return translation_;
  }
  const Eigen::Quaterniond& quaternion() const {
    return quaternion_;
  }
  const Rgba& color() const {
    return color_;
  }
  const std::string& name() const {
    return name_;
  }
  const Shape& shape() const {
    if (!shape_) {
      throw std::runtime_error("ShapeInFrame::shape: Shape is not set!");
    }
    return *shape_;
  }

  void set_frame(const std::string_view frame) {
    frame_ = frame;
  }
  void set_translation(const Eigen::Vector3d& translation) {
    translation_ = translation;
  }
  void set_quaternion(const Eigen::Quaterniond& quaternion) {
    quaternion_ = quaternion;
  }
  void set_color(const Rgba& color) {
    color_ = color;
  }
  void set_name(const std::string_view name) {
    name_ = name;
  }

  template <typename ConcreteShape>
    requires(std::derived_from<ConcreteShape, Shape>
             && !std::same_as<ConcreteShape, Shape>)
  void set_shape(const ConcreteShape& shape) {
    shape_ = shape.Clone();
  }
  void set_shape(std::unique_ptr<Shape> shape) {
    shape_ = std::move(shape);
  }

  template <typename ConcreteShape>
    requires(std::derived_from<ConcreteShape, Shape>
             && !std::same_as<ConcreteShape, Shape>)
  bool Is() const {
    return shape_
           && std::holds_alternative<const ConcreteShape*>(
               shape_->variant_ptr());
  }

  const Capsule& capsule() const {
    const auto* ptr = dynamic_cast<const Capsule*>(shape_.get());
    if (!ptr) {
      throw std::runtime_error(
          "ShapeInFrame::capsule: Shape is not a capsule!");
    }
    return *ptr;
  }
  const Cylinder& cylinder() const {
    const auto* ptr = dynamic_cast<const Cylinder*>(shape_.get());
    if (!ptr) {
      throw std::runtime_error(
          "ShapeInFrame::cylinder: Shape is not a cylinder!");
    }
    return *ptr;
  }
  const Sphere& sphere() const {
    const auto* ptr = dynamic_cast<const Sphere*>(shape_.get());
    if (!ptr) {
      throw std::runtime_error("ShapeInFrame::sphere: Shape is not a sphere!");
    }
    return *ptr;
  }
  const Box& box() const {
    const auto* ptr = dynamic_cast<const Box*>(shape_.get());
    if (!ptr) {
      throw std::runtime_error("ShapeInFrame::box: Shape is not a box!");
    }
    return *ptr;
  }

  // Factory methods for convenient shape creation
  static ShapeInFrame MakeSphere(double radius) {
    return ShapeInFrame(Sphere(radius));
  }

  static ShapeInFrame MakeCylinder(double radius, double height) {
    return ShapeInFrame(Cylinder(radius, height));
  }

  static ShapeInFrame MakeCapsule(double radius, double height) {
    return ShapeInFrame(Capsule(radius, height));
  }

  static ShapeInFrame MakeBox(double width, double depth, double height) {
    return ShapeInFrame(Box(width, depth, height));
  }

 private:
  proto::ShapeInFrame ToProtoImpl() const override;
  void FromProtoImpl(const proto::ShapeInFrame& msg) override;

  std::unique_ptr<Shape> shape_;
  std::string frame_;
  Eigen::Vector3d translation_ {0.0, 0.0, 0.0};
  Eigen::Quaterniond quaternion_ {1.0, 0.0, 0.0, 0.0};
  Rgba color_;
  std::string name_;
};

double CalcVolume(const Shape& shape);
double CalcVolume(const ShapeInFrame& shape_in_frame);
}  // namespace planning_service_client
