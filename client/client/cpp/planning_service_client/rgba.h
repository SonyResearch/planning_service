#pragma once

#include <Eigen/Dense>

#include "planning_service_client/internal/proto_base.h"

namespace planning_service_client {

/**
 * @brief RGB-A color representation.
 *
 * Each channel (r, g, b, a) is a double in the range [0, 1], where a is the
 * alpha (opacity) channel. A value of 1.0 for alpha is fully opaque.
 */
class Rgba : public internal::ProtoBase<proto::Rgba> {
 public:
  Rgba() = default;

  /**
   * @brief Construct with explicit channel values.
   *
   * @param r Red channel in [0, 1].
   * @param g Green channel in [0, 1].
   * @param b Blue channel in [0, 1].
   * @param a Alpha channel in [0, 1]. Defaults to 1.0 (fully opaque).
   */
  Rgba(double r, double g, double b, double a = 1.0)
      : r_(r), g_(g), b_(b), a_(a) {}

  Rgba(const Eigen::Vector4d& rgba)
      : Rgba(rgba(0), rgba(1), rgba(2), rgba(3)) {}

  /** Red channel in [0, 1]. */
  double r() const {
    return r_;
  }
  /** Green channel in [0, 1]. */
  double g() const {
    return g_;
  }
  /** Blue channel in [0, 1]. */
  double b() const {
    return b_;
  }
  /** Alpha (opacity) channel in [0, 1]. */
  double a() const {
    return a_;
  }

  bool operator==(const Rgba& other) const {
    return r_ == other.r_ && g_ == other.g_ && b_ == other.b_ && a_ == other.a_;
  }

  // --- Common colors ---
  static Rgba Red(double alpha = 1.0) {
    return {1.0, 0.0, 0.0, alpha};
  }
  static Rgba Green(double alpha = 1.0) {
    return {0.0, 1.0, 0.0, alpha};
  }
  static Rgba Blue(double alpha = 1.0) {
    return {0.0, 0.0, 1.0, alpha};
  }
  static Rgba White(double alpha = 1.0) {
    return {1.0, 1.0, 1.0, alpha};
  }
  static Rgba Black(double alpha = 1.0) {
    return {0.0, 0.0, 0.0, alpha};
  }

 private:
  proto::Rgba ToProtoImpl() const override;
  void FromProtoImpl(const proto::Rgba& msg) override;

  double r_ {0.0};
  double g_ {0.0};
  double b_ {0.0};
  double a_ {1.0};
};

}  // namespace planning_service_client
