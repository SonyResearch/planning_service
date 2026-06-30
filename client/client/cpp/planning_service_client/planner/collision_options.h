#pragma once

#include "planning_service_client/internal/proto_base.h"
#include "planning_service_client/shape.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief A pair of links for which collision options may be specified. In
 * general, these pairs are unordered - the pair (A, B) is equivalent to (B, A).
 *
 */
class CollisionPair : public internal::ProtoBase<proto::CollisionPair> {
 public:
  CollisionPair() = default;
  CollisionPair(const std::string_view body_1, const std::string_view body_2)
      : body_1_(body_1.data()), body_2_(body_2.data()) {}
  CollisionPair(const std::pair<std::string, std::string>& pair)
      : body_1_(pair.first), body_2_(pair.second) {}

  const std::string& body_1() const {
    return body_1_;
  }
  const std::string& body_2() const {
    return body_2_;
  }

 private:
  std::string body_1_, body_2_;
  proto::CollisionPair ToProtoImpl() const;
  void FromProtoImpl(const proto::CollisionPair& msg);
};

/**
 * @brief A padding value in meters to be applied to a given collision pair.
 *
 */
class CollisionPadding : public internal::ProtoBase<proto::CollisionPadding> {
 public:
  CollisionPadding() = default;
  CollisionPadding(const CollisionPair& pair, double padding)
      : pair_(pair), padding_(padding) {}
  CollisionPadding(const std::string_view body_1, const std::string_view body_2,
                   double padding)
      : pair_(body_1, body_2), padding_(padding) {}

  const CollisionPair& pair() const {
    return pair_;
  }
  const double& padding() const {
    return padding_;
  }

 private:
  CollisionPair pair_;
  double padding_;
  proto::CollisionPadding ToProtoImpl() const override;
  void FromProtoImpl(const proto::CollisionPadding& msg) override;
};

class CollisionOptions : public internal::ProtoBase<proto::CollisionOptions> {
 public:
  CollisionOptions() = default;
  /** Public members. */
  std::vector<CollisionPadding> paddings;
  std::vector<CollisionPair> filtered_pairs;
  std::vector<ShapeInFrame> shapes;

 private:
  proto::CollisionOptions ToProtoImpl() const override;
  void FromProtoImpl(const proto::CollisionOptions& msg) override;
};
}  // namespace planner
}  // namespace planning_service_client
