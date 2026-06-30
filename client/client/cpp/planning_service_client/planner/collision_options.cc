#include "collision_options.h"

namespace planning_service_client {
namespace planner {

proto::CollisionPair CollisionPair::ToProtoImpl() const {
  proto::CollisionPair msg;
  msg.set_body_1(body_1_);
  msg.set_body_2(body_2_);
  return msg;
}
void CollisionPair::FromProtoImpl(const proto::CollisionPair& msg) {
  body_1_ = msg.body_1();
  body_2_ = msg.body_2();
}

proto::CollisionPadding CollisionPadding::ToProtoImpl() const {
  proto::CollisionPadding msg;
  *msg.mutable_pair() = ToProto(pair_);
  msg.set_padding(padding_);
  return msg;
}
void CollisionPadding::FromProtoImpl(const proto::CollisionPadding& msg) {
  pair_ = FromProto<CollisionPair>(msg.pair());
  padding_ = msg.padding();
}

proto::CollisionOptions CollisionOptions::ToProtoImpl() const {
  proto::CollisionOptions msg;
  for (const auto& padding : paddings) {
    msg.add_paddings()->CopyFrom(ToProto(padding));
  }
  for (const auto& pair : filtered_pairs) {
    msg.add_filtered_pairs()->CopyFrom(ToProto(pair));
  }
  for (const auto& shape : shapes) {
    msg.add_shapes()->CopyFrom(ToProto(shape));
  }
  return msg;
}
void CollisionOptions::FromProtoImpl(const proto::CollisionOptions& msg) {
  paddings.clear();
  filtered_pairs.clear();
  for (const auto& padding : msg.paddings()) {
    paddings.push_back(FromProto<CollisionPadding>(padding));
  }
  for (const auto& pair : msg.filtered_pairs()) {
    filtered_pairs.push_back(FromProto<CollisionPair>(pair));
  }
  for (const auto& shape : msg.shapes()) {
    shapes.push_back(FromProto<ShapeInFrame>(shape));
  }
}
}  // namespace planner
}  // namespace planning_service_client
