
#include "constraints.h"

#include "planning_service_client/internal/eigen_utils.h"

namespace planning_service_client {
namespace planner {

PositionConstraint::PositionConstraint(const std::string& frame_A,
                                       const std::string& frame_B,
                                       const Eigen::Vector3d& p_AQ_lower,
                                       const Eigen::Vector3d& p_AQ_upper,
                                       const Eigen::Vector3d& p_BQ)
    : frame_A_(frame_A),
      frame_B_(frame_B),
      p_AQ_lower_(p_AQ_lower),
      p_AQ_upper_(p_AQ_upper),
      p_BQ_(p_BQ) {}

bool PositionConstraint::operator<=(
    const PositionConstraint& other) const noexcept {
  return (frame_A_ == other.frame_A_) && (frame_B_ == other.frame_B_)
         && (p_AQ_lower_.array() >= other.p_AQ_lower_.array()).all()
         && (p_AQ_upper_.array() <= other.p_AQ_upper_.array()).all()
         && p_BQ_.isApprox(other.p_BQ_);
}

proto::PositionConstraint PositionConstraint::ToProtoImpl() const {
  proto::PositionConstraint msg;
  msg.set_frame_a(frame_A_);
  msg.set_frame_b(frame_B_);
  *msg.mutable_p_aq_lower() = internal::EigenVector3dToProto(p_AQ_lower_);
  *msg.mutable_p_aq_upper() = internal::EigenVector3dToProto(p_AQ_upper_);
  *msg.mutable_p_bq() = internal::EigenVector3dToProto(p_BQ_);
  return msg;
}

void PositionConstraint::FromProtoImpl(const proto::PositionConstraint& msg) {
  frame_A_ = msg.frame_a();
  frame_B_ = msg.frame_b();
  p_AQ_lower_ = internal::ProtoToEigenVector3d(msg.p_aq_lower());
  p_AQ_upper_ = internal::ProtoToEigenVector3d(msg.p_aq_upper());
  p_BQ_ = internal::ProtoToEigenVector3d(msg.p_bq());
}

AngleBetweenVectorsConstraint::AngleBetweenVectorsConstraint(
    const std::string& frame_A, const std::string& frame_B,
    const Eigen::Vector3d& a_A, const Eigen::Vector3d& b_B, double angle_lower,
    double angle_upper)
    : frame_A_(frame_A),
      frame_B_(frame_B),
      a_A_(a_A),
      b_B_(b_B),
      angle_lower_(angle_lower),
      angle_upper_(angle_upper) {}

bool AngleBetweenVectorsConstraint::operator<=(
    const AngleBetweenVectorsConstraint& other) const noexcept {
  return (frame_A_ == other.frame_A_) && (frame_B_ == other.frame_B_)
         && a_A_.isApprox(other.a_A_) && b_B_.isApprox(other.b_B_)
         && (angle_lower_ >= other.angle_lower_)
         && (angle_upper_ <= other.angle_upper_);
}

proto::AngleBetweenVectorsConstraint
AngleBetweenVectorsConstraint::ToProtoImpl() const {
  proto::AngleBetweenVectorsConstraint msg;
  msg.set_frame_a(frame_A_);
  msg.set_frame_b(frame_B_);
  *msg.mutable_a_a() = internal::EigenVector3dToProto(a_A_);
  *msg.mutable_b_b() = internal::EigenVector3dToProto(b_B_);
  msg.set_angle_lower(angle_lower_);
  msg.set_angle_upper(angle_upper_);
  return msg;
}

void AngleBetweenVectorsConstraint::FromProtoImpl(
    const proto::AngleBetweenVectorsConstraint& msg) {
  frame_A_ = msg.frame_a();
  frame_B_ = msg.frame_b();
  a_A_ = internal::ProtoToEigenVector3d(msg.a_a());
  b_B_ = internal::ProtoToEigenVector3d(msg.b_b());
  angle_lower_ = msg.angle_lower();
  angle_upper_ = msg.angle_upper();
}

void GeometricConstraints::Add(const PositionConstraint& constraint) {
  position_constraints_.emplace_back(constraint);
}

void GeometricConstraints::Add(
    const AngleBetweenVectorsConstraint& constraint) {
  angle_between_vectors_constraints_.emplace_back(constraint);
}

bool GeometricConstraints::operator<=(
    const GeometricConstraints& other) const noexcept {
  // For each position constraint in this, there must be a tighter one in other
  // Also, for each position constraint in other, there must be a tighter one in
  // this->
  std::vector<bool> matched_this_pc(this->position_constraints_.size(), false);
  std::vector<bool> matched_other_pc(other.position_constraints_.size(), false);
  for (int i = 0; i < std::ssize(position_constraints_); ++i) {
    for (int j = 0; j < std::ssize(other.position_constraints()); ++j) {
      if (position_constraints_[i] <= other.position_constraints()[j]) {
        matched_this_pc[i] = true;
        matched_other_pc[j] = true;
      }
    }
  }
  // Check if all constraints are matched
  if (std::any_of(matched_this_pc.begin(), matched_this_pc.end(), [](bool v) {
        return !v;
      })) {
    return false;
  }
  if (std::any_of(matched_other_pc.begin(), matched_other_pc.end(), [](bool v) {
        return !v;
      })) {
    return false;
  }
  // Repeat for angle between vectors constraints
  std::vector<bool> matched_this_ac(
      this->angle_between_vectors_constraints_.size(), false);
  std::vector<bool> matched_other_ac(
      other.angle_between_vectors_constraints().size(), false);
  for (int i = 0; i < std::ssize(angle_between_vectors_constraints_); ++i) {
    for (int j = 0; j < std::ssize(other.angle_between_vectors_constraints());
         ++j) {
      if (angle_between_vectors_constraints_[i]
          <= other.angle_between_vectors_constraints()[j]) {
        matched_this_ac[i] = true;
        matched_other_ac[j] = true;
      }
    }
  }
  // Check if all constraints are matched
  if (std::any_of(matched_this_ac.begin(), matched_this_ac.end(), [](bool v) {
        return !v;
      })) {
    return false;
  }
  if (std::any_of(matched_other_ac.begin(), matched_other_ac.end(), [](bool v) {
        return !v;
      })) {
    return false;
  }
  // If all constraints are matched, return true
  return true;
}

proto::GeometricConstraints GeometricConstraints::ToProtoImpl() const {
  proto::GeometricConstraints msg;
  for (const auto& pc : position_constraints_) {
    *msg.add_position_constraints() = ToProto(pc);
  }
  for (const auto& ac : angle_between_vectors_constraints_) {
    *msg.add_angle_between_vectors_constraints() = ToProto(ac);
  }
  return msg;
}

void GeometricConstraints::FromProtoImpl(
    const proto::GeometricConstraints& msg) {
  position_constraints_.clear();
  for (const auto& pc_msg : msg.position_constraints()) {
    position_constraints_.emplace_back(FromProto<PositionConstraint>(pc_msg));
  }
  angle_between_vectors_constraints_.clear();
  for (const auto& ac_msg : msg.angle_between_vectors_constraints()) {
    angle_between_vectors_constraints_.emplace_back(
        FromProto<AngleBetweenVectorsConstraint>(ac_msg));
  }
}

}  // namespace planner
}  // namespace planning_service_client
