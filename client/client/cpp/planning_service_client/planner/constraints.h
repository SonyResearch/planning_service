#pragma once

#include "planning_service_client/conf.h"
#include "proto/basic_types.pb.h"
namespace planning_service_client {
namespace planner {

/**
A position constraint between two frames. See
https://drake.mit.edu/doxygen_cxx/classdrake_1_1multibody_1_1_position_constraint.html
for details.
 */
class PositionConstraint
    : public internal::ProtoBase<proto::PositionConstraint> {
 public:
  PositionConstraint() = default;

  PositionConstraint(const std::string& frame_A, const std::string& frame_B,
                     const Eigen::Vector3d& p_AQ_lower,
                     const Eigen::Vector3d& p_AQ_upper,
                     const Eigen::Vector3d& p_BQ);

  const std::string& frame_A() const {
    return frame_A_;
  }
  const std::string& frame_B() const {
    return frame_B_;
  }
  const Eigen::Vector3d& p_AQ_lower() const {
    return p_AQ_lower_;
  }
  const Eigen::Vector3d& p_AQ_upper() const {
    return p_AQ_upper_;
  }
  const Eigen::Vector3d& p_BQ() const {
    return p_BQ_;
  }

  /* Check if this constraint is tighter than other. It means this constraint's
     allowed position range is within other's range, and both constraints are
     defined between the same frames and point.
   */
  bool operator<=(const PositionConstraint& other) const noexcept;

 private:
  proto::PositionConstraint ToProtoImpl() const;

  void FromProtoImpl(const proto::PositionConstraint& msg);

  std::string frame_A_;
  std::string frame_B_;
  Eigen::Vector3d p_AQ_lower_;
  Eigen::Vector3d p_AQ_upper_;
  Eigen::Vector3d p_BQ_;
};

class AngleBetweenVectorsConstraint
    : public internal::ProtoBase<proto::AngleBetweenVectorsConstraint> {
 public:
  AngleBetweenVectorsConstraint() = default;

  AngleBetweenVectorsConstraint(const std::string& frame_A,
                                const std::string& frame_B,
                                const Eigen::Vector3d& a_A,
                                const Eigen::Vector3d& b_B, double angle_lower,
                                double angle_upper);

  const std::string& frame_A() const {
    return frame_A_;
  }
  const std::string& frame_B() const {
    return frame_B_;
  }
  const Eigen::Vector3d& a_A() const {
    return a_A_;
  }
  const Eigen::Vector3d& b_B() const {
    return b_B_;
  }
  double angle_lower() const {
    return angle_lower_;
  }
  double angle_upper() const {
    return angle_upper_;
  }

  /* Check if *this* constraint is tighter than *other*. It means this
     constraint's allowed angle range is within other's range, and both
     constraints are defined between the same frames and vectors.
   */
  bool operator<=(const AngleBetweenVectorsConstraint& other) const noexcept;

 private:
  proto::AngleBetweenVectorsConstraint ToProtoImpl() const;

  void FromProtoImpl(const proto::AngleBetweenVectorsConstraint& msg);

  std::string frame_A_;
  std::string frame_B_;
  Eigen::Vector3d a_A_;
  Eigen::Vector3d b_B_;
  double angle_lower_;
  double angle_upper_;
};

class GeometricConstraints
    : public internal::ProtoBase<proto::GeometricConstraints> {
 public:
  GeometricConstraints() = default;

  void Add(const PositionConstraint& constraint);

  void Add(const AngleBetweenVectorsConstraint& constraint);

  /* Check if this set of constraints is tighter than other. It means that for
     every constraint in *this*, there exists a corresponding constraint in
     *other* that is looser, and for every constraint in *other*, there exists a
     corresponding constraint in *this* that is tighter.
   */
  bool operator<=(const GeometricConstraints& other) const noexcept;

  const std::vector<PositionConstraint>& position_constraints() const {
    return position_constraints_;
  }

  const std::vector<AngleBetweenVectorsConstraint>&
  angle_between_vectors_constraints() const {
    return angle_between_vectors_constraints_;
  }

 private:
  proto::GeometricConstraints ToProtoImpl() const;

  void FromProtoImpl(const proto::GeometricConstraints& msg);

  std::vector<PositionConstraint> position_constraints_;
  std::vector<AngleBetweenVectorsConstraint> angle_between_vectors_constraints_;
};

}  // namespace planner
}  // namespace planning_service_client
