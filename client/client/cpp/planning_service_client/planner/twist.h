#pragma once

#include <Eigen/Dense>

#include <string>

#include "planning_service_client/internal/proto_base.h"
#include "proto/planner.pb.h"

namespace planning_service_client {
namespace planner {

/**
 * @brief A twist specifying motion between two frames.
 * @param frame_A reference frame
 * @param frame_B target frame
 * @param frame_E frame in which twist amounts are specified
 * @param delta_xyz translation amounts in xyz (meters)
 * @param delta_rpy rotation amounts in rpy (radians)
 */
class Twist final : public internal::ProtoBase<proto::Twist> {
 public:
  Twist() = default;

  Twist(const std::string& frame_A, const std::string& frame_B,
        const std::string& frame_E, const Eigen::Vector3d& delta_xyz,
        const Eigen::Vector3d& delta_rpy);

  const std::string& frame_A() const {
    return frame_A_;
  }

  const std::string& frame_B() const {
    return frame_B_;
  }

  const std::string& frame_E() const {
    return frame_E_;
  }

  const Eigen::Vector3d& delta_xyz() const {
    return delta_xyz_;
  }

  const Eigen::Vector3d& delta_rpy() const {
    return delta_rpy_;
  }

 private:
  proto::Twist ToProtoImpl() const override;

  void FromProtoImpl(const proto::Twist& msg) override;

  std::string frame_A_;
  std::string frame_B_;
  std::string frame_E_;
  Eigen::Vector3d delta_xyz_;
  Eigen::Vector3d delta_rpy_;
};

}  // namespace planner
}  // namespace planning_service_client
