#pragma once

#include <Eigen/Dense>

#include "planning_service_client/internal/proto_base.h"

namespace planning_service_client {
class FrameRelativePose : public internal::ProtoBase<proto::FrameRelativePose> {
 public:
  /**
   * @brief A Frame Relative Pose object
   *
   * @param frame_A The name of the first frame
   * @param frame_B The name of the second frame
   * @param X_AB_translation The translation from frame A to frame B
   * @param X_AB_quaternion The quaternion rotation from frame A to frame B.
   *
   * @warning There is no check for the quaternion being normalized. The user is
   * responsible for ensuring that the quaternion is normalized.
   */
  FrameRelativePose() = default;

  FrameRelativePose(const std::string& frame_A, const std::string& frame_B,
                    const Eigen::Vector3d& X_AB_translation,
                    const Eigen::Quaterniond& X_AB_quaternion);

  const std::string& frame_A() const {
    return frame_A_;
  }

  const std::string& frame_B() const {
    return frame_B_;
  }

  const Eigen::Vector3d& X_AB_translation() const {
    return X_AB_translation_;
  }

  const Eigen::Quaterniond& X_AB_quaternion() const {
    return X_AB_quaternion_;
  }

 private:
  proto::FrameRelativePose ToProtoImpl() const override;

  void FromProtoImpl(const proto::FrameRelativePose& msg) override;

  std::string frame_A_;
  std::string frame_B_;
  Eigen::Vector3d X_AB_translation_;
  Eigen::Quaterniond X_AB_quaternion_;
};

class FrameRelativePosesVec
    : public internal::ProtoBase<proto::FrameRelativePosesVec> {
 public:
  FrameRelativePosesVec(const std::vector<FrameRelativePose>& poses)
      : poses_(poses) {}

  // default constructor
  FrameRelativePosesVec() = default;

  const std::vector<FrameRelativePose>& FrameRelativePoses() const {
    return poses_;
  }

 private:
  void FromProtoImpl(const proto::FrameRelativePosesVec& msg) override;
  proto::FrameRelativePosesVec ToProtoImpl() const override;
  std::vector<FrameRelativePose> poses_;
};

}  // namespace planning_service_client
