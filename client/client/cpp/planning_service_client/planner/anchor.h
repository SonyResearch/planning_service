#pragma once

#include <Eigen/Dense>

#include "planning_service_client/conf.h"
#include "planning_service_client/frame_relative_pose.h"

namespace planning_service_client {
namespace planner {

/** An anchor is a set of system configurations and frame relative poses that
 * are used to define a fixed reference for a planning problem to start from,
 * follow through, or end at. */
class Anchor : public internal::ProtoBase<proto::Anchor> {
 public:
  /** The default constructor. */
  Anchor() = default;

  /** Constructs an anchor with a system configuration and a set of frame
   * relative poses.
   * @param system_conf The system configuration.
   * @param wayposes The frame relative poses.
   */
  explicit Anchor(const SystemConf& system_conf,
                  const std::vector<FrameRelativePose>& wayposes = {});

  /** Getters for system_conf */
  const SystemConf& system_conf() const {
    return system_conf_;
  }

  /** Getters for poses */
  const std::vector<FrameRelativePose>& poses() const {
    return poses_;
  }

 private:
  proto::Anchor ToProtoImpl() const override;

  void FromProtoImpl(const proto::Anchor& msg) override;

  SystemConf system_conf_;

  std::vector<FrameRelativePose> poses_;
};

}  // namespace planner
}  // namespace planning_service_client
