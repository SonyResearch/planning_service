#pragma once

#include "planning_service/motion/robot_model.h"
#include "planning_service/motion/splining/time_optimal_spliner.h"
#include "planning_service_client/conf.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/shape.h"
#include "planning_service_client/trajectories.h"

namespace draco {
namespace conversions {

using system_conf_t = std::map<std::string, Eigen::VectorXd>;

using FrameRelativePoses =
    std::vector<std::tuple<const drake::multibody::Frame<double>*,
                           const drake::multibody::Frame<double>*,
                           const drake::math::RigidTransformd>>;

/** @name System Conversions
 *  Methods related to converting between system configurations and
 * trajectories.
 *  @{
 */

/** Enum to specify behavior when converting to generalized position.
 */
enum ToGeneralizedBehavior {
  /** Throw an exception if a model name is missing. */
  kThrowOnMissing = 0,
  /** Assume zero value if a model name is missing. */
  kAssumeZeroOnMissing = 1,
  /** Complete from reference conf if a model name is missing. */
  kCompleteFromReferenceOnMissing = 2,
  /** Only return the partial positions for that particular arm. */
  kArm = 3,
};

/** Convert a SystemConf to a generalized position.
 * @param system_conf the system configuration to convert
 * @return the generalized position.
 */
Eigen::VectorXd ToGeneralizedPosition(
    const motion::RobotModel& robot_model,
    const planning_service_client::SystemConf& system_conf,
    const ToGeneralizedBehavior& behavior =
        ToGeneralizedBehavior::kThrowOnMissing,
    std::optional<planning_service_client::SystemConf> sysconf_ref_opt =
        std::nullopt);

/** Convert a system timed trajectory to a PathParameterizedTrajectory.
 * @param sys_timed_trajectory the system timed trajectory to convert
 * @return the PathParameterizedTrajectory.
 */
drake::trajectories::PathParameterizedTrajectory<double>
ToPathParameterizedTrajectory(
    const motion::splining::TimeOptimalSpliner& time_optimal_spliner,
    const planning_service_client::SystemTimedTrajectory& sys_timed_trajectory,
    const ToGeneralizedBehavior& behavior =
        ToGeneralizedBehavior::kThrowOnMissing);

/** Convert a PathParameterizedTrajectory to a system timed trajectory.
 * @param ppt the PathParameterizedTrajectory to convert
 * @return the system timed trajectory.
 */
planning_service_client::SystemTimedTrajectory ToSystemTimedTrajectory(
    const motion::splining::TimeOptimalSpliner& time_optimal_spliner,
    const drake::trajectories::PathParameterizedTrajectory<double>& ppt);
/** @} */

drake::trajectories::PiecewisePolynomial<double>
ClientPiecewisePolynomialToDrake(
    const planning_service_client::PiecewisePolynomial& poly);

planning_service_client::PiecewisePolynomial DrakePiecewisePolynomialToClient(
    const drake::trajectories::PiecewisePolynomial<double>& poly);

planning_service_client::SystemConf DracoToClientSystemConf(
    const system_conf_t& sysconf);

system_conf_t ClientToDracoSystemConf(
    const planning_service_client::SystemConf& sysconf_client);

FrameRelativePoses ToDracoFrameRelativePoses(
    const motion::RobotModel& robot_model,
    const std::vector<planning_service_client::FrameRelativePose>& frp_vec);

std::vector<planning_service_client::FrameRelativePose>
ToClientFrameRelativePoses(const motion::RobotModel& robot_model,
                           const FrameRelativePoses& frps);

/** Convert a client Shape to its corresponding Drake Shape. */
std::shared_ptr<drake::geometry::Shape> ToDrakeShape(
    const planning_service_client::Shape& shape);

}  // namespace conversions
}  // namespace draco
