
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file viz_service.h

#include <condition_variable>
#include <filesystem>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>

#include <grpcpp/server_context.h>

#include "planning_service/service/visualization/visualizer_hub.h"
#include "proto/visualizer.grpc.pb.h"
#include "utils.h"
namespace fs = std::filesystem;

namespace comms {
template <typename, typename = void>
struct has_conf : std::false_type {};
template <typename T>
struct has_conf<T, std::void_t<decltype(std::declval<T>().conf())>>
    : std::true_type {};
template <typename, typename = void>
struct has_system_conf : std::false_type {};
template <typename T>
struct has_system_conf<T,
                       std::void_t<decltype(std::declval<T>().system_conf())>>
    : std::true_type {};

class VisualizerService : public proto::Visualizer::Service {
 public:
  /**
   * @brief Constructor.
   */
  VisualizerService(
      std::shared_ptr<service::visualization::VisualizerHubInterface> hub)
      : hub_ {hub} {}

  /**
   * @brief Start the visualizer with the model specfiied in the request.
   *
   * @param req Start request
   * @return grpc::Status
   */
  grpc::Status StartVisualizer(grpc::ServerContext*,
                               const proto::StartVisualizerRequest* req,
                               proto::StartVisualizerResponse*);
  /** Stop the visualizer and clear all data. */
  grpc::Status StopVisualizer(grpc::ServerContext*,
                              const proto::StopVisualizerRequest*,
                              proto::StopVisualizerResponse*);
  grpc::Status GetVisualizerStatus(grpc::ServerContext*,
                                   const proto::GetVisualizerStatusRequest*,
                                   proto::GetVisualizerStatusResponse* resp);
  /** Set the model positions. */
  grpc::Status SetConfiguration(grpc::ServerContext*,
                                const proto::SetConfigurationRequest* req,
                                proto::SetConfigurationResponse*);
  /** Stream the model positions. */
  grpc::Status StreamConfigurations(
      grpc::ServerContext*,
      grpc::ServerReader<proto::StreamConfigurationsRequest>* reader,
      proto::StreamConfigurationsResponse*);

  /** Visualize a trajectory. */
  grpc::Status DisplayTrajectory(grpc::ServerContext*,
                                 const proto::DisplayTrajectoryRequest* req,
                                 proto::DisplayTrajectoryResponse*);

  /** Get the pose of a target frame relative to another frame. */
  grpc::Status CalcPose(grpc::ServerContext*, const proto::CalcPoseRequest* req,
                        proto::CalcPoseResponse* resp);

  /** Set an object in the scene. */
  grpc::Status SetObject(grpc::ServerContext*,
                         const proto::SetObjectRequest* req,
                         proto::SetObjectResponse* resp);
  /** Delete an object from the scene. */
  grpc::Status DeleteObject(grpc::ServerContext*,
                            const proto::DeleteObjectRequest* req,
                            proto::DeleteObjectResponse* resp);
  /** Toggle the presence of geometry in the scene. */
  grpc::Status ToggleObject(grpc::ServerContext*,
                            const proto::ToggleObjectRequest* req,
                            proto::ToggleObjectResponse* resp);
  /** Toggle the visibility of a named frame's axes in the scene. */
  grpc::Status ToggleFrame(grpc::ServerContext*,
                           const proto::ToggleFrameRequest* req,
                           proto::ToggleFrameResponse* resp);

 protected:
  grpc::Status DoStreamConfigurations(
      grpc::ServerContext* ctx,
      grpc::ServerReaderInterface<proto::StreamConfigurationsRequest>* reader,
      proto::StreamConfigurationsResponse* resp);

  template <typename T>
  std::enable_if_t<has_conf<T>::value && has_system_conf<T>::value,
                   std::expected<bool, std::string>>
  DoSetConfiguration(const T& req) {
    if (req.has_conf()) {
      const auto conf {psc::FromProto<psc::Conf>(req.conf())};
      return hub_->SetConfiguration(conf.q());
    }
    if (req.has_system_conf()) {
      const auto system_conf {
          psc::FromProto<psc::SystemConf>(req.system_conf())};
      return hub_->SetConfiguration(system_conf);
    }
    return std::unexpected("No configuration provided!");
  }

 private:
  std::shared_ptr<service::visualization::VisualizerHubInterface> hub_;
  const uint16_t viz_status_freq_hz_ {10};

  // Transforms recorded at SetObject time so ToggleObject can restore them.
  mutable std::mutex object_transforms_mutex_;
  std::unordered_map<std::string, drake::math::RigidTransformd>
      object_transforms_;

  mutable std::mutex active_streamer_mutex_;
  std::condition_variable active_streamer_cv_;
  std::string active_streamer_;
  grpc::ServerContext* active_streamer_ctx_ {nullptr};

  /**
   * @brief Resolve a ToggleFrame `frame` field to a canonical Meshcat path.
   *
   * Three forms are accepted:
   *   1. Absolute: must start with "/drake/frames/" — used verbatim.
   *   2. Relative (contains '/'): "frames/model/name" or "model/name" —
   *      "/drake/" or "/drake/frames/" is prepended respectively.
   *   3. Bare name (no '/'): looked up in the loaded model via `hub_`.
   */
  std::optional<std::string> ResolveToggleFramePath(
      std::string_view frame_in) const;
};
}  // namespace comms
