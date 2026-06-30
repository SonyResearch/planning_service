/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file visualizer_client.h

#pragma once
#include <condition_variable>
#include <optional>
#include <queue>
#include <thread>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

#include "planning_service_client/api/client.h"
#include "planning_service_client/conf.h"
#include "planning_service_client/context_id.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/rgba.h"
#include "planning_service_client/shape.h"
#include "planning_service_client/visualizer_status.h"
#include "proto/visualizer.grpc.pb.h"

namespace planning_service_client {
namespace client {

/**
 * @brief Minimal implementation of a gRPC client which may interact with the
 * visualizer service to display, in Meshcat, a selected model and associated
 * planning artifacts (e.g., IRIS regions).
 * For detailed documentation of the API and message types, please see:
 * https://github.com/SonyResearch/planning_service_client/proto/visualizer.proto
 *
 * For interacting with objects in a live Meshcat session, this API uses
 * Meshcat's "scene tree" to reference elements by path; you can find more
 * information about this here:
 *
 * https://drake.mit.edu/doxygen_cxx/classdrake_1_1geometry_1_1_meshcat.html#meshcat_path
 */
class VisualizerClient : public ClientInterface<proto::Visualizer> {
 public:
  /**
   * @brief Constructor.
   *
   * @param addr Address to which new requests will be sent.
   * @param config_json JSON string specifying configuration options. See
   * https://github.com/grpc/proposal/blob/master/A6-client-retries.md for an
   * example on usage.
   */
  VisualizerClient(const std::string_view addr,
                   const std::string_view client_id,
                   const std::string_view config_json = "{}")
      : ClientInterface(addr, client_id, config_json) {}

  ~VisualizerClient() {
    StopStreamConfigurations();
    if (stream_positions_thread_.joinable()) {
      stream_positions_thread_.join();
    }
  }

 public:
  /**
   * @brief Start a visualizer session for a model by providing its unique ID.
   *
   * @param context_id ID of desired model.
   * @param force_reload When true, reload a new session even if one is already
   * running.
   * @return grpc::Status
   */
  void StartVisualizer(const planning_service_client::ContextId& context_id,
                       const bool force_reload = false) const;

  /** Stop the current visualizer session. */
  void StopVisualizer() const;

  /** Get the current status of the visualizer. */
  VisualizerStatus GetVisualizerStatus() const;

  /**
   * @brief Set an object rooted at the given path in the Meshcat visualizer
   * session, with the provided shape, pose, and color.
   *
   * @param path The Meshcat path of the object to set.
   * @param shape_in_frame The shape and its frame information.
   * @param color The color of the object.
   */
  void SetObject(const std::string_view path,
                 const ShapeInFrame& shape_in_frame,
                 const Rgba& color = Rgba::White()) const;

  /**
   * @brief Delete a given object from the Meshcat visualizer session.
   *
   * @param path The Meshcat path of the object to delete.
   */
  void DeleteObject(const std::string_view path) const;

  /**
   * @brief Toggle the visibility of a given object in the Meshcat visualizer
   * session.
   *
   * @param path The Meshcat path of the object to toggle.
   * @param visible If true, make the object visible; if false, hide it.
   */
  void ToggleObject(const std::string_view path, bool visible) const;

  /**
   * @brief Toggle the visibility of a given frame's axes in the Meshcat
   * visualizer session.
   *
   * @param frame The Meshcat path of the frame to toggle.
   * @param visible If true, make the frame axes visible; if false, hide them.
   */
  void ToggleFrame(const std::string_view frame, bool visible) const;

  /**
   * @brief Calculate the pose of frame B relative to frame A in the visualizer
   * session.
   *
   * @param frame_a The name of the reference frame A.
   * @param frame_b The name of the target frame B.
   * @param system_conf_override Optional override for the system configuration
   * to use when calculating the pose; if not provided, the visualizer will use
   * the most recently streamed configuration.
   * @return The pose of frame B relative to frame A.
   */
  FrameRelativePose CalcPose(const std::string_view frame_a,
                             const std::string_view frame_b,
                             const std::optional<SystemConf>&
                                 system_conf_override = std::nullopt) const;

  /** Queue a configuration to be streamed to the visualizer. */
  void QueueStreamedConfiguration(const SystemConf& system_conf) {
    {
      std::lock_guard<std::mutex> lock(queue_mtx_);
      config_queue_.push(system_conf);
    }
    queue_cv_.notify_one();
  }
  /**
   * @brief Stream configurations to the visualizer.
   *
   * This method waits on queued configurations and streams them to the
   * visualizer service; it will run (blocking) until StopStreamConfigurations()
   * is called.
   */
  void StreamConfigurations();

  /** Start streaming of configurations to the visualizer asynchronously. */
  void StreamConfigurationsAsync() {
    stream_positions_thread_ = std::thread([this]() {
      try {
        this->StreamConfigurations();
      } catch (const std::exception& e) {
        std::cerr << "[" << name_
                  << "] StreamConfigurations thread "
                     "terminated with exception: "
                  << e.what() << std::endl;
      }
    });
  }

  /** Stop streaming of configurations to the visualizer. */
  void StopStreamConfigurations() {
    stream_positions_stop_requested_ = true;
    queue_cv_.notify_one();
  }

 protected:
  // Note: channel_/stub_/client_id_ are owned by ClientInterface.
  std::atomic<bool> stream_positions_stop_requested_ {false};
  std::thread stream_positions_thread_;
  std::queue<SystemConf> config_queue_;
  std::mutex queue_mtx_;
  std::condition_variable queue_cv_;
  double stream_freq_hz_ {100};
};
}  // namespace client
}  // namespace planning_service_client
