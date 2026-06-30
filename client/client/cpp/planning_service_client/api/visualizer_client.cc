/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file visualizer_client.cc

#include "planning_service_client/api/visualizer_client.h"

#include <Eigen/Dense>

namespace planning_service_client {
namespace client {

void VisualizerClient::StartVisualizer(
    const planning_service_client::ContextId& context_id,
    const bool force_reload) const {
  proto::StartVisualizerRequest req;
  req.mutable_context_id()->set_value(context_id.value());
  req.set_force_reload(force_reload);
  grpc::ClientContext context;
  SetClientContextDeadline(context);
  proto::StartVisualizerResponse resp;
  const auto status {stub_->StartVisualizer(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("StartVisualizer failed: "
                             + status.error_message());
  }
}

void VisualizerClient::StopVisualizer() const {
  grpc::ClientContext context;
  SetClientContextDeadline(context);
  proto::StopVisualizerRequest req;
  proto::StopVisualizerResponse resp;
  const auto status {stub_->StopVisualizer(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("StopVisualizer failed: "
                             + status.error_message());
  }
}

VisualizerStatus VisualizerClient::GetVisualizerStatus() const {
  grpc::ClientContext context;
  SetClientContextDeadline(context);
  proto::GetVisualizerStatusRequest req;
  proto::GetVisualizerStatusResponse resp;
  const auto status {stub_->GetVisualizerStatus(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("GetVisualizerStatus failed: "
                             + status.error_message());
  }
  return VisualizerStatus(static_cast<VisualizerStatus::Status>(resp.status()),
                          resp.details());
}

void VisualizerClient::SetObject(const std::string_view path,
                                 const ShapeInFrame& shape_in_frame,
                                 const Rgba& color) const {
  grpc::ClientContext context;
  proto::SetObjectRequest req;
  req.set_path(path);
  *req.mutable_shape_in_frame() = ToProto(shape_in_frame);
  *req.mutable_color() = ToProto(color);
  proto::SetObjectResponse resp;
  const auto status {stub_->SetObject(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("SetObject failed: " + status.error_message());
  }
}

void VisualizerClient::DeleteObject(const std::string_view path) const {
  grpc::ClientContext context;
  proto::DeleteObjectRequest req;
  req.set_path(path);
  proto::DeleteObjectResponse resp;
  const auto status {stub_->DeleteObject(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("DeleteObject failed: " + status.error_message());
  }
}

void VisualizerClient::ToggleObject(const std::string_view path,
                                    bool visible) const {
  grpc::ClientContext context;
  SetClientContextDeadline(context);
  proto::ToggleObjectRequest req;
  req.set_path(path);
  req.set_visible(visible);
  proto::ToggleObjectResponse resp;
  const auto status {stub_->ToggleObject(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("ToggleObject failed: " + status.error_message());
  }
}

void VisualizerClient::ToggleFrame(const std::string_view frame,
                                   bool visible) const {
  grpc::ClientContext context;
  SetClientContextDeadline(context);
  proto::ToggleFrameRequest req;
  req.set_frame(frame);
  req.set_visible(visible);
  proto::ToggleFrameResponse resp;
  const auto status {stub_->ToggleFrame(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("ToggleFrame failed: " + status.error_message());
  }
}

void VisualizerClient::StreamConfigurations() {
  grpc::ClientContext context;
  proto::StreamConfigurationsResponse resp;
  auto writer = stub_->StreamConfigurations(&context, &resp);
  while (!stream_positions_stop_requested_) {
    std::unique_lock<std::mutex> lock(queue_mtx_);
    queue_cv_.wait(lock, [this] {
      return !config_queue_.empty() || stream_positions_stop_requested_.load();
    });
    while (!config_queue_.empty()) {
      proto::StreamConfigurationsRequest req;
      req.mutable_system_conf()->CopyFrom(ToProto(config_queue_.front()));
      config_queue_.pop();
      lock.unlock();
      if (!writer->Write(req)) break;
      lock.lock();
    }
  }
  writer->WritesDone();
  const auto status = writer->Finish();
  if (!status.ok()) {
    throw std::runtime_error("StreamConfigurations failed: "
                             + status.error_message());
  }
}

FrameRelativePose VisualizerClient::CalcPose(
    const std::string_view frame_a, const std::string_view frame_b,
    const std::optional<SystemConf>& system_conf_override) const {
  proto::CalcPoseRequest req;
  req.set_frame_a(frame_a);
  req.set_frame_b(frame_b);
  if (system_conf_override.has_value()) {
    req.mutable_system_conf_override()->CopyFrom(
        ToProto(system_conf_override.value()));
  }
  grpc::ClientContext context;
  SetClientContextDeadline(context);
  proto::CalcPoseResponse resp;
  const auto status {stub_->CalcPose(&context, req, &resp)};
  if (!status.ok()) {
    throw std::runtime_error("CalcPose failed: " + status.error_message());
  }
  return FromProto<FrameRelativePose>(resp.pose());
}

}  // namespace client
}  // namespace planning_service_client
