
/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file viz_service.cc
#include "viz_service.h"

#include <drake/common/overloaded.h>
#include <drake/geometry/rgba.h>
#include <drake/geometry/shape_specification.h>

#include <chrono>

#include "grpc_utils.h"
#include "planning_service/draco/client_conversions.h"
#include "planning_service_client/rgba.h"
#include "planning_service_client/shape.h"

namespace psc = planning_service_client;

namespace comms {

std::optional<std::string> VisualizerService::ResolveToggleFramePath(
    std::string_view frame_in) const {
  std::string path;

  if (frame_in.starts_with("/")) {
    // Case 1 — absolute path; must be in the frames subtree.
    if (!frame_in.starts_with("/drake/frames/")) {
      return std::nullopt;
    }
    path = std::string {frame_in};
  } else if (frame_in.find('/') != std::string::npos) {
    // Case 2 — relative path.
    if (frame_in.starts_with("frames/")) {
      path = fmt::format("/drake/{}", frame_in);
    } else {
      path = fmt::format("/drake/frames/{}", frame_in);
    }
  } else {
    // Case 3 — bare frame name; resolve via the loaded model.
    const auto resolved {hub_->ResolveFrameMeshcatPath(frame_in)};
    if (!resolved) {
      return std::nullopt;
    }
    path = *resolved;
  }

  // Normalise: the frame axes parent node in Meshcat always ends with '/'.
  if (!path.ends_with('/')) {
    path += '/';
  }
  return path;
}

grpc::Status VisualizerService::StartVisualizer(
    grpc::ServerContext* ctx, const proto::StartVisualizerRequest* req,
    proto::StartVisualizerResponse*) {
  comms::set_transaction_id_from_context(ctx);
  try {
    // set options — start from the disk-loaded defaults, if available
    auto options = hub_->GetVisualizerOptions();
    options.meshcat_params.visual = true;
    options.enable_sliders = req->options().enable_sliders();
    options.show_iris_regions = req->options().show_iris_regions();
    options.show_prm = req->options().show_prm();
    hub_->SetVisualizerOptions(options);
    // populate data for the visualized model
    service::visualization::VisualizerData viz_data;
    if (req->has_meshcat_params()) {
      logging::log()->info(
          "VS:StartVisualizer: Parsing received meshcat parameters");
      viz_data.robot_meshcat_params =
          utils::ProtoToMeshcatParams(req->meshcat_params());
    }
    if (req->dmd_filename().empty()) {
      viz_data.context_id = draco::PlanContextId(req->context_id().value());
    } else {
      viz_data.dmd_filename = req->dmd_filename();
    }
    if (!hub_->LoadAndSetModelData(viz_data, req->force_reload())) {
      const auto err_msg {"Failed to load robot model from request!"};
      logging::log()->error("VS:StartVisualizer: {}", err_msg);
      return grpc::Status(grpc::StatusCode::RESOURCE_EXHAUSTED, err_msg);
    }
  } catch (const std::exception& e) {
    const auto err_msg {fmt::format(
        "Failed to load model data due to exception: {}", e.what())};
    logging::log()->error("VS:StartVisualizer: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  hub_->WaitOnStatus(service::visualization::VisualizerStatus::Starting);
  logging::log()->info(
      "VS:StartVisualizer: Successfully loaded Draco with hash: {}",
      hub_->ActiveHash());
  return grpc::Status::OK;
}

grpc::Status VisualizerService::StopVisualizer(
    grpc::ServerContext* ctx, const proto::StopVisualizerRequest*,
    proto::StopVisualizerResponse*) {
  comms::set_transaction_id_from_context(ctx);
  logging::log()->info(
      "VS:StopVisualizer: Received request to stop visualizer");
  hub_->RequestStop();
  hub_->WaitOnStatus(service::visualization::VisualizerStatus::Stopping);
  return grpc::Status::OK;
}

grpc::Status VisualizerService::GetVisualizerStatus(
    grpc::ServerContext* ctx, const proto::GetVisualizerStatusRequest*,
    proto::GetVisualizerStatusResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  const auto status = hub_->GetStatus();
  // parse from name
  const auto status_str {
      fmt::format("VISUALIZER_STATUS_{}",
                  common::utils::to_upper(magic_enum::enum_name(status)))};
  proto::VisualizerStatus proto_status;
  const auto parse_result {
      proto::VisualizerStatus_Parse(status_str, &proto_status)};
  if (!parse_result) {
    const auto err_msg {fmt::format(
        "Failed to parse visualizer status from string: {}", status_str)};
    logging::log()->error("VS:GetVisualizerStatus: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  resp->set_status(proto_status);
  return grpc::Status::OK;
}

grpc::Status VisualizerService::SetConfiguration(
    grpc::ServerContext* ctx, const proto::SetConfigurationRequest* req,
    proto::SetConfigurationResponse*) {
  comms::set_transaction_id_from_context(ctx);
  if (const auto set_result {DoSetConfiguration(*req)}; !set_result) {
    return grpc::Status(
        grpc::StatusCode::INTERNAL,
        fmt::format("Failed to set model positions ({})", set_result.error()));
  }
  return grpc::Status::OK;
}

grpc::Status VisualizerService::StreamConfigurations(
    grpc::ServerContext* ctx,
    grpc::ServerReader<proto::StreamConfigurationsRequest>* reader,
    proto::StreamConfigurationsResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  return DoStreamConfigurations(ctx, reader, resp);
}

grpc::Status VisualizerService::DoStreamConfigurations(
    grpc::ServerContext* ctx,
    grpc::ServerReaderInterface<proto::StreamConfigurationsRequest>* reader,
    proto::StreamConfigurationsResponse*) {
  const auto peer = ctx->peer();
  {
    std::unique_lock lock(active_streamer_mutex_);
    if (active_streamer_ctx_ != nullptr) {
      logging::log()->warn(
          "VS:StreamConfigurations: {} is already streaming — dropping it in "
          "favour of new connection from {}",
          active_streamer_, peer);
      // Issue cancellation while holding the mutex to ensure the incumbent
      // handler's ServerContext remains valid for the duration of TryCancel().
      grpc::ServerContext* ctx_to_cancel = active_streamer_ctx_;
      ctx_to_cancel->TryCancel();
      // Wait for the old handler to vacate the slot; the wait will
      // temporarily release active_streamer_mutex_ while blocking.
      // Use a bounded wait: if the incumbent does not vacate within the
      // timeout (e.g. stuck inside Read()), forcibly take over the slot.
      constexpr auto kStreamerEvictTimeout = std::chrono::seconds(5);
      if (!active_streamer_cv_.wait_for(lock, kStreamerEvictTimeout, [this] {
            return active_streamer_.empty();
          })) {
        logging::log()->error(
            "VS:StreamConfigurations: Incumbent streamer {} did not vacate "
            "within {}s after cancellation; forcibly taking over slot for {}",
            active_streamer_, kStreamerEvictTimeout.count(), peer);
        active_streamer_.clear();
        active_streamer_ctx_ = nullptr;
      }
    }
    active_streamer_ = peer;
    active_streamer_ctx_ = ctx;
    logging::log()->info("VS:StreamConfigurations: Client connected: {}", peer);
  }

  proto::StreamConfigurationsRequest req;
  grpc::Status result = grpc::Status::OK;
  while (reader->Read(&req)) {
    if (hub_->GetStatus() != service::visualization::VisualizerStatus::Active) {
      logging::log()->debug(
          "VS:StreamConfigurations: Hub not yet active; dropping packet from "
          "{}",
          peer);
      continue;
    }
    if (const auto set_result {DoSetConfiguration(req)}; !set_result) {
      result = grpc::Status(grpc::StatusCode::INTERNAL,
                            fmt::format("Failed to set model positions ({})",
                                        set_result.error()));
      break;
    }
  }

  {
    std::lock_guard lock(active_streamer_mutex_);
    active_streamer_.clear();
    active_streamer_ctx_ = nullptr;
    if (ctx->IsCancelled()) {
      logging::log()->warn(
          "VS:StreamConfigurations: Client lost (cancelled/dead): {}", peer);
    } else {
      logging::log()->info(
          "VS:StreamConfigurations: Client disconnected cleanly: {}", peer);
    }
    hub_->ClearStreamingState();
  }
  active_streamer_cv_.notify_all();
  return result;
}

grpc::Status VisualizerService::CalcPose(grpc::ServerContext* ctx,
                                         const proto::CalcPoseRequest* req,
                                         proto::CalcPoseResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  try {
    const auto& frame_A {req->frame_a().empty() ? "world" : req->frame_a()};
    const auto& frame_B {req->frame_b().empty() ? "world" : req->frame_b()};
    std::optional<psc::SystemConf> system_conf_override;
    if (req->has_system_conf_override()) {
      system_conf_override =
          psc::FromProto<psc::SystemConf>(req->system_conf_override());
    }
    const auto X_AB {hub_->CalcPose(frame_B, frame_A, system_conf_override)};
    const psc::FrameRelativePose frp {frame_A, frame_B, X_AB.translation(),
                                      X_AB.rotation().ToQuaternion()};
    resp->mutable_pose()->CopyFrom(ToProto(frp));
  } catch (const std::exception& e) {
    const auto err_msg {fmt::format(
        "Failed to compute pose due to error! Exception: {}", e.what())};
    logging::log()->error("VS:CalcPose: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  return grpc::Status::OK;
}

grpc::Status VisualizerService::DisplayTrajectory(
    grpc::ServerContext* ctx, const proto::DisplayTrajectoryRequest*,
    proto::DisplayTrajectoryResponse*) {
  comms::set_transaction_id_from_context(ctx);
  return grpc::Status(grpc::StatusCode::UNIMPLEMENTED,
                      "DisplayTrajectory is not implemented yet!");
}

grpc::Status VisualizerService::SetObject(grpc::ServerContext* ctx,
                                          const proto::SetObjectRequest* req,
                                          proto::SetObjectResponse*) {
  comms::set_transaction_id_from_context(ctx);
  auto path {req->path()};
  const std::string meshcat_root {"/drake/"};
  if (path.starts_with(meshcat_root)) {
    path = path.substr(meshcat_root.size());
  }
  if (path.starts_with("collision/") || path.starts_with("visual/")) {
    const auto err_msg {
        fmt::format("SetObject: Refusing to set path: {}. Modification of "
                    "collision or visual elements is not allowed for safety "
                    "reasons. To toggle visibility, use ToggleObject RPC.",
                    path)};
    logging::log()->error("VS:SetObject: {}", err_msg);
    return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, err_msg);
  }
  if (!path.starts_with("objects/")) {
    path = fmt::format("objects/{}", path);
  }
  const auto shape_in_frame {
      psc::FromProto<psc::ShapeInFrame>(req->shape_in_frame())};
  const auto color {psc::FromProto<psc::Rgba>(req->color())};

  std::shared_ptr<drake::geometry::Shape> drake_shape;
  try {
    drake_shape = draco::conversions::ToDrakeShape(shape_in_frame.shape());
  } catch (const std::exception& e) {
    const auto err_msg {
        fmt::format("SetObject: Failed to build shape: {}", e.what())};
    logging::log()->error("VS:SetObject: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err_msg);
  }

  try {
    // If frame is empty, treat as world.
    const auto& frame {shape_in_frame.frame().empty() ? "world"
                                                      : shape_in_frame.frame()};
    const drake::math::RigidTransformd X_frame_object {
        shape_in_frame.quaternion(), shape_in_frame.translation()};
    const drake::geometry::Rgba drake_color {color.r(), color.g(), color.b(),
                                             color.a()};
    const auto X_WO {hub_->CalcPose(frame, "world") * X_frame_object};
    {
      std::lock_guard lock(object_transforms_mutex_);
      object_transforms_[path] = X_WO;
    }
    hub_->QueueMeshcatTask([path, drake_shape, X_WO,
                            drake_color](drake::geometry::Meshcat& meshcat) {
      logging::log()->debug("VS:SetObject: Setting {} at path: {}",
                            drake_shape->type_name(), path);
      meshcat.SetObject(path, *drake_shape, drake_color);
      meshcat.SetTransform(path, X_WO);
    });
  } catch (const std::exception& e) {
    const auto err_msg {fmt::format(
        "SetObject: Failed to set object in Meshcat: {}", e.what())};
    logging::log()->error("VS:SetObject: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }

  return grpc::Status::OK;
}

grpc::Status VisualizerService::DeleteObject(
    grpc::ServerContext* ctx, const proto::DeleteObjectRequest* req,
    proto::DeleteObjectResponse*) {
  comms::set_transaction_id_from_context(ctx);
  auto path {req->path()};
  const std::string meshcat_root {"/drake/"};
  if (path.starts_with(meshcat_root)) {
    path = path.substr(meshcat_root.size());
  }
  if (path.starts_with("collision/") || path.starts_with("visual/")) {
    const auto err_msg {
        fmt::format("DeleteObject: Refusing to delete path: {}. Deletion of "
                    "collision or visual elements is not allowed for safety "
                    "reasons. To toggle visibility, use ToggleObject RPC.",
                    path)};
    logging::log()->error("VS:DeleteObject: {}", err_msg);
    return grpc::Status(grpc::StatusCode::PERMISSION_DENIED, err_msg);
  }
  if (path.empty()) {
    const auto err_msg {"DeleteObject: Cannot delete an empty path"};
    logging::log()->error("VS:DeleteObject: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err_msg);
  }
  if (!path.starts_with("objects/")) {
    path = fmt::format("objects/{}", path);
  }
  {
    std::lock_guard lock(object_transforms_mutex_);
    object_transforms_.erase(path);
  }
  hub_->QueueMeshcatTask([path](drake::geometry::Meshcat& meshcat) {
    meshcat.Delete(path);
  });
  return grpc::Status::OK;
}

grpc::Status VisualizerService::ToggleObject(
    grpc::ServerContext* ctx, const proto::ToggleObjectRequest* req,
    proto::ToggleObjectResponse*) {
  comms::set_transaction_id_from_context(ctx);
  auto path {req->path()};
  const auto& visible {req->visible()};
  // geometry elements are typically arranged under collision and visual
  // subtrees respectively, but it's a useful shorthand to toggle both at once
  const std::string meshcat_root {"/drake/"};
  if (path.starts_with(meshcat_root)) {
    path = path.substr(meshcat_root.size());
  }

  // Factory to hide/show geometry by teleporting it. SetTransform is used
  // (not SetProperty("visible")) because SetTransform state is authoritative
  // and replayed to reconnecting browsers, whereas SetProperty is ephemeral.
  //
  // Note: no conflict with UpdateFrameAxes — geometry lives under
  // /drake/collision/... and /drake/visual/..., frame axes under
  // /drake/frames/... — the path trees are disjoint.
  static const drake::math::RigidTransformd kHidden {
      Eigen::Translation3d(0, 0, -1e6)};
  const auto make_set_transform_task {
      [](std::string meshcat_path, drake::math::RigidTransformd X_WO)
          -> service::visualization::meshcat_task_t {
        return [meshcat_path = std::move(meshcat_path),
                X_WO](drake::geometry::Meshcat& meshcat) -> void {
          if (!meshcat.HasPath(meshcat_path)) {
            logging::log()->debug(
                "VS:ToggleObject: Path {} does not exist in Meshcat!",
                meshcat_path);
            return;
          }
          meshcat.SetTransform(meshcat_path, X_WO);
        };
      }};

  // const copy
  const auto geo_path {path};
  // Objects placed via SetObject live under the `objects/` subtree — toggle
  // directly without collision/visual fan-out or frame-axes updates.
  if (geo_path.starts_with("objects/")) {
    logging::log()->debug("VS:ToggleObject: Toggling objects path: {}",
                          geo_path);
    std::lock_guard lock(object_transforms_mutex_);
    const auto it = object_transforms_.find(geo_path);
    // Restore to the recorded transform when showing; hide by teleporting.
    // If visible and no stored transform exists, skip (nothing to restore).
    if (!visible) {
      hub_->QueueMeshcatTask(make_set_transform_task(geo_path, kHidden));
    } else if (it != object_transforms_.end()) {
      hub_->QueueMeshcatTask(make_set_transform_task(geo_path, it->second));
    }
    return grpc::Status::OK;
  }
  // If not a 'collision' or 'visual' path, toggle both
  if (!(geo_path.starts_with("collision/")
        || geo_path.starts_with("visual/"))) {
    logging::log()->debug(
        "VS:ToggleObject: Toggling both collision and visual {} for path: {}",
        visible ? "ON" : "OFF", geo_path);
    for (const auto& subtree : {"collision", "visual"}) {
      const auto full_path {fmt::format("{}/{}", subtree, geo_path)};
      hub_->QueueMeshcatTask(make_set_transform_task(
          full_path, visible ? drake::math::RigidTransformd {} : kHidden));
    }
  } else {
    logging::log()->debug("VS:ToggleObject: Toggling path: {}", geo_path);
    hub_->QueueMeshcatTask(make_set_transform_task(
        geo_path, visible ? drake::math::RigidTransformd {} : kHidden));
  }

  // Frames are strictly understood as *visual* elements.
  if (!path.starts_with("collision/")) {
    if (path.starts_with("visual/")) {
      path = path.substr(std::string("visual/").size());
    }
    if (const auto frame_updates {hub_->ToggleFramesByPath(path, visible)};
        !frame_updates.empty()) {
      // Capture by value: frame_updates is a local vector and the lambda runs
      // on the visualizer thread after this stack frame is gone.
      hub_->QueueMeshcatTask(
          [updates = frame_updates](drake::geometry::Meshcat& meshcat) {
            for (const auto& [p, X_WF] : updates) {
              meshcat.SetTransform(p, X_WF);
            }
          });
    }
  }
  return grpc::Status::OK;
}

grpc::Status VisualizerService::ToggleFrame(
    grpc::ServerContext* ctx, const proto::ToggleFrameRequest* req,
    proto::ToggleFrameResponse*) {
  comms::set_transaction_id_from_context(ctx);
  if (req->frame().empty()) {
    const auto err_msg {"ToggleFrame: Frame name cannot be empty"};
    logging::log()->error("VS:ToggleFrame: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err_msg);
  }

  const auto path {ResolveToggleFramePath(req->frame())};
  if (!path) {
    const auto err_msg {
        fmt::format("ToggleFrame: Could not resolve frame '{}'", req->frame())};
    logging::log()->error("VS:ToggleFrame: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err_msg);
  }

  const auto& visible {req->visible()};
  logging::log()->debug("VS:ToggleFrame: Setting '{}' visibility to {}", *path,
                        visible ? "ON" : "OFF");
  hub_->ToggleFrame(*path, visible);
  return grpc::Status::OK;
}
}  // namespace comms
