/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file iris_service.cc

#include "iris_service.h"

#include "grpc_utils.h"
#include "utils.h"

namespace comms {

grpc::Status IrisBuilderService::HandleStartBuildRequest(
    grpc::ServerContext* ctx, const StartBuildRequest* req,
    StartBuildResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  logging::log()->info(
      "IBSI:HandleStartBuildRequest: Received request {} to start IRIS "
      "generation",
      req->id());
  grpc::StatusCode status_code {grpc::StatusCode::OK};
  std::string err_msg {""};
  try {
    const auto req_adapter {utils::ProtoToIrisBuildRequestAdapter(req)};
    resp->set_id(req->id());
    if (const auto result {mgr_->QueueRequest(req_adapter)}; !result) {
      status_code = grpc::StatusCode::RESOURCE_EXHAUSTED;
      err_msg =
          fmt::format("Failed to queue request with error {}", result.error());
    }
  } catch (const std::exception& e) {
    status_code = grpc::StatusCode::INTERNAL;
    err_msg = fmt::format("Caught exception queuing request: {}", e.what());
  }
  if (err_msg.size()) {
    logging::log()->error("IBSI:HandleStartBuildRequest: {}", err_msg);
  }
  return grpc::Status(status_code, err_msg);
}

grpc::Status IrisBuilderService::HandleUpdateRoadmapRequest(
    grpc::ServerContext* ctx, const UpdateRoadmapRequest* req,
    UpdateRoadmapResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  logging::log()->info(
      "IBSI:HandleUpdateRoadmapRequest: Received request to update roadmap "
      "with ID {}",
      req->id());
  grpc::StatusCode status_code {grpc::StatusCode::OK};
  std::string err_msg {""};
  try {
    const auto req_adapter {utils::ProtoToUpdateRoadmapRequestAdapter(req)};
    logging::log()->info("IBSI:HandleUpdateRoadmapRequest: Job type: {}",
                         magic_enum::enum_name(req_adapter.job_type));
    if (!mgr_->UpdateRoadmap(req_adapter.context.id.value(), req_adapter)) {
      status_code = grpc::StatusCode::RESOURCE_EXHAUSTED;
      err_msg = "Failed to update roadmap!";
    }
    resp->set_id(req->id());
  } catch (const std::exception& e) {
    status_code = grpc::StatusCode::INTERNAL;
    err_msg = fmt::format("Caught exception queuing request: {}", e.what());
  }
  if (err_msg.size()) {
    logging::log()->error("IBSI:HandleUpdateRoadmapRequest: {}", err_msg);
  }
  return grpc::Status(status_code, err_msg);
}

// TODO(@davebambrick): implement
grpc::Status IrisBuilderService::HandleReportStatusRequest(
    grpc::ServerContext* ctx, const ReportBuildStatusRequest*,
    ReportBuildStatusResponse*) {
  comms::set_transaction_id_from_context(ctx);
  return grpc::Status(grpc::StatusCode::UNIMPLEMENTED,
                      "Method HandleReportStatusRequest is not implemented!");
}
}  // namespace comms
