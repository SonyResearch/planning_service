/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file registry_service.cc

#include "registry_service.h"

#include "grpc_utils.h"
#include "planning_service/service/utils/utils.h"
#include "utils.h"

namespace comms {

grpc::Status ContextRegistryService::HandleRegisterPlanContextRequest(
    grpc::ServerContext* ctx, const RegisterPlanContextRequest* req,
    RegisterPlanContextResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  logging::log()->info(
      "CRS:HandleRegisterPlanContextRequest: Received request for system: {}",
      req->system());
  try {
    const auto plan_context {utils::ProtoToPlanContext(req->context())};
    const auto registration_result {
        registry_->RegisterPlanContext(req->system(), plan_context)};
    if (!registration_result) {
      const auto err_msg {fmt::format("Failed to save context to disk! {}",
                                      registration_result.error())};
      logging::log()->error("CRS:HandleRegisterPlanContextRequest: {}",
                            err_msg);
      return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
    }
    logging::log()->info(
        "CRS:HandleRegisterPlanContextRequest: Successfully computed hash: "
        "{}",
        registration_result->value);
    proto::PlanContextId id;
    id.set_value(registration_result->value);
    *resp->mutable_context_id() = id;
    return grpc::Status::OK;
  } catch (const std::exception& e) {
    logging::log()->error(
        "CRS:HandleRegisterPlanContextRequest: Failed due to exception! ({})",
        e.what());
    return grpc::Status(grpc::StatusCode::INTERNAL, e.what());
  }
}

grpc::Status ContextRegistryService::RemovePlanContext(
    grpc::ServerContext* ctx, const proto::RemovePlanContextRequest* req,
    proto::RemovePlanContextResponse*) {
  comms::set_transaction_id_from_context(ctx);
  logging::log()->info(
      "CRS:RemovePlanContext: Received request to remove context ID {} for "
      "system: {}",
      req->context_id().value(), req->system());
  grpc::StatusCode status_code {grpc::StatusCode::OK};
  std::string err_msg {""};
  try {
    const bool erase {true};
    registry_->RemovePlanContext(
        req->system(), draco::PlanContextId(req->context_id().value()), erase);
    logging::log()->info(
        "CRS:RemovePlanContext: Successfully removed context ID {} for system: "
        "{}",
        req->context_id().value(), req->system());
  } catch (const std::exception& e) {
    status_code = grpc::StatusCode::INTERNAL;
    err_msg = fmt::format("Caught exception removing context: {}", e.what());
  }
  if (err_msg.size()) {
    logging::log()->error("CRS:RemovePlanContext: {}", err_msg);
  }
  return grpc::Status(status_code, err_msg);
}

grpc::Status ContextRegistryService::GetVersion(
    grpc::ServerContext* ctx, const proto::GetVersionRequest*,
    proto::GetVersionResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  logging::log()->info(
      "CRS:GetVersion: Received request to get the planning service version");

  grpc::StatusCode status_code {grpc::StatusCode::OK};
  std::string err_msg {""};

  try {
    resp->set_version(registry_->version());
  } catch (const std::exception& e) {
    status_code = grpc::StatusCode::INTERNAL;
    err_msg = fmt::format("Caught exception getting the version: {}", e.what());
  }
  if (err_msg.size()) {
    logging::log()->error("MPS:GetVersion: {}", err_msg);
  }
  return grpc::Status(status_code, err_msg);
}

grpc::Status ContextRegistryService::GetPlanContextSummaries(
    grpc::ServerContext* ctx, const proto::GetPlanContextSummariesRequest*,
    proto::GetPlanContextSummariesResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  // iterate over the context IDs and add them to the response
  for (const auto& [hash, plan_context] : registry_->active_contexts_map()) {
    proto::PlanContextSummary summary;
    summary.set_name(plan_context.name);
    const auto& plant {registry_->GetDraco(hash)->robot_model().plant()};
    for (int i {0}; i < plant.num_model_instances(); ++i) {
      const auto idx {drake::multibody::ModelInstanceIndex(i)};
      const auto num_positions {plant.num_positions(idx)};
      if (num_positions > 0) {
        proto::RobotMetadata robot_metadata;
        robot_metadata.set_num_positions(num_positions);
        for (const auto& joint_idx : plant.GetJointIndices(idx)) {
          const auto& joint {plant.get_joint(joint_idx)};
          if (joint.can_rotate() || joint.can_translate()) {
            *robot_metadata.add_joints() = joint.name();
          }
        }
        (*summary.mutable_robot_summaries())[plant.GetModelInstanceName(idx)] =
            robot_metadata;
      }
    }
    summary.mutable_id()->set_value(hash);
    *resp->add_summaries() = summary;
  }
  return grpc::Status::OK;
}

grpc::Status ContextRegistryService::GetPlanningArtifactStatus(
    grpc::ServerContext* ctx,
    const proto::GetPlanningArtifactStatusRequest* req,
    proto::GetPlanningArtifactStatusResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  for (const auto& context_id_pb : req->context_ids()) {
    const auto context_id {draco::PlanContextId(context_id_pb.value())};
    if (!registry_->HasDraco(context_id)) {
      logging::log()->error(
          "CRS:GetPlanningArtifacStatus:Context ID {} not found!",
          context_id.value);
      continue;
    }
    const auto& draco {registry_->GetDraco(context_id)};
    const auto status {draco->GetPlanningArtifactsSizes()};
    const int& num_vertices {status.num_roadmap_vertices};
    const int& num_edges {status.num_roadmap_edges};
    const int& num_regions {status.num_iris_regions};
    logging::log()->info(
        "CRS:GetPlanningArtifactStatus: Context ID {} has {} vertices, {} "
        "edges, and {} regions.",
        context_id.value, num_vertices, num_edges, num_regions);
    // PlanningArtifactSizesToProto
    auto artifact_status_pb {utils::PlanningArtifactSizesToProto(
        num_vertices, num_edges, num_regions)};
    (*resp->mutable_artifacts_status_map())[context_id.value] =
        artifact_status_pb;
  }
  return grpc::Status::OK;
}

grpc::Status ContextRegistryService::HandleMigratePlanningArtifactsRequest(
    grpc::ServerContext* ctx, const proto::MigratePlanningArtifactsRequest* req,
    proto::MigratePlanningArtifactsResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  logging::log()->info(
      "IBSI:HandleMigratePlanningArtifactsRequest: Received request to migrate "
      "planning artifacts");
  grpc::StatusCode status_code {grpc::StatusCode::OK};
  std::string err_msg {""};
  try {
    const auto req_adapter {
        utils::ProtoToMigratePlanningArtifactsRequestAdapter(req)};
    const auto result {registry_->MigratePlanningArtifacts(
        req_adapter.from_context_id, req_adapter.to_context_id,
        req_adapter.num_samples, req_adapter.repair_artifacts)};
    if (!result) {
      err_msg = result.error();
      status_code = grpc::StatusCode::INTERNAL;
    }
    resp->set_id(req->id());
  } catch (const std::exception& e) {
    status_code = grpc::StatusCode::INTERNAL;
    err_msg = fmt::format("Caught exception queuing request: {}", e.what());
  }
  if (err_msg.size()) {
    logging::log()->error("IBSI:HandleMigratePlanningArtifactsRequest: {}",
                          err_msg);
  }
  return grpc::Status(status_code, err_msg);
}
}  // namespace comms
