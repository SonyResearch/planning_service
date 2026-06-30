/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file registry_service.h
#include <filesystem>

#include <grpcpp/server_context.h>

#include "planning_service/service/utils/resource_registry.h"
#include "proto/registry.grpc.pb.h"
#include "utils.h"

namespace fs = std::filesystem;

// registry message types
using proto::RegisterPlanContextRequest;
using proto::RegisterPlanContextResponse;
namespace comms {
/**
 * @brief Implementation of the gRPC service PlanContextRegistry (declared in
 * the `planning_service_client` repository) which generates a unique hash for a
 * given planning context.
 */

class ContextRegistryService final
    : public proto::PlanContextRegistry::Service {
 public:
  /**
   * @brief Constructor.
   * @param registry Resource manager
   */
  ContextRegistryService(
      std::shared_ptr<service::utils::ResourceRegistry> registry)
      : registry_ {registry} {}
  /**
   * @brief Given a RegisterPlanContextRequest, compute the unique ID for
   * the given plan context and return to the client.
   *
   * @param req pointer to request message containing all information
   * uniquely defining a given plan context
   * @param resp pointer to response message which will contain hash
   *
   * @return status code and message to be handled internally
   */
  grpc::Status HandleRegisterPlanContextRequest(
      grpc::ServerContext*, const RegisterPlanContextRequest* req,
      RegisterPlanContextResponse* resp);

  /** Remove a planning context and its associated model/planner instance. */
  grpc::Status RemovePlanContext(grpc::ServerContext*,
                                 const proto::RemovePlanContextRequest* req,
                                 proto::RemovePlanContextResponse* resp);
  /**
   * @brief Returns the version of the planning service.
   *
   * @param req pointer to request message (empty)
   * @param resp pointer to response message which will contain the version
   * @return grpc::Status status code and message to be handled internally
   */
  grpc::Status GetVersion(grpc::ServerContext*, const proto::GetVersionRequest*,
                          proto::GetVersionResponse* resp);

  grpc::Status GetPlanContextSummaries(
      grpc::ServerContext*, const proto::GetPlanContextSummariesRequest* req,
      proto::GetPlanContextSummariesResponse* resp);

  grpc::Status GetPlanningArtifactStatus(
      grpc::ServerContext*, const proto::GetPlanningArtifactStatusRequest* req,
      proto::GetPlanningArtifactStatusResponse* resp);

  grpc::Status HandleMigratePlanningArtifactsRequest(
      grpc::ServerContext*, const proto::MigratePlanningArtifactsRequest* req,
      proto::MigratePlanningArtifactsResponse* resp);

 private:
  std::shared_ptr<service::utils::ResourceRegistry> registry_;
};
}  // namespace comms
