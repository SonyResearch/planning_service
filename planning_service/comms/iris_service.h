/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file iris_service.h

#include <filesystem>

#include <grpcpp/server_context.h>

#include "planning_service/service/iris/iris_manager.h"
#include "proto/builder.grpc.pb.h"

namespace fs = std::filesystem;

// iris builder message types
using proto::ReportBuildStatusRequest;
using proto::ReportBuildStatusResponse;
using proto::StartBuildRequest;
using proto::StartBuildResponse;
using proto::UpdateRoadmapRequest;
using proto::UpdateRoadmapResponse;
namespace comms {
class IrisBuilderService final : public proto::IrisBuilder::Service {
 public:
  /**
   * @brief Constructor.
   * @param mgr IRIS build manager
   */
  IrisBuilderService(std::shared_ptr<service::iris::IrisBuildManager> mgr)
      : mgr_ {mgr} {}

  ~IrisBuilderService() {
    if (mgr_->Running()) {
      mgr_->Stop();
    }
  }
  /**
   * @brief Given a StartBuildFromEdgesRequest, start an IRIS region generation
   * job for the given plan context and seed data.
   *
   * @param req pointer to request message containing the target plan context
   * and seed edges
   * @param resp pointer to response message indicating whether or not job was
   * started successfully
   *
   * @return status code and message to be handled internally
   */
  grpc::Status HandleStartBuildRequest(grpc::ServerContext*,
                                       const StartBuildRequest* req,
                                       StartBuildResponse* resp);

  /**
   * @brief Given a UpdateRoadampRequest, update the roadmap with the solution
   * of the given plan
   *
   * @param req pointer to request message containing the solution of the plan
   * @param resp pointer to response message indicating whether or not the
   * roadmap was updated successfully
   *
   */
  grpc::Status HandleUpdateRoadmapRequest(grpc::ServerContext*,
                                          const UpdateRoadmapRequest* req,
                                          UpdateRoadmapResponse* resp);

  // TODO(@davebambrick): implement
  grpc::Status HandleReportStatusRequest(grpc::ServerContext*,
                                         const ReportBuildStatusRequest*,
                                         ReportBuildStatusResponse*);

 private:
  std::shared_ptr<service::iris::IrisBuildManager> mgr_;
};
}  // namespace comms
