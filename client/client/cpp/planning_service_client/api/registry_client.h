/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file planner_client.h

#pragma once
#include <optional>
#include <string_view>

#include <grpc/grpc.h>
#include <grpcpp/channel.h>
#include <grpcpp/client_context.h>
#include <grpcpp/create_channel.h>
#include <grpcpp/security/credentials.h>

#include "planning_service_client/api/client.h"
#include "proto/registry.grpc.pb.h"

namespace planning_service_client {
namespace client {

/**
 * @brief Minimal implementation of a gRPC client which may interact with the
 * motion planning client to initiate the computation of, and retrieve, motion
 * plans.
 * For detailed documentation of the API and message types, please see:
 * https://github.com/SonyResearch/planning_service_client/proto/planner.proto
 *
 */
class RegistryClient : public ClientInterface<proto::PlanContextRegistry> {
 public:
  RegistryClient(const std::string_view addr, const std::string_view client_id,
                 const std::string_view config_json = "{}")
      : ClientInterface(addr, client_id, config_json) {}

  const std::pair<proto::GetVersionResponse, grpc::Status> GetVersion() {
    proto::GetVersionRequest req;
    proto::GetVersionResponse resp;
    grpc::ClientContext context;
    SetClientContextDeadline(context);
    const auto status {this->stub_->GetVersion(&context, req, &resp)};
    return std::make_pair(resp, status);
  }
};
}  // namespace client
}  // namespace planning_service_client
