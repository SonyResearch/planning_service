/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file comms/grpc_utils.h
///
/// Utilities for extracting per-request metadata from a gRPC ServerContext and
/// wiring it into the logging framework.

#pragma once

#include <grpcpp/server_context.h>

#include "planning_service/common/logging.h"

namespace comms {

/// Extract the "transaction_id" key from the incoming gRPC request metadata
/// and store it as the current thread's transaction ID.  If the key is absent
/// a fresh UUID is generated so that every request is always traceable.
///
/// Call this at the top of every gRPC service handler:
/// @code
///   grpc::Status MyService::MyRpc(grpc::ServerContext* ctx, ...) {
///     comms::set_transaction_id_from_context(ctx);
///     ...
///   }
/// @endcode
inline void set_transaction_id_from_context(const grpc::ServerContext* ctx) {
  if (ctx != nullptr) {
    const auto& metadata = ctx->client_metadata();
    auto it = metadata.find("transaction_id");
    if (it != metadata.end()) {
      logging::set_transaction_id(
          std::string(it->second.begin(), it->second.end()));
      return;
    }
  }
  // No transaction_id in metadata – generate a fresh UUID for this request.
  logging::set_transaction_id();
}

}  // namespace comms
