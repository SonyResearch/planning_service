/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file server.h

#pragma once
#include <grpc/grpc.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>

#include "planning_service/common/logging.h"
namespace comms {

/**
 * @brief Generic wrapper around a set of gRPC services, which exposes an API to
 * start and shutdown the service simply.
 */
class ServerWrapper {
 public:
  /**
   * @brief Constructor. Initializes gRPC server and constructs service using
   * any provided arguments.
   *
   * @param addr Address at which the server will listen to new requests.
   * @param args Arguments to the constructor for the given gRPC service
   * implementation
   */
  ServerWrapper(const std::string& addr) : addr_ {addr} {}
  /**
   * @brief Destructor. Kills the gRPC server if it's running.
   */
  ~ServerWrapper() {
    if (grpc_srv_ != nullptr) {
      logging::log()->info(
          "ServerWrapper:Shutdown: Stopping server listening at address {}",
          addr_);
      grpc_srv_->Shutdown();
    }
  }
  /**
   * @brief Add a service to the underlying gRPC server.
   * @tparam T gRPC Service type
   * @param service Pointer to service of type T
   * @return void
   */
  template <typename T>
  std::enable_if_t<std::is_base_of_v<grpc::Service, T>, void> AddService(
      std::unique_ptr<T> service) {
    if (running_) {
      logging::log()->error(
          "The server has already been started! No further services may be "
          "added");
      return;
    }
    services_.push_back(std::move(service));
  }

  /**
   * @brief Start a gRPC server listening to the address provided at
   * initialization. This call is blocking.
   */
  void Run() {
    if (services_.empty()) {
      throw std::runtime_error("No gRPC services have been added!");
    }
    grpc::ServerBuilder builder;
    // Create health checker
    grpc::EnableDefaultHealthCheckService(true);
    builder.AddListeningPort(addr_, grpc::InsecureServerCredentials());
    builder.SetMaxSendMessageSize(1024 * 1024 * 1024);
    builder.SetMaxMessageSize(1024 * 1024 * 1024);
    builder.SetMaxReceiveMessageSize(1024 * 1024 * 1024);
    // Detect dead clients quickly so zombie streams don't block new
    // connections. Send a keepalive ping after 10s of inactivity, wait 5s for a
    // response, and allow pings even when there are no active RPCs.
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIME_MS, 10'000);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_TIMEOUT_MS, 5'000);
    builder.AddChannelArgument(GRPC_ARG_KEEPALIVE_PERMIT_WITHOUT_CALLS, 1);
    for (const auto& service : services_) {
      builder.RegisterService(service.get());
    }
    grpc_srv_ = builder.BuildAndStart();
    logging::log()->info("ServerWrapper:Run: server started at address {}",
                         addr_);
    running_ = true;
    grpc_srv_->Wait();
  }
  /**
   * @brief Shutdown the underlying gRPC server.
   */
  void Shutdown() {
    grpc_srv_->Shutdown();
  }

 private:
  // address at which the server will listen for new requests
  const std::string addr_ {};
  // flag to prevent services from being added once the server has been started
  bool running_ {false};
  // service implementation
  std::vector<std::unique_ptr<grpc::Service>> services_ {};
  // pointer to server, where the service is registered
  std::unique_ptr<grpc::Server> grpc_srv_ {nullptr};
};
}  // namespace comms
