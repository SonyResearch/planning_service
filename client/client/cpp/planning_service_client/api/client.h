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
template <typename Service>
class ClientInterface {
 public:
  /**
   * @brief Constructor.
   *
   * @param addr Address to which new requests will be sent.
   * @param config_json JSON string specifying configuration options. See
   * https://github.com/grpc/proposal/blob/master/A6-client-retries.md for an
   * example on usage.
   */
  ClientInterface(const std::string_view addr, const std::string_view client_id,
                  const std::string_view config_json = "{}")
      : client_id_ {client_id}, name_ {Service::service_full_name()} {
    grpc::ChannelArguments args;
    args.SetServiceConfigJSON(config_json.data());
    channel_ = grpc::CreateCustomChannel(
        addr.data(), grpc::InsecureChannelCredentials(), args);
    stub_ = Service::NewStub(channel_);
  }

  bool Connect(int num_attempts = 5, int attempt_interval_ms = 1000) {
    for (int attempt = 0; attempt < num_attempts; ++attempt) {
      try {
        // Check channel state for connectivity
        if (channel_->WaitForConnected(
                std::chrono::system_clock::now()
                + std::chrono::milliseconds(attempt_interval_ms))) {
          std::cout << "[" << name_ << "] Successfully connected to server."
                    << std::endl;
          return true;
        }
      } catch (const std::exception& e) {
        std::cout << "[" << name_ << "] Connect attempt " << (attempt + 1)
                  << " failed: " << e.what() << std::endl;
      }
    }
    return false;
  }

 protected:
  /** Assign a new stub. Used for testing. */
  void SetStub(std::unique_ptr<typename Service::StubInterface> stub) {
    stub_ = std::move(stub);
  }

  void SetClientContextDeadline(grpc::ClientContext& context,
                                int client_timeout_ms = 5000) const {
    // Default to 5 sec timeout.
    std::chrono::time_point deadline =
        std::chrono::system_clock::now()
        + std::chrono::milliseconds(client_timeout_ms);
    context.set_deadline(deadline);
  }

  std::atomic<uint32_t> last_id_ {0};
  const std::string client_id_;
  const std::string name_;
  std::shared_ptr<grpc::Channel> channel_;
  std::unique_ptr<typename Service::StubInterface> stub_;
};
}  // namespace client
}  // namespace planning_service_client
