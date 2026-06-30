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
#include "planning_service_client/check_satisfied_response.h"
#include "planning_service_client/context_id.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/planner/motion_plan_result.h"
#include "planning_service_client/planner/motion_problem_definition.h"
#include "proto/planner.grpc.pb.h"

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
class MotionPlannerClient : public ClientInterface<proto::MotionPlanner> {
 public:
  /**
   * @brief Constructor.
   *
   * @param addr Address to which new requests will be sent.
   * @param config_json JSON string specifying configuration options. See
   * https://github.com/grpc/proposal/blob/master/A6-client-retries.md for an
   * example on usage.
   */
  MotionPlannerClient(const std::string_view addr,
                      const std::string_view client_id,
                      const std::string_view config_json = "{}")
      : ClientInterface(addr, client_id, config_json) {}

  /**
   * @brief Given a desired model, calculate the pose of a frame B, relative
   * to a frame A, at some specified configuration.
   *
   * @param context_id Unique ID of desired model
   * @param system_conf Configuration of interest
   * @param frame_B Target frame
   * @param frame_A Relative frame
   * @return const std::pair<proto::CalcRelativePoseResponse, grpc::Status>
   */
  std::pair<proto::CalcRelativePoseResponse, grpc::Status> CalcRelativePose(
      const ContextId& context_id, const proto::SystemConf& system_conf,
      const std::string_view frame_B, const std::string_view frame_A = "world",
      const std::optional<std::string>& transaction_id = std::nullopt) {
    proto::CalcRelativePoseRequest req;
    req.mutable_context_id()->set_value(context_id.value());
    req.mutable_system_conf()->CopyFrom(system_conf);
    req.set_frame_b(frame_B.data());
    req.set_frame_a(frame_A.data());

    proto::CalcRelativePoseResponse resp;
    grpc::ClientContext context;
    SetClientContextDeadline(context);
    if (transaction_id.has_value()) {
      context.AddMetadata("transaction_id", transaction_id.value());
    }
    const auto status {stub_->CalcRelativePose(&context, req, &resp)};
    return std::make_pair(resp, status);
  }

  /**
   * @brief Given a desired model, determine whether the provided
   * configurations satisfy the constraints of that model.
   *
   * @param context_id Unique ID of desired model
   * @param system_conf_vec Vector of configurations to be evaluated
   * @return const std::pair<proto::CheckSatisfiedResponse, grpc::Status>
   */
  CheckSatisfiedResponse CheckSatisfied(
      const ContextId& context_id,
      const std::vector<proto::SystemConf>& system_conf_vec,
      const std::optional<planner::CollisionOptions>& collision_options =
          std::nullopt,
      const std::optional<proto::CheckSatisfiedOptions>& options = std::nullopt,
      const std::optional<std::string>& transaction_id = std::nullopt) {
    proto::CheckSatisfiedRequest req;
    req.mutable_context_id()->set_value(context_id.value());
    for (const auto& system_conf : system_conf_vec) {
      req.add_system_conf_vec()->CopyFrom(system_conf);
    }
    if (collision_options.has_value()) {
      req.mutable_collision_options()->CopyFrom(
          ToProto(collision_options.value()));
    }
    if (options.has_value()) {
      req.mutable_options()->CopyFrom(options.value());
    }
    proto::CheckSatisfiedResponse resp;
    grpc::ClientContext context;
    SetClientContextDeadline(context);
    if (transaction_id.has_value()) {
      context.AddMetadata("transaction_id", transaction_id.value());
    }
    const auto status {stub_->CheckSatisfied(&context, req, &resp)};
    if (!status.ok()) {
      throw std::runtime_error("CheckSatisfied failed: "
                               + std::to_string(status.error_code()) + ": "
                               + status.error_message());
    }
    return FromProto<CheckSatisfiedResponse>(resp);
  }

  /**
   * @brief Solve a motion plan given a unique definition.
   *
   * @param def Motion plan definition.
   * @return const planner::MotionPlanResult
   */
  planner::MotionPlanResult SolvePlan(
      const planner::MotionProblemDefinition& def,
      const std::optional<std::string>& transaction_id = std::nullopt) {
    proto::SolvePlanRequest req;
    req.set_id(new_id());
    *req.mutable_def() = ToProto(def);
    proto::SolvePlanResponse resp;
    grpc::ClientContext context;
    if (transaction_id.has_value()) {
      context.AddMetadata("transaction_id", transaction_id.value());
    }
    const auto status {stub_->SolvePlan(&context, req, &resp)};
    if (!status.ok()) {
      throw std::runtime_error("SolvePlan failed: "
                               + std::to_string(status.error_code()) + ": "
                               + status.error_message());
    }
    return FromProto<planner::MotionPlanResult>(resp.result());
  }

  bool SetPoseInFrame(
      const FrameRelativePose& frp,
      const std::optional<std::string>& transaction_id = std::nullopt) {
    proto::SetPoseInParentFrameRequest req;
    *req.mutable_frame_relative_pose() = ToProto(frp);
    proto::SetPoseInParentFrameResponse resp;
    grpc::ClientContext context;
    if (transaction_id.has_value()) {
      context.AddMetadata("transaction_id", transaction_id.value());
    }
    auto status = stub_->SetPoseInParentFrame(&context, req, &resp);
    return status.ok();
  }

 private:
  const std::string new_id() {
    return client_id_ + "-plan_" + std::to_string(++last_id_);
  }
};
}  // namespace client
}  // namespace planning_service_client
