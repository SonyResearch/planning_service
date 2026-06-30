/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file plan_service.h
#include <grpc/grpc.h>
#include <grpcpp/server_context.h>

#include "planning_service/service/planning/plan_manager.h"
#include "proto/planner.grpc.pb.h"

namespace comms {
/**
 * @brief Implementation of the gRPC service declared in the
 * `planning_service_client` repository which handles requests for motion plans
 * from a robot client.
 */
class MotionPlannerService final : public proto::MotionPlanner::Service {
 public:
  /**
   * @brief Construct an instance of the gRPC service.
   *
   * @param mgr A pointer to the motion plan manager which owns all active
   * threads of execution
   */
  MotionPlannerService(
      std::shared_ptr<service::planning::MotionPlanManager> mgr)
      : mgr_ {mgr} {}

  /**
   * @brief Calculate the pose of a target frame B relative to a frame A.
   *
   * @param req
   * @param resp
   * @return grpc::Status
   */
  grpc::Status CalcRelativePose(grpc::ServerContext*,
                                const proto::CalcRelativePoseRequest* req,
                                proto::CalcRelativePoseResponse* resp);

  grpc::Status CheckSatisfied(grpc::ServerContext*,
                              const proto::CheckSatisfiedRequest* req,
                              proto::CheckSatisfiedResponse* resp);

  grpc::Status SolvePlan(grpc::ServerContext*,
                         const proto::SolvePlanRequest* req,
                         proto::SolvePlanResponse* resp);

  grpc::Status SetPoseInParentFrame(
      grpc::ServerContext*, const proto::SetPoseInParentFrameRequest* req,
      proto::SetPoseInParentFrameResponse* resp);

  std::shared_ptr<service::planning::MotionPlanManager> mgr_ {};
};
}  // namespace comms
