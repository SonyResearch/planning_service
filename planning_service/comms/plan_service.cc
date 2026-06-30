/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file plan_service.cc

#include "plan_service.h"

#include "grpc_utils.h"
#include "planning_service/draco/client_conversions.h"
#include "planning_service_client/common/io_utils.h"
#include "planning_service_client/planner/motion_problem_definition.h"
#include "register_client_planning_problems.h"
#include "utils.h"

namespace psc = planning_service_client;

namespace {
const bool registered_problems = [] {
  planning_service_client::planner::RegisterAllPlanningProblems();
  return true;
}();

const std::string success_green =
    fmt::format(FMT_BOLD | fg(FMT_GREEN), "SUCCESS");
const std::string failure_red = fmt::format(FMT_BOLD | fg(FMT_RED), "FAILURE");
}  // namespace

namespace comms {

grpc::Status MotionPlannerService::CheckSatisfied(
    grpc::ServerContext* ctx, const proto::CheckSatisfiedRequest* req,
    proto::CheckSatisfiedResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  auto draco_context_id = draco::PlanContextId(req->context_id().value());
  logging::log()->info(fmt::format(FMT_BOLD | FMT_ITALIC | fg(FMT_YELLOW),
                                   "MPS:CheckSatisfied: Received request to "
                                   "check satisfaction for context: {}",
                                   draco_context_id));
  const auto& draco_map = mgr_->registry()->draco_map();
  if (!draco_map.count(draco_context_id.value)) {
    return grpc::Status(grpc::StatusCode::NOT_FOUND,
                        "No context found for the given context ID");
  }
  const auto& draco_planner = *draco_map.at(draco_context_id.value);
  std::vector<psc::SystemConf> system_conf_vec {};
  try {
    for (const auto& msg : req->system_conf_vec()) {
      system_conf_vec.push_back(psc::FromProto<psc::SystemConf>(msg));
    }
    std::optional<psc::planner::CollisionOptions> collision_options;
    if (req->has_collision_options()) {
      collision_options = psc::FromProto<psc::planner::CollisionOptions>(
          req->collision_options());
    }
    auto result =
        draco_planner.CheckSatisfied(system_conf_vec, collision_options);
    *resp = planning_service_client::ToProto(result);
  } catch (const std::exception& e) {
    const auto err_msg {
        fmt::format("Failed to run CheckSatisfied: {}", e.what())};
    logging::log()->error("MPS:CheckSatisfied: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  return grpc::Status::OK;
}

grpc::Status MotionPlannerService::CalcRelativePose(
    grpc::ServerContext* ctx, const proto::CalcRelativePoseRequest* req,
    proto::CalcRelativePoseResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  const auto context_id {draco::PlanContextId(req->context_id().value())};
  const auto system_conf {psc::FromProto<psc::SystemConf>(req->system_conf())};
  try {
    const auto pose {mgr_->CalcRelativePose(context_id, system_conf,
                                            req->frame_b(), req->frame_a())};
    *resp->mutable_pose() = utils::RigidTransformToProto(pose);
  } catch (const std::exception& e) {
    const auto err_msg {
        fmt::format("Failed to calculate relative pose: {}", e.what())};
    logging::log()->error("MPS:CalcRelativePose: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  return grpc::Status::OK;
}

grpc::Status MotionPlannerService::SolvePlan(grpc::ServerContext* ctx,
                                             const proto::SolvePlanRequest* req,
                                             proto::SolvePlanResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  auto draco_context_id = draco::PlanContextId(req->def().context_id().value());
  const auto& problem_proto_type {req->def().problem_case()};
  logging::log()->info(fmt::format(
      FMT_BOLD | FMT_ITALIC | fg(FMT_YELLOW),
      "MPS:SolvePlan: Received request to solve {} plan: '{}' with context: {}",
      magic_enum::enum_name(problem_proto_type), req->id(), draco_context_id));
  const auto& draco_map = mgr_->registry()->draco_map();
  if (!draco_map.count(draco_context_id.value)) {
    return grpc::Status(grpc::StatusCode::NOT_FOUND,
                        "No context found for the given context ID");
  }
  const auto& draco_planner = *draco_map.at(draco_context_id.value);
  const auto problems_dir {
      mgr_->registry()->NewProblemDirectory(draco_context_id, req->id())};
  resp->set_id(req->id());
  psc::planner::MotionProblemDefinition motion_def;
  try {
    motion_def =
        psc::FromProto<psc::planner::MotionProblemDefinition>(req->def());
  } catch (const std::exception& e) {
    const auto err_msg {fmt::format("Failed to parse request: {}", e.what())};
    logging::log()->error("MPS:SolvePlan: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INVALID_ARGUMENT, err_msg);
  }
  const auto problem_ptr {motion_def.problem_clone()};
  // Save problem
  psc::common::SaveToJson(problems_dir / "problem.json", motion_def);
  auto problem_path_str = (problems_dir / "problem.json").string();
  std::string label =
      motion_def.name().empty()
          ? fmt::format("{}_{}", req->id(),
                        magic_enum::enum_name(problem_proto_type))
          : motion_def.name();
  psc::planner::MotionPlanResult motion_plan_result;
  try {
    motion_plan_result = draco_planner.SolvePlan(
        *problem_ptr, label, motion_def.maybe_plan_options(),
        motion_def.start_system_conf(), motion_def.maybe_active_trajectory());
  } catch (const std::exception& e) {
    const auto err_msg {fmt::format("Failed to solve plan: {}", e.what())};
    logging::log()->error("MPS:SolvePlan: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  // Save result
  psc::common::SaveToJson(problems_dir / "result.json", motion_plan_result);
  auto result_path_str = (problems_dir / "result.json").string();
  auto jsons_str = fmt::format(FMT_ITALIC | fg(FMT_CYAN),
                               "Problem definition: {}. Motion plan "
                               "result: {}. Meshcat port: {}",
                               problem_path_str, result_path_str,
                               draco_planner.has_draco_visualizer()
                                   ? draco_planner.draco_visualizer().Port()
                                   : -1);
  motion_plan_result.SetMessage(motion_plan_result.message() + jsons_str);
  logging::log()->info("MPS:SolvePlan: motion_plan_result message: {}",
                       motion_plan_result.message());
  try {
    *resp->mutable_result() = psc::ToProto(motion_plan_result);
  } catch (const std::exception& e) {
    const auto err_msg {
        fmt::format("Failed to serialize result: {}", e.what())};
    logging::log()->error("MPS:SolvePlan: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  logging::log()->info(fmt::format(
      FMT_BOLD | FMT_ITALIC | fg(FMT_MAGENTA),
      "MPS:SolvePlan: Done with {} with context: {} result: {}", label,
      draco_context_id,
      motion_plan_result.is_success() ? success_green : failure_red));
  return grpc::Status::OK;
}

grpc::Status MotionPlannerService::SetPoseInParentFrame(
    grpc::ServerContext* ctx, const proto::SetPoseInParentFrameRequest* req,
    proto::SetPoseInParentFrameResponse* resp) {
  comms::set_transaction_id_from_context(ctx);
  DRAKE_THROW_UNLESS(resp != nullptr);
  const auto& draco_map = mgr_->registry()->draco_map();
  try {
    for (const auto& [context_id, draco_planner] : draco_map) {
      logging::log()->info(
          "MPS:SetPoseInParentFrame: Setting pose in parent frame for context: "
          "{}",
          context_id);
      auto frp =
          psc::FromProto<psc::FrameRelativePose>(req->frame_relative_pose());
      draco_planner->SetPoseInParentFrame(frp);
    }
  } catch (const std::exception& e) {
    const auto err_msg {
        fmt::format("Failed to set pose in parent frame: {}", e.what())};
    logging::log()->error("MPS:SetPoseInParentFrame: {}", err_msg);
    return grpc::Status(grpc::StatusCode::INTERNAL, err_msg);
  }
  logging::log()->info(
      "MPS:SetPoseInParentFrame: Pose in parent frame set successfully for all "
      "contexts");
  return grpc::Status::OK;
}

}  // namespace comms
