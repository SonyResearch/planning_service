/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file utils.h

#pragma once
#include <array>
#include <cstdio>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <type_traits>

#include <google/protobuf/util/json_util.h>
#include <magic_enum/magic_enum.hpp>

#include "planning_service/service/types/types.h"
#include "proto/builder.pb.h"
#include "proto/planner.pb.h"
#include "proto/registry.pb.h"
#include "proto/visualizer.pb.h"

namespace comms {
namespace utils {

using system_conf_t = std::map<std::string, Eigen::VectorXd>;
using system_conf_edge_t = std::pair<system_conf_t, system_conf_t>;

/**
 * @brief Create a std::vector<double> from an Eigen::VectorXd.
 */
inline const std::vector<double> e_to_v(const Eigen::VectorXd& e) {
  std::vector<double> v;
  v.resize(e.size());
  Eigen::VectorXd::Map(&v[0], e.size()) = e;
  return v;
}

/**
 * @brief Create an  from a std::vector<T>. Support for int,
 * float, and double.
 */
template <typename T>
const Eigen::VectorXd v_to_e(std::vector<T> v);

/**
 * @brief Create an Eigen::VectorXd from a Protobuf RepeatedField<T>
 * (analagous to a C++ std::vector<T>) for arithmetic types T.
 */
template <typename T>
std::enable_if_t<std::is_arithmetic_v<T>, Eigen::VectorXd> RepeatedToEigen(
    const google::protobuf::RepeatedField<T>& field) {
  return v_to_e<T>(std::vector<T>(field.begin(), field.end()));
}

/**
 * @brief Create a Eigen::VectorXd from its corresponding Protobuf message.
 */
inline const Eigen::VectorXd ProtoToConf(const proto::Conf& msg) {
  return RepeatedToEigen(msg.data());
}

/**
 * @brief Create a system_conf_t from its corresponding Protobuf message.
 */
inline const system_conf_t ProtoToSystemConf(const proto::SystemConf& msg) {
  system_conf_t sys_conf;
  for (const auto& [robot, conf] : msg.data()) {
    sys_conf.emplace(robot, ProtoToConf(conf));
  }
  return sys_conf;
}

/**
 * @brief Create a system_conf_edge_t from its corresponding Protobuf message.
 */
inline const system_conf_edge_t ProtoToSystemConfEdge(
    const proto::SystemConfEdge& msg) {
  return system_conf_edge_t(utils::ProtoToSystemConf(msg.u()),
                            utils::ProtoToSystemConf(msg.v()));
}

inline const proto::RigidTransform RigidTransformToProto(
    const drake::math::RigidTransformd& X_AB) {
  proto::RigidTransform msg;
  const auto& translation {X_AB.translation()};
  const auto& quat {X_AB.rotation().ToQuaternion()};
  proto::Vector3 translation_pb;
  translation_pb.set_x(translation(0));
  translation_pb.set_y(translation(1));
  translation_pb.set_z(translation(2));
  *msg.mutable_translation() = translation_pb;
  proto::Quaternion quat_pb;
  quat_pb.set_w(quat.w());
  quat_pb.set_x(quat.x());
  quat_pb.set_y(quat.y());
  quat_pb.set_z(quat.z());
  *msg.mutable_quat() = quat_pb;
  return msg;
}

/**
 * @brief Create a PlanningProblemDef from its corresponding Protobuf
 * message.
 */
inline const draco::PlanningProblemDef ProtoToPlanningProblemDef(
    const proto::ProblemDef& msg) {
  throw std::runtime_error(
      "ProtoToPlanningProblemDef: Deprecated PlanningProblemDef proto "
      "message is not supported anymore.");
  (void)msg;  // suppress unused variable warning
  draco::PlanningProblemDef problem_def;
  return problem_def;
}

/**
 * @brief Create a motion::PositionConstraintAdapter from its corresponding
 * Protobuf message.
 */
inline const motion::PositionConstraintAdapter ProtoPositionConstraintAdapter(
    const proto::Deprecated_PositionConstraint& adapter_pb) {
  logging::log()->warn(
      "ProtoToAngleBetweenVectorsConstraintAdapter: Using deprecated angle "
      "between vectors constraint. Please update to the latest constraints "
      "format.");
  motion::PositionConstraintAdapter adapter;
  adapter.frame_A = adapter_pb.frame_a();
  adapter.frame_B = adapter_pb.frame_b();
  adapter.position_BQ = RepeatedToEigen(adapter_pb.p_bq());
  adapter.position_AQ_lower = RepeatedToEigen(adapter_pb.p_aq_lower());
  adapter.position_AQ_upper = RepeatedToEigen(adapter_pb.p_aq_upper());
  return adapter;
}

/**
 * @brief Create a AngleBetweenVectorsConstraintAdapter from its corresponding
 * Protobuf message.
 */
inline const motion::AngleBetweenVectorsConstraintAdapter
ProtoToAngleBetweenVectorsConstraintAdapter(
    const proto::Deprecated_AngleBetweenVectorsConstraint& adapter_pb) {
  logging::log()->warn(
      "ProtoToAngleBetweenVectorsConstraintAdapter: Using deprecated angle "
      "between vectors constraint. Please update to the latest constraints "
      "format.");
  motion::AngleBetweenVectorsConstraintAdapter adapter;
  adapter.frame_A = adapter_pb.frame_a();
  adapter.frame_B = adapter_pb.frame_b();
  adapter.a_A = RepeatedToEigen(adapter_pb.a_a());
  adapter.b_B = RepeatedToEigen(adapter_pb.b_b());
  adapter.angle_lower = adapter_pb.angle_lower();
  adapter.angle_upper = adapter_pb.angle_upper();
  return adapter;
}

/**
 * @brief Create a ConstraintsAdapter from its corresponding
 * Protobuf message.
 * @warning This uses deprecated position and angle constraints. ToDo: update
 * this when new constraint types are added.
 */
inline const motion::ConstraintsAdapter ProtoToConstraintsAdapter(
    const proto::ConstraintsSet& constraints_pb) {
  motion::ConstraintsAdapter constraints_adapter;
  // default collision checker
  constraints_adapter.collision_checker = motion::CollisionCheckerAdapter();
  // populate position constraints
  if (constraints_pb.pos_constraints_size()) {
    std::vector<motion::PositionConstraintAdapter> constraints_vec;
    for (const auto& constraint : constraints_pb.pos_constraints()) {
      constraints_vec.push_back(ProtoPositionConstraintAdapter(constraint));
    }
    constraints_adapter.position_constraints = constraints_vec;
  }
  // populate angle constraints
  if (constraints_pb.angle_constraints_size()) {
    std::vector<motion::AngleBetweenVectorsConstraintAdapter> constraints_vec;
    for (const auto& constraint : constraints_pb.angle_constraints()) {
      constraints_vec.push_back(
          ProtoToAngleBetweenVectorsConstraintAdapter(constraint));
    }
    constraints_adapter.angle_constraints = constraints_vec;
  }
  return constraints_adapter;
}

inline const service::MeshFile ProtoToMeshFile(const proto::MeshFile& mesh_pb) {
  std::string extension;
  switch (mesh_pb.format()) {
    case proto::MeshFormat::MESH_FORMAT_OBJ:
      extension = ".obj";
      break;
    case proto::MeshFormat::MESH_FORMAT_MTL:
      extension = ".mtl";
      break;
    case proto::MeshFormat::MESH_FORMAT_GLTF:
      extension = ".gltf";
      break;
    case proto::MeshFormat::MESH_FORMAT_BIN:
      extension = ".bin";
      break;
    default:
      throw std::runtime_error(fmt::format(
          "ProtoToMeshFile: Mesh format: {} is not currently supported!",
          magic_enum::enum_name(mesh_pb.format())));
  }
  bool collision;
  switch (mesh_pb.type()) {
    case proto::MeshType::MESH_TYPE_COLLISION:
      collision = true;
      break;
    case proto::MeshType::MESH_TYPE_VISUAL:
      collision = false;
      break;
    default:
      throw std::runtime_error(fmt::format(
          "ProtoToMeshFile: Mesh type: {} is not currently supported!",
          magic_enum::enum_name(mesh_pb.type())));
  }
  service::MeshFile mesh {.name = mesh_pb.name(),
                          .extension = extension,
                          .package_name = mesh_pb.package_name(),
                          .parent_path = mesh_pb.parent_path(),
                          .collision = collision,
                          .content = mesh_pb.content()};
  return mesh;
}

inline const service::Model ProtoToModel(const proto::Model& model_pb) {
  const auto& file_pb {model_pb.file()};
  service::ModelFormat format;
  switch (file_pb.format()) {
    case proto::ModelFormat::MODEL_FORMAT_URDF:
      format = service::ModelFormat::URDF;
      break;
    default:
      throw std::runtime_error(fmt::format(
          "ProtoToModel: Model of format: {} is not currently supported!",
          magic_enum::enum_name(file_pb.format())));
  }
  const service::ModelFile file {.name = file_pb.name(),
                                 .format = format,
                                 .package_name = file_pb.package_name(),
                                 .parent_path = file_pb.parent_path(),
                                 .content = file_pb.content()};
  std::vector<service::MeshFile> meshes;
  for (const auto& mesh_pb : model_pb.meshes()) {
    meshes.push_back(ProtoToMeshFile(mesh_pb));
  }
  service::Model model {.name = model_pb.name(),
                        .file = file,
                        .meshes = meshes,
                        .default_joint_positions = {},
                        .default_free_body_pose = {}};
  return model;
}

/**
 * @brief Create a PlanContext from its corresponding Protobuf message.
 */
inline const service::PlanContext ProtoToPlanContext(
    const proto::PlanContext& context_pb) {
  const auto name {context_pb.name()};
  if (name.empty()) {
    throw std::runtime_error("No system name provided!");
  }
  if (const auto& context_id {context_pb.id().value()}; context_id != 0) {
    return service::PlanContext(context_id);
  } else {
    std::vector<service::Model> context_models;
    for (const auto& model : context_pb.models()) {
      context_models.push_back(ProtoToModel(model));
    }
    std::string metadata_json_str;
    const auto status {google::protobuf::util::MessageToJsonString(
        context_pb.metadata(), &metadata_json_str)};
    if (!status.ok()) {
      throw std::runtime_error("Failed to convert Protobuf message to JSON: "
                               + status.ToString());
    }
    motion::ConstraintsAdapter constraints_adapter;
    if (!context_pb.constraints_raw().empty()) {
      constraints_adapter =
          drake::yaml::LoadYamlString<motion::ConstraintsAdapter>(
              context_pb.constraints_raw());
    } else {
      constraints_adapter = ProtoToConstraintsAdapter(context_pb.constraints());
    }
    return service::PlanContext(
        name,
        drake::yaml::LoadYamlString<drake::multibody::parsing::ModelDirectives>(
            context_pb.model_directive_raw()),
        context_models, constraints_adapter, json::parse(metadata_json_str));
  }
}

inline service::iris::IrisBuildJobType ProtoToIrisBuildJobType(
    const proto::StartBuildRequest::SeedDataCase& data_case) {
  switch (data_case) {
    // IRIS cases
    case proto::StartBuildRequest::SeedDataCase::kSysconfVec:
      return service::iris::IrisBuildJobType::IRIS_FROM_CONFIGS;
    case proto::StartBuildRequest::SeedDataCase::kSysconfEdgeVec:
      return service::iris::IrisBuildJobType::IRIS_FROM_EDGES;
    case proto::StartBuildRequest::SeedDataCase::kRoadmapData:
      return service::iris::IrisBuildJobType::IRIS_FROM_ROADMAP;
    case proto::StartBuildRequest::SeedDataCase::kProblemDefVec:
      return service::iris::IrisBuildJobType::IRIS_FROM_PROBLEMS;
    default:
      throw std::runtime_error(
          fmt::format("ProtoToIrisBuildJobType: Unsupported seed data type "
                      "provided with value: {}",
                      magic_enum::enum_name(data_case)));
  }
}
inline service::iris::UpdateRoadmapJobType ProtoToUpdateRoadmapJobType(
    const proto::UpdateRoadmapRequest::SeedDataCase& data_case) {
  switch (data_case) {
    case proto::UpdateRoadmapRequest::SeedDataCase::kGenerateFromProblems:
      return service::iris::UpdateRoadmapJobType::UPDATE_FROM_SAVED_PROBLEMS;
    case proto::UpdateRoadmapRequest::SeedDataCase::kRoadmapData:
      return service::iris::UpdateRoadmapJobType::UPDATE_FROM_SAMPLES;
    case proto::UpdateRoadmapRequest::SeedDataCase::kProblemDefVec:
      return service::iris::UpdateRoadmapJobType::UPDATE_FROM_PASSED_PROBLEMS;
    default:
      throw std::runtime_error(
          fmt::format("ProtoToUpdateRoadmapJobType: Unsupported seed data type "
                      "provided with value: {}",
                      magic_enum::enum_name(data_case)));
  }
}

/**
 * @brief Create a IrisBuildRequestAdapter from a request containing seed data
 * as edges.
 */
inline const service::iris::IrisBuildRequestAdapter
ProtoToIrisBuildRequestAdapter(const proto::StartBuildRequest* req) {
  std::vector<system_conf_t> sysconf_vec;
  for (const auto& config_pb : req->sysconf_vec().data()) {
    sysconf_vec.push_back(utils::ProtoToSystemConf(config_pb));
  }
  std::vector<system_conf_edge_t> sysconf_edge_vec;
  for (const auto& edge_pb : req->sysconf_edge_vec().data()) {
    sysconf_edge_vec.push_back(utils::ProtoToSystemConfEdge(edge_pb));
  }

  std::vector<draco::PlanningProblemDef> planning_problem_vec;
  for (const auto& problem_pb : req->problem_def_vec().data()) {
    planning_problem_vec.push_back(ProtoToPlanningProblemDef(problem_pb));
  }

  return service::iris::IrisBuildRequestAdapter(
      req->id(), ProtoToPlanContext(req->context()), req->num_problems(),
      ProtoToIrisBuildJobType(req->seed_data_case()), sysconf_vec,
      sysconf_edge_vec, planning_problem_vec);
}

inline const service::iris::UpdateRoadmapRequestAdapter
ProtoToUpdateRoadmapRequestAdapter(const proto::UpdateRoadmapRequest* req) {
  const int num_samples {req->roadmap_data().num_samples()};
  const int num_problems {req->generate_from_problems().num_problems()};
  const int num_fpp_problems {req->generate_from_problems().num_fpp_problems()};
  const int num_random_ik_seed_samples {
      req->generate_from_problems().num_random_ik_seed_samples()};
  const bool insert_solution_in_roadmap {
      req->generate_from_problems().insert_solution_in_roadmap()};
  std::vector<draco::PlanningProblemDef> planning_problem_vec;
  for (const auto& problem_pb : req->problem_def_vec().data()) {
    planning_problem_vec.push_back(ProtoToPlanningProblemDef(problem_pb));
  }
  return service::iris::UpdateRoadmapRequestAdapter(
      req->id(), ProtoToPlanContext(req->context()),
      ProtoToUpdateRoadmapJobType(req->seed_data_case()), num_samples,
      num_problems, num_fpp_problems, num_random_ik_seed_samples,
      insert_solution_in_roadmap, planning_problem_vec);
}

/** @brief Create a MigratePlanningArtifactsRequestAdapter from its
 * corresponding Protobuf message.
 * @param req the Protobuf message to convert
 *
 */
inline const service::iris::MigratePlanningArtifactsRequestAdapter
ProtoToMigratePlanningArtifactsRequestAdapter(
    const proto::MigratePlanningArtifactsRequest* req) {
  return service::iris::MigratePlanningArtifactsRequestAdapter(
      req->id(), draco::PlanContextId(req->from_context_id().value()),
      draco::PlanContextId(req->to_context_id().value()), req->num_samples(),
      req->repair_artifacts());
}

const motion::RobotMeshcatParams ProtoToMeshcatParams(
    const proto::MeshcatParameters& params_pb);

inline const motion::CheckSatisfiedOptions ProtoToCheckSatisfiedOptions(
    const proto::CheckSatisfiedOptions& options_pb) {
  motion::CheckSatisfiedOptions options;
  options.parallel = options_pb.parallel();
  options.verbose = options_pb.verbose();
  options.tolerance = options_pb.tolerance();
  return options;
}

/**
 * @brief Convert PlanningArtifactStatus to protobuf message.
 *
 * @param status The PlanningArtifactStatus struct to convert
 * @return proto::PlanningArtifactStatus The protobuf representation
 */
const proto::PlanningArtifactStatus PlanningArtifactSizesToProto(
    int num_vertices, int num_edges, int num_regions);

}  // namespace utils
}  // namespace comms
