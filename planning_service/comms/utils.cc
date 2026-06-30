/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file utils.cc

#include "utils.h"

namespace comms {
namespace utils {

template <>
const Eigen::VectorXd v_to_e<double>(std::vector<double> v) {
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}

template <>
const Eigen::VectorXd v_to_e<float>(std::vector<float> v) {
  Eigen::VectorXf evf =
      Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(v.data(), v.size());
  return evf.template cast<double>();
}

template <>
const Eigen::VectorXd v_to_e<int>(std::vector<int> v) {
  Eigen::VectorXi evi =
      Eigen::Map<Eigen::VectorXi, Eigen::Unaligned>(v.data(), v.size());
  return evi.template cast<double>();
}

const motion::RobotMeshcatParams ProtoToMeshcatParams(
    const proto::MeshcatParameters& params_pb) {
  motion::RobotMeshcatParams meshcat_params;
  meshcat_params.visual = params_pb.visual();
  meshcat_params.collision = params_pb.collision();
  motion::body_name_color_map_t color_map;
  for (const auto& [body, color] : params_pb.color_map()) {
    color_map.emplace(body, drake::geometry::Rgba(color.r(), color.g(),
                                                  color.b(), color.a()));
  }
  meshcat_params.color_map = color_map;
  meshcat_params.end_effector_frame_vec = {
      params_pb.end_effector_frame_vec().begin(),
      params_pb.end_effector_frame_vec().end()};
  return meshcat_params;
}

const proto::PlanningArtifactStatus PlanningArtifactSizesToProto(
    int num_vertices, int num_edges, int num_regions) {
  proto::PlanningArtifactStatus status_pb;

  // Fill RoadmapStatus
  proto::RoadmapStatus* roadmap_status = status_pb.mutable_roadmap_status();
  roadmap_status->set_num_vertices(static_cast<uint32_t>(num_vertices));
  roadmap_status->set_num_edges(static_cast<uint32_t>(num_edges));
  // Fill IrisRegionStatus
  proto::IrisRegionStatus* iris_status = status_pb.mutable_iris_status();
  iris_status->set_num_regions(static_cast<uint32_t>(num_regions));
  return status_pb;
}

}  // namespace utils
}  // namespace comms
