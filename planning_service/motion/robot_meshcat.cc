/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <drake/common/random.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/vpolytope.h>

#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "planning_service/common/misc_utils.h"
#include "robot_model.h"

namespace fs = std::filesystem;

namespace motion {

void RobotModel::AddFrameAxesToMeshcat(
    const drake::multibody::Frame<double>& frame, double transparency,
    double axis_length, double axis_radius, bool re_compute) const {
  if (!re_compute
      && std::find(frames_with_axes_ptr_->begin(), frames_with_axes_ptr_->end(),
                   &frame)
             != frames_with_axes_ptr_->end()) {
    logging::log()->debug(
        "RobotModel:AddFrameAxesToMeshcat: Axes for frame {} already exist in "
        "Meshcat, skipping addition.",
        frame.name());
    return;
  }
  if (!meshcat_) {
    logging::log()->warn(
        "RobotModel:AddFrameAxesToMeshcat: No Meshcat instance exists, cannot "
        "add frame axes for frame {}",
        frame.name());
    return;
  }
  // Add x axis as red, y axis as green, z axis as blue
  auto cylinder = drake::geometry::Cylinder(axis_radius, axis_length);
  auto X_x = drake::math::RigidTransformd();
  X_x.set_translation(Eigen::Vector3d(axis_length / 2, 0, 0));
  X_x.set_rotation(
      drake::math::RollPitchYawd(0, -M_PI_2, 0).ToRotationMatrix());
  auto X_y = drake::math::RigidTransformd();
  X_y.set_translation(Eigen::Vector3d(0, axis_length / 2, 0));
  X_y.set_rotation(drake::math::RollPitchYawd(M_PI_2, 0, 0).ToRotationMatrix());
  auto X_z = drake::math::RigidTransformd();
  X_z.set_translation(Eigen::Vector3d(0, 0, axis_length / 2));
  X_z.set_rotation(drake::math::RollPitchYawd(0, 0, 0).ToRotationMatrix());
  const auto& body_with_visual = GetVisualBodyInTheSameMobileGroup(frame);
  // Let's get the pose between frame and body_with_visual
  auto& plant_context =
      plant().GetMyMutableContextFromRoot(calc_pose_context_ptr());
  const auto X_BF = plant().CalcRelativeTransform(
      plant_context, body_with_visual.body_frame(), frame);
  auto path = fmt::format(
      "/drake/visual/{}/{}/axes_{}/",
      plant().GetModelInstanceName(body_with_visual.model_instance()),
      body_with_visual.name(), frame.name());
  logging::log()->debug(
      "RobotModel:AddFrameAxesToMeshcat: Adding axes for frame {} at path {}",
      frame.name(), path);
  meshcat_->SetObject(path + "x", cylinder,
                      drake::geometry::Rgba(1, 0, 0, transparency));  // Red
  meshcat_->SetTransform(path + "x", X_BF * X_x);
  meshcat_->SetObject(path + "y", cylinder,
                      drake::geometry::Rgba(0, 1, 0, transparency));  // Green
  meshcat_->SetTransform(path + "y", X_BF * X_y);
  meshcat_->SetObject(path + "z", cylinder,
                      drake::geometry::Rgba(0, 0, 1, transparency));  // Blue
  meshcat_->SetTransform(path + "z", X_BF * X_z);
  frames_with_axes_ptr_->push_back(&frame);
  PublishMeshcatContext();
}

body_index_color_map_t RobotModel::ParseBodyColorMap() const {
  body_index_color_map_t body_index_color_map;
  const auto& plant {this->plant()};
  const auto& sg_inspector = scene_graph().model_inspector();
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    const auto idx {drake::multibody::ModelInstanceIndex(i)};
    for (const auto& body_index : plant.GetBodyIndices(idx)) {
      auto frame_id = plant.GetBodyFrameIdOrThrow(body_index);
      auto collision_geos = sg_inspector.GetGeometries(
          frame_id, drake::geometry::Role::kProximity);
      for (const auto geo_id : collision_geos) {
        const auto& shape = sg_inspector.GetShape(geo_id);
        auto shape_type_name = shape.type_name();
        // if the shape is a mesh, we log a warning
        if (shape_type_name == "Mesh") {
          const auto* mesh = dynamic_cast<const drake::geometry::Mesh*>(&shape);
          DRAKE_DEMAND(mesh != nullptr);
          logging::log()->debug(
              "RobotModel:ParseBodyColorMap: Body {} has a mesh for collision "
              "geometry {}, which slows down collision checking. Please "
              "consider using a simpler shape (sphere or capsule) for "
              "collision checking.",
              mesh->source().description(), plant.get_body(body_index).name());
        }
      }
      auto all_geos = sg_inspector.GetGeometries(frame_id);
      for (const auto geo_id : all_geos) {
        auto props = sg_inspector.GetIllustrationProperties(geo_id);
        if (props && props->HasProperty("phong", "diffuse")) {
          auto color = props->GetProperty<Eigen::Vector4d>("phong", "diffuse");
          body_index_color_map[body_index]["visual"] =
              drake::geometry::Rgba(color(0), color(1), color(2), color(3));
          body_index_color_map[body_index]["collision"] =
              drake::geometry::Rgba(color(0), color(1), color(2), 0.5);
        } else if (!body_index_color_map.contains(body_index)) {
          body_index_color_map[body_index]["visual"] = drake::geometry::Rgba();
          body_index_color_map[body_index]["collision"] =
              drake::geometry::Rgba(1.0, 1.0, 1.0, 0.5);
        }
      }
    }
  }
  return body_index_color_map;
}

void RobotModel::SetBodyColorsByRole(
    const drake::multibody::RigidBody<double>& body,
    drake::systems::Context<double>& context,
    const std::optional<drake::geometry::Rgba> collision,
    const std::optional<drake::geometry::Rgba> visual) const {
  const auto& plant {this->plant()};
  const auto& scene_graph {this->scene_graph()};
  const auto source_id_opt {plant.get_source_id()};
  DRAKE_THROW_UNLESS(source_id_opt.has_value());
  const auto source_id = source_id_opt.value();
  if (collision) {
    drake::geometry::ProximityProperties proximity_properties;
    proximity_properties.AddProperty("phong", "diffuse", collision.value());
    for (const auto& collision_geo_id :
         plant.GetCollisionGeometriesForBody(body)) {
      scene_graph.AssignRole(&context, source_id, collision_geo_id,
                             proximity_properties,
                             drake::geometry::RoleAssign::kReplace);
    }
  }
  if (visual) {
    drake::geometry::IllustrationProperties illustration_properties;
    illustration_properties.AddProperty("phong", "diffuse", visual.value());
    for (const auto& visual_geo_id : plant.GetVisualGeometriesForBody(body)) {
      scene_graph.AssignRole(&context, source_id, visual_geo_id,
                             illustration_properties,
                             drake::geometry::RoleAssign::kReplace);
    }
  }
}

void RobotModel::SetBodyColorsByProperty(
    const drake::multibody::RigidBody<double>& body,
    const std::optional<drake::geometry::Rgba> collision,
    const std::optional<drake::geometry::Rgba> visual) const {
  const auto model_name {plant().GetModelInstanceName(body.model_instance())};
  if (collision) {
    const auto collision_path {
        fmt::format("/drake/collision/{}/{}", model_name, body.name())};
    std::vector<double> color_vec {collision->r(), collision->g(),
                                   collision->b(), collision->a()};
    meshcat_->SetProperty(collision_path, "color", color_vec);
  }
  if (visual) {
    const auto visual_path {
        fmt::format("/drake/visual/{}/{}", model_name, body.name())};
    std::vector<double> color_vec {visual->r(), visual->g(), visual->b(),
                                   visual->a()};
    meshcat_->SetProperty(visual_path, "color", color_vec);
  }
}

std::unique_ptr<drake::systems::Context<double>>
RobotModel::CreateMeshcatContext(
    const body_index_color_map_t& body_index_color_map) const {
  logging::log()->info(
      "RobotModel:CreateMeshcatContext: Creating meshcat context for {} bodies",
      body_index_color_map.size());
  auto context =
      parsed_model_.default_collision_checker->model().CreateDefaultContext();
  auto& scene_graph_context {
      scene_graph().GetMyMutableContextFromRoot(context.get())};
  for (const auto& [body_index, colors] : body_index_color_map) {
    DRAKE_THROW_UNLESS(colors.contains("visual"));
    DRAKE_THROW_UNLESS(colors.contains("collision"));
    const auto& body {plant().get_body(body_index)};
    SetBodyColorsByRole(body, scene_graph_context, colors.at("collision"),
                        colors.at("visual"));
  }
  return context;
}

void RobotModel::SetMeshcatPositions(const Eigen::VectorXd& q) const {
  const auto& plant {this->plant()};
  DRAKE_THROW_UNLESS(q.size() == holonomic_mapping_.minimal_dim());
  auto q_lifted = holonomic_mapping_.Lift(q);
  if (!common::utils::JointPositionsChanged(q_lifted, GetMeshcatPositions())) {
    return;
  }
  if (meshcat_joint_index_name_map_) {
    for (const auto& [idx, joint_name] : *meshcat_joint_index_name_map_) {
      meshcat_->SetSliderValue(joint_name, q_lifted(idx));
    }
  }
  auto& plant_context {
      plant.GetMyMutableContextFromRoot(meshcat_diagram_context_.get())};
  plant.SetPositions(&plant_context, q_lifted);
}

const Eigen::VectorXd RobotModel::GetMeshcatPositions() const {
  const auto& plant_context {
      plant().GetMyContextFromRoot(*meshcat_diagram_context_)};
  return plant().GetPositions(plant_context);
}

// Get the robot diagram to publish the context used for meshcat
void RobotModel::PublishMeshcatContext() const {
  parsed_model_.default_collision_checker->model().ForcedPublish(
      *meshcat_diagram_context_);
}

void RobotModel::SetMeshcatJointSliders() const {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  DRAKE_THROW_UNLESS(meshcat_joint_index_name_map_ == nullptr);
  const auto& plant {this->plant()};
  const auto q_high {plant.GetPositionUpperLimits()};
  const auto q_low {plant.GetPositionLowerLimits()};
  std::map<int, std::string> q_map;
  const auto q_initial {GetMeshcatPositions()};
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    const auto idx {drake::multibody::ModelInstanceIndex(i)};
    int joint_count {0};
    for (const auto& joint_index : plant.GetJointIndices(idx)) {
      const auto& joint {plant.get_joint(joint_index)};
      for (int j = 0; j < joint.num_positions(); ++j) {
        const auto& model_name {plant.GetModelInstanceName(idx)};
        const auto& joint_name {
            fmt::format("{}::J{}", model_name, ++joint_count)};
        meshcat_->AddSlider(joint_name, q_low(joint.position_start()),
                            q_high(joint.position_start()), 0.001,
                            q_initial(joint.position_start()));
        q_map.emplace(std::make_pair(joint.position_start(), joint_name));
      }
    }
  }
  meshcat_joint_index_name_map_ =
      std::make_unique<std::map<int, std::string>>(q_map);
}

const Eigen::VectorXd RobotModel::GetMeshcatJointSliderPositions() const {
  Eigen::VectorXd q(meshcat_joint_index_name_map_->size());
  for (const auto& [q_index, joint_name] : *meshcat_joint_index_name_map_) {
    const auto q_val {meshcat_->GetSliderValue(joint_name)};
    q(q_index) = q_val;
  }
  return q;
}

void RobotModel::SaveTrajectoryAsMeshcatHtml(
    const std::string filename, const std::vector<Eigen::VectorXd>& q_vec,
    const std::vector<double>& times_vec) const {
  DRAKE_DEMAND(q_vec.size() == times_vec.size());
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  meshcat_->StartRecording();
  for (size_t i = 0; i < q_vec.size(); ++i) {
    const auto q = q_vec.at(i);
    DRAKE_THROW_UNLESS(q.size() == plant().num_positions());
    SetMeshcatPositions(q);
    PublishMeshcatContext();
    meshcat_diagram_context_->SetTime(times_vec.at(i));
  }
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
  const auto html_string = meshcat_->StaticHtml();
  std::ofstream out(filename);
  out << html_string;
  out.close();
}

void RobotModel::SaveTrajectoryAsMeshcatHtml(
    const std::string filename,
    const drake::trajectories::Trajectory<double>& traj, double delta_t) const {
  std::vector<Eigen::VectorXd> q_vec;
  std::vector<double> times_vec;
  for (double t = traj.start_time(); t <= traj.end_time(); t += delta_t) {
    q_vec.push_back(traj.value(t));
    times_vec.push_back(t);
  }
  SaveTrajectoryAsMeshcatHtml(filename, q_vec, times_vec);
}

void RobotModel::DisplayTrajectoryInMeshcat(
    const std::vector<Eigen::VectorXd>& q_vec,
    const std::vector<double>& t_seconds_vec) const {
  DRAKE_DEMAND(q_vec.size() == t_seconds_vec.size());
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  meshcat_->StartRecording();
  for (size_t i = 0; i < q_vec.size(); ++i) {
    const auto q = q_vec.at(i);
    SetMeshcatPositions(q);
    PublishMeshcatContext();
    meshcat_diagram_context_->SetTime(t_seconds_vec.at(i));
  }
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
}

void RobotModel::DisplayTrajectoryInMeshcat(
    const drake::trajectories::Trajectory<double>& ppt, double delta_t) const {
  DRAKE_THROW_UNLESS(meshcat_ != nullptr);
  meshcat_->StartRecording();
  for (double t = ppt.start_time(); t <= ppt.end_time(); t += delta_t) {
    const auto q = ppt.value(t);
    SetMeshcatPositions(q);
    PublishMeshcatContext();
    meshcat_diagram_context_->SetTime(t);
  }
  meshcat_->StopRecording();
  meshcat_->PublishRecording();
}

void RobotModel::ColorCollidingBodiesInMeshcat(
    const drake::planning::RobotClearance& clearance,
    std::optional<double> recording_time) const {
  if (meshcat_ == nullptr) {
    logging::log()->warn(
        "RobotModel:ColorCollidingBodiesInMeshcat: no meshcat "
        "instance to color colliding bodies");
    return;
  }
  std::set<drake::multibody::BodyIndex> colliding_indices;
  for (int i {0}; i < clearance.size(); ++i) {
    const auto robot_index = clearance.robot_indices().at(i);
    const auto other_index = clearance.other_indices().at(i);
    // Ignore collisions with the added geometry body index;
    // those are colored using the collision checker.
    if (robot_index != added_geometry_body_index()
        && !colliding_indices.count(robot_index)) {
      colliding_indices.insert(robot_index);
    }
    if (other_index != added_geometry_body_index()
        && !colliding_indices.count(other_index)) {
      colliding_indices.insert(other_index);
    }
  }
  // start first set of bodies
  if (colliding_bodies_ptr_ == nullptr) {
    colliding_bodies_ptr_ =
        std::make_unique<std::set<drake::multibody::BodyIndex>>();
  }

  // Find bodies that changed state
  std::set<drake::multibody::BodyIndex> newly_colliding;
  std::set<drake::multibody::BodyIndex> newly_free;

  // Bodies that are now colliding but weren't before
  std::set_difference(colliding_indices.begin(), colliding_indices.end(),
                      colliding_bodies_ptr_->begin(),
                      colliding_bodies_ptr_->end(),
                      std::inserter(newly_colliding, newly_colliding.begin()));

  // Bodies that were colliding but aren't anymore
  std::set_difference(colliding_bodies_ptr_->begin(),
                      colliding_bodies_ptr_->end(), colliding_indices.begin(),
                      colliding_indices.end(),
                      std::inserter(newly_free, newly_free.begin()));

  // Highlight colliding bodies in red
  for (const auto& body_index : newly_colliding) {
    const auto& body {plant().get_body(body_index)};
    drake::geometry::ProximityProperties proximity_properties;
    auto color {body_index_color_map_.at(body_index).at("collision")};
    color.update(1.0, 0.0, 0.0);
    SetBodyColorsByProperty(body, color);
  }
  for (const auto& body_index : newly_free) {
    const auto& body {plant().get_body(body_index)};
    SetBodyColorsByProperty(
        body, body_index_color_map_.at(body_index).at("collision"));
  }
  // replace colliding_bodies_ptr_ with colliding_indices
  colliding_bodies_ptr_->clear();
  for (const auto& body_index : colliding_indices) {
    colliding_bodies_ptr_->insert(body_index);
  }
  // For recording purposes only
  if (recording_time.has_value()) {
    for (int i = 0; i < plant().num_bodies(); ++i) {
      const auto body_index = drake::multibody::BodyIndex(i);
      const auto& body = plant().get_body(body_index);
      // Set the color of the body in meshcat
      const auto model_name =
          plant().GetModelInstanceName(body.model_instance());
      const auto path_c =
          fmt::format("/drake/colliding/{}/{}", model_name, body.name());
      if (colliding_indices.count(body_index) == 0) {
        meshcat_->SetProperty(path_c, "visible", false, recording_time.value());
      } else {
        meshcat_->SetProperty(path_c, "visible", true, recording_time.value());
      }
    }
  }
}

}  // namespace motion
