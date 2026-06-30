/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */
#include "robot_model.h"

#include <drake/common/copyable_unique_ptr.h>
#include <drake/common/hash.h>
#include <drake/common/parallelism.h>
#include <drake/common/random.h>
#include <drake/geometry/optimization/cartesian_product.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/geometry/optimization/hyperellipsoid.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/geometry/shape_specification.h>
#include <drake/multibody/parsing/parser.h>
#include <drake/multibody/tree/multibody_tree.h>

#include "drake/geometry/proximity/obj_to_surface_mesh.h"
#include "planning_service/common/misc_utils.h"
namespace motion {

namespace {
class MeshcatPortAllocator {
 public:
  static int GetNextAvailablePort(const int x) {
    static std::atomic<int> next_port {x};
    return next_port++;
  }
};

std::shared_ptr<drake::geometry::Meshcat> MakeMeshcat(
    const RobotMeshcatParams& meshcat_params) {
  const auto reserved_ports {
      meshcat_params.reserved_ports.value_or(std::vector<int>())};
  if (meshcat_params.port.has_value()) {
    const auto port {meshcat_params.port.value()};
    if (common::utils::contains(reserved_ports, port)) {
      throw std::runtime_error(
          "A port was requested which is also in reserved ports");
    }
    return std::make_shared<drake::geometry::Meshcat>(port);
  }
  int port {-1};
  do {
    port = MeshcatPortAllocator::GetNextAvailablePort(7000);
  } while (common::utils::contains(reserved_ports, port));
  return std::make_shared<drake::geometry::Meshcat>(port);
}

std::map<drake::multibody::ModelInstanceIndex, drake::planning::DofMask>
MakeModelInstanceDofMasks(
    const drake::multibody::MultibodyPlant<double>& plant) {
  std::map<drake::multibody::ModelInstanceIndex, drake::planning::DofMask>
      instance_dof_masks;
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    auto model_instance = drake::multibody::ModelInstanceIndex(i);
    int num_positions = plant.num_positions(model_instance);
    if (num_positions > 0) {
      instance_dof_masks.emplace(
          model_instance,
          drake::planning::DofMask::MakeFromModel(plant, model_instance));
    }
  }
  logging::log()->info(
      "RobotModel:MakeModelInstanceDofMasks: Created dof masks for {} model "
      "instances.",
      instance_dof_masks.size());
  return instance_dof_masks;
}

// This function partitions the bodies in the plant into groups of bodies that
// are connected by weld or fixed joints. Each group of bodies can be treated
// as a single mobile body. The function returns a vector of vectors of body
// indices, where each inner vector represents a group of bodies that are
// connected by weld or fixed joints.
std::vector<std::vector<drake::multibody::BodyIndex>> PartitionMobileBodies(
    const drake::multibody::MultibodyPlant<double>& plant) {
  std::vector<std::vector<drake::multibody::BodyIndex>> body_groups;
  const auto joint_indices = plant.GetJointIndices();
  for (const auto& joint_index : joint_indices) {
    const auto& joint = plant.get_joint(joint_index);
    const auto parent_body = joint.parent_body().index();
    const auto child_body = joint.child_body().index();
    if (joint.type_name() == "weld") {
      // Both parent and child bodies are welded together, so they should be in
      // the same group
      bool group_found = false;
      for (auto& body_group : body_groups) {
        if (std::find(body_group.begin(), body_group.end(), parent_body)
            != body_group.end()) {
          body_group.push_back(child_body);
          group_found = true;
          break;
        } else if (std::find(body_group.begin(), body_group.end(), child_body)
                   != body_group.end()) {
          body_group.push_back(parent_body);
          group_found = true;
          break;
        }
      }
      if (!group_found) {
        // Create a new group with both bodies
        body_groups.push_back({parent_body, child_body});
      }
    } else {
      // parent goes to one group, child goes to another
      bool parent_group_found = false;
      for (auto& body_group : body_groups) {
        if (std::find(body_group.begin(), body_group.end(), parent_body)
            != body_group.end()) {
          parent_group_found = true;
          break;
        }
      }
      if (!parent_group_found) {
        // Make a new group with parent body
        body_groups.push_back({parent_body});
      }
      bool child_group_found = false;
      for (auto& body_group : body_groups) {
        if (std::find(body_group.begin(), body_group.end(), child_body)
            != body_group.end()) {
          child_group_found = true;
          break;
        }
      }
      if (!child_group_found) {
        // Make a new group with child body
        body_groups.push_back({child_body});
      }
    }
    // Now let's go through the body groups. If any two have any body in common,
    // merge them.
    for (int i = 0; i < std::ssize(body_groups); ++i) {
      for (int j = i + 1; j < std::ssize(body_groups); ++j) {
        auto& group_i = body_groups[i];
        auto& group_j = body_groups[j];
        // Check if they have any body in common
        bool has_common_body = false;
        for (const auto& body_index_i : group_i) {
          if (std::find(group_j.begin(), group_j.end(), body_index_i)
              != group_j.end()) {
            has_common_body = true;
            break;
          }
        }
        if (has_common_body) {
          // Merge the two groups, but remove duplicates
          group_i.insert(group_i.end(), group_j.begin(), group_j.end());
          // Remove duplicates
          std::sort(group_i.begin(), group_i.end());
          group_i.erase(std::unique(group_i.begin(), group_i.end()),
                        group_i.end());
          // Remove group_j from body_groups
          body_groups.erase(body_groups.begin() + j);
          --j;  // Adjust index after erasing
        }
      }
    }
  }
  // Let's log the body groups
  logging::log()->info(
      "RobotModel: Partitioned {} bodies into {} mobile groups",
      plant.num_bodies(), body_groups.size());
  int sum_body_group_sizes = 0;
  for (const auto& body_group : body_groups) {
    sum_body_group_sizes += body_group.size();
  }
  logging::log()->debug("RobotModel: Total number of bodies in groups: {}",
                        sum_body_group_sizes);
  // print the bodies in each group:
  for (int i = 0; i < std::ssize(body_groups); ++i) {
    const auto& body_group = body_groups[i];
    logging::log()->debug("RobotModel: Body group {} has {} bodies: ", i,
                          body_group.size());
    for (const auto& body_index : body_group) {
      const auto& body =
          plant.get_body(drake::multibody::BodyIndex(body_index));
      logging::log()->debug("RobotModel: Body group {}: {}", i, body.name());
    }
  }
  return body_groups;
}

drake::multibody::ModelInstanceIndex find_parent_model_index(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::ModelInstanceIndex& idx) {
  for (const auto& joint_index : plant.GetJointIndices(idx)) {
    const auto& joint {plant.get_joint(joint_index)};
    auto parent_idx = joint.parent_body().model_instance();
    if (parent_idx != idx) {
      return parent_idx;
    }
  }
  auto msg = fmt::format(
      "RobotModel:FindParentModelIndex: No parent model index found for {}: {}",
      idx, plant.GetModelInstanceName(idx));
  throw std::runtime_error(msg);
}

std::pair<std::vector<drake::multibody::ModelInstanceIndex>,
          std::vector<std::vector<drake::multibody::ModelInstanceIndex>>>
group_kinematic_chains(const drake::multibody::MultibodyPlant<double>& plant) {
  std::vector<std::vector<drake::multibody::ModelInstanceIndex>> open_chains;
  // go through the model instances and partition them into env and open chains
  std::vector<drake::multibody::ModelInstanceIndex> env_indices;
  auto world_idx = drake::multibody::world_model_instance();
  auto default_idx = drake::multibody::default_model_instance();
  env_indices.push_back(world_idx);
  env_indices.push_back(default_idx);
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    auto idx = drake::multibody::ModelInstanceIndex(i);
    if (idx == world_idx || idx == default_idx) {
      continue;
    }
    auto model_idx = drake::multibody::ModelInstanceIndex(i);
    auto parent_idx = find_parent_model_index(plant, model_idx);
    // Check if parent is part of environment.
    if (std::find(env_indices.begin(), env_indices.end(), parent_idx)
        != env_indices.end()) {
      // We may have found a new chain!
      logging::log()->debug(
          "RobotModel:FindArms: The parent of {}: {} is {}:{} and actually "
          "part of environment",
          model_idx, plant.GetModelInstanceName(model_idx), parent_idx,
          plant.GetModelInstanceName(parent_idx));
      if (plant.num_positions(model_idx) == 0) {
        // No dofs. This actually belongs to the environment as well.
        env_indices.push_back(model_idx);
      } else {
        auto new_open_chain =
            std::vector<drake::multibody::ModelInstanceIndex>();
        new_open_chain.push_back(model_idx);
        open_chains.push_back(new_open_chain);
      }
    } else {
      bool assigned = false;
      for (auto& chain : open_chains) {
        if (std::find(chain.begin(), chain.end(), parent_idx) != chain.end()) {
          chain.push_back(model_idx);
          assigned = true;
          logging::log()->debug(
              "RobotModel:FindArms: Added {}:{} to arm with parent {}:{}",
              model_idx, plant.GetModelInstanceName(model_idx), parent_idx,
              plant.GetModelInstanceName(parent_idx));
          break;
        }
      }
      if (!assigned) {
        auto msg = fmt::format(
            "RobotModel:FindArms: The parent of {}:{} which is {}:{}"
            " is not part of the environment or any open kinematic chain",
            model_idx, plant.GetModelInstanceName(model_idx), parent_idx,
            plant.GetModelInstanceName(parent_idx));
        throw std::runtime_error(msg);
      }
    }
  }
  return std::make_pair(env_indices, open_chains);
}

}  // namespace

RobotModel::RobotModel(
    const std::string& xml_file,
    const drake::multibody::parsing::ModelDirectives& dmd,
    const std::optional<RobotMeshcatParams>& robot_meshcat_params,
    const std::vector<std::pair<std::string, int>>& continuous_revolute_joints,
    bool implicit_parallelism)
    : package_map_ {CreatePackageMapFromXmlFile(xml_file)},
      robot_meshcat_params_ {robot_meshcat_params},
      meshcat_ {robot_meshcat_params_ ? MakeMeshcat(*robot_meshcat_params_)
                                      : nullptr},
      parsed_model_ {ParseModel(dmd, package_map_, implicit_parallelism)},
      body_index_color_map_ {robot_meshcat_params_.has_value()
                                 ? ParseBodyColorMap()
                                 : body_index_color_map_t {}},
      continuous_revolute_joint_indices_ {
          CalcContinuousRevoluteJointIndices(continuous_revolute_joints)},
      instance_dof_masks_ {MakeModelInstanceDofMasks(
          parsed_model_.default_collision_checker->plant())},
      arms_ {FindArms(parsed_model_.default_collision_checker->plant(), dmd,
                      package_map_)},
      mobile_body_groups_ {PartitionMobileBodies(
          parsed_model_.default_collision_checker->plant())},
      holonomic_mapping_ {
          HolonomicMapping(parsed_model_.default_collision_checker->plant())},
      meshcat_diagram_context_ {
          robot_meshcat_params_.has_value()
              ? CreateMeshcatContext(body_index_color_map_)
              : nullptr},
      calc_pose_context_ {parsed_model_.default_collision_checker->model()
                              .CreateDefaultContext()} {
  if (robot_meshcat_params_.has_value()) {
    for (const auto& frame_scoped_name :
         robot_meshcat_params_->end_effector_frame_vec) {
      logging::log()->info(
          "RobotModel: Adding end effector frame {} to Meshcat visualization",
          frame_scoped_name);
      if (!HasScopedFrameNamed(frame_scoped_name)) {
        continue;
      }
      const auto& frame = GetScopedFrameByName(frame_scoped_name);
      AddFrameAxesToMeshcat(frame);
      frames_with_axes_ptr_->push_back(&frame);
    }
  } else {
    logging::log()->warn(
        "RobotModel: No Meshcat instance created, skipping end effector frame "
        "addition.");
  }
}

RobotModel::RobotArm::RobotArm(
    const drake::multibody::MultibodyPlant<double>& original_plant,
    const std::vector<drake::multibody::ModelInstanceIndex>& model_instances,
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
    const std::string& name, ArmIndex index)
    : original_plant_(original_plant),
      model_instances_(model_instances),
      plant_(std::move(plant)),
      arm_holonomic_mapping_ {HolonomicMapping(*plant_)},
      name_(name),
      index_(index) {
  logging::log()->info(
      "RobotModel: RobotArm: Created arm {}, name: {} with {} model "
      "instances and {} degrees of freedom",
      index_, name_, model_instances_.size(), plant_->num_positions());
  dof_mask_ = drake::planning::DofMask(original_plant_.num_positions(), false);
  for (const auto& model_instance : model_instances_) {
    dof_mask_ = dof_mask_.Union(drake::planning::DofMask::MakeFromModel(
        original_plant_, model_instance));
  }
  DRAKE_DEMAND(dof_mask_.count() == plant_->num_positions());
  DRAKE_DEMAND(dof_mask_.size() == original_plant_.num_positions());
}

Eigen::VectorXd RobotModel::RobotArm::GetPositionFromOriginalPlant(
    const Eigen::VectorXd& q) const {
  DRAKE_DEMAND(q.size() == original_plant_.num_positions());
  return dof_mask_.GetFromArray(q);
}

std::vector<RobotModel::RobotArm> RobotModel::FindArms(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::parsing::ModelDirectives& dmd,
    const drake::multibody::PackageMap& package_map) const {
  std::vector<RobotArm> arms;
  auto [env, open_chains] = group_kinematic_chains(plant);
  logging::log()->info("RobotModel:FindArms: Found {} open kinematic chains",
                       open_chains.size());
  for (int i = 0; i < std::ssize(open_chains); ++i) {
    auto& chain = open_chains[i];
    std::string name = "arm";
    for (const auto& model_instance : chain) {
      name += "_" + plant.GetModelInstanceName(model_instance);
    }
    // Go through the DMD and find all the model instances that belong to this
    // arm.
    drake::multibody::parsing::ModelDirectives sub_dmd;
    for (auto& directive : dmd.directives) {
      if (directive.add_collision_filter_group.has_value()) {
        continue;  // skip collision filter groups for arm plants - they don't
                   // matter because collision checking is done at the robot
                   // level anyways.
      } else if (directive.add_directives.has_value()) {
        // ToDo(Sadra): Implement this if needed.
        throw std::runtime_error(
            "RobotModel:FindArms: add_directives not supported yet. Consider "
            "rewriting the DMD or develop code to support it.");
      } else if (directive.add_frame.has_value()) {
        const auto& frame = drake::multibody::parsing::GetScopedFrameByName(
            plant, directive.add_frame.value().name);
        if (std::find(chain.begin(), chain.end(), frame.model_instance())
            != chain.end()) {
          // Ths frame belongs to this arm.
          sub_dmd.directives.push_back(directive);
        }
      } else if (directive.add_model.has_value()) {
        const auto& model =
            plant.GetModelInstanceByName(directive.add_model.value().name);
        if (std::find(chain.begin(), chain.end(), model) != chain.end()) {
          // Ths model belongs to this arm.
          sub_dmd.directives.push_back(directive);
        }
      } else if (directive.add_model_instance.has_value()) {
        throw std::runtime_error(
            "RobotModel:FindArms: add_model_instance not supported yet. "
            "Consider removing it from the DMD.");
      } else if (directive.add_weld.has_value()) {
        const auto& frame_parent =
            drake::multibody::parsing::GetScopedFrameByName(
                plant, directive.add_weld.value().parent);
        const auto& frame_child =
            drake::multibody::parsing::GetScopedFrameByName(
                plant, directive.add_weld.value().child);
        if (std::find(chain.begin(), chain.end(), frame_child.model_instance())
            != chain.end()) {
          // Ths weld belongs to this arm. The parent frame also MUST belong to
          // this arm or the environment.
          sub_dmd.directives.push_back(directive);
          DRAKE_DEMAND(std::find(chain.begin(), chain.end(),
                                 frame_parent.model_instance())
                           != chain.end()
                       || std::find(env.begin(), env.end(),
                                    frame_parent.model_instance())
                              != env.end());
        }
      } else {
        throw std::runtime_error(
            "RobotModel:FindArms: Unknown directive encountered in DMD.");
      }
    }
    drake::multibody::MultibodyPlant<double>* arm_plant =
        new drake::multibody::MultibodyPlant<double>(0.001);
    drake::multibody::Parser parser(arm_plant);
    parser.package_map() = package_map;
    drake::multibody::parsing::ProcessModelDirectives(sub_dmd, &parser);
    arm_plant->Finalize();
    arms.push_back(RobotArm(
        plant, chain,
        std::unique_ptr<drake::multibody::MultibodyPlant<double>>(arm_plant),
        name, ArmIndex(i)));
  }
  return arms;
}

RobotModel::ParsedModel RobotModel::ParseModel(
    const drake::multibody::parsing::ModelDirectives& dmd,
    const drake::multibody::PackageMap& package_map,
    bool implicit_parallelism) {
  const auto start {std::chrono::high_resolution_clock::now()};
  logging::log()->debug("RobotModel:ParseModel: Parsing model");
  auto robot_diagram_builder {
      drake::planning::RobotDiagramBuilder<double>(0.001)};
  auto& mutable_plant {robot_diagram_builder.plant()};
  mutable_plant.set_discrete_contact_approximation(
      drake::multibody::DiscreteContactApproximation::kSap);
  auto& parser {robot_diagram_builder.parser()};
  parser.package_map() = package_map;
  const auto model_instance_info_vec {
      drake::multibody::parsing::ProcessModelDirectives(dmd, &parser)};
  for (const auto& model_instance_info : model_instance_info_vec) {
    logging::log()->debug("RobotModel:ParseModel: Parsed {} from {}",
                          model_instance_info.model_name,
                          model_instance_info.model_path);
  }
  // Add a dedicated 'added_geometry' body for dynamically-added collision
  // shapes; Organizing these shapes in this way allows us to manage them
  // easier, and provide more accurate information in viz and debugging
  const auto added_geo_instance =
      mutable_plant.AddModelInstance("added_geometry");
  const auto& added_geo_body = mutable_plant.AddRigidBody(
      "added_geometry", added_geo_instance,
      drake::multibody::SpatialInertia<double>::Zero());
  mutable_plant.WeldFrames(mutable_plant.world_frame(),
                           added_geo_body.body_frame());
  const auto added_geometry_body_index = added_geo_body.index();
  mutable_plant.Finalize();
  // if meshcat is enabled, load the visualizers
  if (meshcat_) {
    auto& builder {robot_diagram_builder.builder()};
    const auto& scene_graph {robot_diagram_builder.scene_graph()};
    if (robot_meshcat_params_->visual) {
      drake::geometry::MeshcatVisualizerParams visual_params;
      visual_params.role = drake::geometry::Role::kIllustration;
      visual_params.prefix = "visual";
      visual_params.default_color = drake::geometry::Rgba(0.8, 0.2, 0.2, 1.0);
      drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
          &builder, scene_graph, meshcat_, visual_params);
      logging::log()->info("RobotModel:ParseModel: Added visual visualizer");
    }
    if (robot_meshcat_params_->collision) {
      drake::geometry::MeshcatVisualizerParams collision_params;
      collision_params.role = drake::geometry::Role::kProximity;
      collision_params.prefix = "collision";
      collision_params.default_color =
          drake::geometry::Rgba(1.0, 1.0, 1.0, 0.5);
      drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
          &builder, scene_graph, meshcat_, collision_params);
      // Let's do another collision visualizer, with all red colors
      drake::geometry::MeshcatVisualizerParams collision_red_params;
      collision_red_params.role = drake::geometry::Role::kProximity;
      collision_red_params.prefix = "colliding";
      collision_red_params.default_color =
          drake::geometry::Rgba(1.0, 0.0, 0.0, 0.5);
      collision_red_params.visible_by_default = false;
      drake::geometry::MeshcatVisualizer<double>::AddToBuilder(
          &builder, scene_graph, meshcat_, collision_red_params);
      logging::log()->info("RobotModel:ParseModel: Added collision visualizer");
    }
  }
  const auto& plant {robot_diagram_builder.plant()};
  auto [env, open_chains] = group_kinematic_chains(plant);
  std::vector<drake::multibody::ModelInstanceIndex> robot_indices_vec;
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    const auto model_instance {drake::multibody::ModelInstanceIndex(i)};
    for (const auto& chain : open_chains) {
      if (std::find(chain.begin(), chain.end(), model_instance)
          != chain.end()) {
        robot_indices_vec.push_back(model_instance);
        break;
      }
    }
  }
  drake::planning::CollisionCheckerParams collision_checker_params;
  auto robot_diagram {robot_diagram_builder.Build()};
  collision_checker_params.model = std::move(robot_diagram);
  const drake::planning::ConfigurationDistanceFunction dist {
      [](const Eigen::VectorXd& a, const Eigen::VectorXd& b) {
        return (b - a).norm();
      }};
  collision_checker_params.configuration_distance_function = dist;
  collision_checker_params.edge_step_size = 0.01;
  collision_checker_params.env_collision_padding = 0.00;
  collision_checker_params.self_collision_padding = 0.00;
  collision_checker_params.robot_model_instances = robot_indices_vec;
  collision_checker_params.implicit_context_parallelism =
      drake::Parallelism(implicit_parallelism);
  // Build the map of group names to their constituent indices
  std::map<std::string, std::vector<drake::multibody::BodyIndex>> body_map;
  const auto& parser_groups {parser.GetCollisionFilterGroups()};
  for (const auto& [group_name, group] : parser_groups.groups()) {
    body_map[group_name] = {};
    for (const auto& body_name : group) {
      const auto body_index {
          drake::multibody::parsing::GetScopedFrameByName(plant, body_name)
              .body()
              .index()};
      body_map[group_name].push_back(body_index);
    }
  }
  // For convenience, collect the set of all body indices
  std::vector<drake::multibody::BodyIndex> body_idxs;
  for (int i {0}; i < plant.num_model_instances(); ++i) {
    const auto& body_idxs_i {
        plant.GetBodyIndices(drake::multibody::ModelInstanceIndex(i))};
    body_idxs.reserve(body_idxs.size()
                      + distance(body_idxs_i.begin(), body_idxs_i.end()));
    body_idxs.insert(body_idxs.end(), body_idxs_i.begin(), body_idxs_i.end());
  }
  const auto duration {std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start)};
  logging::log()->info("RobotModel:ParseModel: Parsed model in {} ms",
                       duration.count());
  return ParsedModel {
      .default_collision_checker =
          std::make_unique<drake::planning::SceneGraphCollisionChecker>(
              std::move(collision_checker_params)),
      .body_indices = std::move(body_idxs),
      .filter_group_body_indices = std::move(body_map),
      .added_geometry_body_index = added_geometry_body_index,
  };
}

Eigen::VectorXd RobotModel::ToGeneralizedPosition(
    const system_conf_t& sysconf) const {
  for (const auto& [robot_name, conf] : sysconf) {
    if (!plant().HasModelInstanceNamed(robot_name)) {
      throw std::runtime_error(fmt::format(
          "RobotModel:ToGeneralizedPosition: Sysconf includes "
          "robot name: {} which is not specified in the robot model.",
          robot_name));
    }
  }
  Eigen::VectorXd q {Eigen::VectorXd::Zero(plant().num_positions())};
  for (int i {0}; i < plant().num_model_instances(); ++i) {
    const auto model_instance {drake::multibody::ModelInstanceIndex(i)};
    if (plant().num_positions(model_instance) > 0) {
      const auto& robot_name {plant().GetModelInstanceName(model_instance)};
      if (sysconf.count(robot_name) != 1) {
        throw RobotModel::PartialSysconfError(fmt::format(
            "RobotModel:ToGeneralizedPosition: Sysconf fails to include "
            "robot name: {}, which is specified in the robot model.",
            robot_name));
      }
      plant().SetPositionsInArray(model_instance, sysconf.at(robot_name), &q);
    }
  }
  return q;
}

bool RobotModel::IsSysconfComplete(const system_conf_t& sysconf) const {
  for (int i {0}; i < plant().num_model_instances(); ++i) {
    const auto model_instance {drake::multibody::ModelInstanceIndex(i)};
    if (plant().num_positions(model_instance) > 0) {
      const auto& robot_name {plant().GetModelInstanceName(model_instance)};
      if (sysconf.count(robot_name) != 1) {
        return false;
      }
    }
  }
  return true;
}

system_conf_t RobotModel::ToSystemConf(const Eigen::VectorXd& q) const {
  system_conf_t sysconf;
  for (int i {0}; i < plant().num_model_instances(); ++i) {
    const auto model_instance {drake::multibody::ModelInstanceIndex(i)};
    if (plant().num_positions(model_instance) > 0) {
      sysconf.emplace(plant().GetModelInstanceName(model_instance),
                      plant().GetPositionsFromArray(model_instance, q));
    }
  }
  return sysconf;
}

system_conf_t RobotModel::ReduceSystemConf(const system_conf_t& sysconf) const {
  system_conf_t reduced_sysconf;
  for (const auto& [robot_name, conf] : sysconf) {
    DRAKE_THROW_UNLESS(plant().HasModelInstanceNamed(robot_name));
    const auto model_instance {plant().GetModelInstanceByName(robot_name)};
    DRAKE_THROW_UNLESS(plant().num_positions(model_instance) > 0);
    reduced_sysconf.emplace(
        robot_name, holonomic_mapping_.ReduceInstance(model_instance, conf));
  }
  return reduced_sysconf;
}

system_conf_t RobotModel::LiftSystemConf(const system_conf_t& sysconf) const {
  system_conf_t lifted_sysconf;
  for (const auto& [robot_name, conf] : sysconf) {
    DRAKE_THROW_UNLESS(plant().HasModelInstanceNamed(robot_name));
    const auto model_instance {plant().GetModelInstanceByName(robot_name)};
    DRAKE_THROW_UNLESS(plant().num_positions(model_instance) > 0);
    lifted_sysconf.emplace(
        robot_name, holonomic_mapping_.LiftInstance(model_instance, conf));
  }
  return lifted_sysconf;
}

int RobotModel::GetModelStartIndex(
    const drake::multibody::ModelInstanceIndex& idx) const {
  return GetModelStartIndex(plant(), idx);
}

int RobotModel::GetModelStartIndex(
    const drake::multibody::MultibodyPlant<double>& plant,
    const drake::multibody::ModelInstanceIndex& idx) {
  // drake now guarantees that the block of positions inside generalized
  // positions is connected. For example, if the whole robot has 4 positions,
  // and a model index has 2 positions, then those positions (marked by x) will
  // appear as e.g., xxoo or oxxo or ooxx, but never xoxo or oxox or xoox.
  DRAKE_THROW_UNLESS(plant.num_positions(idx) > 0);
  auto dof_mask = drake::planning::DofMask::MakeFromModel(plant, idx);
  DRAKE_DEMAND(dof_mask.size() == plant.num_positions());
  // Return the first position that is not masked out.
  int start_position = -1;
  for (int i = 0; i < dof_mask.size(); ++i) {
    if (dof_mask[i]) {
      start_position = i;
      break;
    }
  }
  DRAKE_DEMAND(start_position >= 0);
  return start_position;
}

void RobotModel::SetIdleModelsConfigToRef(
    Eigen::VectorXd* q, const Eigen::VectorXd& q_ref,
    const std::set<drake::multibody::ModelInstanceIndex>&
        active_model_instances) const {
  if (active_model_instances.empty()) {
    // All model instances are idle
    *q = q_ref;
  }
  auto q_lifted = holonomic_mapping_.Lift(*q);
  auto q_ref_lifted = holonomic_mapping_.Lift(q_ref);
  // First, let's find the inactive model instances
  for (const auto& [model_idx, _] : instance_dof_masks_) {
    // If belongs to inactive model instance
    if (active_model_instances.count(model_idx) == 0) {
      // Then let's set the positions to the reference configuration
      auto q_ref_instance =
          plant().GetPositionsFromArray(model_idx, q_ref_lifted);
      plant().SetPositionsInArray(model_idx, q_ref_instance, &q_lifted);
    }
  }
  *q = holonomic_mapping_.Reduce(q_lifted);
}

Eigen::VectorXd RobotModel::SetIdleModelsConfigToRef(
    const Eigen::VectorXd& q, const Eigen::VectorXd& q_ref,
    const std::set<drake::multibody::ModelInstanceIndex>&
        active_model_instances) const {
  Eigen::VectorXd q_new = q;
  SetIdleModelsConfigToRef(&q_new, q_ref, active_model_instances);
  return q_new;
}

void RobotModel::SetIdleArmsConfigToRef(
    Eigen::VectorXd* q, const Eigen::VectorXd& q_ref,
    const std::set<drake::multibody::ModelInstanceIndex>&
        active_model_instances) const {
  DRAKE_DEMAND(num_arms() > 0);
  // First, let's find the inactive arms
  std::vector<ArmIndex> active_arm_indices;
  for (const auto& model_instance : active_model_instances) {
    auto arm_index = get_arm_index(model_instance);
    if (!arm_index.is_valid()) {
      // This model instance does not belong to any arm. It is part of the
      // environment.
      continue;
    }
    active_arm_indices.push_back(arm_index);
  }
  std::vector<ArmIndex> idle_arm_indices;
  for (int i = 0; i < num_arms(); ++i) {
    auto index_i = ArmIndex(i);
    if (std::find(active_arm_indices.begin(), active_arm_indices.end(), index_i)
        == active_arm_indices.end()) {
      idle_arm_indices.push_back(index_i);
    }
  }
  auto q_lifted = holonomic_mapping_.Lift(*q);
  auto q_ref_lifted = holonomic_mapping_.Lift(q_ref);
  for (int i = 0; i < plant().num_model_instances(); ++i) {
    const auto model_instance {drake::multibody::ModelInstanceIndex(i)};
    // If belongs to inactive arm
    if (plant().num_positions(model_instance) > 0) {
      auto arm = get_arm_index(model_instance);
      if (std::find(idle_arm_indices.begin(), idle_arm_indices.end(), arm)
          != idle_arm_indices.end()) {
        // Then let's set the positions to the reference configuration
        auto q_ref_instance =
            plant().GetPositionsFromArray(model_instance, q_ref_lifted);
        plant().SetPositionsInArray(model_instance, q_ref_instance, &q_lifted);
      }
    }
  }
  *q = holonomic_mapping_.Reduce(q_lifted);
  logging::log()->info(
      "RobotModel:SetIdleArmsConfigToRef: Set {} idle arms to reference "
      "configuration for {} total dofs.",
      idle_arm_indices.size(), q->size());
}

drake::math::RigidTransformd RobotModel::CalcRelativeTransform(
    const Eigen::VectorXd& q, const drake::multibody::Frame<double>& frame_A,
    const drake::multibody::Frame<double>& frame_B) const {
  auto q_lifted = holonomic_mapping_.Lift(q);
  auto& plant_context {
      plant().GetMyMutableContextFromRoot(calc_pose_context_.get())};
  plant().SetPositions(&plant_context, q_lifted);
  return plant().CalcRelativeTransform(plant_context, frame_A, frame_B);
}

const drake::multibody::Frame<double>& RobotModel::GetScopedFrameByName(
    const std::string_view name) const {
  return drake::multibody::parsing::GetScopedFrameByName(plant(), name.data());
}

bool RobotModel::HasFilterGroupName(const std::string& group_name) const {
  return parsed_model_.filter_group_body_indices.count(group_name) > 0;
}

void RobotModel::SetFixedOffsetFramePoseInParentFrame(
    const drake::multibody::Frame<double>& frame,
    const drake::math::RigidTransformd& offset) const {
  DRAKE_THROW_UNLESS(&frame.GetParentPlant() == &plant());
  // Check if I can cast the frame to FixedOffsetFrame
  const auto* fixed_offset_frame =
      dynamic_cast<const drake::multibody::FixedOffsetFrame<double>*>(&frame);
  if (!fixed_offset_frame) {
    auto msg = fmt::format(
        "RobotModel:SetFixedOffsetFramePoseInParentFrame: Frame {} is not a "
        "FixedOffsetFrame. Cannot set pose in parent frame.",
        frame.name());
    throw std::runtime_error(msg);
  }
  logging::log()->info(
      "RobotModel:SetFixedOffsetFramePoseInParentFrame: Setting pose of "
      "frame {} in parent frame {} to {}",
      fixed_offset_frame->name(), fixed_offset_frame->parent_frame().name(),
      offset);
  fixed_offset_frame->SetPoseInParentFrame(calc_pose_context_.get(), offset);
  // Let's add the frames to meshcat
  if (meshcat_) {
    AddFrameAxesToMeshcat(*fixed_offset_frame, 0.8, 0.05);
    AddFrameAxesToMeshcat(fixed_offset_frame->parent_frame(), 0.3, 0.05);
  }
}

const drake::multibody::RigidBody<double>&
RobotModel::GetVisualBodyInTheSameMobileGroup(
    const drake::multibody::Frame<double>& frame) const {
  DRAKE_THROW_UNLESS(&frame.GetParentPlant() == &plant());
  const auto frame_body_index = frame.body().index();
  // First, find the index of mobile body groups.
  std::optional<int> maybe_mobile_group_index;
  for (int i = 0; i < std::ssize(mobile_body_groups_); ++i) {
    const auto& body_group = mobile_body_groups_[i];
    if (std::find(body_group.begin(), body_group.end(), frame_body_index)
        != body_group.end()) {
      // We found the body group that contains the frame's body.
      maybe_mobile_group_index = i;
      break;
    }
  }
  if (!maybe_mobile_group_index.has_value()) {
    auto msg = fmt::format(
        "RobotModel:GetVisualBodyInTheSameMobileGroup: The body {} of frame {} "
        "is not part of any mobile body group.",
        frame.body().name(), frame.name());
    throw std::runtime_error(msg);
  }
  // Now, go through the bodies in this mobile group and find a collision body.
  int mobile_group_index = maybe_mobile_group_index.value();
  for (const auto& body_index : mobile_body_groups_[mobile_group_index]) {
    const auto& body = plant().get_body(body_index);
    if (plant().GetVisualGeometriesForBody(body).size() > 0) {
      logging::log()->debug(
          "RobotModel:GetVisualBodyInTheSameMobileGroup: Found visual body {} "
          "in the same mobile group as frame {}",
          body.name(), frame.name());
      return body;
    }
  }
  throw std::runtime_error(fmt::format(
      "RobotModel:GetVisualBodyInTheSameMobileGroup: Frame {} is in a "
      "mobile body group, but no collision body found in that group.",
      frame.name()));
}

drake::multibody::PackageMap RobotModel::CreatePackageMapFromXmlFile(
    const std::string& xml_file) const {
  drake::multibody::PackageMap package_map;
  package_map.AddPackageXml(xml_file);
  return package_map;
}

// using HashAlgorithm = drake::hash_value::HashAlgorithm;
void RobotModel::HashKinematics(drake::DelegatingHasher* hasher) const {
  struct KinematicInfo {
    drake::multibody::BodyIndex parent_index;
    std::vector<drake::math::RigidTransformd> joint_transforms;
    drake::multibody::JointIndex parent_joint_index;
    bool has_collision_downstream = false;
  };
  std::map<drake::multibody::BodyIndex, KinematicInfo> kinematic_info_tree;
  const auto joint_indices = plant().GetJointIndices();
  const auto& plant {this->plant()};
  for (const auto& joint_index : joint_indices) {
    const auto& joint = plant.get_joint(joint_index);
    const auto& frame_on_parent = joint.frame_on_parent();
    const auto& frame_on_child = joint.frame_on_child();
    const auto child_body_idx = frame_on_child.body().index();
    const auto parent_body_idx = frame_on_parent.body().index();
    KinematicInfo kinematic_info;
    kinematic_info.parent_index = parent_body_idx;
    kinematic_info.parent_joint_index = joint_index;
    // Check if the joint is weld
    const auto* weld_joint =
        dynamic_cast<const drake::multibody::WeldJoint<double>*>(&joint);
    if (weld_joint && !weld_joint->X_FM().IsExactlyIdentity()) {
      kinematic_info.joint_transforms.push_back(weld_joint->X_FM());
    }
    if (!frame_on_parent.GetFixedPoseInBodyFrame().IsExactlyIdentity()) {
      kinematic_info.joint_transforms.push_back(
          frame_on_parent.GetFixedPoseInBodyFrame());
    }
    if (!frame_on_child.GetFixedPoseInBodyFrame().IsExactlyIdentity()) {
      kinematic_info.joint_transforms.push_back(
          frame_on_child.GetFixedPoseInBodyFrame());
    }
    // First, let's check the node itself
    kinematic_info.has_collision_downstream =
        plant.GetCollisionGeometriesForBody(plant.get_body(child_body_idx))
            .size()
        > 0;
    // Insert it into the tree
    DRAKE_DEMAND(kinematic_info_tree.count(child_body_idx) == 0);
    kinematic_info_tree[child_body_idx] = kinematic_info;
  }
  // Thw world frame is the root
  KinematicInfo world_node;
  world_node.parent_index = plant.world_body().index();
  world_node.parent_joint_index = drake::multibody::JointIndex(0);
  world_node.joint_transforms = {};
  world_node.has_collision_downstream = true;  // to stop propagation
  kinematic_info_tree[plant.world_body().index()] = world_node;
  // Now, let's propagate the collision info up the tree
  for (const auto& [body_index, kinematic_info] : kinematic_info_tree) {
    if (body_index == plant.world_body().index()) {
      continue;  // skip the world
    }
    if (!kinematic_info.has_collision_downstream) {
      continue;  // nothing to propagate
    }
    auto parent_index = kinematic_info.parent_index;
    while (parent_index != plant.world_body().index()) {
      auto& parent_info = kinematic_info_tree[parent_index];
      if (parent_info.has_collision_downstream) {
        break;  // already marked
      }
      parent_info.has_collision_downstream = true;
      parent_index = parent_info.parent_index;
    }
  }
  // Now, let's hash the kinematic info of each node that has collision
  // downstream
  for (const auto& [body_index, kinematic_info] : kinematic_info_tree) {
    if (!kinematic_info.has_collision_downstream) {
      continue;  // skip nodes without collision downstream
    }
    for (const auto& x_tform : kinematic_info.joint_transforms) {
      hash_append(*hasher, x_tform);
    }
    const auto& joint_type_name =
        plant.get_joint(kinematic_info.parent_joint_index).type_name();
    drake::hash_append(*hasher, joint_type_name);
  }
  // Debugging: Uncomment to output the kinematic tree in dot format
  // std::ofstream dot_file("/logs/shokunin/kinematic_tree.dot");
  // dot_file << "digraph KinematicTree {\n";
  // for (const auto& [body_index, kinematic_info] : kinematic_info_tree) {
  //   const auto& body_name = plant.get_body(body_index).name();
  //   const auto& parent_name =
  //       plant.get_body(kinematic_info.parent_index).name();
  //   const bool has_collision =
  //       plant.GetCollisionGeometriesForBody(plant.get_body(body_index)).size()
  //       > 0;
  //   // If has_collision_downstream, make the node color red
  //   // If has_collision itself, make the node rectangle, otherwise ellipse
  //   dot_file << fmt::format(
  //       "\"{}\" [shape={}, style=filled, fillcolor={}];\n", body_name,
  //       has_collision ? "rectangle" : "ellipse",
  //       kinematic_info.has_collision_downstream ? "red" : "white");
  //   dot_file << fmt::format("\"{}\" -> \"{}\";\n", parent_name, body_name);
  // }
  // dot_file << "}\n";
  // dot_file.close();
}

std::vector<int> RobotModel::CalcContinuousRevoluteJointIndices(
    const std::vector<std::pair<std::string, int>>& continuous_revolute_joints)
    const {
  std::vector<int> continuous_revolute_joint_indices;
  for (const auto& [robot_name, position_index] : continuous_revolute_joints) {
    DRAKE_THROW_UNLESS(plant().HasModelInstanceNamed(robot_name));
    const auto model_instance_index {
        plant().GetModelInstanceByName(robot_name)};
    continuous_revolute_joint_indices.push_back(
        GetModelStartIndex(model_instance_index) + position_index);
  }
  return continuous_revolute_joint_indices;
}

bool RobotModel::AreAllVisualShapesEncapsulatedByCollisionShapes(
    double tol, const std::optional<Eigen::VectorXd>& evaluation_config) const {
  using ConvexSets = drake::geometry::optimization::ConvexSets;
  // Let's go with ever body
  const auto& plant = this->plant();
  const auto& scene_graph = this->scene_graph();
  const auto& mobile_body_groups = PartitionMobileBodies(plant);
  const auto& sg_query_port = scene_graph.get_query_output_port();
  if (evaluation_config.has_value()) {
    logging::log()->info(
        "RobotModel:AreAllVisualShapesEncapsulatedByCollisionShapes: Using "
        "evaluation configuration to set the plant state.");
    DRAKE_THROW_UNLESS(evaluation_config->size()
                       == holonomic_mapping().minimal_dim());
    // Let's set the plant to the evaluation configuration
    auto q_lifted = holonomic_mapping_.Lift(evaluation_config.value());
    auto& plant_context {
        plant.GetMyMutableContextFromRoot(calc_pose_context_.get())};
    plant.SetPositions(&plant_context, q_lifted);
    logging::log()->info(
        "RobotModel:AreAllVisualShapesEncapsulatedByCollisionShapes: Plant set "
        "to evaluation configuration (lifted): {}",
        q_lifted);
  }
  // Create a context for the scene graph
  const auto& sg_query =
      sg_query_port.Eval<drake::geometry::QueryObject<double>>(
          scene_graph.GetMyContextFromRoot(*calc_pose_context_));
  const auto& sg_inspector = scene_graph.model_inspector();
  std::vector<ConvexSets> mobile_collision_sets(mobile_body_groups.size(),
                                                ConvexSets());
  std::vector<std::string> uncovered_mesh_msgs = {};
  ConvexSets all_collision_sets;
  for (int i = 0; i < plant.num_bodies(); ++i) {
    auto body_index = drake::multibody::BodyIndex(i);
    const auto& body = plant.get_body(body_index);
    auto collision_geos = plant.GetCollisionGeometriesForBody(body);
    ConvexSets collision_sets;
    for (const auto collision_geo : collision_geos) {
      // Check if the collision geometry is encapsulated by the visual
      // geometries
      const auto& shape = sg_inspector.GetShape(collision_geo);
      if (shape.type_name() == "Box") {
        auto hpolyhedron =
            std::make_unique<drake::geometry::optimization::HPolyhedron>(
                sg_query, collision_geo);
        collision_sets.push_back(
            drake::copyable_unique_ptr<
                drake::geometry::optimization::ConvexSet>(hpolyhedron));
      } else if (shape.type_name() == "Sphere") {
        auto hyperellipsoid =
            std::make_unique<drake::geometry::optimization::Hyperellipsoid>(
                sg_query, collision_geo);
        collision_sets.push_back(
            drake::copyable_unique_ptr<
                drake::geometry::optimization::ConvexSet>(hyperellipsoid));
      } else if (shape.type_name() == "Cylinder") {
        auto cartesian_product =
            std::make_unique<drake::geometry::optimization::CartesianProduct>(
                sg_query, collision_geo);
        collision_sets.push_back(
            drake::copyable_unique_ptr<
                drake::geometry::optimization::ConvexSet>(cartesian_product));
      } else if (shape.type_name() == "Mesh") {
        // We make convex hull of meshes, but they are inefficient for
        // encapsulation checks, so we give a warning
        logging::log()->warn(
            "RobotModel: Body {} has a mesh collision shape, which is not "
            "ideal. Please please use basic shapes (box, cylinder, "
            "sphere) for collision shapes to improve performance ",
            body.name());
        auto vpolytope =
            std::make_unique<drake::geometry::optimization::VPolytope>(
                sg_query, collision_geo);
        collision_sets.push_back(
            drake::copyable_unique_ptr<
                drake::geometry::optimization::ConvexSet>(vpolytope));
      } else {
        logging::log()->warn(
            "RobotModel: Body {} collison has {}, which is not "
            "supported yet to check encapsulation. Skipping it.",
            body.name(), shape.type_name());
      }
    }
    if (!evaluation_config.has_value()) {
      // Let's now insert the collision sets into the mobile body groups
      for (int j = 0; j < std::ssize(mobile_body_groups); ++j) {
        const auto& body_group = mobile_body_groups[j];
        if (std::find(body_group.begin(), body_group.end(), body_index)
            != body_group.end()) {
          mobile_collision_sets[j].insert(mobile_collision_sets[j].end(),
                                          collision_sets.begin(),
                                          collision_sets.end());
          break;
        }
      }
    } else {
      all_collision_sets.insert(all_collision_sets.end(),
                                collision_sets.begin(), collision_sets.end());
    }
  }
  // Let's log how many collision sets we have for each mobile body group
  for (int i = 0; i < std::ssize(mobile_body_groups); ++i) {
    const auto& body_group = mobile_body_groups[i];
    logging::log()->debug(
        "RobotModel: Mobile body group {} has bodies and {} collision sets", i,
        body_group.size(), mobile_collision_sets[i].size());
  }
  for (int i = 0; i < plant.num_bodies(); ++i) {
    logging::log()->debug(
        "RobotModel: Checking body {}/{}: {}", i, plant.num_bodies(),
        plant.get_body(drake::multibody::BodyIndex(i)).name());
    auto body_index = drake::multibody::BodyIndex(i);
    const auto& body = plant.get_body(body_index);
    auto visual_geos = plant.GetVisualGeometriesForBody(body);
    int index_mobile_body_group = -1;
    for (int j = 0; j < std::ssize(mobile_body_groups); ++j) {
      const auto& body_group = mobile_body_groups[j];
      if (std::find(body_group.begin(), body_group.end(), body_index)
          != body_group.end()) {
        index_mobile_body_group = j;
        break;
      }
    }
    DRAKE_DEMAND(index_mobile_body_group != -1);
    // Now let's check if all visual shapes are encapsulated by collision shapes
    const auto source_id_opt {plant.get_source_id()};
    const auto source_id {*source_id_opt};
    for (const auto visual_geo : visual_geos) {
      // Check if the visual geometry is encapsulated by the collision
      // geometries
      const auto& shape = sg_inspector.GetShape(visual_geo);
      auto X_frame_shape = sg_inspector.GetPoseInFrame(visual_geo);
      auto frame_id = sg_inspector.GetFrameId(visual_geo);
      const auto* body_of_frame = plant.GetBodyFromFrameId(frame_id);
      if (!body_of_frame) {
        throw std::runtime_error(fmt::format(
            "RobotModel: Body not found for frame id {}", frame_id));
      }
      if (body_of_frame->index() != body_index) {
        throw std::runtime_error(fmt::format(
            "RobotModel: Visual geometry frame's body {} does not match "
            "expected body {}",
            body_of_frame->name(), body.name()));
      }
      const auto X_world_body = body.body_frame().CalcPoseInWorld(
          plant.GetMyContextFromRoot(*calc_pose_context_));
      const auto X_world_shape = X_world_body * X_frame_shape;
      if (shape.type_name() == "Mesh") {
        const auto& mesh = static_cast<const drake::geometry::Mesh&>(shape);
        std::string file_path = mesh.source().description();
        auto triangle_surface_mesh =
            drake::geometry::ReadObjToTriangleSurfaceMesh(file_path);
        logging::log()->info(
            "RobotModel:: body: {} has a mesh file {} with {} vertices and "
            "{} faces",
            body.name(), mesh.source().path().filename(),
            triangle_surface_mesh.num_vertices(),
            triangle_surface_mesh.num_elements());
        triangle_surface_mesh.TransformVertices(X_world_shape);
        bool is_mesh_covered = true;
        std::vector<drake::geometry::SurfaceTriangle> uncovered_polygons;
        for (int i = 0; i < triangle_surface_mesh.num_elements(); ++i) {
          const auto& polygon = triangle_surface_mesh.element(i);
          bool is_polygon_touching = false;
          for (int k = 0; k < polygon.num_vertices(); ++k) {
            int vertex_index = polygon.vertex(k);
            const auto& vertex = triangle_surface_mesh.vertex(vertex_index);
            for (const auto& collision_set :
                 all_collision_sets.size() > 0
                     ? all_collision_sets
                     : mobile_collision_sets[index_mobile_body_group]) {
              if (collision_set->PointInSet(vertex, tol)) {
                is_polygon_touching = true;
                break;
              }
            }
            if (is_polygon_touching) {
              break;
            }
          }
          if (!is_polygon_touching) {
            logging::log()->debug(
                "RobotModel:: Polygon {}/{} of visual shape id {} of body {} "
                "is not inside any {} collision sets",
                i, triangle_surface_mesh.num_elements(), visual_geo,
                body.name(),
                mobile_collision_sets[index_mobile_body_group].size());
            is_mesh_covered = false;
            uncovered_polygons.push_back(polygon);
          }
        }
        int num_uncovered = uncovered_polygons.size();
        if (num_uncovered > 0 && meshcat_) {
          DRAKE_DEMAND(!is_mesh_covered);
          // Display them in meshcat
          logging::log()->error(
              "RobotModel:: Displaying {} uncovered polygons of visual mesh "
              "of body {} in meshcat",
              uncovered_polygons.size(), body.name());
          auto path = fmt::format(
              "/drake/uncovered/{}/{}",
              plant.GetModelInstanceName(body.model_instance()), body.name());
          auto vertices = triangle_surface_mesh.vertices();
          // vertices are now in world coordinate. Need to bring them back to
          // the frame of the shape
          for (auto& vertex : vertices) {
            vertex *= 1.0001;  // Slightly scale up for better visibility
          }
          auto uncovered_mesh = drake::geometry::TriangleSurfaceMesh<double>(
              std::move(uncovered_polygons), std::move(vertices));
          // uncovered_mesh.TransformVertices(X_world_shape.inverse());
          meshcat_->SetObject(path, uncovered_mesh,
                              drake::geometry::Rgba(0.5, 0.1, 0.2, 1.0));
          // meshcat_->SetTransform(path, X_frame_shape);
        }
        if (!is_mesh_covered) {
          auto msg = fmt::format(
              "Mesh file {} of body {} is not (fully) "
              "encapsulated by collision shapes. {}/{} polygons are uncovered.",
              mesh.source().path().filename(), body.name(), num_uncovered,
              triangle_surface_mesh.num_elements());
          uncovered_mesh_msgs.push_back(msg);
          logging::log()->error("RobotModel:: {}", msg);
        } else {
          logging::log()->info(
              "RobotModel:: Mesh file {} of body {} is encapsulated by "
              "basic collision shapes",
              mesh.source().path().filename(), body.name());
        }
      }
    }
  }
  for (const auto& msg : uncovered_mesh_msgs) {
    logging::log()->warn("RobotModel:: {}", msg);
  }
  return uncovered_mesh_msgs.empty();
}

}  // namespace motion
