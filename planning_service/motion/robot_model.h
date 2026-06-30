/*
 * Copyright © 2025 Sony Research. All rights reserved.
 */

/// @file robot_model.h

#pragma once
#include <drake/common/hash.h>
#include <drake/common/name_value.h>
#include <drake/common/trajectories/path_parameterized_trajectory.h>
#include <drake/common/trajectories/trajectory.h>
#include <drake/common/type_safe_index.h>
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/meshcat_visualizer.h>
#include <drake/geometry/optimization/hpolyhedron.h>
#include <drake/math/rigid_transform.h>
#include <drake/multibody/parsing/collision_filter_groups.h>
#include <drake/multibody/parsing/model_directives.h>
#include <drake/multibody/parsing/model_instance_info.h>
#include <drake/multibody/parsing/package_map.h>
#include <drake/multibody/parsing/process_model_directives.h>
#include <drake/multibody/parsing/scoped_names.h>
#include <drake/multibody/tree/fixed_offset_frame.h>
#include <drake/planning/collision_checker.h>
#include <drake/planning/dof_mask.h>
#include <drake/planning/robot_clearance.h>
#include <drake/planning/robot_diagram.h>
#include <drake/planning/robot_diagram_builder.h>
#include <drake/planning/scene_graph_collision_checker.h>

#include <omp.h>

#include "planning_service/common/logging.h"
#include "planning_service/motion/holonomic_mapping.h"

namespace motion {

using system_conf_t = std::map<std::string, Eigen::VectorXd>;

using body_name_color_map_t = std::map<std::string, drake::geometry::Rgba>;

// Map of body index to its respective collision and visual colors
using body_index_color_map_t =
    std::map<drake::multibody::BodyIndex,
             std::map<std::string, drake::geometry::Rgba>>;

using ArmIndex = drake::TypeSafeIndex<class ArmInstanceTag>;

struct RobotMeshcatParams {
  // Ports which may not be used, typically because they are
  // allocated to another process.
  std::optional<std::vector<int>> reserved_ports {};
  // Optionally specify an explicit port to use.
  std::optional<int> port {};
  // @warning: if set to true, the meshcat visualizer can be very slow
  // on some machines. It is recommended to set this to false on a
  // laptop that has a weak Graphics card.
  bool visual {false};
  // load collision geos
  bool collision {false};
  // colors of the bodies
  body_name_color_map_t color_map {};
  // colors of Cartesian-projected PRMs; if a color is not defined here, then a
  // random one will be selected instead
  body_name_color_map_t prm_color_map {};
  // the list of frames to be displayed in meshcat
  std::vector<std::string> end_effector_frame_vec {};
  // Load them from YAML file
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(reserved_ports));
    a->Visit(DRAKE_NVP(port));
    a->Visit(DRAKE_NVP(visual));
    a->Visit(DRAKE_NVP(collision));
    a->Visit(DRAKE_NVP(color_map));
    a->Visit(DRAKE_NVP(prm_color_map));
    a->Visit(DRAKE_NVP(end_effector_frame_vec));
  }
};

class RobotModel {
 public:
  /**
   * @brief Build a robot model with meshcat visualizer from a drake model
   * directives object.
   * @param xml_file the path to the xml file that contains the assets for the
   * robot model
   * @param dmd the model directives.
   * @param robot_meshcat_params [optional] the meshcat parameters for the robot
   * model. If not provided, the robot model will not have a meshcat visualizer.
   * @param continuous_revolute_joints the list of continuous revolute joints
   * expressed as a pair of model instance name and joint index.
   * @param implicit_parallelism if true, configure the collision checker to use
   * implicit context parallelism (i.e., the collision checker will allocate
   * contexts under-the-hood, instead of requiring explicit developer allocation
   * and management).
   */
  RobotModel(const std::string& xml_file,
             const drake::multibody::parsing::ModelDirectives& dmd,
             const std::optional<RobotMeshcatParams>& robot_meshcat_params =
                 std::nullopt,
             const std::vector<std::pair<std::string, int>>&
                 continuous_revolute_joints = {},
             bool implicit_parallelism = false);

  ~RobotModel() = default;

  /** An arm is a collection of model instances that serially connected in an
   * open kinematic chain. */
  class RobotArm {
   public:
    /** Returns the model instances of the RobotModel. */
    const std::vector<drake::multibody::ModelInstanceIndex>& model_instances()
        const {
      return model_instances_;
    }

    bool HasModelInstance(
        const drake::multibody::ModelInstanceIndex& model_instance) const {
      return std::find(model_instances_.begin(), model_instances_.end(),
                       model_instance)
             != model_instances_.end();
    }

    /** Given the generalized positions of the robot model, returns the
     * generalized positions of the arm.
     * @param q the generalized positions of the robot model
     * @return the generalized positions of the arm
     */
    Eigen::VectorXd GetPositionFromOriginalPlant(
        const Eigen::VectorXd& q) const;

    const drake::multibody::MultibodyPlant<double>& plant() const {
      return *plant_;
    }

    const std::string& name() const {
      return name_;
    }

    ArmIndex index() const {
      return index_;
    }

    /** Returns if index in the general coordinates of the original plant
     * is part of this arm.
     */
    bool IsGeneralIndexInArm(int index) const {
      return dof_mask_[index];
    }

    const drake::planning::DofMask& dof_mask() const {
      return dof_mask_;
    }

    const HolonomicMapping& arm_holonomic_mapping() const {
      return arm_holonomic_mapping_;
    }

    friend class RobotModel;

   private:
    // Constructor from model instances
    RobotArm(const drake::multibody::MultibodyPlant<double>& full_plant,
             const std::vector<drake::multibody::ModelInstanceIndex>&
                 model_instances,
             std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant,
             const std::string& name, ArmIndex index);

    const drake::multibody::MultibodyPlant<double>& original_plant_;
    std::vector<drake::multibody::ModelInstanceIndex> model_instances_;
    std::unique_ptr<drake::multibody::MultibodyPlant<double>> plant_;
    const HolonomicMapping arm_holonomic_mapping_;
    const std::string name_;
    const ArmIndex index_;
    drake::planning::DofMask dof_mask_;
  };

  class PartialSysconfError : public std::runtime_error {
   public:
    PartialSysconfError(const std::string& msg) : std::runtime_error(msg) {}
  };

  /** Given a map of model instance names to joint configutations, returns the
   robot model's generalized positions.
   @param sys_conf the map of model instance names to joint configurations
   @throws std::runtime_error if the number of positions does not match the
   number of positions in the robot model.
   @throws std::exception the system conf does not specify a configuration *for
   each* robot instance in the system.
   */
  Eigen::VectorXd ToGeneralizedPosition(const system_conf_t& sys_conf) const;

  /** Given a map of model instance names to joint configutations, returns true
   if the system configuration is complete, i.e. it specifies a configuration
   for each robot instance in the system.
   @param sysconf the map of model instance names to joint configurations
   */
  bool IsSysconfComplete(const system_conf_t& sysconf) const;

  /** Given robot model's generalized positions, returns a map of model instance
   names to joint configutations.
   @param q the robot model's generalized positions
   */
  system_conf_t ToSystemConf(const Eigen::VectorXd& q) const;

  /** Returns a reduced system configuration such that the dimension of each
   * model instance is reduced according to the holonomic mapping.
   */
  system_conf_t ReduceSystemConf(const system_conf_t& sysconf) const;

  /** Returns a lifted system configuration such that the dimension of each
   * model instance is lifted according to the holonomic mapping.
   */
  system_conf_t LiftSystemConf(const system_conf_t& sysconf) const;

  /** Returns the index of the generalized position that corresponds to the
   first position of the robot model instance.
   @param idx robot model instance index
   @throws std::runtime_error if the model instance index does not have
   positions or does not exist in the robot model.
   */
  int GetModelStartIndex(const drake::multibody::ModelInstanceIndex& idx) const;

  static int GetModelStartIndex(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::ModelInstanceIndex& idx);

  const drake::multibody::Frame<double>& GetScopedFrameByName(
      const std::string_view name) const;

  /**
   * @brief Check if a frame with the given scoped name exists in the robot
   * model.
   *
   * @param name Scoped name of the frame.
   * @return true if the frame exists, false otherwise.
   */
  bool HasScopedFrameNamed(const std::string_view name) const {
    return drake::multibody::parsing::GetScopedFrameByNameMaybe(plant(),
                                                                name.data())
           != nullptr;
  }

  /** Get a frame relative transform between two frames given the
   * generalized positions of the robot model.
   * @param q the generalized positions of the robot model
   * @param frame_A the frame A, the reference frame
   * @param frame_B the frame B, the target frame
   * @return the relative transform between frame A and frame B
   * @throws std::runtime_error if the number of positions does not match the
   * number of positions in the robot model.
   * @throws std::exception if either frame does not exist in the robot model.
   */
  drake::math::RigidTransformd CalcRelativeTransform(
      const Eigen::VectorXd& q, const drake::multibody::Frame<double>& frame_A,
      const drake::multibody::Frame<double>& frame_B) const;

  /** Set the positions of the idle model instances (that are
   * not part of the active_model_instances) to the reference
   * configuration q_ref.
   * @param q The configuration to be modified.
   * @param q_ref The reference configuration.
   * @param active_model_instances The active model instances.
   */
  void SetIdleModelsConfigToRef(
      Eigen::VectorXd* q, const Eigen::VectorXd& q_ref,
      const std::set<drake::multibody::ModelInstanceIndex>&
          active_model_instances) const;

  /** An overload that returns the modified configuration instead of
   * modifying in place.
   * @return The modified configuration.
   */
  Eigen::VectorXd SetIdleModelsConfigToRef(
      const Eigen::VectorXd& q, const Eigen::VectorXd& q_ref,
      const std::set<drake::multibody::ModelInstanceIndex>&
          active_model_instances) const;

  /**
   * @brief Given a configuration q, a reference configuration q_ref, and a set
   * of active model instances, sets the position of the model instances that
   * belong to the inactive arm (defined by those arms that do not contain any
   * of the active model instances) to the reference configuration q_ref.
   *
   * @param q
   * @param q_reference
   * @param active_model_instances The model instances that are part of the
   * active arm.
   */
  void SetIdleArmsConfigToRef(
      Eigen::VectorXd* q, const Eigen::VectorXd& q_ref,
      const std::set<drake::multibody::ModelInstanceIndex>&
          active_model_instances) const;

  // ---------- Meshcat Visualization ----------

  /** Sets the positions of the robot in the meshcat visualizer.
  @throws std::runtime_error if the number of positions does not match the
  number of positions in the robot model.
  @throws std::exception this robot model instance does not have meshcat
  visualizer initialized. */
  void SetMeshcatPositions(const Eigen::VectorXd& q) const;

  /** Get the joint positions of the robot in the Meshcat visualizer. */
  const Eigen::VectorXd GetMeshcatPositions() const;

  /** Publish the meshcat context to the meshcat visualizer. */
  void PublishMeshcatContext() const;

  /**
   * @brief Enable sliders for the given Meshcat session to manually set the
   * positions of each joint in the robot.
   */
  void SetMeshcatJointSliders() const;

  /** Get the joint positions from the meshcat sliders. */
  const Eigen::VectorXd GetMeshcatJointSliderPositions() const;

  /** Display the robot moving according trajectory in meshcat and
  save it to html.
  @param filename the name of the html file to be saved
  @param q_vec the vector of generalized positions
  @param t_seconds_vec the vector of times corresponding to the generalized
  positions
  @throws std::runtime_error if the number of positions does not match the
  number of positions in the robot model.
  @throws std::exception this robot model instance does not have meshcat
  visualizer initialized.
  @throws std::exception the number of times does not match the number of
  positions.
  */
  void SaveTrajectoryAsMeshcatHtml(
      const std::string filename, const std::vector<Eigen::VectorXd>& q_vec,
      const std::vector<double>& t_seconds_vec) const;

  /** Display the robot moving according trajectory in meshcat and
   * save it to html.
   * @param filename the name of the html file to be saved
   * @param traj the trajectory of generalized positions
   * @param delta_t the time step between each point in the trajectory.
   * The default value is 0.05 seconds (20 Hz).
   */
  void SaveTrajectoryAsMeshcatHtml(
      const std::string filename,
      const drake::trajectories::Trajectory<double>& traj,
      double delta_t = 0.05) const;

  /** Display the robot executing a given trajectory in meshcat
   * @param q_vec the vector of generalized positions of a trajectory
   * @param t_seconds_vec the vector of times corresponding to the generalized
   * positions
   */
  void DisplayTrajectoryInMeshcat(
      const std::vector<Eigen::VectorXd>& q_vec,
      const std::vector<double>& t_seconds_vec) const;

  void DisplayTrajectoryInMeshcat(
      const drake::trajectories::Trajectory<double>& ppt,
      double delta_t = 0.05) const;

  // ----------------- getters -----------------

  /** read-only access to the meshcat associated with this robot model */
  std::shared_ptr<drake::geometry::Meshcat> meshcat() const {
    return meshcat_;
  }

  /** read-only access to the plant */
  const drake::multibody::MultibodyPlant<double>& plant() const {
    return parsed_model_.default_collision_checker->plant();
  }

  /** read-only access to the scene graph */
  const drake::geometry::SceneGraph<double>& scene_graph() const {
    return parsed_model_.default_collision_checker->model().scene_graph();
  }

  /** read-only access to the collision checker
  @warning This is the default collision checker. It may not have the
  appropriate filters set. To use a specific collision checker, define it as
  part of
  @RobotConstraints and pass it to the appropriate functions.
  */
  const drake::planning::SceneGraphCollisionChecker& default_collision_checker()
      const {
    return *parsed_model_.default_collision_checker;
  }

  const drake::systems::Context<double>& meshcat_diagram_context() const {
    return *meshcat_diagram_context_;
  }

  /** mutable access to the meshcat diagram context */
  drake::systems::Context<double>& mutable_meshcat_diagram_context() const {
    return *meshcat_diagram_context_;
  }

  void SetMeshcatTime(double t) const {
    meshcat_diagram_context_->SetTime(t);
  }

  /** Returns the continuous revolute joint indices. */
  const std::vector<int>& continuous_revolute_joint_indices() const {
    return continuous_revolute_joint_indices_;
  }

  /** Returns the number of arms in the robot model. An
   arm is a collection of model instances that serially connected in an
    open kinematic chain. */
  int num_arms() const {
    return std::ssize(arms_);
  }

  /** Returns the model instances of the arm at the given index. */
  const RobotArm& GetArm(ArmIndex arm_index) const {
    DRAKE_THROW_UNLESS(arm_index < num_arms());
    return arms_.at(int(arm_index));
  }

  /** Returns the arm index of the given model instance.
   * @param model_instance the model instance index
   * @returns the arm index of the given model instance. If the model instance
   * does not belong to any arm (part of the environment), then -1 is returned.
   */
  ArmIndex get_arm_index(
      const drake::multibody::ModelInstanceIndex& model_instance) const {
    for (int i = 0; i < num_arms(); ++i) {
      if (arms_[i].HasModelInstance(model_instance)) {
        return ArmIndex(i);
      }
    }
    return ArmIndex();
  }

  const HolonomicMapping& holonomic_mapping() const {
    return holonomic_mapping_;
  }

  /** hasher for RobotModel. Refer to drake::hasher for more details.
  It only hashes the kinematics of the robot model. */
  template <class HashAlgorithm>
  friend void hash_append(HashAlgorithm& hasher,
                          const RobotModel& robot_model) noexcept {
    drake::DelegatingHasher delegating_hasher(
        [&hasher](const void* data, const size_t length) {
          return hasher(data, length);
        });
    robot_model.HashKinematics(&delegating_hasher);
  }

  void ColorCollidingBodiesInMeshcat(
      const drake::planning::RobotClearance& clearance,
      std::optional<double> recording_time = std::nullopt) const;

  bool HasFilterGroupName(const std::string& group_name) const;

  const std::vector<drake::multibody::BodyIndex>&
  GetBodyIndicesFromFilterGroupName(const std::string& group_name) const {
    return parsed_model_.filter_group_body_indices.at(group_name);
  }

  const std::vector<drake::multibody::BodyIndex>& GetBodyIndices() const {
    return parsed_model_.body_indices;
  }

  /** Sets the pose of a FixedOffsetFrame in the parent frame.
   * Throws std::runtime_error if the frame is not a FixedOffsetFrame.
   * @param frame the frame to set the pose of
   * @param offset the offset to set the pose to, expressed in the parent frame
   */
  void SetFixedOffsetFramePoseInParentFrame(
      const drake::multibody::Frame<double>& frame,
      const drake::math::RigidTransformd& offset) const;

  /** Given a frame, returns a visual body that is in the same mobile group as
   * the frame.
   * @param frame the frame to find the visual body for
   * @returns a body that has visual geometries and is in the same mobile group
   * as the frame's body.
   * @throws std::runtime_error if the frame's body is not part of any mobile
   * group, or if the mobile group does not have any visual body.
   */
  const drake::multibody::RigidBody<double>& GetVisualBodyInTheSameMobileGroup(
      const drake::multibody::Frame<double>& frame) const;

  drake::systems::Context<double>* calc_pose_context_ptr() const {
    return calc_pose_context_.get();
  }

  /** Index of the dedicated body used to anchor dynamically-added collision
   * shapes. It is welded to world with no geometry, and exists in every
   * collision checker clone. */
  drake::multibody::BodyIndex added_geometry_body_index() const {
    return parsed_model_.added_geometry_body_index;
  }

  const std::map<drake::multibody::ModelInstanceIndex,
                 drake::planning::DofMask>&
  instance_dof_masks() const {
    return instance_dof_masks_;
  }

  /** Adds the axes of the given frame to the meshcat visualizer.
   * @param frame the frame to add the axes of
   */
  void AddFrameAxesToMeshcat(const drake::multibody::Frame<double>& frame,
                             double transparency = 1.0,
                             double axis_length = 0.1,
                             double axis_radius = 0.002,
                             bool re_compute = false) const;

  body_index_color_map_t& mutable_body_index_color_map() const {
    return body_index_color_map_;
  }

  /** Checks if all visual shapes are encapsulated by collision shapes.
   * @return true if all visual shapes are encapsulated by meshes, false
   * otherwise.
   * @param tol the tolerance for the encapsulation check. The default value is
   * 1e-3 meters.
   * @note positive tolerance means that the collision shapes are interpreted as
   * inflated, i.e. the visual shapes are allowed to be outside of the
   * collision shapes by the given tolerance.
   * @note This function checks if all visual shapes are encapsulated by
   * collision shapes, i.e. if the visual shapes are contained within the
   collision shapes.
   */
  bool AreAllVisualShapesEncapsulatedByCollisionShapes(
      double tol = 1e-3,
      const std::optional<Eigen::VectorXd>& evaluation_config =
          std::nullopt) const;

 private:
  /**
   * @brief For a given body, set its visual and collision colors in the given
   * context.
   *
   * Because this assigns roles directly to the underlying scene graph, it is
   * persistent in visualization. However, it is also slow.
   *
   * @param body
   * @param visual
   * @param collision
   */
  void SetBodyColorsByRole(
      const drake::multibody::RigidBody<double>& body,
      drake::systems::Context<double>& context,
      const std::optional<drake::geometry::Rgba> collision = std::nullopt,
      const std::optional<drake::geometry::Rgba> visual = std::nullopt) const;

  /**
   * @brief For a given body, set its visual and collision colors by modifying
   * the Meshcat properties directly. This is much faster than
   * SetBodyColorsByRole, but the color changes are not persistent in
   * visualization. Prefer this approach for consistent updates to the
   * visualizer.
   *
   * @param body Rigid body to set colors for
   * @param collision Collision color to set for the body.
   * @param visual Visual color to set for the body.
   */
  void SetBodyColorsByProperty(
      const drake::multibody::RigidBody<double>& body,
      const std::optional<drake::geometry::Rgba> collision = std::nullopt,
      const std::optional<drake::geometry::Rgba> visual = std::nullopt) const;

  struct ParsedModel {
    std::unique_ptr<drake::planning::SceneGraphCollisionChecker>
        default_collision_checker;
    /** Vector of all indices in the plant. */
    std::vector<drake::multibody::BodyIndex> body_indices;
    /** Maps group names to the set of indices of the constituent bodies of that
     * group. */
    std::map<std::string, std::vector<drake::multibody::BodyIndex>>
        filter_group_body_indices;
    /** Index of the dedicated body used to anchor dynamically-added collision
     * shapes. Welded to world, no geometry. */
    drake::multibody::BodyIndex added_geometry_body_index;
  };

  ParsedModel ParseModel(const drake::multibody::parsing::ModelDirectives& dmd,
                         const drake::multibody::PackageMap& package_map,
                         bool implicit_parallelism);

  std::vector<RobotArm> FindArms(
      const drake::multibody::MultibodyPlant<double>& plant,
      const drake::multibody::parsing::ModelDirectives& dmd,
      const drake::multibody::PackageMap& package_map) const;

  drake::multibody::PackageMap CreatePackageMapFromXmlFile(
      const std::string& xml_file) const;

  /**
   * @brief Create the context whose data will be used to populate the Meshcat
   * web browser.
   *
   * This method assigns the initial geometry roles which control the behavior
   * of the visualizer.
   *
   * @param body_index_color_map Map of body indices to their respective colors.
   * @return std::unique_ptr<drake::systems::Context<double>>
   */
  std::unique_ptr<drake::systems::Context<double>> CreateMeshcatContext(
      const body_index_color_map_t& body_index_color_map) const;

  body_index_color_map_t ParseBodyColorMap() const;

  void HashKinematics(drake::DelegatingHasher* hasher) const;

  std::vector<int> CalcContinuousRevoluteJointIndices(
      const std::vector<std::pair<std::string, int>>&
          continuous_revolute_joints) const;

  // ---------- Data Members set at ctor ----------
  const drake::multibody::PackageMap package_map_;
  const std::optional<RobotMeshcatParams> robot_meshcat_params_;
  std::shared_ptr<drake::geometry::Meshcat> meshcat_;
  ParsedModel parsed_model_;
  mutable body_index_color_map_t body_index_color_map_;
  const std::vector<int> continuous_revolute_joint_indices_;
  const std::map<drake::multibody::ModelInstanceIndex, drake::planning::DofMask>
      instance_dof_masks_ {};
  const std::vector<RobotArm> arms_;
  std::vector<std::vector<drake::multibody::BodyIndex>> mobile_body_groups_;
  const HolonomicMapping holonomic_mapping_;
  // Context used to set positions of visualized model
  std::unique_ptr<drake::systems::Context<double>> meshcat_diagram_context_;
  // Context used to calculate relative poses
  std::unique_ptr<drake::systems::Context<double>> calc_pose_context_;
  // Visualizer members
  mutable std::unique_ptr<std::map<int, std::string>>
      meshcat_joint_index_name_map_;
  mutable std::unique_ptr<std::set<drake::multibody::BodyIndex>>
      colliding_bodies_ptr_ = nullptr;
  std::unique_ptr<std::vector<const drake::multibody::Frame<double>*>>
      frames_with_axes_ptr_ = std::make_unique<
          std::vector<const drake::multibody::Frame<double>*>>();
};

}  // namespace motion

namespace std {
template <>
struct hash<motion::RobotModel> : public drake::DefaultHash {};
}  // namespace std

template <>
struct fmt::formatter<motion::system_conf_t> {
  template <typename ParseContext>
  constexpr auto parse(ParseContext& ctx) {
    return ctx.begin();
  }

  template <typename FormatContext>
  auto format(motion::system_conf_t const& sys_conf, FormatContext& ctx) const {
    std::string fmt_msg {};
    for (const auto& name_conf_pair : sys_conf) {
      std::string robot_msg {fmt::format("\n\t\t{}: {}", name_conf_pair.first,
                                         name_conf_pair.second.transpose())};
      fmt_msg += robot_msg;
    }
    return fmt::format_to(ctx.out(), "system_conf: {} robots {}",
                          sys_conf.size(), fmt_msg);
  }
};
