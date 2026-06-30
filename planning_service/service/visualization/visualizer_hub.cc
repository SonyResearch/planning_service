/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file visualizer_hub.cc

#include "visualizer_hub.h"

#include <drake/geometry/rgba.h>
#include <drake/geometry/shape_specification.h>
#include <drake/math/rotation_matrix.h>
#include <drake/multibody/tree/revolute_joint.h>

#include <unordered_set>

#include "planning_service/common/file_utils.h"
#include "planning_service/common/misc_utils.h"
#include "planning_service/common/time_utils.h"
#include "planning_service/draco/client_conversions.h"

namespace {
// Geometry dimensions shared between SetupFrameAxes and UpdateFrameAxes.
constexpr double kFrameAxisLength {0.05};
constexpr double kFrameAxisRadius {0.001};
}  // namespace

namespace service {
namespace visualization {

bool VisualizerHub::LoadAndSetModelData(const VisualizerData& visualizer_data,
                                        const bool force_reload) {
  if (status_ == VisualizerStatus::Active) {
    if (options_->default_init) {
      logging::log()->error(
          "VH:LoadAndSetModelData: Default init is enabled; cannot load new "
          "model!");
      return false;
    }
    if (!force_reload) {
      logging::log()->error(
          "VH:LoadAndSetModelData: A visualizer is already "
          "running, and another may not be started!");
      return false;
    }
    logging::log()->warn(
        "VH:LoadAndSetModelData: Force reload requested. "
        "Switching to new model");
    // set it, wait for it to update
    SetStatus(VisualizerStatus::Stopping);
    status_.wait(VisualizerStatus::Stopping);
  }
  logging::log()->info("VH:LoadAndSetModelData: Loading new model...");
  try {
    draco::DracoAdapter draco_adapter;
    draco_adapter.system = system_name_;
    if (visualizer_data.context_id) {
      // First try to load from `visualization` subfolder, then from
      // all contexts
      const auto context_path_root {data_path_ / "contexts"};
      const auto urdf_base_dir {data_path_ / "urdf"};
      PlanContext context {visualizer_data.context_id->value};
      if (!utils::LoadContext(context, context_path_root / "visualization",
                              urdf_base_dir)) {
        logging::log()->warn(
            "VH:LoadAndSetModelData: Context ID {} not found in "
            "`visualization` subfolder; trying all contexts",
            visualizer_data.context_id->value);
        if (!utils::LoadContext(context, context_path_root, urdf_base_dir)) {
          logging::log()->error(
              "VH:LoadAndSetModelData: Context ID {} could not be found "
              "in any known context folder!",
              visualizer_data.context_id->value);
          return false;
        }
      }
      const bool require_parameters {false};
      draco_adapter = utils::MakeDracoAdapterFromContext(
          system_name_, context, require_parameters,
          draco::VisualizerMode::kNative, data_path_);
      // If collision geos are disabled, get rid of the collision checker
      if (!options_->meshcat_params.collision) {
        draco_adapter.constraints_adapter.collision_checker = std::nullopt;
      }
    } else {
      draco_adapter.dmd =
          drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
              data_path_ / "dmd" / *visualizer_data.dmd_filename);
      // default collision checking for DMD-only
      draco_adapter.constraints_adapter.collision_checker =
          motion::CollisionCheckerAdapter();
      draco_adapter.xml_file = data_path_ / "urdf" / "package.xml";
    }
    motion::RobotMeshcatParams robot_meshcat_params {options_->meshcat_params};
    if (visualizer_data.robot_meshcat_params) {
      logging::log()->info(
          "VH:LoadAndSetModelData: Overriding default meshcat parameters with "
          "parameters from request");
      robot_meshcat_params = *visualizer_data.robot_meshcat_params;
    }
    // Discard reserved ports and set port to 7000
    robot_meshcat_params.reserved_ports.reset();
    robot_meshcat_params.port = 7000;
    draco_adapter.robot_meshcat_params = robot_meshcat_params;
    draco_adapter_ = draco_adapter;
    // signal that adapter is ready for use
    SetStatus(VisualizerStatus::Starting);
    return true;
  } catch (const std::exception& e) {
    logging::log()->error(
        "VH:LoadAndSetModelAdapter: Failed to load model data due "
        "to exception: {}",
        e.what());
    return false;
  }
}

void VisualizerHub::SetRobotStreamActive(const std::string& robot,
                                         const bool active) {
  DRAKE_THROW_UNLESS(draco_ != nullptr);
  const auto prefix = robot + "-";
  const auto& plant = draco_->robot_model().plant();
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    const auto idx = drake::multibody::ModelInstanceIndex(i);
    const auto model_name = plant.GetModelInstanceName(idx);
    if (model_name != robot && !model_name.starts_with(prefix)) {
      continue;
    }
    if (!options_->stream_opacity) {
      continue;
    }
    const auto path = fmt::format("/drake/visual/{}", model_name);
    draco_->robot_model().meshcat()->SetProperty(path, "modulated_opacity",
                                                 active ? 1.0 : 0.5);
  }
}

void VisualizerHub::SetupJointLimitIndicators() {
  DRAKE_THROW_UNLESS(draco_ != nullptr);
  joint_limit_indicators_.clear();
  const auto& model = draco_->robot_model();
  const auto meshcat = model.meshcat();
  if (!meshcat) return;
  const auto& plant = model.plant();
  const auto q = model.GetMeshcatPositions();
  const auto& world_frame = plant.world_frame();
  const auto q_low = plant.GetPositionLowerLimits();
  const auto q_high = plant.GetPositionUpperLimits();
  const auto disk = drake::geometry::Cylinder(kJointLimitDiskRadius,
                                              kJointLimitDiskThickness);
  // Find the "next" link in the kinematic chain after the child body of the
  // joint, given our DH-calibrated links.
  const auto find_link_n =
      [&](const drake::multibody::RigidBody<double>& link_n_d)
      -> const drake::multibody::RigidBody<double>& {
    for (const auto& ji : plant.GetJointIndices(link_n_d.model_instance())) {
      const auto& j = plant.get_joint(ji);
      if (j.num_positions() == 0 && &j.parent_body() == &link_n_d)
        return j.child_body();
    }
    return link_n_d;  // no _d joint found; midpoint degenerates to same point
  };
  for (int i = 0; i < plant.num_model_instances(); ++i) {
    const auto model_idx = drake::multibody::ModelInstanceIndex(i);
    const auto& model_name = plant.GetModelInstanceName(model_idx);
    for (const auto& joint_index : plant.GetJointIndices(model_idx)) {
      const auto& joint = plant.get_joint(joint_index);
      if (joint.num_positions() != 1) continue;
      const int pos_idx = joint.position_start();
      const double q_lo = q_low[pos_idx];
      const double q_hi = q_high[pos_idx];
      if (!std::isfinite(q_lo) || !std::isfinite(q_hi)) continue;
      const auto* rev =
          dynamic_cast<const drake::multibody::RevoluteJoint<double>*>(&joint);
      if (!rev) continue;
      // Orient the disk about the joint axis of rotation.
      const Eigen::Vector3d axis = rev->revolute_axis();
      const Eigen::Quaterniond q_to_axis =
          Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), axis);
      const drake::math::RigidTransformd X_J_disk(
          drake::math::RotationMatrixd(q_to_axis), Eigen::Vector3d::Zero());
      const auto path =
          fmt::format("/drake/joint_limits/{}/{}", model_name, joint.name());
      // Anchor the disk to the child frame so it will rotate accordingly.
      const auto& child_frame = joint.frame_on_child();
      const auto& link_n_frame = find_link_n(joint.child_body()).body_frame();
      const auto X_W_Jc =
          model.CalcRelativeTransform(q, world_frame, child_frame);
      const Eigen::Vector3d disk_pos =
          0.5
          * (X_W_Jc.translation()
             + model.CalcRelativeTransform(q, world_frame, link_n_frame)
                   .translation());
      const auto X_W_disk = drake::math::RigidTransformd(
          (X_W_Jc * X_J_disk).rotation(), disk_pos);
      const drake::geometry::Rgba rgba {0.0, 1.0, 0.0,
                                        kJointLimitIndicatorAlpha};
      meshcat->SetObject(path, disk, rgba);
      meshcat->SetTransform(path, X_W_disk);
      meshcat->SetProperty(path, "visible", false);
      const auto cone = drake::geometry::MeshcatCone(kJointLimitArrowHeight,
                                                     kJointLimitArrowRadius,
                                                     kJointLimitArrowRadius);
      for (int ci = 0; ci < 4; ++ci) {
        const auto cone_path = fmt::format("{}/arrow{}", path, ci);
        meshcat->SetObject(cone_path, cone, rgba);
        meshcat->SetProperty(cone_path, "visible", false);
      }
      joint_limit_indicators_.emplace_back(
          JointLimitIndicator {pos_idx, q_lo, q_hi, path, child_frame.index(),
                               link_n_frame.index(), X_J_disk});
      logging::log()->debug(
          "VH:SetupJointLimitIndicators: Added indicator for joint {} at {}",
          joint.name(), path);
    }
  }
  logging::log()->info("VH:SetupJointLimitIndicators: {} indicators added",
                       joint_limit_indicators_.size());
}

void VisualizerHub::UpdateJointLimitIndicators(const Eigen::VectorXd& q) {
  if (joint_limit_indicators_.empty()) return;
  const auto meshcat = draco_->robot_model().meshcat();
  DRAKE_THROW_UNLESS(meshcat != nullptr);
  const auto& model = draco_->robot_model();
  const auto& plant = model.plant();
  const auto& world_frame = plant.world_frame();
  for (const auto& ind : joint_limit_indicators_) {
    const auto& child_frame {plant.get_frame(ind.child_frame_index)};
    const auto& link_n_frame {plant.get_frame(ind.link_n_frame_index)};
    const auto X_W_Jc =
        model.CalcRelativeTransform(q, world_frame, child_frame);
    const Eigen::Vector3d disk_pos =
        0.5
        * (X_W_Jc.translation()
           + model.CalcRelativeTransform(q, world_frame, link_n_frame)
                 .translation());
    const auto X_W_disk = drake::math::RigidTransformd(
        (X_W_Jc * ind.X_J_disk).rotation(), disk_pos);
    meshcat->SetTransform(ind.path, X_W_disk);
    // Four cones at 90° intervals on the rim, all pointing tangentially in
    // the safe rotation direction. safe_sign = +1 → CCW (increasing q).
    const double q_val = q[ind.position_index];
    const double q_mid = 0.5 * (ind.q_min + ind.q_max);
    const double safe_sign = (q_mid > q_val) ? -1.0 : 1.0;
    for (int ci = 0; ci < 4; ++ci) {
      const double theta = ci * M_PI_2;
      const auto R_cone =
          drake::math::RotationMatrixd::MakeZRotation(theta)
          * drake::math::RotationMatrixd::MakeXRotation(-safe_sign * M_PI_2);
      // Cone is rooted at its tip; shift so the cone's midpoint sits on the
      // rim: tip = rim - R_cone * (0, 0, height/2).
      const Eigen::Vector3d rim(std::cos(theta) * kJointLimitDiskRadius,
                                std::sin(theta) * kJointLimitDiskRadius, 0.0);
      const Eigen::Vector3d tip =
          rim - R_cone * Eigen::Vector3d(0.0, 0.0, kJointLimitArrowHeight / 2);
      meshcat->SetTransform(fmt::format("{}/arrow{}", ind.path, ci),
                            drake::math::RigidTransformd(R_cone, tip));
    }
    // Show/hide and recolor based on proximity to limits.
    const double range = ind.q_max - ind.q_min;
    const double margin = std::min(q_val - ind.q_min, ind.q_max - q_val);
    const double frac = (range > 0.0) ? (margin / range) : 0.0;
    if (frac >= kJointLimitWarnFraction) {
      // Comfortably within limits: hide the indicator entirely.
      meshcat->SetProperty(ind.path, "visible", false);
      for (int ci = 0; ci < 4; ++ci)
        meshcat->SetProperty(fmt::format("{}/arrow{}", ind.path, ci), "visible",
                             false);
    } else {
      meshcat->SetProperty(ind.path, "visible", true);
      // Interpolate from transparent yellow to max alpha red.
      const double g = frac / kJointLimitWarnFraction;
      const auto alpha = (1.0 - frac) * kJointLimitIndicatorAlpha;
      const std::vector<double> rgba {1.0, g, 0.0, alpha};
      meshcat->SetProperty(ind.path, "color", rgba);
      for (int ci = 0; ci < 4; ++ci) {
        const auto cone_path = fmt::format("{}/arrow{}", ind.path, ci);
        meshcat->SetProperty(cone_path, "visible", true);
        meshcat->SetProperty(cone_path, "color", rgba);
      }
    }
  }
}

// Build the three constant local-frame axis transforms (same for every axes).
static void CalcLocalAxisTransforms(drake::math::RigidTransformd& X_x,
                                    drake::math::RigidTransformd& X_y,
                                    drake::math::RigidTransformd& X_z) {
  X_x.set_translation(Eigen::Vector3d(kFrameAxisLength / 2, 0, 0));
  X_x.set_rotation(
      drake::math::RollPitchYawd(0, -M_PI_2, 0).ToRotationMatrix());
  X_y.set_translation(Eigen::Vector3d(0, kFrameAxisLength / 2, 0));
  X_y.set_rotation(drake::math::RollPitchYawd(M_PI_2, 0, 0).ToRotationMatrix());
  X_z.set_translation(Eigen::Vector3d(0, 0, kFrameAxisLength / 2));
}

void VisualizerHub::SetupFrameAxes() {
  const auto& robot_model {draco_->robot_model()};
  const auto& plant {robot_model.plant()};
  auto& meshcat {*robot_model.meshcat()};
  const auto& world_frame {plant.world_frame()};

  const drake::geometry::Cylinder cylinder(kFrameAxisRadius, kFrameAxisLength);
  drake::math::RigidTransformd X_local_x, X_local_y, X_local_z;
  CalcLocalAxisTransforms(X_local_x, X_local_y, X_local_z);

  frame_axes_.clear();
  // Precompute welded-to-world body indices once. Frames on these bodies are
  // static: their world pose is invariant to q and only needs setting at
  // setup time
  const auto welded_body_ptrs {plant.GetBodiesWeldedTo(plant.world_body())};
  std::unordered_set<drake::multibody::BodyIndex> welded_body_indices;
  welded_body_indices.reserve(welded_body_ptrs.size());
  for (const auto* body : welded_body_ptrs) {
    welded_body_indices.insert(body->index());
  }
  const auto q {robot_model.GetMeshcatPositions()};
  const bool hidden_by_toggle {!options_->enable_frame_toggle_web};
  for (drake::multibody::FrameIndex idx {0}; idx < plant.num_frames(); ++idx) {
    const auto& frame {plant.get_frame(idx)};
    // Skip world-body frames and dynamically-added geometry frames.
    const bool is_world_body {frame.body().index()
                              == plant.world_body().index()};
    const auto& model_name {plant.GetModelInstanceName(frame.model_instance())};
    const bool is_added_geometry {model_name == "added_geometry"
                                  || frame.name() == "added_geometry"};
    // Skip frames whose names match any configured ignore pattern.
    const std::string fname {frame.name()};
    const bool has_pattern {
        std::any_of(options_->frame_name_ignore_patterns.begin(),
                    options_->frame_name_ignore_patterns.end(),
                    [&fname](const std::string& pattern) {
                      return fname.find(pattern) != std::string::npos;
                    })};
    const bool has_prefix {
        std::any_of(options_->frame_name_ignore_prefixes.begin(),
                    options_->frame_name_ignore_prefixes.end(),
                    [&fname](const std::string& prefix) {
                      return fname.starts_with(prefix);
                    })};
    const bool has_suffix {
        std::any_of(options_->frame_name_ignore_suffixes.begin(),
                    options_->frame_name_ignore_suffixes.end(),
                    [&fname](const std::string& suffix) {
                      return fname.ends_with(suffix);
                    })};
    const bool is_ignored = has_pattern || has_prefix || has_suffix;
    if (is_world_body || is_added_geometry || is_ignored) {
      continue;
    }

    const auto path {
        fmt::format("/drake/frames/{}/{}/", model_name, frame.name())};

    // Axis cylinders live at fixed local offsets; the parent transform carries
    // the world pose so UpdateFrameAxes only needs one SetTransform per frame.
    meshcat.SetObject(path + "x", cylinder, drake::geometry::Rgba(1, 0, 0, 1));
    meshcat.SetTransform(path + "x", X_local_x);
    meshcat.SetObject(path + "y", cylinder, drake::geometry::Rgba(0, 1, 0, 1));
    meshcat.SetTransform(path + "y", X_local_y);
    meshcat.SetObject(path + "z", cylinder, drake::geometry::Rgba(0, 0, 1, 1));
    meshcat.SetTransform(path + "z", X_local_z);
    // Set the parent to the current world pose.
    const auto X_WF {
        draco_->robot_model().CalcRelativeTransform(q, world_frame, frame)};
    meshcat.SetTransform(path, X_WF);
    // If using the web interface, hide with visibility property; otherwise,
    // teleport
    if (options_->enable_frame_toggle_web) {
      meshcat.SetProperty(path, "visible", false);
    } else {
      meshcat.SetTransform(
          path, drake::math::RigidTransformd(Eigen::Vector3d(0, 0, -1e6)));
    }
    const bool is_static {welded_body_indices.count(frame.body().index()) > 0};
    // In non-web mode frames start teleported away; hidden_by_toggle=true
    // prevents UpdateFrameAxes from restoring them on the very first tick.
    frame_axes_.emplace(path, FrameAxesInfo {idx, is_static, hidden_by_toggle});
  }

  if (hidden_by_toggle) {
    meshcat.SetProperty("/drake/frames", "hidden", true);
  }

  const auto n_static {
      std::count_if(frame_axes_.begin(), frame_axes_.end(), [](const auto& p) {
        return p.second.is_static;
      })};
  logging::log()->info(
      "VH:SetupFrameAxes: Registered {} frame axes under /drake/frames/... ({} "
      "static, {} kinematic), hidden by default",
      frame_axes_.size(), n_static, frame_axes_.size() - n_static);
}

void VisualizerHub::UpdateFrameAxes() {
  if (frame_axes_.empty()) {
    return;
  }

  const auto& robot_model {draco_->robot_model()};
  const auto& plant {robot_model.plant()};
  auto& meshcat {*robot_model.meshcat()};
  const auto& world_frame {plant.world_frame()};
  const auto q {robot_model.GetMeshcatPositions()};
  for (const auto& [path, info] : frame_axes_) {
    // Static frames are welded to the world — transform never changes with q.
    if (info.is_static) {
      continue;
    }
    // Frames teleported by ToggleFramesByPath must not be restored here.
    if (info.hidden_by_toggle) {
      continue;
    }
    const auto& frame {plant.get_frame(info.index)};
    const auto X_WF {
        draco_->robot_model().CalcRelativeTransform(q, world_frame, frame)};
    // One SetTransform on the parent moves all three axis children.
    meshcat.SetTransform(path, X_WF);
  }
}

std::vector<std::pair<std::string, drake::math::RigidTransformd> >
VisualizerHub::ToggleFramesByPath(std::string_view path, bool active) {
  std::vector<std::pair<std::string, drake::math::RigidTransformd> > result;
  if (!draco_) {
    return result;
  }
  const auto frames_prefix {fmt::format("/drake/frames/{}", path)};
  const auto& robot_model {draco_->robot_model()};
  const auto& plant {robot_model.plant()};
  const auto& world_frame {plant.world_frame()};
  const auto q {robot_model.GetMeshcatPositions()};
  for (auto& [p, info] : frame_axes_) {
    if (!p.starts_with(frames_prefix)) {
      continue;
    }
    info.hidden_by_toggle = !active;
    drake::math::RigidTransformd transform;
    if (!active) {
      transform.set_translation(Eigen::Vector3d(0, 0, -1e6));
    } else {
      const auto& frame {plant.get_frame(info.index)};
      transform = robot_model.CalcRelativeTransform(q, world_frame, frame);
    }
    result.emplace_back(p, std::move(transform));
  }
  return result;
}

bool VisualizerHub::ToggleFrame(std::string_view path, bool visible) {
  if (!draco_) {
    return false;
  }
  if (!frame_axes_.contains(std::string(path))) {
    logging::log()->warn(
        "VH:ToggleFrame: No frame found for path '{}'; cannot toggle "
        "visibility",
        path);
    return false;
  }
  auto& info {frame_axes_.at(std::string(path))};
  const auto& robot_model {draco_->robot_model()};

  if (options_->enable_frame_toggle_web) {
    // When allowing for frame toggling via the GUI, just use the visibility
    // property
    QueueMeshcatTask(
        [path = std::string(path), visible](drake::geometry::Meshcat& meshcat) {
          meshcat.SetProperty(path, "visible", visible);
        });
  } else {
    // When not using the web interface, toggle visibility by teleporting frames
    // in the same way that we hide objects via ToggleObject.
    info.hidden_by_toggle = !visible;
    drake::math::RigidTransformd X_WF;
    if (!visible) {
      X_WF.set_translation(Eigen::Vector3d(0, 0, -1e6));
    } else {
      const auto& plant {robot_model.plant()};
      const auto q {robot_model.GetMeshcatPositions()};
      const auto& frame {plant.get_frame(info.index)};
      X_WF = robot_model.CalcRelativeTransform(q, plant.world_frame(), frame);
    }
    QueueMeshcatTask(
        [path = std::string(path), X_WF](drake::geometry::Meshcat& meshcat) {
          meshcat.SetTransform(path, X_WF);
        });
  }
  return true;
}

std::optional<std::string> VisualizerHub::ResolveFrameMeshcatPath(
    std::string_view frame_name) const {
  if (!draco_) {
    return std::nullopt;
  }
  const auto& robot_model {draco_->robot_model()};
  if (!robot_model.HasScopedFrameNamed(frame_name)) {
    return std::nullopt;
  }
  const auto& frame {robot_model.GetScopedFrameByName(frame_name)};
  const auto& plant {robot_model.plant()};
  const auto& model_name {plant.GetModelInstanceName(frame.model_instance())};
  return fmt::format("/drake/frames/{}/{}/", model_name, frame.name());
}

bool VisualizerHub::Setup() {
  bool success {true};
  // Set up sliders
  if (options_->enable_sliders) {
    draco_->robot_model().SetMeshcatJointSliders();
    logging::log()->info("VH:Setup: Added joint sliders.");
  }
  // Set up default positions
  if (draco_adapter_->options.default_configuration) {
    default_positions_ = draco_->robot_model().ToGeneralizedPosition(
        *draco_adapter_->options.default_configuration);
    draco_->robot_model().SetMeshcatPositions(*default_positions_);
    if (options_->enable_sliders) {
      AddButton(reset_to_default_button_name_, 0);
    }
    logging::log()->info("VH:Setup: Configured robot's default positions.");
  }
  // Streaming
  if (options_->enable_set_configuration) {
    // Add a button indicating stream status
    position_stream_button_name_ = position_stream_inactive_button_name_;
    AddButton(position_stream_button_name_, 0);
    // Publish once so geometry nodes exist in Meshcat's scene tree before
    // we attempt to set properties on them.
    draco_->robot_model().PublishMeshcatContext();
    // Dim all robots on startup; they brighten up as pushes arrive.
    const auto robot_confs = draco_->robot_model().ToSystemConf(
        draco_->robot_model().GetMeshcatPositions());
    for (const auto& [robot, _] : robot_confs) {
      SetRobotStreamActive(robot, false);
    }
  }
  // Set up joint limit indicators
  if (options_->show_joint_limits) {
    SetupJointLimitIndicators();
    logging::log()->info("VH:Setup: Added joint limit indicators.");
  }
  // Register all plant frames under /drake/frames/.
  SetupFrameAxes();
  // Set initial camera pose from options.
  if (options_->default_camera_position) {
    const auto& camera_position {options_->default_camera_position.value()};
    const auto camera_target {
        options_->default_camera_target.value_or(Eigen::Vector3d::Zero())};
    draco_->robot_model().meshcat()->SetCameraPose(camera_position,
                                                   camera_target);
    logging::log()->info(
        "VH:Setup: Set initial camera position ({}) and target ({}).",
        camera_position.transpose(), camera_target.transpose());
  }
  // @TODO(@davebambrick): Restore HTML export when we have something
  // interesting to export (e.g., a recording).
  // AddButton(export_html_button_name_, -1);

  logging::log()->info(
      "VH:Setup: Completed setup for visualizer instance at: {}",
      draco_->robot_model().meshcat()->web_url());
  return success;
}

void VisualizerHub::Run() {
  while (true) {
    const auto status {status_.load()};
    switch (status) {
      case VisualizerStatus::Idle: {
        break;
      }
      case VisualizerStatus::Stopping: {
        if (!HasDraco()) {
          logging::log()->warn(
              "VH:Run: Stop requested, but no Draco instance is active!");
          SetStatus(VisualizerStatus::Idle);
        } else {
          // kill the Draco
          draco_.reset();
          logging::log()->info(
              "VH:Run: Stop request processed; model has been reset");
          // reset data after stop has been processed
          draco_adapter_ = std::nullopt;
          default_positions_ = std::nullopt;
          joint_limit_indicators_.clear();
          frame_axes_.clear();
          SetStatus(VisualizerStatus::Idle);
        }
        break;
      }
      case VisualizerStatus::Starting: {
        logging::log()->info("VH:Run: Loading prepared adapter");
        try {
          draco_ = std::make_unique<draco::Draco>(*draco_adapter_);
        } catch (const std::exception& e) {
          logging::log()->error(
              "VH:Run: Failed to load model due to exception: {}", e.what());
          SetStatus(VisualizerStatus::Idle);
          break;
        }
        if (!Setup()) {
          logging::log()->error(
              "VH:Run: Failed to setup with specified options!");
          break;
        }
        SetStatus(VisualizerStatus::Active);
        break;
      }
      case VisualizerStatus::Active: {
        // run the "main loop"
        const auto meshcat_update_interval_ms {
            static_cast<int>(1e3 / meshcat_active_freq_hz_)};
        const auto& robot_model {draco_->robot_model()};
        while (status_ != VisualizerStatus::Stopping) {
          const auto t_start {std::chrono::steady_clock::now()};
          const auto q_before {robot_model.GetMeshcatPositions()};
          // Read export to HTML button
          if (HasButton(export_html_button_name_)
              && robot_model.meshcat()->GetButtonClicks(
                     export_html_button_name_)
                     > 0) {
            logging::log()->info("VH:Run: Exporting session to HTML...");
            const auto html_path {ExportToHtml()};
            logging::log()->info("VH:Run: Static session exported to: {}",
                                 html_path);
          }
          // grab values from sliders
          if (options_->enable_sliders) {
            auto q {robot_model.GetMeshcatJointSliderPositions()};
            // Read reset to default button
            if (HasButton(reset_to_default_button_name_)
                && robot_model.meshcat()->GetButtonClicks(
                       reset_to_default_button_name_)
                       > 0) {
              logging::log()->info(
                  "VH:Run: Resetting robot model to default positions...");
              q = *default_positions_;
              // Reset clicks to zero
              AddButton(reset_to_default_button_name_, 0);
            }
            if (common::utils::JointPositionsChanged(q_before, q)) {
              robot_model.SetMeshcatPositions(q);
            }
          }
          // set target position if requested
          if (options_->enable_set_configuration) {
            // Hold the mutex only long enough to copy out the next position and
            // timing snapshot
            std::optional<Eigen::VectorXd> position_to_set;
            RobotPushTimeMap push_time_snapshot;
            {
              std::unique_lock<std::mutex> lock(position_mtx_);
              position_cv_.wait_for(
                  lock, chrono_ms(meshcat_update_interval_ms / 5), [this]() {
                    return pending_position_.has_value();
                  });
              position_to_set = std::exchange(pending_position_, std::nullopt);
              push_time_snapshot = robot_last_push_time_;
            }
            if (position_to_set) {
              logging::log()->debug("VH:Run: Setting position: {}",
                                    position_to_set->transpose());
              draco_->robot_model().SetMeshcatPositions(*position_to_set);
            }
            const auto now = std::chrono::high_resolution_clock::now();
            // Update stream status
            const auto robot_confs = draco_->robot_model().ToSystemConf(
                draco_->robot_model().GetMeshcatPositions());
            const size_t total = robot_confs.size();
            size_t active_count {0};
            for (const auto& [robot, _] : robot_confs) {
              const auto it = push_time_snapshot.find(robot);
              const auto active {
                  (it != push_time_snapshot.end()
                   && std::chrono::duration_cast<chrono_ms>(now - it->second)
                              .count()
                          <= position_push_timeout_ms_)};
              active_count += static_cast<size_t>(active);
              SetRobotStreamActive(robot, active);
            }
            const auto desired_button =
                (active_count > 0)
                    ? fmt::format("Position Stream: {}/{} Active", active_count,
                                  total)
                    : position_stream_inactive_button_name_;

            // Swap the button if the label changed
            if (desired_button != position_stream_button_name_) {
              if (HasButton(position_stream_button_name_)) {
                DeleteButton(position_stream_button_name_);
              }
              position_stream_button_name_ = desired_button;
              AddButton(position_stream_button_name_, 0);
            } else if (!HasButton(position_stream_button_name_)) {
              AddButton(position_stream_button_name_, 0);
            }
          }
          const auto q_now {robot_model.GetMeshcatPositions()};
          // run any queued tasks (non-blocking: don't stall publish for tasks)
          {
            std::scoped_lock<std::mutex> lock(meshcat_task_queue_mtx_);
            while (!meshcat_task_queue_.empty()) {
              if (const auto meshcat = draco_->robot_model().meshcat()) {
                meshcat_task_queue_.front()(*meshcat);
              }
              meshcat_task_queue_.pop();
            }
          }
          // Only update position-dependent visuals when q has actually changed.
          if (common::utils::JointPositionsChanged(q_before, q_now)) {
            if (options_->show_joint_limits) {
              UpdateJointLimitIndicators(q_now);
            }
            // Update all non-static, non-hidden moving frame transforms.
            UpdateFrameAxes();
          }
          // CheckSastisfied
          draco_->robot_constraints().CheckSatisfied(q_now);
          // publish new positions
          draco_->robot_model().PublishMeshcatContext();
          // Sleep for remainder
          const auto elapsed_ms {std::chrono::duration_cast<chrono_ms>(
                                     std::chrono::steady_clock::now() - t_start)
                                     .count()};
          const auto sleep_time_ms {std::max(
              0, meshcat_update_interval_ms - static_cast<int>(elapsed_ms))};
          std::this_thread::sleep_for(chrono_ms(sleep_time_ms));
        }
        break;
      }
      default: {
        throw std::runtime_error(fmt::format(
            "Unknown status: {}", magic_enum::enum_name(status_.load())));
      }
    }
    WaitOnStatus(status);
  }
}

const drake::math::RigidTransformd VisualizerHub::CalcPose(
    const std::string_view frame_B_name, const std::string_view frame_A_name,
    const std::optional<psc::SystemConf>& system_conf_override) const {
  DRAKE_THROW_UNLESS(draco_ != nullptr);
  logging::log()->info("VH:CalcPose: Getting pose for {} relative to {}",
                       frame_B_name, frame_A_name);
  const auto& robot_model {draco_->robot_model()};
  Eigen::VectorXd q;
  if (system_conf_override) {
    psc::SystemConf curr_system_conf;
    for (const auto& [robot, conf] :
         robot_model.ToSystemConf(robot_model.GetMeshcatPositions())) {
      curr_system_conf[robot] = conf;
    }
    q = draco::conversions::ToGeneralizedPosition(
        robot_model, *system_conf_override,
        draco::conversions::ToGeneralizedBehavior::
            kCompleteFromReferenceOnMissing,
        curr_system_conf);
    logging::log()->info("VH:CalcPose: Evaluating pose at override config: {}",
                         q.transpose());
  } else {
    q = robot_model.GetMeshcatPositions();
  }
  const auto& frame_A {robot_model.GetScopedFrameByName(frame_A_name)};
  const auto& frame_B {robot_model.GetScopedFrameByName(frame_B_name)};
  const auto& pose {robot_model.CalcRelativeTransform(q, frame_A, frame_B)};
  logging::log()->info("VH:CalcPose: Got pose {}", pose);
  return pose;
}

std::expected<bool, std::string> VisualizerHub::SetConfiguration(
    const psc::SystemConf& system_conf) {
  DRAKE_THROW_UNLESS(draco_ != nullptr);
  // Keep current positions for robots not specified in system_conf
  psc::SystemConf curr_system_conf;
  for (const auto& [robot, conf] : draco_->robot_model().ToSystemConf(
           draco_->robot_model().GetMeshcatPositions())) {
    curr_system_conf[robot] = conf;
  }
  const auto q {draco::conversions::ToGeneralizedPosition(
      draco_->robot_model(), system_conf,
      draco::conversions::ToGeneralizedBehavior::
          kCompleteFromReferenceOnMissing,
      curr_system_conf)};
  return SetConfiguration(q);
}

std::expected<bool, std::string> VisualizerHub::SetConfiguration(
    const Eigen::VectorXd& q) {
  if (!options_->enable_set_configuration) {
    return std::unexpected("Setting configuration is disabled!");
  }
  DRAKE_THROW_UNLESS(draco_ != nullptr);
  const auto& robot_model {draco_->robot_model()};
  if (q.size() != robot_model.plant().num_positions()) {
    return std::unexpected(
        fmt::format("Invalid configuration size: expected {}, got {}",
                    robot_model.plant().num_positions(), q.size()));
  }
  // Update last push time, even if we don't actually set the new position.
  {
    std::scoped_lock<std::mutex> lock(position_mtx_);
    const auto now = std::chrono::high_resolution_clock::now();
    for (const auto& [robot, _] : robot_model.ToSystemConf(q)) {
      robot_last_push_time_[robot] = now;
    }
  }
  bool pushed {false};
  // Now determine whether to actually set the new position
  {
    std::scoped_lock<std::mutex> lock(position_mtx_);
    if (!pending_position_
        || common::utils::JointPositionsChanged(*pending_position_, q)) {
      pushed = true;
      pending_position_ = q;
    }
  }
  if (pushed) {
    position_cv_.notify_one();
  }
  return true;
}

const fs::path VisualizerHub::ExportToHtml() {
  DRAKE_THROW_UNLESS(draco_ != nullptr);
  const auto html_string = draco_->robot_model().meshcat()->StaticHtml();
  const auto meshcat_sessions_dir {data_path_ / "meshcat"};
  if (!fs::is_directory(meshcat_sessions_dir)) {
    fs::create_directories(meshcat_sessions_dir);
  }
  const auto meshcat_session_path {
      meshcat_sessions_dir
      / fmt::format("meshcat_session-{}.html",
                    common::utils::datetime_str(common::utils::DATETIME_FMT))};
  common::utils::SaveToFile(meshcat_session_path, html_string);
  // Remove the old button, add a new one titled "HTML exported" which does
  // nothing
  DeleteButton(export_html_button_name_);
  AddButton(export_html_success_button_name_, -1);
  return meshcat_session_path;
}
}  // namespace visualization
}  // namespace service
