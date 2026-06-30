#include "planning_service/draco/visualizer/draco_visualizer.h"

#include "planning_service/common/misc_utils.h"
#include "planning_service/draco/client_conversions.h"
namespace draco {
namespace visualizer {

namespace psc = planning_service_client;

namespace {
const double MAX_TRAJECTORY_TIME_LIMIT = 60.0;  // seconds

// Apply the collision matrices snapshot from the planner to the visualizer's
// own robot_constraints. Called once in DracoVisualizer::Main before
// dispatching to any display function, so all display logic sees the same
// collision state the planner had when the item was enqueued.
// The returned ScopedOverride pair must be kept alive for the duration of the
// display call.
[[nodiscard]] CollisionOptionsScope ApplySnapshotScoped(
    const motion::RobotConstraints& robot_constraints,
    const std::optional<draco::CollisionOptionsSnapshot>& snapshot) {
  common::ScopedOverride<Eigen::MatrixXd> padding_scope(
      robot_constraints.collision_padding_matrix());
  common::ScopedOverride<Eigen::MatrixXi> filter_scope(
      robot_constraints.collision_filter_matrix());
  common::ScopedOverride<std::vector<motion::ShapeDescription>>
      collision_shapes_scope(robot_constraints.added_collision_shapes());
  if (!snapshot.has_value()) {
    return {std::move(padding_scope), std::move(filter_scope),
            std::move(collision_shapes_scope)};
  }
  padding_scope.on_event([&robot_constraints]() {
    robot_constraints.SetActivePaddingMatrix();
  });
  filter_scope.on_event([&robot_constraints]() {
    robot_constraints.SetActiveCollisionFilterMatrix();
  });
  collision_shapes_scope.on_set([&robot_constraints]() {
    robot_constraints.AddCollisionShapes();
  });
  collision_shapes_scope.on_restore([&robot_constraints]() {
    robot_constraints.RemoveAllAddedCollisionShapes();
  });
  padding_scope.set(snapshot->padding_matrix);
  filter_scope.set(snapshot->filter_matrix);
  collision_shapes_scope.set(snapshot->collision_shapes);
  return {std::move(padding_scope), std::move(filter_scope),
          std::move(collision_shapes_scope)};
}

/**
 * @brief Perform a single update of the visualizer, optionally setting a new
 * robot configuration before applying new changes.
 *
 * @param robot_constraints Constraints instance.
 * @param q_new Optional new configuration.
 */
void Update(const motion::RobotConstraints& robot_constraints,
            const std::optional<Eigen::VectorXd> q_new = std::nullopt) {
  const auto& robot_model = robot_constraints.robot_model();
  if (q_new) {
    robot_model.SetMeshcatPositions(*q_new);
  }
  const auto q {robot_model.GetMeshcatPositions()};
  const auto clearance {
      robot_constraints.collision_checker().CalcContextRobotClearance(
          &robot_constraints.mutable_collision_checker_context(0), q, 0.0)};
  robot_model.ColorCollidingBodiesInMeshcat(clearance);
  robot_constraints.UpdateConstraintsOnMeshcat();
  robot_model.PublishMeshcatContext();
  if (clearance.size() > 0) {
    logging::log()->debug("DracoVisualizer:Update: Collisions detected:\n{}",
                          robot_constraints.PrintRobotClearance(clearance, q));
  }
}

// Pass systraj and label by value to avoid dangling references
void DisplaySystemTrajectory(const motion::RobotConstraints& robot_constraints,
                             const psc::SystemTimedTrajectory sys_traj,
                             double dt,
                             const std::string label = "Trajectory") {
  const auto& robot_model = robot_constraints.robot_model();
  const auto& plant = robot_model.plant();
  for (const auto& [name, traj] : sys_traj.data()) {
    if (!plant.HasModelInstanceNamed(name)) {
      throw std::runtime_error(fmt::format(
          "DracoVisualizer: DisplaySystemTrajectory: "
          "Model instance [{}] not found in meshcat robot model hashed [{}]",
          name, robot_constraints.constraints_hash()));
    }
    auto model_idx = plant.GetModelInstanceByName(name);
    if (traj.dim() != plant.num_positions(model_idx)) {
      throw std::runtime_error(
          fmt::format("DracoVisualizer: DisplaySystemTrajectory: "
                      "Trajectory for model instance {} has wrong dimension. "
                      "Expected {}, got {}.",
                      name, plant.num_positions(model_idx), traj.dim()));
    }
  }
  // Let's get sys_traj start_time and end_time
  std::optional<double> start_time;
  for (const auto& [_, traj] : sys_traj.data()) {
    if (start_time.has_value()) {
      if (std::abs(traj.start_time() - start_time.value()) > 1e-6) {
        throw std::runtime_error(
            fmt::format("DracoVisualizer: DisplaySystemTrajectory: "
                        "Trajectories have different start times. "
                        "This is not allowed!"
                        "Expected {}, got {}.",
                        start_time.value(), traj.start_time()));
      }
    } else {
      start_time = traj.start_time();
    }
  }
  if (!start_time.has_value()) {
    throw std::runtime_error(
        "DracoVisualizer: DisplaySystemTrajectory: Empty system trajectory "
        "provided. ");
  }
  // get end time
  double end_time = 0.0;
  for (const auto& [_, traj] : sys_traj.data()) {
    if (traj.end_time() > end_time) {
      end_time = traj.end_time();
    }
  }
  if (end_time > MAX_TRAJECTORY_TIME_LIMIT) {
    logging::log()->warn(
        "DracoVisualizer: DisplaySystemTrajectory: End time is greater than {} "
        "seconds, which is our current limit. Not displaying the trajectory "
        "in Meshcat. Please use a shorter trajectory.",
        MAX_TRAJECTORY_TIME_LIMIT);
    return;
  }
  // Get values (including end_time)
  std::vector<std::pair<motion::system_conf_t, double>> sys_conf_time_pairs;
  for (double t = start_time.value(); t <= end_time; t += dt) {
    motion::system_conf_t sys_conf;
    for (const auto& [name, traj] : sys_traj.data()) {
      sys_conf[name] = traj.Value(t);
    }
    sys_conf_time_pairs.emplace_back(sys_conf, t - start_time.value());
  }
  auto meshcat = robot_constraints.robot_model().meshcat();
  motion::CheckSatisfiedOptions check_satisfied_options_meshcat;
  check_satisfied_options_meshcat.color_collisions_meshcat = true;
  Eigen::VectorXd q =
      Eigen::VectorXd::Zero(robot_model.plant().num_positions());
  std::string time_label = "Time (s)";
  std::string end_label = "Double Click to Exit This Plan";
  std::string pause_label = "Pause";
  std::string play_label = "Play";
  std::string display_label = fmt::format("{}", label);
  meshcat->AddSlider(time_label, 0, end_time - start_time.value(), dt, 0.0);
  meshcat->AddButton(pause_label);
  meshcat->AddButton(play_label);
  meshcat->AddButton(end_label);
  meshcat->AddButton(display_label);
  bool play = false;
  while (meshcat->GetButtonClicks(end_label) < 2) {
    double t = meshcat->GetSliderValue(time_label);
    int i = static_cast<int>(t / dt);
    i = std::min(i, static_cast<int>(sys_conf_time_pairs.size() - 1));
    const auto& [sysconf, _] = sys_conf_time_pairs[i];
    for (const auto& [name, conf] : sysconf) {
      if (!robot_model.plant().HasModelInstanceNamed(name)) {
        throw std::runtime_error(
            fmt::format("DracoVisualizer: DisplaySystemTrajectory: "
                        "Model instance [{}] not found in meshcat robot model.",
                        name));
      }
      const auto model_instance =
          robot_model.plant().GetModelInstanceByName(name);
      if (conf.rows() != robot_model.plant().num_positions(model_instance)) {
        throw std::runtime_error(fmt::format(
            "DracoVisualizer: DisplaySystemTrajectory: "
            "Configuration for model instance {} has wrong dimension. "
            "Expected {}, got {}.",
            name, robot_model.plant().num_positions(model_instance),
            conf.rows()));
      }
      robot_model.plant().SetPositionsInArray(model_instance, conf, &q);
    }
    Update(robot_constraints, q);
    if (meshcat->GetButtonClicks(pause_label)) {
      play = false;
      // Reset the button
      meshcat->AddButton(pause_label);
    }
    bool did_reset = false;
    if (meshcat->GetButtonClicks(play_label)) {
      if (play) {
        // Already playing — pressing Play again resets to the start
        meshcat->SetSliderValue(time_label, 0.0);
        did_reset = true;
      } else {
        play = true;
      }
      // Reset the button
      meshcat->AddButton(play_label);
    }
    if (play && !did_reset) {
      meshcat->SetSliderValue(time_label, std::min(end_time, t + dt));
    }
    int dt_ms = static_cast<int>(dt * 1000);
    std::this_thread::sleep_for(std::chrono::milliseconds(dt_ms));
  }
  logging::log()->info(
      "DracoVisualizer: DisplaySystemTrajectory: Stopping playback.");
  // Remove sliders and buttons
  meshcat->DeleteSlider(time_label);
  meshcat->DeleteButton(pause_label);
  meshcat->DeleteButton(play_label);
  meshcat->DeleteButton(end_label);
  meshcat->DeleteButton(display_label);
}

void DisplayConfiguration(const motion::RobotConstraints& robot_constraints,
                          const Eigen::VectorXd& q) {
  DRAKE_THROW_UNLESS(
      q.size() == robot_constraints.robot_model().plant().num_positions());
  Update(robot_constraints, q);
}

std::pair<drake::geometry::Cylinder, drake::math::RigidTransform<double>>
CreateXAxisCylinder(double length, double radius) {
  // Make a cylinder in the shape of an axis
  drake::geometry::Cylinder cylinder(radius, length);
  // create rigid transform that sets it in x-direction
  drake::math::RigidTransform<double> X_x;
  X_x.set_translation(Eigen::Vector3d(length / 2, 0, 0));
  X_x.set_rotation(
      drake::math::RollPitchYaw<double>(0, -M_PI_2, 0.0).ToRotationMatrix());
  return {cylinder, X_x};
}

// Create Y and Z axes cylinders
std::pair<drake::geometry::Cylinder, drake::math::RigidTransform<double>>
CreateYAxisCylinder(double length, double radius) {
  // Make a cylinder in the shape of an axis
  drake::geometry::Cylinder cylinder(radius, length);
  // create rigid transform that sets it in y-direction
  drake::math::RigidTransform<double> X_y;
  X_y.set_translation(Eigen::Vector3d(0, length / 2, 0));
  X_y.set_rotation(
      drake::math::RollPitchYaw<double>(M_PI_2, 0, 0).ToRotationMatrix());
  return {cylinder, X_y};
}

std::pair<drake::geometry::Cylinder, drake::math::RigidTransform<double>>
CreateZAxisCylinder(double length, double radius) {
  // Make a cylinder in the shape of an axis
  drake::geometry::Cylinder cylinder(radius, length);
  // create rigid transform that sets it in z-direction
  drake::math::RigidTransform<double> X_z;
  X_z.set_translation(Eigen::Vector3d(0, 0, length / 2));
  X_z.set_rotation(
      drake::math::RollPitchYaw<double>(0, 0, 0).ToRotationMatrix());
  return {cylinder, X_z};
}

void DisplayFailedIKSolution(const motion::RobotConstraints& robot_constraints,
                             const FailedIK& failed_ik) {
  const auto& [q, frps_index] = failed_ik;
  DRAKE_THROW_UNLESS(
      q.size() == robot_constraints.robot_model().plant().num_positions());
  Update(robot_constraints, q);
  auto meshcat = robot_constraints.robot_model().meshcat();
  const auto& plant = robot_constraints.robot_model().plant();
  const auto& sg_inspector =
      robot_constraints.robot_model().scene_graph().model_inspector();
  std::vector<std::string> path_As;
  for (int i = 0; i < std::ssize(frps_index); ++i) {
    const auto& [frame_A_name, frame_B_name, X_AB] = frps_index[i];
    const auto& frame_A = plant.get_frame(frame_A_name);
    const auto& frame_B = plant.get_frame(frame_B_name);
    robot_constraints.robot_model().AddFrameAxesToMeshcat(frame_A);
    robot_constraints.robot_model().AddFrameAxesToMeshcat(frame_B);
    const auto& model_A_name =
        plant.GetModelInstanceName(frame_A.model_instance());
    auto path_A = fmt::format("/drake/visual/{}/{}/FailedIK_{}/", model_A_name,
                              frame_A.name(), i);
    path_As.push_back(path_A);
    const auto& body_B = frame_B.body();
    auto frame_id = plant.GetBodyFrameIdOrThrow(body_B.index());
    auto B_collision_geos =
        sg_inspector.GetGeometries(frame_id, drake::geometry::Role::kProximity);
    logging::log()->info(
        "DracoVisualizer: DisplayFailedIKSolution: Displaying failed IK "
        "solution for frames {} and {} at path {} with {} collision "
        "geometries.",
        frame_A.scoped_name().to_string(), frame_B.scoped_name().to_string(),
        path_A, B_collision_geos.size());
    for (const auto& geo_id : B_collision_geos) {
      const auto& shape = sg_inspector.GetShape(geo_id);
      meshcat->SetObject(path_A, shape,
                         drake::geometry::Rgba(1.0, 0.3, 0.0, 0.5));
      meshcat->SetTransform(path_A, X_AB);
    }
    double length = 0.1;    // Length of the axis
    double radius = 0.002;  // Radius of the axis
    // Need to add frame axes for the failing frame at path_A
    auto [x_cylinder, X_x] = CreateXAxisCylinder(length, radius);
    auto [y_cylinder, X_y] = CreateYAxisCylinder(length, radius);
    auto [z_cylinder, X_z] = CreateZAxisCylinder(length, radius);
    const auto path_A_x = fmt::format("{}/x", path_A);
    const auto path_A_y = fmt::format("{}/y", path_A);
    const auto path_A_z = fmt::format("{}/z", path_A);
    meshcat->SetObject(path_A_x, x_cylinder,
                       drake::geometry::Rgba(1, 0, 0, 0.4));  // Red
    meshcat->SetTransform(path_A_x, X_AB * X_x);
    meshcat->SetObject(path_A_y, y_cylinder,
                       drake::geometry::Rgba(0, 1, 0, 0.4));  // Green
    meshcat->SetTransform(path_A_y, X_AB * X_y);
    meshcat->SetObject(path_A_z, z_cylinder,
                       drake::geometry::Rgba(0, 0, 1, 0.4));  // Blue
    meshcat->SetTransform(path_A_z, X_AB * X_z);
  }
  std::string click_msg = "Double click to exit this failed IK solution.";
  meshcat->AddButton(click_msg);
  while (meshcat->GetButtonClicks(click_msg) < 2) {
    // Wait for double click to exit
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  // Clean added path
  meshcat->DeleteButton(click_msg);
  for (const auto& path_A : path_As) {
    meshcat->Delete(path_A);
  }
}

}  // namespace

DracoVisualizer::DracoVisualizer(
    const std::string& xml_file,
    const drake::multibody::parsing::ModelDirectives& dmd,
    const motion::RobotMeshcatParams& robot_meshcat_params,
    const motion::ConstraintsAdapter& constraints_adapter,
    std::optional<std::map<std::string, Eigen::VectorXd>> default_configuration,
    int max_queue_size, int viz_frequency, int meshcat_beat_interval_ms)
    : max_queue_size_(static_cast<long unsigned int>(max_queue_size)),
      viz_delta_time_(1.0 / viz_frequency),
      meshcat_beat_interval_ms_(meshcat_beat_interval_ms) {
  if (main_thread_.joinable()) {
    logging::log()->warn(
        "DracoVisualizer: Run: Main thread is already running, not starting a "
        "new one.");
    return;
  }
  main_thread_ = std::thread(&DracoVisualizer::Main, this, xml_file, dmd,
                             robot_meshcat_params, constraints_adapter,
                             std::move(default_configuration));
  while (!is_running_.load()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
  }
}

void DracoVisualizer::AddSetFixedOffsetFramePoseInParentFrame(
    const std::string& frame_scoped_name,
    const drake::math::RigidTransformd& offset) {
  if (!is_running_.load()) {
    throw std::runtime_error(
        "DracoVisualizer: SetFixedOffsetFramePoseInParentFrame: Visualizer is "
        "not running. Cannot set frame pose.");
  }
  std::lock_guard<std::mutex> lock(mutex_);
  // Now we can safely call the method on the robot model
  set_pose_in_parent_frame_requests_.emplace(
      std::make_pair(frame_scoped_name, offset));
}

void DracoVisualizer::Main(
    const std::string& xml_file,
    const drake::multibody::parsing::ModelDirectives& dmd,
    const motion::RobotMeshcatParams& robot_meshcat_params,
    const motion::ConstraintsAdapter& constraints_adapter,
    std::optional<std::map<std::string, Eigen::VectorXd>>
        default_configuration) {
  // Need to make sure the meshcat settings are correct
  auto robot_model = motion::RobotModel(xml_file, dmd, robot_meshcat_params);
  auto robot_constraints =
      motion::RobotConstraints(robot_model, constraints_adapter, 1);
  DRAKE_DEMAND(robot_constraints.num_threads() == 1);
  DRAKE_DEMAND(robot_model.meshcat() != nullptr);
  auto meshcat_colored_msg = fmt::format(
      FMT_ITALIC | fg(FMT_BLUE), "Draco instance {} at meshcat port {}",
      robot_constraints.constraints_hash(), robot_model.meshcat()->port());
  logging::log()->info("DracoVisualizer: {}", meshcat_colored_msg);
  // Apply default configuration if provided
  std::optional<Eigen::VectorXd> default_q;
  if (default_configuration.has_value()) {
    default_q = robot_model.ToGeneralizedPosition(*default_configuration);
    robot_model.SetMeshcatPositions(*default_q);
  }
  Update(robot_constraints);
  auto meshcat = robot_model.meshcat();
  meshcat_port_ = meshcat->port();
  const std::string id_button_label =
      "ID: " + std::to_string(robot_constraints.constraints_hash());
  meshcat->AddButton(id_button_label);
  is_running_.store(true);
  std::map<std::string, AnnotatedVisualizable> label_viz_map;
  std::deque<std::string> label_order;
  std::optional<std::string> label_to_display = std::nullopt;
  // Initialize the scope here so that for non-blocking displays (specifically,
  // configurations), the options are applied for the duration of the display
  std::optional<CollisionOptionsScope> current_display_scope;
  while (true) {
    std::unique_lock<std::mutex> lock(mutex_);
    bool should_awake = cv_.wait_for(
        lock, std::chrono::milliseconds(meshcat_beat_interval_ms_),
        [this, &meshcat]() {
          return !incoming_queue_.empty()
                 || !set_pose_in_parent_frame_requests_.empty() || stop_.load();
        });
    if (stop_.load()) {
      logging::log()->debug("DracoVisualizer: Stopping visualizer.");
      break;
    }
    if (should_awake) {
      // Drain new additions
      while (!incoming_queue_.empty()) {
        logging::log()->debug(
            "DracoVisualizer: Main thread processing new visualizable object.");
        auto [label, annotated] = incoming_queue_.front();
        incoming_queue_.pop_front();
        label_viz_map[label] = annotated;
        label_order.push_back(label);
        meshcat->AddButton(label);
        logging::log()->debug(
            "DracoVisualizer:Main: '{}' available to display for {}", label,
            meshcat_colored_msg);
      }
      // Evict if too many
      while (std::size(label_order) > max_queue_size_) {
        const std::string oldest = label_order.front();
        label_order.pop_front();
        label_viz_map.erase(oldest);
        const auto meshcat_buttons = meshcat->GetButtonNames();
        DRAKE_DEMAND(common::utils::contains(meshcat_buttons, oldest));
        meshcat->DeleteButton(oldest);
      }
      // Check the axes queue
      while (!set_pose_in_parent_frame_requests_.empty()) {
        const auto [frame_name, offset] =
            set_pose_in_parent_frame_requests_.front();
        set_pose_in_parent_frame_requests_.pop();
        const auto& frame = robot_model.GetScopedFrameByName(frame_name);
        robot_model.SetFixedOffsetFramePoseInParentFrame(frame, offset);
        // Get the parent frame and display its axes too
        const auto* fixed_offset_frame =
            dynamic_cast<const drake::multibody::FixedOffsetFrame<double>*>(
                &frame);
        const auto& parent_frame = fixed_offset_frame->parent_frame();
        const double axis_length {0.07};
        const double axis_radius {0.0005};
        robot_model.AddFrameAxesToMeshcat(frame, 0.7, axis_length, axis_radius,
                                          true);
        robot_model.AddFrameAxesToMeshcat(parent_frame, 0.3, axis_length,
                                          axis_radius, true);
      }
    }
    lock.unlock();
    // Treat the ID button as a "reset" to the default configuration if it
    // exists
    if (meshcat->GetButtonClicks(id_button_label) > 0) {
      meshcat->AddButton(id_button_label);  // Reset click count.
      if (default_q.has_value()) {
        current_display_scope.reset();
        Update(robot_constraints, *default_q);
      }
    }
    std::optional<std::string> clicked_label = std::nullopt;
    // Now check if any button was clicked since last time
    for (const auto& [label, _] : label_viz_map) {
      if (meshcat->GetButtonClicks(label)) {
        meshcat->AddButton(label);  // Reset the button
        clicked_label = label;
        break;
      }
    }
    // Now let's read the queue of visualizable objects
    if (clicked_label.has_value()) {
      const auto& annotated = label_viz_map[clicked_label.value()];
      logging::log()->info(
          "DracoVisualizer: Playing visualizable object: {} on meshcat port {}",
          clicked_label.value(), meshcat->port());
      // Enter "play mode": hide buttons
      for (const auto& [label, _] : label_viz_map) {
        auto meshcat_buttons = meshcat->GetButtonNames();
        DRAKE_DEMAND(common::utils::contains(meshcat_buttons, label));
        meshcat->DeleteButton(label);
      }
      const auto& label = clicked_label.value();
      // Release the previous scope first so the Overrideables are unowned
      // before the new scope tries to acquire
      current_display_scope.reset();
      current_display_scope =
          ApplySnapshotScoped(robot_constraints, annotated.snapshot);
      bool needs_reset {false};
      if (std::holds_alternative<psc::SystemTimedTrajectory>(
              annotated.visualizable)) {
        const auto sys_traj =
            std::get<psc::SystemTimedTrajectory>(annotated.visualizable);
        DisplaySystemTrajectory(robot_constraints, sys_traj, viz_delta_time_,
                                label);
        needs_reset = true;
      } else if (std::holds_alternative<Eigen::VectorXd>(
                     annotated.visualizable)) {
        const auto q = std::get<Eigen::VectorXd>(annotated.visualizable);
        DisplayConfiguration(robot_constraints, q);
        // Scope intentionally kept alive — shapes stay visible while idle.
      } else if (std::holds_alternative<FailedIK>(annotated.visualizable)) {
        const auto failed_ik = std::get<FailedIK>(annotated.visualizable);
        DisplayFailedIKSolution(robot_constraints, failed_ik);
        needs_reset = true;
      } else {
        logging::log()->error(
            "DracoVisualizer: Main: Unknown visualizable type for label {}",
            label);
      }
      // Back to idle mode, bring back the buttons
      for (const auto& [label, _] : label_viz_map) {
        meshcat->AddButton(label);
      }
      // Reset to default configuration if it exists.
      if (needs_reset) {
        current_display_scope.reset();
        Update(robot_constraints, default_q);
      }
    }
  }
  is_running_.store(false);
  logging::log()->debug("DracoVisualizer: Stopped visualizer.");
}

}  // namespace visualizer
}  // namespace draco
