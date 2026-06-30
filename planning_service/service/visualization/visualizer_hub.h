/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file visualizer_hub.h

#pragma once
#include <drake/common/yaml/yaml_io.h>
#include <drake/geometry/meshcat.h>

#include <queue>
#include <unordered_map>

#include <expected>
#include <magic_enum/magic_enum.hpp>

#include "planning_service/common/string_utils.h"
#include "planning_service/service/utils/utils.h"

namespace fs = std::filesystem;
namespace psc = planning_service_client;
namespace service {
namespace visualization {

/** Task to be executed on a Meshcat instance. */
using meshcat_task_t = std::function<void(drake::geometry::Meshcat&)>;

/**
 * @brief Data required to spawn a given model in the Meshcat visualizer.
 * Includes a reference to the model geometry (either from a full context or
 * from a DMD), as well as some optional parameters for the visualizer. If no
 * Meshcat parameters are provided, the visualizer will attempt to load from
 * disk and, failing that, use defaults.
 *
 */
struct VisualizerData {
  /** When this is set, load the model defined by the DMD in this context. */
  std::optional<draco::PlanContextId> context_id;
  /** When this is set, load the model defined by the DMD with this name. */
  std::optional<std::string> dmd_filename;
  /** Optional parameters for the Meshcat visualizer. */
  std::optional<motion::RobotMeshcatParams> robot_meshcat_params;
  VisualizerData() = default;
};

enum class VisualizerStatus {
  None,      // Only used for initialization
  Idle,      // No model running, waiting for new data
  Starting,  // Data has been set, coming online
  Active,    // Model loaded and actively publishing
  Stopping   // Stop in progress
};

/** Small virtual interface to enable mocking in unit tests. */
class VisualizerHubInterface {
 public:
  virtual ~VisualizerHubInterface() = default;

  virtual bool LoadAndSetModelData(const VisualizerData& visualizer_data,
                                   const bool force_reload = false) = 0;

  virtual std::expected<bool, std::string> SetConfiguration(
      const psc::SystemConf& system_conf) = 0;
  virtual std::expected<bool, std::string> SetConfiguration(
      const Eigen::VectorXd& q) = 0;

  virtual const drake::math::RigidTransformd CalcPose(
      const std::string_view frame_B_name,
      const std::string_view frame_A_name = "world",
      const std::optional<psc::SystemConf>& system_conf_override =
          std::nullopt) const = 0;

  virtual bool HasDraco() const = 0;
  virtual size_t ActiveHash() const = 0;
  virtual void RequestStop() = 0;
  virtual void WaitOnStatus(const VisualizerStatus& status) const = 0;
  virtual VisualizerStatus GetStatus() const = 0;
  virtual void SetVisualizerOptions(const VisualizerOptions& options) = 0;
  virtual VisualizerOptions GetVisualizerOptions() const = 0;
  virtual void QueueMeshcatTask(const meshcat_task_t& f) = 0;
  /** Reset streaming state so the stream-status button immediately shows
   * Inactive. */
  virtual void ClearStreamingState() = 0;

  /**
   * @brief Resolve a named frame to its Meshcat path under /drake/frames/.
   *
   * Looks up the frame by scoped name (e.g. "wrist_link" or
   * "my_robot::wrist_link") in the loaded plant and returns the full Meshcat
   * path "/drake/frames/{model}/{frame}/". Returns std::nullopt when no model
   * is loaded or the frame cannot be found.
   */
  virtual std::optional<std::string> ResolveFrameMeshcatPath(
      std::string_view frame_name) const = 0;

  /**
   * @brief Show or hide a single named frame's axes in Meshcat.
   *
   * @param path    Meshcat path e.g. "/drake/frames/my_robot/wrist_link/".
   * @param visible True to show the triad, false to hide it.
   * @return true if the frame was found, false otherwise.
   */
  virtual bool ToggleFrame(std::string_view path, bool visible) = 0;

  /**
   * @brief Show or hide all frame axes whose Meshcat paths start with
   * /drake/frames/{path}.
   *
   * Always hides by teleporting to (0, 0, -1e6) and shows by recomputing the
   * current world pose, regardless of enable_frame_toggle_web. SetProperty is
   * not used here because the associated bodies may themselves be teleported
   * away during operation, and a visibility-only toggle would leave the axes
   * floating at the wrong location.
   *
   * Sets hidden_by_toggle on each matching frame so UpdateFrameAxes does not
   * overwrite the teleport each tick.
   *
   * @param path    Path component after stripping any leading "visual/" or
   *                "collision/" prefix, e.g. "panda_east".
   * @param active  True to restore (recompute world pose), false to hide
   *                (teleport to (0, 0, -1e6)).
   * @return (meshcat_path, transform) pairs to be applied via SetTransform.
   */
  virtual std::vector<std::pair<std::string, drake::math::RigidTransformd>>
  ToggleFramesByPath(std::string_view path, bool active) = 0;
};

/**
 * @brief Hub to store model data for a given visualizer session as well as
 * other viz options
 */
class VisualizerHub final : public VisualizerHubInterface {
 public:
  /**
   * @brief Constructor.
   * @param system_name  Enabled system name
   */
  VisualizerHub(const std::string& system_name,
                const std::optional<VisualizerOptions>& options = std::nullopt)
      : system_name_ {system_name},
        data_path_ {data_path_root_ / system_name_} {
    options_ = std::make_unique<VisualizerOptions>();
    if (options) {
      logging::log()->info("VH:Constructor: Using passed visualizer options");
      options_ = std::make_unique<VisualizerOptions>(*options);
    } else {
      logging::log()->warn(
          "VH:Constructor: No visualizer_options.yaml found; using "
          "defaults.");
    }
    // Check for default init
    if (options_->default_init) {
      logging::log()->info("VH:Constructor: Default init enabled");
      std::string deployment;
      const char* result = std::getenv("DEPLOYMENT");
      if (result == nullptr || result[0] == '\0') {
        throw std::runtime_error(
            "VH:Constructor: DEPLOYMENT environment variable not set for "
            "default init!");
      }
      deployment = common::utils::to_lower(std::string(result));
      logging::log()->info(
          "VH:Constructor: Loading default context for deployment {}",
          deployment);
      const auto context_it = options_->default_contexts.find(deployment);
      if (context_it == options_->default_contexts.end()) {
        throw std::runtime_error(
            "VH:Constructor: No default context found for deployment "
            + deployment);
      }
      VisualizerData visualizer_data;
      visualizer_data.context_id = draco::PlanContextId {context_it->second};

      if (!LoadAndSetModelData(visualizer_data)) {
        throw std::runtime_error(
            "VH:Constructor: Failed to load default model on initialization!");
      }
    }
  }
  VisualizerHub(VisualizerHub& other) = delete;
  VisualizerHub(VisualizerHub&& other) = delete;
  ~VisualizerHub() = default;

  /**
   * @brief Main loop for the visualizer hub. Waits to receive a new request for
   * visualization, and upon receipt, configures the visualizer according to a
   * set of options and publishes the visual model to the Meshcat port.
   *
   */
  void Run();

 private:
  /**
   * @brief Setup the model given the current set of visualizer options, which
   * controls things like IRIS robots and other optional assets.
   *
   * @return true if all configured options were applied successfully, false
   * otherwise
   */
  bool Setup();

  /** Apply opacity to all Meshcat visual paths belonging to `robot` (the
   * robot's own model plus any tool models prefixed with "<robot>-").
   * @param robot  model instance name of the robot
   * @param active true → full opacity, false → dimmed
   */
  void SetRobotStreamActive(const std::string& robot, bool active);
  /**
   * @brief Register all plant frames in Meshcat under /drake/frames/.
   * Each frame is hidden individually via SetProperty so users can toggle
   * them in Meshcat's scene tree. Moving frames are stored in
   * frame_axes_ for per-tick transform updates.
   */
  void SetupFrameAxes();

  /**
   * @brief Recompute and push world-space transforms for all non-static,
   * non-toggle-hidden moving frame triads.
   */
  void UpdateFrameAxes();

  /** Set the status. */
  void SetStatus(const VisualizerStatus& status) {
    logging::log()->debug("VH:SetStatus: {} -> {}",
                          magic_enum::enum_name(status_.load()),
                          magic_enum::enum_name(status));
    status_ = status;
    status_.notify_all();
  }

  const fs::path ExportToHtml();

  bool HasButton(const std::string_view button_name) const {
    DRAKE_THROW_UNLESS(draco_ != nullptr);
    const auto button_names {draco_->robot_model().meshcat()->GetButtonNames()};
    return std::find(button_names.begin(), button_names.end(),
                     button_name.data())
           != button_names.end();
  }

  void ResetButtons() {
    DRAKE_THROW_UNLESS(draco_ != nullptr);
    for (const auto& name : draco_->robot_model().meshcat()->GetButtonNames()) {
      draco_->robot_model().meshcat()->DeleteButton(name);
    }
    for (const auto& [name, keycode] : button_order_) {
      draco_->robot_model().meshcat()->AddButton(name, keycode);
    }
  }

  void DeleteButton(const std::string_view button_name) {
    DRAKE_THROW_UNLESS(draco_ != nullptr);
    // Remove from button order
    button_order_.erase(
        std::remove_if(button_order_.begin(), button_order_.end(),
                       [&](const auto& pair) {
                         return pair.first == button_name.data();
                       }),
        button_order_.end());
    draco_->robot_model().meshcat()->DeleteButton(button_name.data());
  }
  /**
   * @brief Add a button to the meshcat visualizer GUI.
   *
   * @param button_name
   * @param idx
   * @param keycode
   */
  void AddButton(const std::string_view button_name, int idx = -1,
                 const std::string_view keycode = "") {
    DRAKE_THROW_UNLESS(draco_ != nullptr);
    auto it {button_order_.begin()};
    if (idx < 0) {
      // support backwards indexing
      it = button_order_.end() + idx + 1;
    } else {
      it += idx;
    }
    if (it < button_order_.begin() || it > button_order_.end()) {
      throw std::runtime_error(
          fmt::format("Invalid button index: {} for button order of size {}",
                      idx, button_order_.size()));
    }
    button_order_.insert(it, {button_name.data(), keycode.data()});
    ResetButtons();
  }

 public:
  /**
   * @brief Load all model data for visualization. Given the content of
   * visualizer data, load any additional required resources from disk, and
   * create and set the corresponding Draco adapter.
   *
   * @param visualizer_data Data specifying the model/scene to be visualized
   * @param force_reload If true, override the running model and reload the
   * new scene
   * @return true on success, false otherwise
   */
  bool LoadAndSetModelData(const VisualizerData& visualizer_data,
                           const bool force_reload = false) override;

  /** Set the meshcat model positions with a system configuration. */
  std::expected<bool, std::string> SetConfiguration(
      const psc::SystemConf& system_conf) override;

  /** Set the meshcat model positions with a fully specified joint
   * configuration. */
  std::expected<bool, std::string> SetConfiguration(
      const Eigen::VectorXd& q) override;

  /**
   * @brief Given a target frame (frame_B) and relative frame (frame_A), compute
   * the pose of B relative to A (X_AB).
   *
   * @param frame_B_name The target frame.
   * @param frame_A_name The relative frame. Defaults to 'world'.
   * @return const drake::math::RigidTransformd
   */
  const drake::math::RigidTransformd CalcPose(
      const std::string_view frame_B_name,
      const std::string_view frame_A_name = "world",
      const std::optional<psc::SystemConf>& system_conf_override =
          std::nullopt) const override;

  /** Return true if a Draco has been loaded. */
  inline bool HasDraco() const override {
    return draco_ != nullptr;
  }

  /** @copydoc VisualizerHubInterface::ResolveFrameMeshcatPath */
  std::optional<std::string> ResolveFrameMeshcatPath(
      std::string_view frame_name) const override;

  /** @copydoc VisualizerHubInterface::ToggleFrame */
  bool ToggleFrame(std::string_view path, bool visible) override;

  /** @copydoc VisualizerHubInterface::ToggleFramesByPath */
  std::vector<std::pair<std::string, drake::math::RigidTransformd>>
  ToggleFramesByPath(std::string_view path, bool active) override;

  /** Get the active hash. */
  inline size_t ActiveHash() const override {
    DRAKE_THROW_UNLESS(draco_ != nullptr);
    return draco_->robot_constraints().constraints_hash();
  }

  /**
   * @brief Set the flag indicating whether or not a stop is requested.
   *
   * Used as a trigger to kill the active visualizer model from outside the main
   * `Run` loop.
   */
  inline void RequestStop() override {
    logging::log()->warn("VH:RequestStop: Setting to true");
    SetStatus(VisualizerStatus::Stopping);
  }

  /**
   * @brief Wait for the value of the visualizer's status to change from
   * `status` to something else. If the status is not equivalent to the passed
   * `status`, this does nothing.
   */
  inline void WaitOnStatus(const VisualizerStatus& status) const override {
    logging::log()->debug(
        "VH:WaitOnStatus: Waiting for status to change from {}",
        magic_enum::enum_name(status));
    // Time the wait
    const auto start = hr_clock::now();
    status_.wait(status);
    const auto end = hr_clock::now();
    const auto duration =
        std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    logging::log()->debug("VH:WaitOnStatus: Waited for {} ms ",
                          duration.count());
  }

  inline VisualizerStatus GetStatus() const override {
    return status_.load();
  }

  /** Set the visualizer options. */
  inline void SetVisualizerOptions(const VisualizerOptions& options) override {
    options_ = std::make_unique<VisualizerOptions>(options);
  }

  /** Get the current visualizer options. */
  inline VisualizerOptions GetVisualizerOptions() const override {
    return *options_;
  }

  /**
   * @brief Queue a meshcat task to be executed in the visualizer thread.
   *
   * @param f The task to be executed, as a function that takes a Meshcat
   * instance.
   *
   */
  void QueueMeshcatTask(const meshcat_task_t& f) override {
    {
      std::scoped_lock<std::mutex> lock(meshcat_task_queue_mtx_);
      meshcat_task_queue_.push(f);
    }
    meshcat_task_queue_cv_.notify_one();
  }

  /** Reset streaming state so that the stream-status button immediately
   * flips back to Inactive rather than waiting for the timeout. */
  void ClearStreamingState() override {
    std::scoped_lock<std::mutex> lock(position_mtx_);
    robot_last_push_time_.clear();
  }

  const draco::Draco& draco() const {
    DRAKE_THROW_UNLESS(draco_ != nullptr);
    return *draco_;
  }

 private:
  /**
   * @brief Setup joint limit indicators for each joint in the model. These are
   * represented as a thin disk about the axis of rotation of the joint. They
   * will appear as yellow when within a certain fraction of the limit, and red
   * at said limit.
   *
   */
  void SetupJointLimitIndicators();

  /**
   * @brief Reposition and recolor each indicator disk to match the current
   * robot configuration.
   * @param q  Full generalized positions from the plant (as returned by
   *           RobotModel::GetMeshcatPositions()). */
  void UpdateJointLimitIndicators(const Eigen::VectorXd& q);

  struct JointLimitIndicator {
    int position_index;  ///< Index into the plant's full position vector.
    double q_min;        ///< Lower position limit (rad).
    double q_max;        ///< Upper position limit (rad).
    std::string path;    ///< Meshcat path for the disk geometry.
    /// Frame index of frame_on_child (= link_N_d body frame). Drives both the
    /// disk's orientation (rotates with the joint) and one end of the midpoint
    /// used for the disk's world-space position.
    drake::multibody::FrameIndex child_frame_index;
    /// Body frame index of link_N (child of the `_d` fixed joint) — the other
    /// end of the midpoint. Disk sits halfway between link_N_d and link_N.
    drake::multibody::FrameIndex link_n_frame_index;
    /// Fixed rotation aligning the cylinder's symmetry axis (Z) with the
    /// revolute axis, expressed in frame_on_child.
    drake::math::RigidTransformd X_J_disk;
  };

  std::vector<JointLimitIndicator> joint_limit_indicators_;

  /// Fraction of the joint range within which the indicator appears (yellow →
  /// red gradient as the joint approaches its limit).
  static constexpr double kJointLimitIndicatorAlpha {0.5};
  static constexpr double kJointLimitWarnFraction {0.15};
  /// Outer radius of the disk indicator (metres).
  static constexpr double kJointLimitDiskRadius {0.125};
  /// Thickness (height) of the disk indicator (metres).
  static constexpr double kJointLimitDiskThickness {0.004};
  /// Height of the directional arrow cone.
  static constexpr double kJointLimitArrowHeight {kJointLimitDiskRadius * 0.4};
  /// Base radius of the directional arrow cone.
  static constexpr double kJointLimitArrowRadius {kJointLimitArrowHeight * 0.2};
  // ---- end joint limit indicators ----

  const std::string system_name_;
  const fs::path data_path_root_ {"/data"};
  const fs::path data_path_;
  // frequency at which the active meshcat model is updated
  const uint8_t meshcat_active_freq_hz_ {100};
  // frequency at which the status is evaluated
  const uint8_t status_freq_hz_ {10};
  // draco adapter
  std::optional<draco::DracoAdapter> draco_adapter_ {std::nullopt};
  // options, which may be set independently of model
  std::unique_ptr<VisualizerOptions> options_ {};
  // unique_ptr to Draco
  std::unique_ptr<draco::Draco> draco_ {nullptr};
  std::optional<Eigen::VectorXd> default_positions_ {};
  // Frame axes under /drake/frames/, mapped by their path.
  struct FrameAxesInfo {
    drake::multibody::FrameIndex index;
    /** True if the frame does not need to be recomputed. */
    bool is_static {false};
    /** True when hidden as part of a ToggleObject/ToggleFrame call.
     *  UpdateFrameAxes skips these so the teleport transform is not
     *  overwritten each tick. */
    bool hidden_by_toggle {false};
  };
  std::map<std::string, FrameAxesInfo> frame_axes_;
  // flag to trigger a start or stop
  std::atomic<bool> stop_requested_ {false};
  // Statuses control the state of the visualizer hub
  std::atomic<VisualizerStatus> status_ {VisualizerStatus::Idle};
  std::atomic<VisualizerStatus> last_status_ {VisualizerStatus::None};
  // requested joint position — single pending slot; producer overwrites,
  // consumer takes (sets to nullopt). No queue needed: only the latest
  // position matters for visualization.
  std::mutex position_mtx_;
  std::condition_variable position_cv_;
  std::optional<Eigen::VectorXd> pending_position_ {std::nullopt};
  using RobotPushTimeMap = std::unordered_map<
      std::string, std::chrono::time_point<std::chrono::high_resolution_clock>>;
  // Per-robot last push time.
  RobotPushTimeMap robot_last_push_time_;
  const int position_push_timeout_ms_ {1000};
  // tasks for the visualizer
  std::mutex meshcat_task_queue_mtx_;
  std::condition_variable meshcat_task_queue_cv_;
  std::queue<meshcat_task_t> meshcat_task_queue_;
  // Buttons
  const std::string export_html_button_name_ {"Export to HTML"};
  const std::string export_html_success_button_name_ {"HTML Exported."};
  const std::string reset_to_default_button_name_ {"Reset Positions"};
  std::string position_stream_button_name_;
  const std::string position_stream_inactive_button_name_ {
      "Position Stream: Inactive"};
  std::vector<std::pair<std::string, std::string>> button_order_;
};

}  // namespace visualization
}  // namespace service
