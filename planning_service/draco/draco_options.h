#pragma once
#include <drake/common/yaml/yaml_io.h>
#include <fmt/format.h>

#include <algorithm>
#include <filesystem>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace draco {

/** Defines visualization behavior. */
enum VisualizerMode {
  kNone,   /* No visualization. */
  kDraco,  /* Create a unique Draco visualizer in its own thread. */
  kNative, /* Use the native visualizer of RobotModel. */
};

struct VisualizerOptions {
  /** The visualizer mode to use. */
  VisualizerMode mode {kNone};
  /** Whether to use the meshcat visualizer or not. */
  bool add_plans_to_visualizer {true};

  /** @warning This option would put planning on hold until the user
   * clicks the "Go" button in the meshcat visualizer. This is useful for
   * debugging and visualization purposes, but it can be annoying if you want to
   * run the planner without waiting for user input.
   */
  bool wait_for_user_go_input_in_meshcat {false};

  /** The size of the plans buffer in the visualizer. */
  int plans_buffer_size {10};

  /** The frequency at which the trajectory is displayed in the visualizer.
   * This is the number of times per second the meshcat positions are updated.
   * For example, if this is set to 20, the trajectory will be displayed using
   * positions displayed at 50 ms intervals (1000 ms / 20 = 50 ms).
   */
  int display_trajectory_frequency {
      20};  // Divide 1000 by this number to get the time in ms

  /** The beat interval of the meshcat in ms, to check if meshcat button is
   * pressed or not.
   */
  int meshcat_beat_interval_ms {100};

  template <typename Archive>
  void Serialize(Archive* a) {
    std::string mode_name {magic_enum::enum_name(mode)};
    a->Visit(drake::MakeNameValue("mode", &mode_name));
    mode = magic_enum::enum_cast<VisualizerMode>(mode_name).value_or(
        VisualizerMode::kNone);
    a->Visit(DRAKE_NVP(add_plans_to_visualizer));
    a->Visit(DRAKE_NVP(wait_for_user_go_input_in_meshcat));
    a->Visit(DRAKE_NVP(plans_buffer_size));
    a->Visit(DRAKE_NVP(display_trajectory_frequency));
    a->Visit(DRAKE_NVP(meshcat_beat_interval_ms));
  }
};

// ToDO: @anyone: Add more options to DracoAdapter
struct PlannerOptions {
  /** Whether to load the straight path planner or not. */
  bool use_straight_path_planner {true};

  /** Whether to load the GCS planner or not. */
  bool use_gcs_planner {true};

  /** Whether to load the sample-based planner or not. */
  bool use_sample_based_planner {true};

  double deconfliction_step {0.0};  // in config space units

  double deconfliction_offset {0.0};  // in config space units

  double async_collison_checking_time_step = 0.0;  // in seconds

  double async_latency_check_per_call = 0.0;  // in seconds

  double latency_comms = 0.0;  // in seconds

  double partial_solution_time_buffer =
      0.3;  // in seconds, added as buffer to the time of collision when
            // considering partial solutions

  double update_spline_per_conf = 0.0;  // in seconds.

  double update_spline_extra_time = 0.0;  // in seconds

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(use_straight_path_planner));
    a->Visit(DRAKE_NVP(use_gcs_planner));
    a->Visit(DRAKE_NVP(use_sample_based_planner));
    a->Visit(DRAKE_NVP(deconfliction_step));
    a->Visit(DRAKE_NVP(deconfliction_offset));
    a->Visit(DRAKE_NVP(async_collison_checking_time_step));
    a->Visit(DRAKE_NVP(async_latency_check_per_call));
    a->Visit(DRAKE_NVP(latency_comms));
    a->Visit(DRAKE_NVP(partial_solution_time_buffer));
    a->Visit(DRAKE_NVP(update_spline_per_conf));
    a->Visit(DRAKE_NVP(update_spline_extra_time));
  }
};

struct DracoOptions {
  VisualizerOptions visualizer_options = VisualizerOptions();
  PlannerOptions planner_options = PlannerOptions();
  // Mapping of models to their default configurations; note the difference
  // between Drake's default_positions field, which maps joint names to default
  // position values.
  std::optional<std::map<std::string, Eigen::VectorXd>>
      default_configuration {};

  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(visualizer_options));
    a->Visit(DRAKE_NVP(planner_options));
    a->Visit(DRAKE_NVP(default_configuration));
  }
};

}  // namespace draco
