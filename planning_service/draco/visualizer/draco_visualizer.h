#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

#include "planning_service/draco/draco.h"
#include "planning_service_client/planner/motion_plan_result.h"

namespace draco {
namespace visualizer {

using FailedIK =
    std::pair<Eigen::VectorXd,
              std::vector<std::tuple<drake::multibody::FrameIndex,
                                     drake::multibody::FrameIndex,
                                     drake::math::RigidTransformd>>>;

using FrameAxesAdapter =
    std::tuple<std::string, double, double>;  // name, transp

/** A planner based meshcat visualizer for Draco. This visualizer will run in
 * its own thread and will display the results of the planner in a meshcat
 * instance. It will also allow the user to interact with the visualizer by
 * clicking buttons to kill the visualizer or to display the results of the
 * planner.
 */
class DracoVisualizer {
 public:
  using Visualizable =
      std::variant<Eigen::VectorXd,
                   planning_service_client::SystemTimedTrajectory, FailedIK>;

  /** A Visualizable bundled with the collision matrix state that was active
   * on the planner's RobotConstraints when this item was enqueued.
   */
  struct AnnotatedVisualizable {
    Visualizable visualizable;
    std::optional<draco::CollisionOptionsSnapshot> snapshot;
  };

  /** Constructor for the DracoVisualizer.
   * @param xml_file the path to the URDF or SDF file of the robot
   * @param dmd the model directives to use for parsing the robot model
   * @param robot_meshcat_params the parameters for the robot meshcat
   * @param constraints_adapter the constraints adapter to use for the robot
   * @param max_queue_size the maximum size of the incoming queue
   * @param viz_frequency the frequency of the visualizer in Hz
   * @param meshcat_beat_interval_ms the beat interval of the meshcat in ms, to
   * check if meshcat should be destroyed or not.
   */
  DracoVisualizer(const std::string& xml_file,
                  const drake::multibody::parsing::ModelDirectives& dmd,
                  const motion::RobotMeshcatParams& robot_meshcat_params,
                  const motion::ConstraintsAdapter& constraints_adapter,
                  std::optional<std::map<std::string, Eigen::VectorXd>>
                      default_configuration = std::nullopt,
                  int max_queue_size = 10, int viz_frequency = 20,
                  int meshcat_beat_interval_ms = 100);

  ~DracoVisualizer() {
    stop_.store(true);
    logging::log()->debug(
        "DracoVisualizer: Destructor called, waiting for main thread to join.");
    if (main_thread_.joinable()) {
      main_thread_.join();
    }
    logging::log()->debug(
        "DracoVisualizer: Destructor called, main thread joined.");
  }

  void Kill() {
    logging::log()->debug("DracoVisualizer: Kill called.");
    stop_.store(true);
    cv_.notify_all();
  }

  template <typename T>
  void Add(
      const T& visualizable, const std::string& label = "N/A",
      std::optional<draco::CollisionOptionsSnapshot> snapshot = std::nullopt) {
    static_assert(
        std::is_same_v<T, planning_service_client::SystemTimedTrajectory>
            || std::is_same_v<T, Eigen::VectorXd>
            || std::is_same_v<T, FailedIK>,
        "DracoVisualizer: Add: type must be either SystemTimedTrajectory or "
        "Eigen::VectorXd or a FailedIK");
    {
      std::lock_guard<std::mutex> lock(mutex_);
      // if the queue is too big, evict the oldest element
      while (incoming_queue_.size() >= max_queue_size_) {
        logging::log()->debug(
            "DracoVisualizer: Add: queue is too big size {}; evicting oldest "
            "element",
            incoming_queue_.size());
        incoming_queue_.pop_front();
      }
      Visualizable visualizable_variant = visualizable;
      std::string label_with_counter =
          fmt::format("{}: {}", queue_counter_++, label);
      incoming_queue_.emplace_back(
          label_with_counter,
          AnnotatedVisualizable {visualizable_variant, std::move(snapshot)});
      logging::log()->debug(
          "DracoVisualizer: Added {} and queue size is now {}",
          label_with_counter, incoming_queue_.size());
    }
    cv_.notify_one();
    logging::log()->debug(
        "DracoVisualizer: Add: Notified main thread of new visualizable "
        "object.");
  }

  int Port() const {
    return meshcat_port_;
  }

  bool IsRunning() const {
    return is_running_.load();
  }

  void AddSetFixedOffsetFramePoseInParentFrame(
      const std::string& frame_name,
      const drake::math::RigidTransformd& offset);

 private:
  void Main(const std::string& xml_file,
            const drake::multibody::parsing::ModelDirectives& dmd,
            const motion::RobotMeshcatParams& robot_meshcat_params,
            const motion::ConstraintsAdapter& constraints_adapter,
            std::optional<std::map<std::string, Eigen::VectorXd>>
                default_configuration);

  const long unsigned int max_queue_size_;
  const double viz_delta_time_;
  const int meshcat_beat_interval_ms_;
  std::deque<std::pair<std::string, AnnotatedVisualizable>> incoming_queue_;
  std::queue<std::pair<std::string, drake::math::RigidTransformd>>
      set_pose_in_parent_frame_requests_;
  std::atomic<bool> stop_ {false};
  std::atomic<bool> is_running_ {false};
  // queue of objects that can be visualized
  std::thread main_thread_;
  std::mutex mutex_;
  std::condition_variable cv_;
  int queue_counter_ {0};
  int meshcat_port_ {-1};
};

}  // namespace visualizer
}  // namespace draco
