#include <gtest/gtest.h>

#include <mutex>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

struct Record {
  std::vector<int> traj_number;
  std::vector<Eigen::VectorXd> q;
  std::vector<Eigen::Vector3d> ee_xyz;
  std::vector<double> theta;
  std::vector<double> time;

  template <typename Archive>
  void Serialize(Archive* archive) {
    archive->Visit(DRAKE_NVP(traj_number));
    archive->Visit(DRAKE_NVP(q));
    archive->Visit(DRAKE_NVP(ee_xyz));
    archive->Visit(DRAKE_NVP(theta));
    archive->Visit(DRAKE_NVP(time));
  }
};

struct Options {
  double run_time = 10.0;     // Total time to run the circle trajectory
  int max_num_updates = 100;  // Number of updates to perform on the trajectory
  double radius = 0.1;
  int num_waypoints = 10;  // Number of waypoints to generate
  double theta_step = M_PI / 20;
  bool run_toppra = false;  // Whether to run TOPPRA or not
  std::optional<double> uniform_timing =
      std::nullopt;   // If set, use uniform timing
  int sleep_ms = 15;  // Sleep time between updates, which would allow the
                      // trajectory to move on
  std::optional<double> percentage_trajectory_forward =
      std::nullopt;  // Percentage of the trajectory to move forward before
                     // updating. If not set, will use sleep_ms.
  double search_step_size =
      0.02;  // In seconds. It will search the waypoints trajectory with this
             // resolution to find a landing cubic spline from current state to
             // the new trajectory.
  double theta_lead = 0.0;

  template <typename Archive>
  void Serialize(Archive* archive) {
    archive->Visit(DRAKE_NVP(run_time));
    archive->Visit(DRAKE_NVP(max_num_updates));
    archive->Visit(DRAKE_NVP(radius));
    archive->Visit(DRAKE_NVP(num_waypoints));
    archive->Visit(DRAKE_NVP(theta_step));
    archive->Visit(DRAKE_NVP(run_toppra));
    archive->Visit(DRAKE_NVP(uniform_timing));
    archive->Visit(DRAKE_NVP(sleep_ms));
    archive->Visit(DRAKE_NVP(percentage_trajectory_forward));
    archive->Visit(DRAKE_NVP(search_step_size));
    archive->Visit(DRAKE_NVP(theta_lead));
  }
};

class CircleRunner {
 public:
  CircleRunner(const std::unique_ptr<DracoPlanner>& planner,
               const drake::math::RigidTransformd& circle_center_pose)
      : planner_(planner),
        circle_center_pose_(circle_center_pose),
        frame_A_(&planner_->robot_model().GetScopedFrameByName("world")),
        frame_B_(&planner_->robot_model().GetScopedFrameByName(
            "franka_tool_location")) {}

  void UpdateCurrentTrajectory(
      const planning_service_client::SystemTimedTrajectory& sys_traj) {
    logging::log()->critical(
        "Update num {} happened to trajectory with start_time = {}, end_time = "
        "{}",
        traj_number_ + 1, sys_traj.at("franka").start_time(),
        sys_traj.at("franka").end_time());
    // Update the current trajectory
    current_sys_traj_ = sys_traj;
    traj_number_++;
  }

  void PublishCurrentConf() {
    while (!stop_.load()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      double t_now = time_now();
      if (current_sys_traj_.size() == 0) {
        logging::log()->warn(
            "CircleRunner:PublishCurrentConf: No current "
            "trajectory to publish.");
        continue;
      }
      auto traj = current_sys_traj_.at("franka");
      if (t_now < traj.start_time() || t_now > traj.end_time()) {
        logging::log()->warn(
            "CircleRunner:PublishCurrentConf: Current time {} is out of "
            "trajectory bounds [start: {}, end: {}].",
            t_now, traj.start_time(), traj.end_time());
        continue;
      }
      q_now_ = traj.Value(t_now);
      pose_ = planner_->robot_model().CalcRelativeTransform(q_now_, *frame_A_,
                                                            *frame_B_);
      planner_->mutable_draco_visualizer().Add(q_now_);
      // Save the record
      record_.traj_number.push_back(traj_number_);
      record_.q.push_back(q_now_);
      record_.ee_xyz.push_back(pose_.translation());
      record_.theta.push_back(CalcTheta());
      record_.time.push_back(t_now);
    }
  }

  void StartClock() {
    clock_start_ = std::chrono::system_clock::now();
  }

  double time_now() const {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::system_clock::now() - clock_start_)
               .count()
           / 1000.0;
  }

  Eigen::VectorXd q_now() const {
    return q_now_;
  }

  drake::math::RigidTransformd CalcPose() const {
    return planner_->robot_model().CalcRelativeTransform(q_now_, *frame_A_,
                                                         *frame_B_);
  }

  double CalcTheta() const {
    auto delta_now = pose_.translation() - circle_center_pose_.translation();
    return std::atan2(delta_now(1), delta_now(0));
  }

  planning_service_client::SystemTimedTrajectory current_sys_traj() const {
    return current_sys_traj_;
  }

  const DracoPlanner& planner() const {
    return *planner_;
  }

  int traj_number() const {
    return traj_number_;
  }

  void StopPublishing() {
    stop_.store(true);
    logging::log()->info(
        "CircleRunner:StopPublishing: Stopping the publishing thread.");
  }

  void SaveRecordToFile(const std::string& file_path) const {
    drake::yaml::SaveYamlFile(file_path, record_);
    logging::log()->info("CircleRunner:SaveRecordToFile: Record saved to {}",
                         file_path);
  }

  double CalcAngularVelocity() const {
    // Use a moving average of the last 5 theta values to calculate the velocity
    int num_values = 5;
    if (std::ssize(record_.theta) < num_values) {
      return 0;
    }
    double theta_dot_sum = 0.0;
    for (int i = std::ssize(record_.theta) - num_values;
         i < std::ssize(record_.theta); ++i) {
      double theta_diff = record_.theta[i] - record_.theta[i - 1];
      double time_diff = record_.time[i] - record_.time[i - 1];
      double theta_dot = theta_diff / time_diff;
      theta_dot_sum += theta_dot;
    }
    return theta_dot_sum / num_values;
  }

 private:
  const std::unique_ptr<DracoPlanner>& planner_;
  const drake::math::RigidTransformd& circle_center_pose_;
  const drake::multibody::Frame<double>* frame_A_;
  const drake::multibody::Frame<double>* frame_B_;
  std::chrono::time_point<std::chrono::system_clock> clock_start_ =
      std::chrono::system_clock::now();
  Eigen::VectorXd q_now_ = Eigen::VectorXd::Zero(7);
  drake::math::RigidTransformd pose_ = drake::math::RigidTransformd::Identity();
  planning_service_client::SystemTimedTrajectory current_sys_traj_;
  std::atomic<bool> stop_ {false};
  int traj_number_ = 0;
  Record record_;
};

int do_main() {
  auto options = drake::yaml::LoadYamlFile<Options>(
      "planning_service/test_data/run_circle_options.yaml");
  // When wayposes are given.
  planning_service_client::planner::UpdateTrajTowardWaypointsProblem def;
  auto adapter = test::Franka();
  adapter.options.visualizer_options.mode = VisualizerMode::kDraco;
  const auto planner = std::make_unique<DracoPlanner>(adapter);
  // Let's create a simple initial trajectory.
  Eigen::VectorXd q(7);
  q << -1.0, 0.3, 0.2, -2.0, -0.5, 1.8, -0.4;
  // Let's solve the start to goal problem
  planning_service_client::SystemConf sysconf;
  sysconf["franka"] = q;
  auto first_traj =
      planning_service_client::ConstantSystemTimedTrajectory(sysconf);
  // Create a linear trajectory in configuration space
  auto q_circle_center = first_traj["franka"].Value(0.0);
  DRAKE_DEMAND(q_circle_center.isApprox(q));
  std::string frame_A_name = "world";
  std::string frame_B_name = "franka_tool_location";
  const auto& frame_A =
      planner->robot_model().GetScopedFrameByName(frame_A_name);
  const auto& frame_B =
      planner->robot_model().GetScopedFrameByName(frame_B_name);
  std::map<std::string, Eigen::VectorXd> sys_conf;
  const auto pose_circle_center = planner->robot_model().CalcRelativeTransform(
      q_circle_center, frame_A, frame_B);
  // Now let's update the trajectory toward the circle center with some
  // translation
  // Parameters for the circle
  double radius = options.radius;
  double theta_step = options.theta_step;
  // Now we get to circle runner
  CircleRunner circle_runner(planner, pose_circle_center);
  circle_runner.StartClock();
  circle_runner.UpdateCurrentTrajectory(first_traj);
  // Publish runs in a separate thread
  std::thread publish_thread(&CircleRunner::PublishCurrentConf, &circle_runner);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  while (circle_runner.time_now() < options.run_time
         && circle_runner.traj_number() < options.max_num_updates) {
    if (options.percentage_trajectory_forward.has_value()) {
      double percentage = options.percentage_trajectory_forward.value();
      double time_to_finish =
          circle_runner.current_sys_traj().at("franka").end_time()
          - circle_runner.time_now();
      double time_to_advance = time_to_finish * percentage;
      std::this_thread::sleep_for(
          std::chrono::milliseconds(int(time_to_advance * 500)));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(options.sleep_ms));
    }
    // Select waypoints
    std::vector<planning_service_client::FrameRelativePose> wayposes;
    // Find theta of now
    double theta_now = circle_runner.CalcTheta();
    double angular_velocity = circle_runner.CalcAngularVelocity();
    logging::log()->info(
        "CircleRunner:do_main: Current theta: {}, angular velocity: {}",
        theta_now, angular_velocity);
    for (int j = 0; j < options.num_waypoints; ++j) {
      double theta = theta_now + j * theta_step + options.theta_lead;
      logging::log()->info("Theta lead at {}: {}", j, theta - theta_now);
      Eigen::Vector3d delta_translation(radius * std::cos(theta),
                                        radius * std::sin(theta), 0);
      Eigen::Vector3d translation =
          pose_circle_center.translation() + delta_translation;
      auto rotation = pose_circle_center.rotation();
      auto X_AB = drake::math::RigidTransformd(rotation, translation);
      wayposes.push_back(planning_service_client::FrameRelativePose(
          frame_A_name, frame_B_name, X_AB.translation(),
          X_AB.rotation().ToQuaternion()));
    }
    double time_now = circle_runner.time_now();
    auto current_sys_traj = circle_runner.current_sys_traj();
    std::vector<double> durations {};
    if (options.uniform_timing.has_value()) {
      durations = std::vector<double>(wayposes.size() - 1,
                                      options.uniform_timing.value());
    }
    auto def =
        planning_service_client::planner::UpdateTrajTowardWaypointsProblem(
            current_sys_traj, wayposes, time_now, durations,
            options.search_step_size, options.run_toppra);
    // Solve the problem
    auto motion_plan_result = circle_runner.planner().SolvePlan(def);
    logging::log()->info("Message: {}", motion_plan_result.message());
    if (!motion_plan_result.is_success()) {
      logging::log()->error(
          "CircleRunner:do_main: Failed to solve the motion "
          "plan problem: {}",
          motion_plan_result.message());
    } else {
      current_sys_traj = motion_plan_result.system_timed_trajectory();
      circle_runner.UpdateCurrentTrajectory(current_sys_traj);
    }
  }
  // Kill the publish thread
  circle_runner.StopPublishing();
  publish_thread.join();
  // Save the record to file
  circle_runner.SaveRecordToFile(
      "planning_service/test_data/circle_record.yaml");
  // keep the session open
  // while (true) {
  //   std::this_thread::sleep_for(std::chrono::milliseconds(100));
  // }
  return 1;
}

}  // namespace planner
}  // namespace draco

int main() {
  return draco::planner::do_main();
}
