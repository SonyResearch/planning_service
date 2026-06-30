#include <gtest/gtest.h>

#include "planning_service_client/conf.h"
#include "planning_service_client/planner/start_to_goal_plan.h"
#include "planning_service_client/planner/update_traj_toward_waypoints.h"
#include "planning_service_client/trajectories.h"

namespace planning_service_client {
namespace test {

// Some correct q(s) and s(t) trajectories
TimedTrajectory Traj1() {
  std::vector<double> breaks {0.0, 1.0, 6.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(3, 7));
  auto path = PiecewisePolynomial(coefficients_vec, breaks);
  // A time scaling trajectory
  std::vector<double> time_breaks {0.0, 2.0};
  std::vector<Eigen::MatrixXd> s_coefficients_vec;
  s_coefficients_vec.push_back(Eigen::Vector3d(0.0, 1.0, 1.0).transpose());
  auto time_scaling = PiecewisePolynomial(s_coefficients_vec, time_breaks);
  return TimedTrajectory(path, time_scaling);
}

TimedTrajectory Traj2() {
  std::vector<double> breaks {0.0, 1.0, 10.0};
  std::vector<Eigen::MatrixXd> coefficients_vec;
  coefficients_vec.push_back(Eigen::MatrixXd::Zero(3, 1));
  coefficients_vec.push_back(Eigen::MatrixXd::Ones(3, 7));
  auto path = PiecewisePolynomial(coefficients_vec, breaks);
  // A time scaling trajectory
  std::vector<double> time_breaks {0.0, 2.0};
  std::vector<Eigen::MatrixXd> s_coefficients_vec;
  s_coefficients_vec.push_back(Eigen::Vector3d(0.0, 1.0, 2.0).transpose());
  auto time_scaling = PiecewisePolynomial(s_coefficients_vec, time_breaks);
  return TimedTrajectory(path, time_scaling);
}

Conf RandomConf(int dim = 1) {
  return Conf(Eigen::VectorXd::Random(dim));
}

SystemConf RandomSystemConf(int dim = 1) {
  SystemConf system_conf;
  system_conf["a"] = RandomConf(dim);
  return system_conf;
}

bool NearEqualSystemConf(const SystemConf& a, const SystemConf& b,
                         double tol = 1e-6) {
  if (a.size() != b.size()) return false;
  for (const auto& [key, conf_a] : a) {
    if (!b.has_key(key)) return false;
    const auto& conf_b = b.at(key);
    if (conf_a.q().isApprox(conf_b.q(), tol)) {
      continue;
    } else {
      return false;
    }
  }
  return true;
}

planner::StartToGoalProblem RandomStartToGoalProblem() {
  std::vector<FrameRelativePose> poses_end;
  poses_end.push_back(FrameRelativePose("a", "b", Eigen::Vector3d::Random(),
                                        Eigen::Quaterniond::UnitRandom()));
  auto start_anchor = planner::Anchor(test::RandomSystemConf(2),
                                      std::vector<FrameRelativePose>());
  auto goal_anchor = planner::Anchor(test::RandomSystemConf(2), poses_end);
  bool replace_invalid_goal = true;
  return planner::StartToGoalProblem(start_anchor, goal_anchor,
                                     replace_invalid_goal);
}

planner::UpdateTrajTowardWaypointsProblem
RandomUpdateTrajTowardWaypointsProblem() {
  SystemTimedTrajectory sys_timed_traj;
  sys_timed_traj["a"] = test::Traj1();
  std::vector<SystemConf> waypoints;
  waypoints.push_back(test::RandomSystemConf(2));
  return planner::UpdateTrajTowardWaypointsProblem(sys_timed_traj, waypoints,
                                                   0.1);
}

}  // namespace test
}  // namespace planning_service_client
