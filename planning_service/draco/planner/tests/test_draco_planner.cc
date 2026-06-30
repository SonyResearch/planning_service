#include <gtest/gtest.h>

#include "planning_service/draco/client_conversions.h"
#include "planning_service/draco/planner/draco_planner.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {
namespace planner {

using motion::system_conf_t;

TEST(TestDracoPlanner, CombineSequentialSystemTimedTrajectories) {
  const auto planner = DracoPlanner(test::DualPandas());
  Eigen::VectorXd conf_1(14);
  conf_1 << -0.44111, 0.500755, -0.22664, -1.82248, -1.63484, 2.00563,
      -0.027943, 2.07683, 0.664395, 1.0827, -2.46029, 1.83885, 1.57929, 0.1637;
  Eigen::VectorXd conf_2(14);
  conf_2 << -0.441242, 0.48272, -0.227708, -1.8237, -1.6324, 2.00664,
      -0.0452711, 2.05392, 0.637206, 1.11224, -2.46728, 1.85754, 1.57788,
      0.181677;
  Eigen::VectorXd conf_3(14);
  conf_3 << -0.440533, 0.464783, -0.227737, -1.82565, -1.62703, 2.00606,
      -0.0628275, 2.02439, 0.604434, 1.14967, -2.47798, 1.88258, 1.57462,
      0.202441;
  Eigen::VectorXd conf_4(14);
  conf_4 << -0.440819, 0.448484, -0.228333, -1.82524, -1.62397, 2.00773,
      -0.0817298, 1.99749, 0.575301, 1.18474, -2.48788, 1.90604, 1.57277,
      0.220157;
  Eigen::VectorXd conf_5(14);
  conf_5 << -0.439494, 0.43107, -0.228201, -1.82561, -1.61941, -2e-06, -9e-05,
      1e-06, -9e-05, -9e-05, -9e-05, -9e-05, -9e-05, -9e-05;
  Eigen::VectorXd conf_6(14);
  conf_6 << -0.439019, 0.414557, -0.228886, -1.82437, -1.61471, 2.00614,
      -0.118414, 1.93613, 0.51864, 1.26065, -2.49786, 1.95028, 1.56661,
      0.264721;
  Eigen::VectorXd conf_7(14);
  conf_7 << -0.439629, 0.400945, -0.228968, -1.82073, -1.61016, 2.00616,
      -0.136884, 1.90409, 0.47223, 1.29808, -2.5074, 1.9821, 1.54899, 0.291149;
  Eigen::VectorXd conf_8(14);
  conf_8 << -0.43913, 0.384724, -0.22884, -1.81888, -1.60533, -2e-06, -9e-05,
      1e-06, -9e-05, -9e-05, -9e-05, -9e-05, -9e-05, -9e-05;
  // The first trajectory
  std::vector<Eigen::VectorXd> q_vec_1 {conf_1, conf_2, conf_3, conf_4, conf_5};
  // The second trajectory
  std::vector<Eigen::VectorXd> q_vec_2 {conf_5, conf_6, conf_7, conf_8};

  // Use drake cubic spline and uniform timing for the first trajectory
  Eigen::VectorXd times_1;
  times_1.resize(q_vec_1.size());
  double dt_1 = 1.0;  // uniform time step
  for (size_t i = 0; i < q_vec_1.size(); ++i) {
    times_1(i) = i * dt_1;
  }
  // Make matrices for q_vec_1 and q_vec_2
  Eigen::MatrixXd q_matrix_1(q_vec_1.front().size(), q_vec_1.size());
  for (size_t i = 0; i < q_vec_1.size(); ++i) {
    q_matrix_1.col(i) = q_vec_1[i];
  }
  Eigen::MatrixXd q_matrix_2(q_vec_2.front().size(), q_vec_2.size());
  for (size_t i = 0; i < q_vec_2.size(); ++i) {
    q_matrix_2.col(i) = q_vec_2[i];
  }
  // Make matrices for times_1 and times_2
  Eigen::MatrixXd times_matrix_1(1, times_1.size());
  for (size_t i = 0; i < static_cast<size_t>(times_1.size()); ++i) {
    times_matrix_1(0, i) = times_1(i);
  }
  // Use drake cubic spline and uniform timing for the second trajectory
  Eigen::VectorXd times_2;
  times_2.resize(q_vec_2.size());
  double dt_2 = 1.0;  // uniform time step
  for (size_t i = 0; i < q_vec_2.size(); ++i) {
    times_2(i) = i * dt_2;
  }
  Eigen::MatrixXd times_matrix_2(1, times_2.size());
  for (size_t i = 0; i < static_cast<size_t>(times_2.size()); ++i) {
    times_matrix_2(0, i) = times_2(i);
  }

  auto path_1 = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times_1, q_matrix_1);
  auto time_1 = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times_1, times_matrix_1);

  EXPECT_TRUE(path_1.value(0).isApprox(q_vec_1.front()))
      << "Start does not match the first configuration";
  EXPECT_TRUE(
      path_1.value(times_1(times_1.size() - 1)).isApprox(q_vec_1.back()))
      << "End does not match the last configuration";

  auto path_2 = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times_2, q_matrix_2);
  auto time_2 = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times_2, times_matrix_2);

  EXPECT_TRUE(path_2.value(0).isApprox(q_vec_2.front()))
      << "Start does not match the first configuration";
  EXPECT_TRUE(
      path_2.value(times_2(times_2.size() - 1)).isApprox(q_vec_2.back()))
      << "End does not match the last configuration";

  // Combine the two trajectories
  logging::log()->info("Combining the two trajectories");
  auto combined_trajectory =
      motion::splining::internal::CombineSequentialSystemTimedTrajectories(
          {std::make_pair(path_1, time_1), std::make_pair(path_2, time_2)});
  // Check that the combined trajectory is not empty
  EXPECT_FALSE(combined_trajectory.first.empty())
      << "Combined trajectory path is empty";
  EXPECT_FALSE(combined_trajectory.second.empty())
      << "Combined trajectory time is empty";
  // Check that the start matches the first configuration of the first
  // trajectory
  EXPECT_TRUE(combined_trajectory.first.value(0).isApprox(q_vec_1.front()))
      << "Start does not match the first configuration of the first trajectory";
  // Check that the end matches the last configuration of the second trajectory
  double combined_end_time =
      times_1(times_1.size() - 1) + times_2(times_2.size() - 1);
  EXPECT_TRUE(combined_trajectory.first.value(combined_end_time)
                  .isApprox(q_vec_2.back()))
      << "End does not match the last configuration of the second trajectory";
  // Check that the value of the combined trajectory at the time the first
  // trajectory ends is the same as the final configuration of the first
  // trajectory
  EXPECT_TRUE(combined_trajectory.first.value(times_1(times_1.size() - 1))
                  .isApprox(q_vec_1.back()))
      << "Value at the end of the first trajectory does not match";
  // Check that the value of the combined trajectory at the time the first
  // trajectory ends is the same as the first configuration of the second
  // trajectory
  EXPECT_TRUE(combined_trajectory.first.value(times_1(times_1.size() - 1))
                  .isApprox(q_vec_2.front()))
      << "Value at the end of the first trajectory does not match";
  // Make sure combining in the opposite order does not work as the continuity
  // check would fail (returns an empty trajectory)
  EXPECT_THROW(
      motion::splining::internal::CombineSequentialSystemTimedTrajectories(
          {std::make_pair(path_2, time_2), std::make_pair(path_1, time_1)}),
      std::runtime_error);  // Expect an exception to be thrown
}

TEST(TestDracoPlanner, DracoVisualizer) {
  auto adapter = test::DualWallflowers();
  auto planner_native = DracoPlanner(adapter);
  EXPECT_FALSE(planner_native.has_draco_visualizer());
  adapter.options.visualizer_options.mode = draco::VisualizerMode::kDraco;
  adapter.robot_meshcat_params = motion::RobotMeshcatParams();
  auto planner_draco = DracoPlanner(adapter);
  EXPECT_TRUE(planner_draco.has_draco_visualizer());
  planner_draco.mutable_draco_visualizer().Kill();
}

TEST(CheckSatisfiedTest, CollisionOptions) {
  const auto draco {DracoPlanner(test::Wallflower())};
  Eigen::Vector2d valid_q {Eigen::Vector2d(1.0, 0.4)};
  Eigen::Vector2d colliding_q {Eigen::Vector2d(0.0, 0.4)};
  psc::SystemConf valid;
  psc::SystemConf colliding;
  valid["robot"] = valid_q;
  colliding["robot"] = colliding_q;

  EXPECT_TRUE(draco.CheckSatisfied({valid}).satisfied());
  EXPECT_FALSE(draco.CheckSatisfied({colliding}).satisfied());
  psc::planner::CollisionOptions collision_options;
  // Filtering
  collision_options.filtered_pairs.emplace_back("ball", "main");
  EXPECT_TRUE(draco.CheckSatisfied({colliding}, collision_options).satisfied());
  collision_options.filtered_pairs.clear();
  // Collision at this configuration has a penetration of ~55 mm
  collision_options.paddings.emplace_back("ball", "main", -0.06);
  auto result_0 = draco.CheckSatisfied({colliding}, collision_options);
  EXPECT_TRUE(result_0.satisfied());
  EXPECT_TRUE(result_0.offending_resource_names().empty());
  EXPECT_TRUE(result_0.failed_constraint_strings().empty());
  collision_options.paddings.clear();
  // Reattempt with a shallower padding, which should fail
  collision_options.paddings.emplace_back("ball", "main", -0.01);
  auto result_1 = draco.CheckSatisfied({colliding}, collision_options);
  EXPECT_FALSE(result_1.satisfied());
  EXPECT_FALSE(result_1.offending_resource_names().empty());
  EXPECT_EQ(result_1.offending_resource_names().front(), "robot");
  EXPECT_FALSE(result_1.failed_constraint_strings().empty());
  // Finally, ensure that the collision is detected with no special options
  // provided
  EXPECT_FALSE(draco.CheckSatisfied({colliding}).satisfied());
}

TEST(TestDracoPlanner, PrivateMaybeFixArmsInPath) {
  const auto planner = DracoPlanner(test::DualWallflowers());
  // Make a fake trajectory that has left arm moving to the same place
  std::vector<double> times {0.0, 1.0, 2.0};
  std::vector<Eigen::MatrixXd> path_vec;
  // Flower 1 comes back to where it started, flower 2 goes to a different
  // config,
  path_vec.push_back(Eigen::Vector4d(1.0, 0.3, 0.0, 0.2));
  path_vec.push_back(Eigen::Vector4d(1.5, 0.4, 1.0, 0.3));
  path_vec.push_back(Eigen::Vector4d(1, 0.3, 2.0, 0.4));
  auto path = drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(times, path_vec);
  // Now fix the path to make flower 1 not move, it should be valid
  auto fixed_path_opt = planner.MaybeFixArmsInPath(path);
  EXPECT_TRUE(fixed_path_opt.has_value());
  const auto& fixed_path = fixed_path_opt.value();
  // The same timings, as it will not change
  EXPECT_EQ(fixed_path.get_number_of_segments(), 2);
  EXPECT_EQ(fixed_path.start_time(), 0.0);
  EXPECT_EQ(fixed_path.end_time(), 2.0);
  // Test fixed path's first 2 joints remain constant
  for (double t = 0.0; t <= 2.0; t += 0.5) {
    EXPECT_DOUBLE_EQ(fixed_path.value(t)(0), 1.0);
    EXPECT_DOUBLE_EQ(fixed_path.value(t)(1), 0.3);
  }
  // The fixed path is valid
  EXPECT_TRUE(planner.robot_constraints().CheckSatisfiedTrajectory(fixed_path));
}

class DracoTest : public ::testing::Test {
 protected:
  void SetUp() override {
    draco_adapter = CreateAdapter();
    EXPECT_NO_THROW(draco = std::make_unique<DracoPlanner>(draco_adapter));
  }
  /**
   * @brief Virtual. Update the Draco adapter with any desired attributes before
   * constructing the Draco instance.
   *
   */
  virtual DracoAdapter CreateAdapter() = 0;

  DracoAdapter draco_adapter;
  std::unique_ptr<DracoPlanner> draco;
};

class DracoWallflowerTest : public DracoTest {
 protected:
  DracoAdapter CreateAdapter() override {
    DracoAdapter adapter = test::Wallflower();
    adapter.context_dir = "planning_service/test_data/wallflower/";
    adapter.thunder_dat_file = adapter.context_dir / "thunder_prm.dat";
    return adapter;
  }
};

class DracoDualPandaTest : public DracoTest {
 protected:
  void SetUp() override {
    DracoAdapter adapter = CreateAdapter();
    // Construct a start and goal which moves the left arm only.
    q_right_start = Eigen::VectorXd::Zero(7);
    q_right_end = Eigen::VectorXd::Zero(7);
    q_right_start << -2, 0.5, 0.5, -2.5, -2.0, 3.0, 0.0;
    q_right_end = q_right_start;

    q_left_start = Eigen::VectorXd::Zero(7);
    q_left_end = Eigen::VectorXd::Zero(7);
    q_left_start << -2.5, 0, 0, -2, -2, 3, 0;
    q_left_end << -1.5, 0, 0, -2, -2, 3, 0;
    start["franka_right"] = q_right_start;
    start["franka_left"] = q_left_start;
    goal["franka_right"] = q_right_end;
    goal["franka_left"] = q_left_end;
  }

  DracoAdapter CreateAdapter() override {
    DracoAdapter adapter = test::DualPandas();
    adapter.context_dir = "planning_service/test_data/dual_pandas/";
    adapter.thunder_dat_file = adapter.context_dir / "thunder_prm.dat";
    return adapter;
  }

  Eigen::VectorXd q_right_start, q_left_start, q_right_end, q_left_end;
  system_conf_t start, goal;
  const std::string left_tool_frame {"franka_left::franka_tool_location"};
};

TEST_F(DracoWallflowerTest, Basics) {
  // We only have joint limit constraints
  EXPECT_EQ(draco->robot_constraints().get_non_collision_constraints().size(),
            1);
  const auto joint_limit_constraint =
      draco->robot_constraints().get_non_collision_constraints().front();
  // the string contains multibody_entity_name
  EXPECT_TRUE(
      joint_limit_constraint->get_description().find("multibody_entity_name:")
      != std::string::npos);
}

TEST_F(DracoWallflowerTest, FastEstimatePlan1) {
  // The case when GCC path is the shortest path.
  Eigen::VectorXd q_1 = Eigen::Vector2d {M_PI / 2 - 0.1, 0.4};  // inside r1
  Eigen::VectorXd q_2 =
      Eigen::Vector2d {-M_PI / 2 + 0.1, 0.4};  // A bit outside r3
  // The midpoint is not valid.
  EXPECT_FALSE(draco->robot_constraints().CheckSatisfied((q_1 + q_2) / 2, 0));
  auto result_path_opt = draco->FastEstimatePlan(q_1, q_2);
  EXPECT_TRUE(result_path_opt.has_value());
  double cost = result_path_opt->end_time();
  // The cost must be greater than the distance between q_1 and q_2
  EXPECT_GT(cost, (q_1 - q_2).norm());
  EXPECT_TRUE(result_path_opt->value(0).isApprox(q_1));
  EXPECT_TRUE(result_path_opt->value(cost).isApprox(q_2));
}

TEST_F(DracoWallflowerTest, FastEstimatePlan2) {
  // The case when straight path is the shortest path.
  Eigen::VectorXd q_1 = Eigen::Vector2d {0.1, 0.2};   // inside r1
  Eigen::VectorXd q_2 = Eigen::Vector2d {0.3, 0.35};  // inside r1
  auto result_opt = draco->FastEstimatePlan(q_1, q_2);
  EXPECT_TRUE(result_opt.has_value());
  // The cost must be equal to the max time for each joint
  auto vel_bound =
      draco->time_optimal_spliner().joint_dynamic_limits().velocity_bound;
  double min_time = 0;
  for (int i = 0; i < q_1.rows(); ++i) {
    double joint_time = std::abs(q_1(i) - q_2(i)) / vel_bound(i);
    min_time = std::max(min_time, joint_time);
  }
  EXPECT_NEAR(result_opt->end_time(), min_time, 1e-4);
}

TEST_F(DracoWallflowerTest, FastEstimatePlan3) {
  // The case when start and end are the same.
  Eigen::VectorXd q_1 = Eigen::Vector2d {0.1, 0.2};
  Eigen::VectorXd q_2 = q_1;
  auto result_opt = draco->FastEstimatePlan(q_1, q_2);
  EXPECT_TRUE(result_opt.has_value());
  EXPECT_NEAR(result_opt->start_time(), 0, 1e-4);
  EXPECT_NEAR(result_opt->end_time(), 0, 1e-4);
  EXPECT_TRUE(result_opt->value(0).isApprox(q_1));
}

}  // namespace planner
}  // namespace draco
