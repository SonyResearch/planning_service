
#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/motion/planning/ompl/informed_rrt_star.h"
#include "planning_service/motion/planning/ompl/thunder_planner.h"
#include "planning_service/motion/planning/ompl/validity_checker.h"
#include "planning_service/motion/splining/cubic_spliner.h"

namespace motion {
namespace planning {
namespace ompl {
using robot_conf_t = Eigen::VectorXd;
using robot_conf_vec_t = std::vector<robot_conf_t>;  // "trajectory"
using system_conf_t = std::map<std::string, robot_conf_t>;

Eigen::VectorXd v_to_e(std::vector<double> v) {
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}

std::vector<double> e_to_v(Eigen::VectorXd e) {
  std::vector<double> v;
  v.resize(e.size());
  Eigen::VectorXd::Map(&v[0], e.size()) = e;
  return v;
}

std::vector<Eigen::VectorXd> MatrixToEigenVectors(
    const Eigen::MatrixXd& matrix) {
  std::vector<Eigen::VectorXd> vector;
  for (size_t i {}; i < static_cast<size_t>(matrix.cols()); i++) {
    vector.push_back(matrix.col(i));
  }
  return vector;
}

Eigen::MatrixXd EigenVectorsToMatrix(
    const std::vector<Eigen::VectorXd>& vector) {
  Eigen::MatrixXd matrix(vector[0].rows(), vector.size());
  for (size_t i {}; i < vector.size(); i++) {
    matrix.col(i) = vector[i];
  }
  return matrix;
}

double CalculatePathLength(std::vector<Eigen::VectorXd> q_vec) {
  double path_length = 0;
  for (size_t i = 0; i < q_vec.size() - 1; i++) {
    path_length += (q_vec[i] - q_vec[i + 1]).norm();
  }
  return path_length;
}

/** Test the roadmap recall on a simple example with prismatic joints and only a
 * few vertices and edges. This allows us to have a nice equivalence between
 * joint space and workspace, and hence test and visualize the recall behavior
 * easily.*/
class TestOmplCost : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    robot_model_ = std::make_unique<motion::RobotModel>(xml_file, dmd);
  }

  const motion::RobotModel& robot_model() {
    return *robot_model_;
  }

  const std::vector<Eigen::VectorXd>& nodes() {
    return nodes_;
  }

  ConstraintsAdapter constraints_adapter_cost() {
    return constraints_adapter_cost_;
  }

  ConstraintsAdapter constraints_adapter_default() {
    return constraints_adapter_default_;
  }

 private:
  std::vector<Eigen::VectorXd> CreateNodes() {
    // path 1->2 should be more expensive than 1->3->2
    Eigen::VectorXd conf_1(2);
    conf_1 << -1.2, 0.0;
    Eigen::VectorXd conf_2(2);
    conf_2 << -1.2, 1.5;
    Eigen::VectorXd conf_3(2);
    conf_3 << -1.6, 0.75;

    return {conf_1, conf_2, conf_3};
  }

  ConstraintsAdapter CreateCostConstraintsAdapter() {
    ConstraintsAdapter constraints_adapter_cost;
    constraints_adapter_cost.collision_checker = CollisionCheckerAdapter {};
    const double zero_penalty {50.0};
    const double gamma {0.5};
    const double influence {0.2};
    motion::planning::MinimumValuePenaltyParams params {
        .m = zero_penalty, .gamma = gamma, .x0 = influence};
    constraints_adapter_cost.collision_checker.value()
        .minimum_value_penalty_params = params;
    return constraints_adapter_cost;
  }

  ConstraintsAdapter CreateDefaultConstraintsAdapter() {
    ConstraintsAdapter constraints_adapter_default;
    constraints_adapter_default.collision_checker = CollisionCheckerAdapter {};
    return constraints_adapter_default;
  }

  std::vector<Eigen::VectorXd> nodes_ {CreateNodes()};
  std::unique_ptr<motion::RobotModel> robot_model_;
  ConstraintsAdapter constraints_adapter_cost_ {CreateCostConstraintsAdapter()};
  ConstraintsAdapter constraints_adapter_default_ {
      CreateDefaultConstraintsAdapter()};
};

TEST_F(TestOmplCost, TestCustomRoadmapLength) {
  ThunderParameters thunder_params;
  thunder_params.max_scratch_planning_time = 5;
  thunder_params.num_parallel_plans = 0;
  thunder_params.db_file_path =
      "planning_service/test_data/2d_prismatic_3_nodes.dat";
  thunder_params.use_cost_in_roadmap = true;
  thunder_params.add_edge_with_cost = true;
  thunder_params.set_compute_solution_cost = true;
  thunder_params.roadmap_granularity_base = 0.05;
  thunder_params.sparse_delta_fraction_base = 1.0;

  ThunderPlanner thunder_planner_default(
      robot_model(), constraints_adapter_default(), thunder_params);

  const auto timed_ptc {ob::timedPlannerTerminationCondition(5)};
  for (const auto& node : nodes()) {
    auto* ompl_state {
        thunder_planner_default.space_information()->allocState()};
    thunder_planner_default.space_information()->getStateSpace()->copyFromReals(
        ompl_state, e_to_v(node));
    thunder_planner_default.experience_database()
        ->getSPARSdb()
        ->addStateToRoadmap(timed_ptc, ompl_state);
  }

  EXPECT_EQ(thunder_planner_default.experience_database()
                ->getSPARSdb()
                ->getNumVertices(),
            4);  // 1 additional node from Spars

  EXPECT_EQ(thunder_planner_default.experience_database()
                ->getSPARSdb()
                ->getNumEdges(),
            3);  // 3 edges from 1->2, 1->3, 2->3

  const auto solution_vec_default {
      thunder_planner_default.SolveRecallPlan(nodes()[0], nodes()[1])};
  EXPECT_TRUE(solution_vec_default.has_value()) << "No solution found!";
  EXPECT_EQ(solution_vec_default.value().size(),
            4);  // 2 with repeated start and goal
}

TEST_F(TestOmplCost, TestCustomRoadmapCost) {
  ThunderParameters thunder_params;
  thunder_params.max_scratch_planning_time = 5;
  thunder_params.num_parallel_plans = 0;
  thunder_params.db_file_path =
      "planning_service/test_data/2d_prismatic_3_nodes.dat";
  thunder_params.use_cost_in_roadmap = true;
  thunder_params.add_edge_with_cost = true;
  thunder_params.set_compute_solution_cost = true;
  thunder_params.roadmap_granularity_base = 0.05;
  thunder_params.sparse_delta_fraction_base = 1.0;

  // create a constraints adapter with custom cost

  ThunderPlanner thunder_planner_cost(robot_model(), constraints_adapter_cost(),
                                      thunder_params);

  const auto timed_ptc {ob::timedPlannerTerminationCondition(5)};
  for (const auto& node : nodes()) {
    auto* ompl_state {thunder_planner_cost.space_information()->allocState()};
    thunder_planner_cost.space_information()->getStateSpace()->copyFromReals(
        ompl_state, e_to_v(node));
    thunder_planner_cost.experience_database()->getSPARSdb()->addStateToRoadmap(
        timed_ptc, ompl_state);
  }

  EXPECT_EQ(thunder_planner_cost.experience_database()
                ->getSPARSdb()
                ->getNumVertices(),
            4);  // 1 additional node from Spars

  EXPECT_EQ(
      thunder_planner_cost.experience_database()->getSPARSdb()->getNumEdges(),
      3);  // 3 edges from 1->2, 1->3, 2->3

  const auto solution_vec_cost {
      thunder_planner_cost.SolveRecallPlan(nodes()[0], nodes()[1])};
  EXPECT_TRUE(solution_vec_cost.has_value()) << "No solution found!";
  // log the configurations from the solution
  for (const auto& conf : solution_vec_cost.value()) {
    logging::log()->info("Solution conf: {}", conf.transpose());
  }
  EXPECT_EQ(solution_vec_cost.value().size(),
            5);  // 3 with repeated start and goal
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
