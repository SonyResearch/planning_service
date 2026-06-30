
#include <drake/geometry/optimization/geodesic_convexity.h>
#include <drake/geometry/optimization/vpolytope.h>
#include <drake/geometry/shape_specification.h>
#include <drake/solvers/binding.h>
#include <drake/solvers/solve.h>
#include <gtest/gtest.h>

#include <chrono>
#include <set>

#include "planning_service/motion/iris/iris_builder.h"
#include "planning_service/motion/planning/thunder_planner.h"

namespace motion {
namespace planning {

using conf_edge_t = std::pair<Eigen::VectorXd, Eigen::VectorXd>;

std::vector<conf_edge_t> MakeEdgesFromConfSequence(
    const std::vector<Eigen::VectorXd>& conf_sequence) {
  std::vector<conf_edge_t> edges;
  for (size_t i = 0; i < conf_sequence.size() - 1; ++i) {
    edges.push_back({conf_sequence[i], conf_sequence[i + 1]});
  }
  return edges;
}

/** Test the Iris generation from data structures methods */
class TestGenerationFromRoadmap : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
    const std::string meshcat_params_file {
        "planning_service/test_data/meshcat.yaml"};
    const auto robot_meshcat_params {
        drake::yaml::LoadYamlFile<motion::RobotMeshcatParams>(
            meshcat_params_file)};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    robot_model_ = std::make_unique<motion::RobotModel>(xml_file, dmd);

    ConstraintsAdapter constraints_adapter_cost;
    constraints_adapter_cost.collision_checker = CollisionCheckerAdapter {};
    const double zero_penalty {50.0};
    const double gamma {0.5};
    const double influence {0.2};
    MinimumValuePenaltyParams params {
        .m = zero_penalty, .gamma = gamma, .x0 = influence};
    constraints_adapter_cost.collision_checker.value()
        .minimum_value_penalty_params = params;
    robot_constraints_cost_ = std::make_unique<RobotConstraints>(
        robot_model(), constraints_adapter_cost, 1);
  }

  const motion::RobotModel& robot_model() {
    return *robot_model_;
  }

  const std::vector<conf_edge_t>& roadmap_edges() {
    return roadmap_edges_;
  }

  const std::vector<Eigen::VectorXd>& nodes() {
    return nodes_;
  }

  // Getter to robot constraints
  const RobotConstraints& robot_constraints_cost() {
    return *robot_constraints_cost_;
  }

  std::unique_ptr<iris::IrisBuilder> CreateIrisBuilder(
      const std::string& path) {
    iris::IrisBuilderOptions iris_builder_options;
    std::string filename = "temp";
    return std::make_unique<iris::IrisBuilder>(*robot_constraints_cost_,
                                               iris_builder_options, path);
  }

 private:
  std::vector<conf_edge_t> CreateRoadmapEdges() {
    std::vector<conf_edge_t> edges;
    Eigen::VectorXd conf_1(2);
    conf_1 << 0.0, -1.0;
    Eigen::VectorXd conf_2(2);
    conf_2 << 0.0, 0.0;
    edges.push_back({conf_1, conf_2});

    Eigen::VectorXd conf_3(2);
    conf_3 << 1.5, 0.0;
    edges.push_back({conf_2, conf_3});
    edges.push_back({conf_1, conf_3});

    Eigen::VectorXd conf_4(2);
    conf_4 << -1.5, 0.0;
    edges.push_back({conf_1, conf_4});
    edges.push_back({conf_2, conf_4});

    Eigen::VectorXd conf_5(2);
    conf_5 << 1.5, 1.5;
    edges.push_back({conf_3, conf_5});

    Eigen::VectorXd conf_6(2);
    conf_6 << -1.5, 1.5;
    edges.push_back({conf_4, conf_6});
    edges.push_back({conf_5, conf_6});

    return edges;
  }

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

  std::vector<Eigen::VectorXd> nodes_ {CreateNodes()};
  std::vector<conf_edge_t> roadmap_edges_ {CreateRoadmapEdges()};
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_cost_;
};

TEST_F(TestGenerationFromRoadmap, IsEdgeInsideRegion) {
  const std::string& iris_regions_path {
      "planning_service/test_data/2d_prismatic_robot/"
      "IsEdgeInsideRegion_regions.yaml"};
  const auto& iris_builder {CreateIrisBuilder(iris_regions_path)};
  Eigen::Vector2d conf_1;
  conf_1 << -0.5, 0.0;
  Eigen::Vector2d conf_2;
  conf_2 << 0.5, 0.0;
  conf_edge_t edge {conf_1, conf_2};

  auto box_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(-1, -0.5), Eigen::Vector2d(1, 0.5));

  auto box_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(-0.2, -0.2), Eigen::Vector2d(0.2, 0.2));

  iris_builder->AddRegion(box_1, "box_1");
  iris_builder->AddRegion(box_2, "box_2");

  const auto& regions {iris_builder->adapter().regions_vec()};

  const auto& inspector = iris_builder->inspector();

  EXPECT_TRUE(inspector.IsEdgeInsideRegion(edge, regions[0]))
      << "The edge should be covered by this iris region (box_1)";

  EXPECT_FALSE(inspector.IsEdgeInsideRegion(edge, regions[1]))
      << "The edge should not be covered by this iris region (box_2)";

  EXPECT_TRUE(inspector.IsEdgeCoveredByAnyIrisRegion(edge, regions))
      << "The edge should be covered by one of the iris regions";
}

TEST_F(TestGenerationFromRoadmap, GenerateFromEdge) {
  const std::string iris_regions_path {
      "planning_service/test_data/2d_prismatic_robot/"
      "GenerateFromEdge_regions.yaml"};
  const auto& iris_builder {CreateIrisBuilder(iris_regions_path)};

  Eigen::Vector2d valid_start;
  Eigen::Vector2d valid_goal;
  valid_start << 1.3, 0.4;
  valid_goal << -1.3, 0.4;
  const std::pair<Eigen::VectorXd, Eigen::VectorXd> valid_edge {valid_start,
                                                                valid_goal};
  EXPECT_TRUE(robot_constraints_cost().CheckSatisfiedEdge(valid_start,
                                                          valid_goal, 0.01));

  const auto polytope_opt {iris_builder->CalcIrisRegionFromEdge(valid_edge)};

  EXPECT_TRUE(polytope_opt.has_value());

  // Check that both start and goal are in the generated polytope
  const auto polytope {polytope_opt.value()};
  const auto q_start_eigen {valid_edge.first};
  const auto q_goal_eigen {valid_edge.second};
  const auto& inspector = iris_builder->inspector();
  const auto start_eval {
      inspector.EvalConfigAgainstRegion(q_start_eigen, polytope)};
  const auto goal_eval {
      inspector.EvalConfigAgainstRegion(q_goal_eigen, polytope)};

  // If both start and goal are inside, then the whole edge is also inside the
  // polytope. This is because the polytope is convex
  EXPECT_TRUE(start_eval.inside) << "Start is not inside the polytope";
  EXPECT_TRUE(goal_eval.inside) << "Goal is not inside the polytope";
}

TEST_F(TestGenerationFromRoadmap, GenerateFromPath) {
  const std::string& iris_regions_path {
      "planning_service/test_data/2d_prismatic_robot/"
      "GenerateFromPath_regions.yaml"};
  const auto& iris_builder {CreateIrisBuilder(iris_regions_path)};

  std::vector<Eigen::VectorXd> conf_sequence;
  conf_sequence.push_back(Eigen::Vector2d(0.5, 0.2));
  conf_sequence.push_back(Eigen::Vector2d(-1.5, 0.2));
  conf_sequence.push_back(Eigen::Vector2d(-1.5, 1.6));
  conf_sequence.push_back(Eigen::Vector2d(0.5, 1.6));

  const auto edge_sequence {MakeEdgesFromConfSequence(conf_sequence)};

  iris_builder->BuildFromPath(conf_sequence);
  const auto& inspector = iris_builder->inspector();
  const auto& iris_regions {iris_builder->adapter().regions_vec()};
  for (const auto& edge : edge_sequence) {
    EXPECT_TRUE(robot_constraints_cost().CheckSatisfiedEdge(edge.first,
                                                            edge.second, 0.01));
    EXPECT_TRUE(inspector.IsEdgeCoveredByAnyIrisRegion(edge, iris_regions));
  }
  logging::log()->info("# of vertices: {}", conf_sequence.size());
  logging::log()->info("# of edges: {}", edge_sequence.size());
  logging::log()->info("# of iris regions: {}", iris_regions.size());
}

TEST_F(TestGenerationFromRoadmap, GenerateFromRoadmap) {
  const std::string& iris_regions_path {
      "planning_service/test_data/2d_prismatic_robot/"
      "GenerateFromRoadmap_regions.yaml"};

  const auto& iris_builder {CreateIrisBuilder(iris_regions_path)};

  iris_builder->BuildFromEdges(roadmap_edges());
  const auto& inspector = iris_builder->inspector();
  const auto& regions {iris_builder->adapter().regions_vec()};
  for (const auto& edge : roadmap_edges()) {
    EXPECT_TRUE(robot_constraints_cost().CheckSatisfiedEdge(edge.first,
                                                            edge.second, 0.01));
    EXPECT_TRUE(inspector.IsEdgeCoveredByAnyIrisRegion(edge, regions));
  }
}

TEST_F(TestGenerationFromRoadmap, GenerateFromClique) {
  // get the maximal clique containing edge (1,2) and generate an IRIS region
  // from it
  const std::string& iris_regions_path {
      "planning_service/test_data/2d_prismatic_robot/"
      "GenerateFromClique_regions.yaml"};
  iris::IrisBuilderOptions iris_builder_options;
  iris_builder_options.use_generation_from_cliques = true;
  std::string filename = "temp";
  std::vector<Eigen::VectorXd> clique_points {nodes()};
  auto iris_builder = std::make_unique<iris::IrisBuilder>(
      robot_constraints_cost(), iris_builder_options, iris_regions_path,
      clique_points);
  const auto first_node {clique_points[0]};
  auto clique_polytope_zo_opt = iris_builder->CalcIrisRegionFromConfig(
      first_node, iris::IrisBuilder::IrisMethod::kIrisZo);
  EXPECT_TRUE(clique_polytope_zo_opt.has_value());
  auto clique_polytope_np2_opt = iris_builder->CalcIrisRegionFromConfig(
      first_node, iris::IrisBuilder::IrisMethod::kIrisNp2);
  EXPECT_TRUE(clique_polytope_np2_opt.has_value());
  auto clique_polytope_np_opt = iris_builder->CalcIrisRegionFromConfig(
      first_node, iris::IrisBuilder::IrisMethod::kIrisNp);
  EXPECT_TRUE(clique_polytope_np_opt.has_value());
}

}  // namespace planning
}  // namespace motion
