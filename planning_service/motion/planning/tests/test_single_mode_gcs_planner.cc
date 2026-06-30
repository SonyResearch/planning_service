#include <gtest/gtest.h>

#include "planning_service/common/logging.h"
#include "planning_service/motion/planning/single_mode_gcs_planner.h"

namespace motion {
namespace planning {

class TestGcsPlannerBase : public ::testing::Test {
 public:
  explicit TestGcsPlannerBase(bool use_lazy_edges, int num_intersection_samples)
      : use_lazy_edges_(use_lazy_edges),
        num_intersection_samples_(num_intersection_samples) {}

  void SetUp() {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/wallflower/dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
        std::make_pair("robot", 0)};  // only the first AA joint is continuous
    robot_model_ = std::make_unique<RobotModel>(xml_file, dmd, std::nullopt,
                                                continuous_revolute_joints);
    ConstraintsAdapter constraints_adapter;
    constraints_adapter.plan_name = "";
    CollisionCheckerAdapter collision_checker_adapter;
    constraints_adapter.collision_checker = collision_checker_adapter;
    robot_constraints_ = std::make_unique<RobotConstraints>(
        *robot_model_, constraints_adapter, 1);
    // set 1, q0: from 0 to 90 degrees
    auto set1 = drake::geometry::optimization::HPolyhedron::MakeBox(
        Eigen::Vector2d(0, 0), Eigen::Vector2d(M_PI / 2.0, 1));
    // set 2: q0: from 60 to 180 degrees
    auto set2 = drake::geometry::optimization::HPolyhedron::MakeBox(
        Eigen::Vector2d(-5.0 * M_PI / 3.0, 0.3), Eigen::Vector2d(-M_PI, 0.75));
    // set 3: q0: from -90 to 45 degrees
    auto set3 = drake::geometry::optimization::HPolyhedron::MakeBox(
        Eigen::Vector2d(3.0 * M_PI / 2, -0.1),
        Eigen::Vector2d(9.0 * M_PI / 4, 0.5));
    iris_regions_adapter_ = iris::IrisRegionsAdapter();
    auto hash = robot_constraints_->constraints_hash();
    auto crji =
        robot_constraints_->robot_model().continuous_revolute_joint_indices();
    iris_regions_adapter_.AddRegion(set1, "set1", hash, crji,
                                    num_intersection_samples_);
    iris_regions_adapter_.AddRegion(set2, "set2", hash, crji,
                                    num_intersection_samples_);
    iris_regions_adapter_.AddRegion(set3, "set3", hash, crji,
                                    num_intersection_samples_);
    Eigen::Vector2d joint_velocity_bounds = Eigen::Vector2d(1.0, 1.0);
    std::string gcs_options_file =
        "planning_service/test_data/gcs_options.yaml";
    auto gcs_options =
        drake::yaml::LoadYamlFile<GcsPlannerOptions>(gcs_options_file);
    gcs_options.lazy_gcc_edges = use_lazy_edges_;
    single_mode_gcs_planner_ = std::make_unique<SingleModeGcsPlanner>(
        *robot_constraints_, iris_regions_adapter_, joint_velocity_bounds,
        gcs_options);
  }

  const RobotModel& robot_model() const {
    return *robot_model_;
  }

  SingleModeGcsPlanner& single_mode_gcs_planner() {
    return *single_mode_gcs_planner_;
  }

  internal::GraphOfConfigs& graph_of_configs() {
    return *(single_mode_gcs_planner().graph_of_configs_);
  }

  const drake::geometry::optimization::GraphOfConvexSets& graph_of_convex_sets()
      const {
    return single_mode_gcs_planner_->gcs_traj_opt_->graph_of_convex_sets();
  }

  void AddStartTerminal(const Eigen::VectorXd& q) {
    single_mode_gcs_planner_->AddTerminal(
        q, SingleModeGcsPlanner::TerminalType::kStart);
  }

  void AddEndTerminal(const Eigen::VectorXd& q) {
    single_mode_gcs_planner_->AddTerminal(
        q, SingleModeGcsPlanner::TerminalType::kEnd);
  }

  std::unique_ptr<drake::geometry::optimization::ConvexSet> CreateConnectingSet(
      const Eigen::VectorXd& q1, const Eigen::VectorXd& q2,
      const double epsilon) {
    return single_mode_gcs_planner_->CreateConnectingSet(
        q1, q2, ConnectingSetType::kNarrowBox, epsilon);
  }

  std::vector<const drake::geometry::optimization::GraphOfConvexSets::Vertex*>
  CalcGcsVerticesPath(const Eigen::VectorXd& q1, const Eigen::VectorXd& q2) {
    SingleModeGcsPlanner::TerminalNode start_terminal =
        single_mode_gcs_planner_->AddTerminal(
            q1, SingleModeGcsPlanner::TerminalType::kStart);
    SingleModeGcsPlanner::TerminalNode end_terminal =
        single_mode_gcs_planner_->AddTerminal(
            q2, SingleModeGcsPlanner::TerminalType::kEnd);
    const auto* start_gcc = start_terminal.q_gcc;
    const auto* end_gcc = end_terminal.q_gcc;
    const auto [_, gcc_path] =
        single_mode_gcs_planner_->graph_of_configs_->CalcShortestPath(
            start_gcc, end_gcc, &single_mode_gcs_planner_->add_edges_func_);
    if (gcc_path.empty()) {
      logging::log()->error("Failed to find a path from {} to {}",
                            q1.transpose(), q2.transpose());
      return {};
    }
    return single_mode_gcs_planner_->ConvertToGcsVertices(
        gcc_path, start_terminal, end_terminal);
  }

  int intersection_samples() const {
    return num_intersection_samples_;
  }

 private:
  iris::IrisRegionsAdapter iris_regions_adapter_;
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  std::unique_ptr<SingleModeGcsPlanner> single_mode_gcs_planner_;
  bool use_lazy_edges_ {false};
  int num_intersection_samples_;
};

class TestGcsPlanner : public TestGcsPlannerBase {
 protected:
  TestGcsPlanner() : TestGcsPlannerBase(false, 1) {}
};

class TestGcsPlannerSamples : public TestGcsPlannerBase {
 protected:
  TestGcsPlannerSamples() : TestGcsPlannerBase(false, 5) {}
};

class TestGcsPlannerLazy : public TestGcsPlannerBase {
 protected:
  TestGcsPlannerLazy() : TestGcsPlannerBase(true, 1) {}
};

double calc_trajectory_length(
    const drake::trajectories::CompositeTrajectory<double>& trajectory,
    int num_samples = 100) {
  double start_time = trajectory.start_time();
  double end_time = trajectory.end_time();

  if (num_samples <= 1) {
    throw std::invalid_argument("Number of samples must be greater than 1");
  }

  double interval = (end_time - start_time) / (num_samples - 1);
  double length = 0.0;

  Eigen::VectorXd previous_point = trajectory.value(start_time);

  for (int i = 1; i < num_samples; ++i) {
    double time = start_time + i * interval;
    Eigen::VectorXd current_point = trajectory.value(time);

    length += ((previous_point - current_point).norm());
    previous_point = current_point;
  }

  return length;
}  // namespace

TEST_F(TestGcsPlanner, Basics) {
  // The number of vertices should be equal to the number of edges in the gcs
  logging::log()->info("num vertices: {}",
                       graph_of_convex_sets().Edges().size());
  EXPECT_EQ(graph_of_configs().Vertices().size(),
            graph_of_convex_sets().Edges().size() / 2);
  int total_num_connections {0};
  for (const auto* vertex : graph_of_convex_sets().Vertices()) {
    // vertices in gcc correspond to edges in gcs
    const auto num_vertices {std::ssize(vertex->outgoing_edges())};
    total_num_connections += num_vertices * (num_vertices - 1);
  }
  EXPECT_EQ(graph_of_configs().Edges().size(), total_num_connections);
  // check the number of strongly connected components
  EXPECT_EQ(graph_of_configs().CalcStronglyConnectedComponents().size(), 1);
  // solve an A* problem
  const auto* start = graph_of_configs().Vertices().front();
  const auto* goal = graph_of_configs().Vertices().back();
  logging::log()->info("Solving A* from {} to {}", start->name(), goal->name());
  const auto [_, path] = graph_of_configs().CalcShortestPath(start, goal);
  logging::log()->info("path size from {} to {}: {}", start->name(),
                       goal->name(), path.size());
  for (const auto* vertex : path) {
    logging::log()->info("{}", vertex->name());
  }
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlannerSamples, BasicsWithSampling) {
  // The number of vertices should be equal to the number of edges in the gcs
  logging::log()->info("num vertices: {}",
                       graph_of_convex_sets().Edges().size());
  EXPECT_EQ(graph_of_configs().Vertices().size(),
            intersection_samples() * graph_of_convex_sets().Edges().size() / 2);
  int total_num_connections {0};
  for (const auto* vertex : graph_of_convex_sets().Vertices()) {
    // vertices in gcc correspond to edges in gcs
    const auto num_vertices {std::ssize(vertex->outgoing_edges())};
    // there are (intersection_samples)^2 pairs of configs per GCS vertex
    for (int j = 0; j < intersection_samples() * intersection_samples(); j++) {
      total_num_connections += num_vertices * (num_vertices - 1);
    }
  }
  // All vertices within intersections are connected to each other
  total_num_connections +=
      graph_of_configs().Vertices().size() * (intersection_samples() - 1);
  EXPECT_EQ(graph_of_configs().Edges().size(), total_num_connections);
  // check the number of strongly connected components
  EXPECT_EQ(graph_of_configs().CalcStronglyConnectedComponents().size(), 1);
  // solve an A* problem
  const auto* start = graph_of_configs().Vertices().front();
  const auto* goal = graph_of_configs().Vertices().back();
  logging::log()->info("Solving A* from {} to {}", start->name(), goal->name());
  const auto [_, path] = graph_of_configs().CalcShortestPath(start, goal);
  logging::log()->info("path size from {} to {}: {}", start->name(),
                       goal->name(), path.size());
  for (const auto* vertex : path) {
    logging::log()->info("{}", vertex->name());
  }
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlannerLazy, CreateConnectingSet1) {
  const Eigen::Vector2d q_a = Eigen::Vector2d(0, 0.0);
  const Eigen::Vector2d q_b = Eigen::Vector2d(0.0, 3.2);
  auto narrow_polytope = CreateConnectingSet(q_a, q_b, 0.1);
  EXPECT_TRUE(narrow_polytope->PointInSet(q_a));
  EXPECT_TRUE(narrow_polytope->PointInSet(q_b));
  // Verify it is a HPolyhedron
  const auto* narrow_polytope_hpoly =
      dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(
          narrow_polytope.get());
  EXPECT_TRUE(narrow_polytope_hpoly != nullptr);
  // upper corner: [0.1, 3.3], lower corner: [-0.1, -0.1]
  // Test corners are in, a bit further out are not
  double tol {1e-6};
  EXPECT_TRUE(
      narrow_polytope_hpoly->PointInSet(Eigen::Vector2d(-0.1, -0.1), tol));
  EXPECT_TRUE(
      narrow_polytope_hpoly->PointInSet(Eigen::Vector2d(-0.1, 3.3), tol));
  EXPECT_TRUE(
      narrow_polytope_hpoly->PointInSet(Eigen::Vector2d(0.1, 3.3), tol));
  EXPECT_FALSE(
      narrow_polytope_hpoly->PointInSet(Eigen::Vector2d(-0.2, 0.0), tol));
  EXPECT_FALSE(
      narrow_polytope_hpoly->PointInSet(Eigen::Vector2d(0.0, -0.2), tol));
  EXPECT_FALSE(
      narrow_polytope_hpoly->PointInSet(Eigen::Vector2d(0.0, 3.4), tol));
}

TEST_F(TestGcsPlannerLazy, CreateConnectingSet2) {
  const Eigen::Vector2d q_a = Eigen::Vector2d(1.0, 2.0);
  const Eigen::Vector2d q_b = Eigen::Vector2d(3.0, -1.0);
  auto narrow_polytope = CreateConnectingSet(q_a, q_b, 0.01);
  EXPECT_TRUE(narrow_polytope->PointInSet(q_a));
  EXPECT_TRUE(narrow_polytope->PointInSet(q_b));
  // Verify it is a HPolyhedron
  const auto* narrow_polytope_hpoly =
      dynamic_cast<const drake::geometry::optimization::HPolyhedron*>(
          narrow_polytope.get());
  EXPECT_TRUE(narrow_polytope_hpoly != nullptr);
}

TEST_F(TestGcsPlannerLazy, DoAddConnectingSet) {
  // The number of vertices should be equal to the number of edges in the gcs
  const Eigen::Vector2d q_a = Eigen::Vector2d(0, 0.2);
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.2 * M_PI, 0.2);
  // Add terminals
  AddStartTerminal(q_a);
  AddEndTerminal(q_b);
  // 3 regions + 2 points + 1 connecting set
  EXPECT_EQ(graph_of_convex_sets().Vertices().size(), 6);
  // 4 within regions + 2 start + 1 end + 1 from connecting set
  // because the start is connected to first and third
  EXPECT_EQ(graph_of_convex_sets().Edges().size(), 8);
  // Get the vertex correspondong to the first region
  const auto* vertex = graph_of_convex_sets().Vertices().front();
  ASSERT_TRUE(vertex != nullptr) << "No vertex was computed!";
  // The vertex should have 2 outgoing edges and 3 incoming edges
  EXPECT_EQ(std::ssize(vertex->outgoing_edges()), 2);
  EXPECT_EQ(std::ssize(vertex->incoming_edges()), 3);
  // Plot the graphs
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlannerLazy, CalcGcsPath1) {
  // start from the intersection of region 0 and region 2
  const Eigen::Vector2d q_a = Eigen::Vector2d(0, 0.2);
  // end just a bit further than region 1
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.2 * M_PI, 0.2);
  // expected path: q_a --> region 0 --> region 1 --> end_visible --> q_b
  const auto path = CalcGcsVerticesPath(q_a, q_b);
  EXPECT_EQ(path.size(), 5);
  int i {1};
  for (const auto* vertex : path) {
    ASSERT_TRUE(vertex != nullptr)
        << "No vertex was computed for step " << i << "in the path!";
  }
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlannerLazy, CalcGcsPath2) {
  // Reverse of the previous test
  const Eigen::Vector2d q_a = Eigen::Vector2d(0, 0.2);
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.2 * M_PI, 0.2);
  const auto path = CalcGcsVerticesPath(q_b, q_a);
  EXPECT_EQ(path.size(), 5);
  int i {1};
  for (const auto* vertex : path) {
    ASSERT_TRUE(vertex != nullptr)
        << "No vertex was computed for step " << i << "in the path!";
  }
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlannerLazy, CalcGcsPath3) {
  // Start from the intersection of first and second sets
  const Eigen::Vector2d q_a = Eigen::Vector2d(M_PI / 2 - 0.1, 0.5);
  // end in visible place from the second set
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.2 * M_PI, 0.2);
  // Expect the shortest path to be q_a --> region 1 --> end_visible --> q_b
  const auto path = CalcGcsVerticesPath(q_a, q_b);
  EXPECT_EQ(path.size(), 4);
  int i {1};
  for (const auto* vertex : path) {
    ASSERT_TRUE(vertex != nullptr)
        << "No vertex was computed for step " << i << "in the path!";
  }
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlannerLazy, CalcGcsPath4) {
  // Start from the intersection of region 0 and region 2
  const Eigen::Vector2d q_a = Eigen::Vector2d(M_PI / 2 - 0.1, 0.5);
  // end in the intersection of the region 0 and region 2
  const Eigen::Vector2d q_b = Eigen::Vector2d(0.3 * M_PI, 0.2);
  // Expect the shortest path to be q_a --> region 0 --> q_b
  const auto path = CalcGcsVerticesPath(q_a, q_b);
  EXPECT_EQ(path.size(), 3);
  int i {1};
  for (const auto* vertex : path) {
    ASSERT_TRUE(vertex != nullptr)
        << "No vertex was computed for step " << i << "in the path!";
  }
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlannerLazy, CalcGcsPath6) {
  // Right on the edge of region 0
  const Eigen::Vector2d q_a = Eigen::Vector2d(0, -1e-5);
  const Eigen::Vector2d q_b = Eigen::Vector2d(M_PI + 0.3, 0.2);
  // It must handle q_a properly
  std::vector<const drake::geometry::optimization::GraphOfConvexSets::Vertex*>
      path;
  EXPECT_NO_THROW(path = CalcGcsVerticesPath(q_a, q_b));
  // path should have 6 vertices
  EXPECT_EQ(path.size(), 6);
}

TEST_F(TestGcsPlannerLazy, CalcGcsPath5) {
  // Both out of the sets
  const Eigen::Vector2d q_a = Eigen::Vector2d(-M_PI / 2 - 0.1, 0.35);
  const Eigen::Vector2d q_b = Eigen::Vector2d(M_PI + 0.3, 0.2);
  // Expect the shortest path to be q_a --> visible_a--> region 2 --> region 0
  // --> region 1 --> end_visible --> q_b Because we have not yet implemented
  // the shortest path with CheckSatisfied
  const auto path = CalcGcsVerticesPath(q_a, q_b);
  EXPECT_EQ(path.size(), 7);
  int i {1};
  for (const auto* vertex : path) {
    ASSERT_TRUE(vertex != nullptr)
        << "No vertex was computed for step " << i << "in the path!";
  }
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
}

TEST_F(TestGcsPlanner, CalcOptimalPath) {
  // first record the number of vertices and edges
  int num_gcs_vertices = graph_of_convex_sets().Vertices().size();
  int num_gcs_edges = graph_of_convex_sets().Edges().size();
  int num_gcc_vertices = graph_of_configs().Vertices().size();
  int num_gcc_edges = graph_of_configs().Edges().size();
  // Both out of the sets
  const Eigen::Vector2d q_a = Eigen::Vector2d(M_PI / 2 - 0.1, 0.5);
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.2 * M_PI, 0.4);
  const auto trajectory_opt =
      single_mode_gcs_planner().CalcOptimalPath(q_a, q_b);
  EXPECT_TRUE(trajectory_opt.has_value());
  const auto& trajectory = trajectory_opt.value();
  // q_a --> region 1 --> end_visible --> q_b
  // it must two segments
  EXPECT_EQ(trajectory.get_number_of_segments(), 2);
  // The inital point must be q_a + wrapped
  // TODO: this will be fixed once my drake PR lands:
  // https://github.com/RobotLocomotion/drake/pull/21137
  Eigen::Vector2d wrapping = Eigen::Vector2d(-1.0, 0) * 2 * M_PI;
  EXPECT_TRUE(
      trajectory.value(trajectory.start_time()).isApprox(q_a + wrapping, 1e-4));
  // The final point must be q_b
  EXPECT_TRUE(trajectory.value(trajectory.end_time()).isApprox(q_b, 1e-4));
  logging::log()->debug("graphviz of gcs: \n{}",
                        graph_of_convex_sets().GetGraphvizString());
  logging::log()->debug("graphviz of gcc: \n{}",
                        graph_of_configs().GetGraphvizString());
  // verify that the number of vertices and edges did not change.
  // This is testing that the terminal cleanup is working correctly.
  EXPECT_EQ(num_gcs_vertices, graph_of_convex_sets().Vertices().size());
  EXPECT_EQ(num_gcs_edges, graph_of_convex_sets().Edges().size());
  EXPECT_EQ(num_gcc_vertices, graph_of_configs().Vertices().size());
  EXPECT_EQ(num_gcc_edges, graph_of_configs().Edges().size());
}

TEST_F(TestGcsPlannerLazy, FastEstimatePathLength) {
  // For two points not in the regions, fast estimate must be
  // greater than the distance
  const Eigen::Vector2d q_a = Eigen::Vector2d(M_PI / 2 + 0.1, 0.3);
  const Eigen::Vector2d q_b = Eigen::Vector2d(-M_PI / 2 - 0.1, 0.2);
  std::optional<double> cost_opt =
      single_mode_gcs_planner().FastEstimatePathLength(q_a, q_b);
  EXPECT_TRUE(cost_opt.has_value());
  // The cost must be greater than the L infinity distance
  EXPECT_GE(cost_opt.value(), (q_a - q_b).lpNorm<Eigen::Infinity>());
}

TEST(TestSingleModeGcsPlanner, CalcOptimalPathSingleRegion) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = std::make_unique<RobotModel>(xml_file, dmd);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter;
  constraints_adapter.collision_checker = collision_checker_adapter;
  auto robot_constraints =
      std::make_unique<RobotConstraints>(*robot_model, constraints_adapter, 1);
  auto set = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  iris::IrisRegionsAdapter iris_regions_adapter;
  iris_regions_adapter.AddRegion(set, "only_region");
  Eigen::Vector2d joint_velocity_bounds = Eigen::Vector2d(1.0, 1.0);
  auto single_mode_gcs_planner =
      SingleModeGcsPlanner(*robot_constraints, iris_regions_adapter,
                           joint_velocity_bounds, GcsPlannerOptions());
  // test when the start and end are the same
  const Eigen::Vector2d q_a = Eigen::Vector2d(0.5, 0.4);
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.5, 0.2);
  const auto trajectory_opt = single_mode_gcs_planner.CalcOptimalPath(q_a, q_a);
  EXPECT_TRUE(trajectory_opt.has_value());
  const auto& trajectory = trajectory_opt.value();
  // it must be a single point
  EXPECT_EQ(trajectory.get_number_of_segments(), 1);
  // The inital point must be q_a
  EXPECT_TRUE(trajectory.value(trajectory.start_time()).isApprox(q_a, 1e-4));
  // The final point must be q_a
  EXPECT_TRUE(trajectory.value(trajectory.end_time()).isApprox(q_a, 1e-4));
  // test when the start and end are different
  const auto trajectory_opt2 =
      single_mode_gcs_planner.CalcOptimalPath(q_a, q_b);
  EXPECT_TRUE(trajectory_opt2.has_value());
  const auto& trajectory2 = trajectory_opt2.value();
  // it must two segments
  EXPECT_EQ(trajectory2.get_number_of_segments(), 2);
}

TEST(TestSingleModeGcsPlanner, TestIntersectionSampling) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd_data {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const std::string k_ancillary_arm_name {"robot"};
  const auto aa_j0_pair = std::make_pair(k_ancillary_arm_name, 0);
  const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
      aa_j0_pair};  // only the first AA joint is continuous
  auto robot_model_instance =
      RobotModel(xml_file, dmd_data, std::nullopt, continuous_revolute_joints);
  ConstraintsAdapter constraints_adapter_instance;
  constraints_adapter_instance.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter_instance;
  constraints_adapter_instance.collision_checker =
      collision_checker_adapter_instance;
  auto robot_constraints_instance =
      RobotConstraints(robot_model_instance, constraints_adapter_instance, 1);
  auto set_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  auto set_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.2, 5.5));
  auto set_3 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(3.0, 5.2), Eigen::Vector2d(6.0, 7.0));
  auto iris_regions_adapter = iris::IrisRegionsAdapter();
  iris_regions_adapter.AddRegion(set_1, "set1", 0, {}, 5);
  iris_regions_adapter.AddRegion(set_2, "set2", 0, {}, 5);
  iris_regions_adapter.AddRegion(set_3, "set3", 0, {}, 5);
  Eigen::Vector2d joint_velocity_bounds = Eigen::Vector2d(1.0, 1.0);
  std::string gcs_options_file_one_sample =
      "planning_service/test_data/gcs_options.yaml";
  auto gcs_options_one_sample_data =
      drake::yaml::LoadYamlFile<GcsPlannerOptions>(gcs_options_file_one_sample);
  auto single_mode_gcs_planner_instance =
      SingleModeGcsPlanner(robot_constraints_instance, iris_regions_adapter,
                           joint_velocity_bounds, gcs_options_one_sample_data);
  const Eigen::Vector2d q_a = Eigen::Vector2d(0.25, 0.25);
  const Eigen::Vector2d q_b = Eigen::Vector2d(5.8, 6.9);
  const auto trajectory_opt =
      single_mode_gcs_planner_instance.CalcOptimalPath(q_a, q_b);
  EXPECT_TRUE(trajectory_opt.has_value());

  const auto& trajectory = trajectory_opt.value();

  double trajectory_1_length = calc_trajectory_length(trajectory);

  std::string gcs_options_file_many_sample =
      "planning_service/test_data/gcs_options.yaml";
  auto gcs_options_many_sample_data =
      drake::yaml::LoadYamlFile<GcsPlannerOptions>(
          gcs_options_file_many_sample);
  auto single_mode_gcs_planner_many_sample_instance =
      SingleModeGcsPlanner(robot_constraints_instance, iris_regions_adapter,
                           joint_velocity_bounds, gcs_options_many_sample_data);

  const auto trajectory_opt_many_sample =
      single_mode_gcs_planner_many_sample_instance.CalcOptimalPath(q_a, q_b);
  EXPECT_TRUE(trajectory_opt_many_sample.has_value());

  const auto& trajectory_many_sample = trajectory_opt_many_sample.value();

  double trajectory_many_length =
      calc_trajectory_length(trajectory_many_sample);

  EXPECT_LE(trajectory_many_length, trajectory_1_length);
}

TEST(TestSingleModeGcsPlanner, TestConvexHullsFull) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd_data {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const std::string k_ancillary_arm_name {"robot"};
  const auto aa_j0_pair = std::make_pair(k_ancillary_arm_name, 0);
  const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
      aa_j0_pair};  // only the first AA joint is continuous
  auto robot_model_instance =
      RobotModel(xml_file, dmd_data, std::nullopt, continuous_revolute_joints);
  ConstraintsAdapter constraints_adapter_instance;
  constraints_adapter_instance.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter_instance;
  constraints_adapter_instance.collision_checker =
      collision_checker_adapter_instance;
  auto robot_constraints_instance =
      RobotConstraints(robot_model_instance, constraints_adapter_instance, 1);
  auto set_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  auto set_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.2, 5.5));
  auto set_3 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(2.7, 5.0), Eigen::Vector2d(5.0, 7.0));
  auto iris_regions_adapter = iris::IrisRegionsAdapter();
  iris_regions_adapter.AddRegion(set_1, "set1");
  iris_regions_adapter.AddRegion(set_2, "set2");
  iris_regions_adapter.AddRegion(set_3, "set3");
  Eigen::Vector2d joint_velocity_bounds = Eigen::Vector2d(1.0, 1.0);
  std::string gcs_options_file_one_sample =
      "planning_service/test_data/gcs_options.yaml";
  auto gcs_options_one_sample_data =
      drake::yaml::LoadYamlFile<GcsPlannerOptions>(gcs_options_file_one_sample);
  gcs_options_one_sample_data.use_convex_hull_gcs = true;
  gcs_options_one_sample_data.cost_type = 1;
  auto single_mode_gcs_planner_instance =
      SingleModeGcsPlanner(robot_constraints_instance, iris_regions_adapter,
                           joint_velocity_bounds, gcs_options_one_sample_data);
  const Eigen::Vector2d q_a = Eigen::Vector2d(0.25, 0.25);
  const Eigen::Vector2d q_b = Eigen::Vector2d(4.8, 6.9);
  const auto trajectory_opt =
      single_mode_gcs_planner_instance.CalcOptimalPath(q_a, q_b);
  const auto trajectory_opt_cHull =
      single_mode_gcs_planner_instance.CalcOptimalPathCHulls(q_a, q_b);

  const auto trajectory_opt2 =
      single_mode_gcs_planner_instance.CalcOptimalPath(q_b, q_a);
  const auto trajectory_opt2_cHull =
      single_mode_gcs_planner_instance.CalcOptimalPathCHulls(q_b, q_a);

  EXPECT_TRUE(trajectory_opt.has_value());
  EXPECT_TRUE(trajectory_opt_cHull.has_value());
  EXPECT_TRUE(trajectory_opt2.has_value());
  EXPECT_TRUE(trajectory_opt2_cHull.has_value());

  const auto& trajectory = trajectory_opt.value();
  double trajectory_1_length = calc_trajectory_length(trajectory);

  const auto& trajectory_cHull = trajectory_opt_cHull.value();
  double trajectory_1_length_cHull = calc_trajectory_length(trajectory_cHull);

  EXPECT_LE(trajectory_1_length, trajectory_1_length_cHull + 1e-3);

  const auto& trajectory2 = trajectory_opt2.value();
  double trajectory_2_length = calc_trajectory_length(trajectory2);

  const auto& trajectory2_cHull = trajectory_opt2_cHull.value();
  double trajectory_2_length_cHull = calc_trajectory_length(trajectory2_cHull);

  EXPECT_LE(trajectory_2_length, trajectory_2_length_cHull + 1e-3);
}

TEST(TestSingleModeGcsPlanner, TestConvexHullsVisibility) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
  const auto dmd_data {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const std::string k_ancillary_arm_name {"robot"};
  const auto aa_j0_pair = std::make_pair(k_ancillary_arm_name, 0);
  const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
      aa_j0_pair};  // only the first AA joint is continuous
  auto robot_model_instance =
      RobotModel(xml_file, dmd_data, std::nullopt, continuous_revolute_joints);
  ConstraintsAdapter constraints_adapter_instance;
  constraints_adapter_instance.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter_instance;
  constraints_adapter_instance.collision_checker =
      collision_checker_adapter_instance;
  auto robot_constraints_instance =
      RobotConstraints(robot_model_instance, constraints_adapter_instance, 1);
  // set 1, q0: from 0 to 90 degrees
  auto set_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0.5, 0.25), Eigen::Vector2d(1, 1));
  // set 2: q0: from 60 to 180 degrees
  auto set_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.2, 4.5));
  // set 3: q0: from -90 to 45 degrees
  auto set_3 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(2.7, 4.0), Eigen::Vector2d(5.2, 5.2));
  auto iris_regions_adapter = iris::IrisRegionsAdapter();
  iris_regions_adapter.AddRegion(set_1, "set1");
  iris_regions_adapter.AddRegion(set_2, "set2");
  iris_regions_adapter.AddRegion(set_3, "set3");
  Eigen::Vector2d joint_velocity_bounds = Eigen::Vector2d(1.0, 1.0);
  std::string gcs_options_file_one_sample =
      "planning_service/test_data/gcs_options.yaml";
  auto gcs_options_one_sample_data =
      drake::yaml::LoadYamlFile<GcsPlannerOptions>(gcs_options_file_one_sample);
  gcs_options_one_sample_data.use_convex_hull_gcs = true;
  auto single_mode_gcs_planner_instance =
      SingleModeGcsPlanner(robot_constraints_instance, iris_regions_adapter,
                           joint_velocity_bounds, gcs_options_one_sample_data);
  const Eigen::Vector2d q_a = Eigen::Vector2d(-0.6, 0.25);
  const Eigen::Vector2d q_b = Eigen::Vector2d(5.1, 5.1);
  const auto trajectory_opt_cHull =
      single_mode_gcs_planner_instance.CalcOptimalPathCHulls(q_a, q_b);
  const auto trajectory_opt2_cHull =
      single_mode_gcs_planner_instance.CalcOptimalPathCHulls(q_b, q_a);
  EXPECT_TRUE(trajectory_opt_cHull.has_value());
  EXPECT_TRUE(trajectory_opt2_cHull.has_value());
}

TEST(TestSingleModeGcsPlanner, TestConvexHullsSmall) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd_data {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const std::string k_ancillary_arm_name {"robot"};
  const auto aa_j0_pair = std::make_pair(k_ancillary_arm_name, 0);
  const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
      aa_j0_pair};  // only the first AA joint is continuous
  auto robot_model_instance =
      RobotModel(xml_file, dmd_data, std::nullopt, continuous_revolute_joints);
  ConstraintsAdapter constraints_adapter_instance;
  constraints_adapter_instance.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter_instance;
  constraints_adapter_instance.collision_checker =
      collision_checker_adapter_instance;
  auto robot_constraints_instance =
      RobotConstraints(robot_model_instance, constraints_adapter_instance, 1);
  // set 1, q0: from 0 to 90 degrees
  auto set_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  // set 2: q0: from 60 to 180 degrees
  auto set_2 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0.75, 0.75), Eigen::Vector2d(3.2, 5.5));
  auto iris_regions_adapter = iris::IrisRegionsAdapter();
  iris_regions_adapter.AddRegion(set_1, "set1");
  iris_regions_adapter.AddRegion(set_2, "set2");
  Eigen::Vector2d joint_velocity_bounds = Eigen::Vector2d(1.0, 1.0);
  std::string gcs_options_file_one_sample =
      "planning_service/test_data/gcs_options.yaml";
  auto gcs_options_one_sample_data =
      drake::yaml::LoadYamlFile<GcsPlannerOptions>(gcs_options_file_one_sample);
  gcs_options_one_sample_data.use_convex_hull_gcs = true;
  auto single_mode_gcs_planner_instance =
      SingleModeGcsPlanner(robot_constraints_instance, iris_regions_adapter,
                           joint_velocity_bounds, gcs_options_one_sample_data);
  const Eigen::Vector2d q_a = Eigen::Vector2d(0.25, 0.25);
  const Eigen::Vector2d q_b = Eigen::Vector2d(3.0, 5.0);
  const auto trajectory_opt_cHull =
      single_mode_gcs_planner_instance.CalcOptimalPathCHulls(q_a, q_b);

  EXPECT_TRUE(trajectory_opt_cHull.has_value());
}

TEST(TestSingleModeGcsPlanner, TestConvexHullsSingleRegion) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd_data {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const std::string k_ancillary_arm_name {"robot"};
  const auto aa_j0_pair = std::make_pair(k_ancillary_arm_name, 0);
  const std::vector<std::pair<std::string, int>> continuous_revolute_joints {
      aa_j0_pair};  // only the first AA joint is continuous
  auto robot_model_instance =
      RobotModel(xml_file, dmd_data, std::nullopt, continuous_revolute_joints);
  ConstraintsAdapter constraints_adapter_instance;
  constraints_adapter_instance.plan_name = "";
  CollisionCheckerAdapter collision_checker_adapter_instance;
  constraints_adapter_instance.collision_checker =
      collision_checker_adapter_instance;
  auto robot_constraints_instance =
      RobotConstraints(robot_model_instance, constraints_adapter_instance, 1);
  // set 1, q0: from 0 to 90 degrees
  auto set_1 = drake::geometry::optimization::HPolyhedron::MakeBox(
      Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 1));
  auto iris_regions_adapter = iris::IrisRegionsAdapter();
  iris_regions_adapter.AddRegion(set_1, "set1");
  Eigen::Vector2d joint_velocity_bounds = Eigen::Vector2d(1.0, 1.0);
  std::string gcs_options_file_one_sample =
      "planning_service/test_data/gcs_options.yaml";
  auto gcs_options_one_sample_data =
      drake::yaml::LoadYamlFile<GcsPlannerOptions>(gcs_options_file_one_sample);
  gcs_options_one_sample_data.use_convex_hull_gcs = true;
  auto single_mode_gcs_planner_instance =
      SingleModeGcsPlanner(robot_constraints_instance, iris_regions_adapter,
                           joint_velocity_bounds, gcs_options_one_sample_data);
  const Eigen::Vector2d q_a = Eigen::Vector2d(0.25, 0.25);
  const Eigen::Vector2d q_b = Eigen::Vector2d(1.0, 1.0);
  const auto trajectory_opt_cHull =
      single_mode_gcs_planner_instance.CalcOptimalPathCHulls(q_a, q_b);

  EXPECT_TRUE(trajectory_opt_cHull.has_value());
}

}  // namespace planning
}  // namespace motion
