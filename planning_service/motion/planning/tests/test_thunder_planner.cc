

/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/motion/planning/thunder_planner.h"

namespace ot = ::ompl::tools;
namespace ob = ::ompl::base;
namespace og = ::ompl::geometric;

namespace motion {
namespace planning {
namespace ompl {

std::vector<double> e_to_v(Eigen::VectorXd e) {
  std::vector<double> v;
  v.resize(e.size());
  Eigen::VectorXd::Map(&v[0], e.size()) = e;
  return v;
}

class AlwaysTrueValidityChecker : public ob::StateValidityChecker {
 public:
  explicit AlwaysTrueValidityChecker(const ob::SpaceInformationPtr& sip)
      : ob::StateValidityChecker {sip} {};
  bool isValid(const ob::State* state) const override {
    logging::log()->trace(si_->distance(state, state));
    return true;
  };
};

// Nodes inside ball of some radius are invalid. All else is valid.
class OutsideBallChecker : public ob::StateValidityChecker {
 public:
  OutsideBallChecker(const ob::SpaceInformationPtr& sip,
                     const double radius = 1.0)
      : ob::StateValidityChecker {sip} {
    radius_ = radius;
  };
  double radius_ {};
  bool isValid(const ob::State* state) const override {
    std::vector<double> state_vec;
    si_->getStateSpace()->copyToReals(state_vec, state);

    double norm {};
    for (const auto& coordinate : state_vec) {
      norm += std::pow(coordinate, 2);
    }
    norm = std::sqrt(norm);
    return norm > radius_;
  };
};

// When this optimization objective is applied to an edge, it returns the edge
// length multiplied by a scalar_
class ConstantOptimizationObjectiveMB : public ob::StateCostIntegralObjective {
 public:
  explicit ConstantOptimizationObjectiveMB(const ob::SpaceInformationPtr& sip,
                                           const double scalar)
      : ob::StateCostIntegralObjective(sip, true) {
    scalar_ = scalar;
  }

  ob::Cost stateCost(const ob::State* s) const {
    logging::log()->trace(si_->distance(s, s));
    return ob::Cost(scalar_);
  }
  // We override the motion cost here, in case the default motionCost function,
  // or the default values of the flags used in it, change in ompl. Applying the
  // trapezoidal rule on two vertices of cost 1.0 returns the edge length
  // between them
  ob::Cost motionCost(const ob::State* s1, const ob::State* s2) const override {
    return this->trapezoid(this->stateCost(s1), this->stateCost(s2),
                           si_->distance(s1, s2));
  }
  double scalar_;
};

// Cost objective that is low away from some half-space, gets inifinitely high
// close to it.
class DistanceFromHalfSpaceObjectiveMB : public ob::StateCostIntegralObjective {
 public:
  explicit DistanceFromHalfSpaceObjectiveMB(
      const ob::SpaceInformationPtr& sip,
      const double separation_coordinate = 0.0)
      : ob::StateCostIntegralObjective(sip, true) {
    separation_coordinate_ = separation_coordinate;
  }

  ob::Cost stateCost(const ob::State* s) const {
    std::vector<double> state_vec;
    si_->getStateSpace()->copyToReals(state_vec, s);
    if (!state_vec.size()) {
      return ob::Cost(0);  // throw?
    }
    if (state_vec[0] <= separation_coordinate_) {
      return ob::Cost(
          std::numeric_limits<double>::max());  // in forbidden region
    }
    // cost is high when state is close to forbidden half-space, low away from
    // it
    return ob::Cost(10 / std::pow(state_vec[0] - separation_coordinate_, 2));
  }

  // forbidden half space is x_0 <= seperation_coordinate_, where state = (x0,
  // x1, ..., x_n)
  double separation_coordinate_ {};
};

// Cost objective that is low away from some ball at the origin, inifintely high
// close to it
class DistanceFromBallObjectiveMB : public ob::MinimaxObjective {
 public:
  explicit DistanceFromBallObjectiveMB(const ob::SpaceInformationPtr& sip,
                                       const double ball_radius = 1.0)
      : ob::MinimaxObjective(sip) {
    ball_radius_ = ball_radius;
  }

  ob::Cost stateCost(const ob::State* s) const {
    std::vector<double> state_vec;
    si_->getStateSpace()->copyToReals(state_vec, s);
    if (!state_vec.size()) {
      return ob::Cost(0);  // throw?
    }

    double state_norm {};
    for (const auto& coordinate : state_vec) {
      state_norm += std::pow(coordinate, 2);
    }
    state_norm = std::sqrt(state_norm);
    if (state_norm <= ball_radius_) {
      return ob::Cost(
          std::numeric_limits<double>::max());  // inside forbidden ball
    }
    // Gets infinitely high as you get close to the ball of radius ball_radius_,
    // and gets lower away from it
    return ob::Cost(1 / std::pow(state_norm - ball_radius_, 4));
  }
  // forbidden ball is the ball centered at the origin with radius ball_radius_
  double ball_radius_ {};
};

void SetupThunderAndLoadRoadmap(std::shared_ptr<ot::Thunder> thunder,
                                const std::string& db_file_path) {
  thunder->setSparseDelta(1.0);
  thunder->setNumParallelPlans(2);
  thunder->setRRT();
  thunder->setup();

  thunder->getProblemDefinition()->setComputeSolutionCost(true);
  thunder->getExperienceDB()->getSPARSdb()->setMigrateRoadmapOnLoad(true);
  thunder->getExperienceDB()->getSPARSdb()->setUseCostInRoadmap(true);
  thunder->getExperienceDB()->getSPARSdb()->setDenseRoadmap(true);
  thunder->getExperienceDB()->getSPARSdb()->setAddEdgeWithCost(true);
  thunder->getExperienceDB()->getSPARSdb()->setRoadmapGranularity(0.25);

  thunder->setFilePath(db_file_path);
  thunder->getExperienceDB()->load(thunder->getFilePath());
  thunder->getExperienceDB()->getSPARSdb()->setup();
}

void SetupThunder(std::shared_ptr<ot::Thunder> thunder,
                  ob::StateValidityCheckerPtr checker,
                  ob::OptimizationObjectivePtr objective,
                  const std::string& db_file_path = "") {
  auto si {thunder->getSpaceInformation()};
  si->setStateValidityChecker(checker);
  si->setStateValidityCheckingResolution(0.01);
  si->setup();

  thunder->setOptimizationObjective(objective);
  thunder->getProblemDefinition()->setComputeSolutionCost(true);
  thunder->getExperienceDB()->getSPARSdb()->setRoadmapGranularity(0.1);
  thunder->getExperienceDB()->getSPARSdb()->setMigrateRoadmapOnLoad(true);
  thunder->getExperienceDB()->getSPARSdb()->setUseCostInRoadmap(true);
  thunder->getExperienceDB()->getSPARSdb()->setDenseRoadmap(true);
  thunder->getExperienceDB()->getSPARSdb()->setAddEdgeWithCost(true);

  thunder->setFilePath(db_file_path);
  thunder->getExperienceDB()->load(thunder->getFilePath());
  thunder->getExperienceDB()->getSPARSdb()->setup();
}

std::shared_ptr<ot::Thunder> CreateThunderFromRobotConstraints(
    const RobotConstraints& robot_constraints) {
  const auto space {
      std::make_shared<RobotStateSpace>(robot_constraints.robot_model())};
  auto thunder {std::make_shared<ot::Thunder>(space)};
  // sparse_delta = 1.0 give each node maximum visibility when inserted: it can
  // connect to all nodes in the roadmap to which it is connected by a valid
  // edge.
  thunder->setSparseDelta(1.0);
  thunder->setNumParallelPlans(0);
  thunder->setRRT();
  thunder->setup();
  return thunder;
}

TEST(TestThunderPlanner, CalcNearestValidConf) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_wallflowers/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = RobotModel(xml_file, dmd);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "test_with_default_collision_checker";
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  auto robot_constraints = RobotConstraints(robot_model, constraints_adapter);
  ThunderParameters thunder_parameters;
  thunder_parameters.max_scratch_planning_time = 5.0;
  thunder_parameters.verbosity = 3;
  std::string db_file_path =
      "planning_service/test_data/dual_wallflowers/thunder_prm.dat";
  auto thunder = std::make_unique<ThunderPlanner>(
      robot_constraints, thunder_parameters, db_file_path);
  logging::log()->info("Number of vertices in the roadmap: {}",
                       thunder->vertices_confs().size());
  // Let's sample a invalid one
  drake::RandomGenerator gen {0};
  SampleOptions sample_options;
  sample_options.parallel = false;
  sample_options.return_invalid = true;
  auto invalid_samples =
      robot_constraints.GenerateSamples(&gen, 5, sample_options);
  CheckSatisfiedOptions options;
  options.verbose = true;
  for (const auto& invalid_sample : invalid_samples) {
    auto nearest_valid_conf = thunder->CalcNearestValidConf(invalid_sample);
    EXPECT_TRUE(nearest_valid_conf.has_value());
    logging::log()->info("invalid conf: \t {} \nnearest valid conf: {}",
                         invalid_sample.transpose(),
                         nearest_valid_conf.value().transpose());
    EXPECT_TRUE(robot_constraints.CheckSatisfied(nearest_valid_conf.value(), 0,
                                                 options));
  }
  // Now let's fix the model instance indices. Fix the 1st arm.
  const auto& arm1 = robot_model.GetArm(ArmIndex(0));
  for (const auto& invalid_sample : invalid_samples) {
    auto nearest_valid_conf =
        thunder->CalcNearestValidConf(invalid_sample, arm1.model_instances());
    if (!nearest_valid_conf.has_value()) {
      logging::log()->info(
          "No nearest valid conf found for {} while fixing first arm",
          invalid_sample.transpose());
      EXPECT_FALSE(
          robot_constraints.CheckSatisfied(invalid_sample, 0, options));
      continue;
    }
    EXPECT_TRUE(nearest_valid_conf.has_value());
    EXPECT_TRUE(robot_constraints.CheckSatisfied(nearest_valid_conf.value(), 0,
                                                 options));
    // Check that the model instance indices are fixed
    EXPECT_TRUE(
        nearest_valid_conf.value().head(2).isApprox(invalid_sample.head(2)));
  }
  const auto& arm2 = robot_model.GetArm(ArmIndex(1));
  // Now let's fix the second arm.
  for (const auto& invalid_sample : invalid_samples) {
    auto nearest_valid_conf =
        thunder->CalcNearestValidConf(invalid_sample, arm2.model_instances());
    if (!nearest_valid_conf.has_value()) {
      logging::log()->info(
          "No nearest valid conf found for {} while fixing second arm",
          invalid_sample.transpose());
      EXPECT_FALSE(
          robot_constraints.CheckSatisfied(invalid_sample, 0, options));
      continue;
    }
    EXPECT_TRUE(nearest_valid_conf.has_value());
    logging::log()->info("invalid conf: \t {} \nnearest valid conf: {}",
                         invalid_sample.transpose(),
                         nearest_valid_conf.value().transpose());
    EXPECT_TRUE(robot_constraints.CheckSatisfied(nearest_valid_conf.value(), 0,
                                                 options));
    // Check that the model instance indices are fixed
    EXPECT_TRUE(
        nearest_valid_conf.value().tail(2).isApprox(invalid_sample.tail(2)));
  }
}

TEST(TestThunderPlanner, CalcNearestValidConf_MultipleConfs) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/dual_wallflowers/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  auto robot_model = RobotModel(xml_file, dmd);
  ConstraintsAdapter constraints_adapter;
  constraints_adapter.plan_name = "test_with_default_collision_checker";
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  auto robot_constraints = RobotConstraints(robot_model, constraints_adapter);
  ThunderParameters thunder_parameters;
  std::string db_file_path =
      "planning_service/test_data/dual_wallflowers/thunder_prm.dat";
  auto thunder = std::make_unique<ThunderPlanner>(
      robot_constraints, thunder_parameters, db_file_path);
  logging::log()->info("Number of vertices in the roadmap: {}",
                       thunder->vertices_confs().size());
  // Let's sample a invalid one
  Eigen::VectorXd invalid_sample1(4), invalid_sample2(4), invalid_sample3(4);
  invalid_sample1 << 2.0, 0.4, 1.0, 0.4;   // Does not collide
  invalid_sample2 << 2.1, 0.35, 1.0, 0.4;  // Collides
  invalid_sample3 << 2.2, 0.3, 1.0, 0.4;   // Collides
  // Fixed model instance indices: fix second flower
  std::set<drake::multibody::ModelInstanceIndex> fixed_models;
  fixed_models.insert(robot_model.plant().GetModelInstanceByName("flower1"));
  std::vector<Eigen::VectorXd> invalid_confs {invalid_sample1, invalid_sample2,
                                              invalid_sample3};
  auto nearest_valid_conf =
      thunder->CalcNearestValidConf(invalid_confs, fixed_models);
  EXPECT_TRUE(nearest_valid_conf.has_value());
  logging::log()->info("................. nearest valid conf: {}",
                       nearest_valid_conf.value().transpose());
  // Test that projecting each invalid conf to the nearest valid conf
  const auto delta = nearest_valid_conf.value() - invalid_sample1;
  auto invalid_sample2_delta = invalid_sample2 + delta;
  auto invalid_sample3_delta = invalid_sample3 + delta;
  // Check that they are valid
  EXPECT_TRUE(robot_constraints.CheckSatisfied(nearest_valid_conf.value()));
  EXPECT_TRUE(robot_constraints.CheckSatisfied(invalid_sample2_delta));
  EXPECT_TRUE(robot_constraints.CheckSatisfied(invalid_sample3_delta));
  // Trying invalid_confs without fixed model instance indices should throw
  EXPECT_THROW(thunder->CalcNearestValidConf(invalid_confs, {}),
               std::exception);
}

TEST(TestThunderPlanner, Plan) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {"planning_service/test_data/wallflower/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  ConstraintsAdapter dut;
  dut.plan_name = "test";
  dut.collision_checker = CollisionCheckerAdapter {};
  const auto dut_string {drake::yaml::SaveYamlString(dut)};
  logging::log()->info("dut_string: \n{}", dut_string);
  // CI only has 1 core. But the test will pass anyways
  const auto n_threads {std::thread::hardware_concurrency()};
  auto robot_constraints {
      std::make_shared<RobotConstraints>(*robot_model, dut, n_threads)};
  CheckSatisfiedOptions options;
  options.parallel = false;
  options.verbose = true;
  options.num_threads = n_threads;

  // number of problems to solve
  const size_t n_problems {1};
  drake::RandomGenerator gen {0};
  SampleOptions sample_options;
  sample_options.parallel = false;
  // generate a vector of valid start samples using
  // robot_constraints->GenerateValidSamples
  const auto q_start_eigen {
      robot_constraints->GenerateSamples(&gen, n_problems, sample_options)[0]};
  // generate a vector of valid goal samples using
  // robot_constraints->GenerateValidSamples
  const auto q_goal_eigen {
      robot_constraints->GenerateSamples(&gen, n_problems, sample_options)[0]};

  const auto thunder_ss {CreateThunderFromRobotConstraints(*robot_constraints)};
  // get SpaceInformation
  ob::SpaceInformationPtr si {thunder_ss->getSpaceInformation()};
  auto& space {si->getStateSpace()};

  ob::ScopedState<> start(space);
  std::vector<double> start_aa_conf_v(e_to_v(q_start_eigen));
  start = start_aa_conf_v;
  ob::ScopedState<> goal(space);
  std::vector<double> goal_aa_conf_v(e_to_v(q_goal_eigen));
  goal = goal_aa_conf_v;
  thunder_ss->setStartAndGoalStates(start, goal, 0.001);

  // auto ball_validity_checker {std::make_shared<OutsideBallChecker>(si, 1.0)};
  auto rc_validity_checker {
      std::make_shared<ValidityChecker>(si, *robot_constraints)};
  auto unit_opt_obj {
      std::make_shared<ConstantOptimizationObjectiveMB>(si, 1.0)};
  std::string db_file_path {"planning_service/test_data/10dof_generated.dat"};

  SetupThunder(thunder_ss, rc_validity_checker, unit_opt_obj, db_file_path);
  const auto timed_ptc {ob::timedPlannerTerminationCondition(
      4.0)};  // try to recall for 3 seconds

  ASSERT_TRUE(thunder_ss->getSpaceInformation()->isValid(start.get()));
  ASSERT_TRUE(thunder_ss->getSpaceInformation()->isValid(goal.get()));

  // this next part will crash
  thunder_ss->solve(timed_ptc);
  const auto solutions {thunder_ss->getProblemDefinition()->getSolutions()};
  ASSERT_TRUE(solutions.size());
  const auto solution {solutions[0]};
  auto path {std::static_pointer_cast<og::PathGeometric>(solution.path_)};
  std::vector<ob::State*> path_states {path->getStates()};
  logging::log()->info("Solution has size: {}, and cost = {}, and length = {}",
                       path_states.size(), solution.cost_, solution.length_);
  int counter {};
  for (auto& path_state : path_states) {
    counter++;
    std::vector<double> this_state_vec;
    space->copyToReals(this_state_vec, path_state);
    // convert to eigen
    Eigen::VectorXd state_eigen = Eigen::Map<Eigen::VectorXd>(
        this_state_vec.data(), this_state_vec.size());
    // print the state
    logging::log()->info("{}: {}", counter, state_eigen.transpose());
  }
}

TEST(ThunderPlanner, TestAddConfsToRoadmap) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter {};
  constraints_adapter.plan_name = "test";
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  const auto n_threads {std::thread::hardware_concurrency()};
  auto robot_constraints {std::make_shared<RobotConstraints>(
      *robot_model, constraints_adapter, n_threads)};
  ThunderParameters thunder_parameters {};
  thunder_parameters.roadmap_granularity_base = 0.1;
  thunder_parameters.verbosity = 4;
  std::string db_file_path =
      "planning_service/test_data/2d_prismatic_robot/"
      "thunder_prm_AddConfsToRoadmap.dat";
  // if file does not exist, create it
  if (!std::ifstream(db_file_path)) {
    std::ofstream ofs(db_file_path);
    ofs.close();
  }
  auto thunder_planner = std::make_unique<ThunderPlanner>(
      *robot_constraints, thunder_parameters, db_file_path);
  const auto num_vertices_before_vec = thunder_planner->vertices_confs().size();
  const auto num_vertices_before_thunder =
      thunder_planner->planner_data().numVertices();
  const auto num_vertices_before_sparsdb =
      thunder_planner->experience_database()->getSPARSdb()->getNumVertices();
  const auto num_edges_before = thunder_planner->planner_data().numEdges();
  // Add a new conf
  Eigen::VectorXd new_conf(2);
  new_conf << 0.5, 0.5;
  thunder_planner->AddConfsToRoadmap({new_conf});
  const auto num_vertices_after1_vec = thunder_planner->vertices_confs().size();
  const auto num_vertices_after1_thunder =
      thunder_planner->planner_data().numVertices();
  const auto num_vertices_after1_sparsdb =
      thunder_planner->experience_database()->getSPARSdb()->getNumVertices();
  EXPECT_EQ(num_vertices_after1_vec, num_vertices_before_vec + 1);
  EXPECT_EQ(num_vertices_after1_thunder, num_vertices_before_thunder + 1);
  EXPECT_EQ(num_vertices_after1_sparsdb, num_vertices_before_sparsdb + 1);
  // Check that the last vertex is the one we added
  const auto& added_conf =
      thunder_planner->GetVertexConf(num_vertices_after1_vec - 1);
  logging::log()->info("Added conf: {}", added_conf.transpose());
  EXPECT_TRUE(added_conf.isApprox(new_conf));

  // Add another new conf different than the previous one
  Eigen::VectorXd new_conf2(2);
  new_conf2 << 0.75, 0.5;

  thunder_planner->AddConfsToRoadmap({new_conf2});
  const auto num_vertices_after2_vec = thunder_planner->vertices_confs().size();
  const auto num_vertices_after2_thunder =
      thunder_planner->planner_data().numVertices();
  const auto num_vertices_after2_sparsdb =
      thunder_planner->experience_database()->getSPARSdb()->getNumVertices();
  EXPECT_EQ(num_vertices_after2_vec, num_vertices_after1_vec + 1);
  EXPECT_EQ(num_vertices_after2_thunder, num_vertices_after1_thunder + 1);
  EXPECT_EQ(num_vertices_after2_sparsdb, num_vertices_after1_sparsdb + 1);
  // Check that the last vertex is the one we added
  const auto& added_conf2 =
      thunder_planner->GetVertexConf(num_vertices_after2_vec - 1);
  logging::log()->info("Added conf2: {}", added_conf2.transpose());
  EXPECT_TRUE(added_conf2.isApprox(new_conf2));
  // Check that the number of edges increased
  const auto num_edges_after2 = thunder_planner->planner_data().numEdges();
  EXPECT_GT(num_edges_after2, num_edges_before);

  // Adding the same conf again should not increase the number of vertices
  thunder_planner->AddConfsToRoadmap({new_conf});
  const auto num_vertices_after_adding_same_vec =
      thunder_planner->vertices_confs().size();
  logging::log()->info("Number of vertices after adding same conf: {}",
                       num_vertices_after_adding_same_vec);
  EXPECT_EQ(num_vertices_after_adding_same_vec, num_vertices_after2_vec);
  const auto num_vertices_after_adding_same_thunder =
      thunder_planner->planner_data().numVertices();
  EXPECT_EQ(num_vertices_after_adding_same_thunder,
            num_vertices_after2_thunder);
  const auto num_vertices_after_adding_same_sparsdb =
      thunder_planner->experience_database()->getSPARSdb()->getNumVertices();
  EXPECT_EQ(num_vertices_after_adding_same_sparsdb,
            num_vertices_after2_sparsdb);
  const auto num_edges_after_adding_same =
      thunder_planner->planner_data().numEdges();
  EXPECT_EQ(num_edges_after_adding_same, num_edges_after2);
  // delete the database file to avoid interference with other tests
  std::remove(db_file_path.c_str());
}

TEST(ThunderPlanner, TestConstrainedPlanning) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  // Testing projection of a conf on a constrained set
  const std::string plan_adapter_file {
      "planning_service/test_data/2d_prismatic_robot/constraints_adapter.yaml"};
  const auto plan_adapter {
      drake::yaml::LoadYamlFile<ConstraintsAdapter>(plan_adapter_file)};

  const auto n_threads {std::thread::hardware_concurrency()};
  auto robot_constraints {std::make_shared<RobotConstraints>(
      *robot_model, plan_adapter, n_threads)};

  CheckSatisfiedOptions options;
  options.parallel = true;
  options.verbose = false;
  options.num_threads = n_threads;

  Eigen::VectorXd q_start_eigen(2);
  Eigen::VectorXd q_goal_eigen(2);
  q_start_eigen << -0.129, -0.427;
  q_goal_eigen << 0.124, 0.467;

  const auto thunder_ss {CreateThunderFromRobotConstraints(*robot_constraints)};
  // get SpaceInformation
  ob::SpaceInformationPtr si {thunder_ss->getSpaceInformation()};
  auto& space {si->getStateSpace()};
  ob::ScopedState<> start(space);
  std::vector<double> start_aa_conf_v(e_to_v(q_start_eigen));
  start = start_aa_conf_v;
  ob::ScopedState<> goal(space);
  std::vector<double> goal_aa_conf_v(e_to_v(q_goal_eigen));
  goal = goal_aa_conf_v;
  thunder_ss->setStartAndGoalStates(start, goal, 0.001);

  auto rc_validity_checker {
      std::make_shared<ValidityChecker>(si, *robot_constraints)};
  // path length objective
  auto path_length_opt_obj {
      std::make_shared<ConstantOptimizationObjectiveMB>(si, 1.0)};
  std::string db_file_path {""};  // no recall

  SetupThunder(thunder_ss, rc_validity_checker, path_length_opt_obj,
               db_file_path);
  const auto timed_ptc {ob::timedPlannerTerminationCondition(1.0)};

  ASSERT_TRUE(thunder_ss->getSpaceInformation()->isValid(start.get()));
  ASSERT_TRUE(thunder_ss->getSpaceInformation()->isValid(goal.get()));

  // solve parallel plans and get the planning time
  // time bracketed code
  thunder_ss->solve(timed_ptc);
  const auto solutions {thunder_ss->getProblemDefinition()->getSolutions()};
  ASSERT_TRUE(solutions.size());
}

TEST(ThunderPlanner, WithGripper) {
  const std::string xml_file {"planning_service/test_data/package.xml"};
  const std::string dmd_file {
      "planning_service/test_data/franka_with_gripper/dmd.yaml"};
  const auto dmd {
      drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
          dmd_file)};
  const auto robot_model {std::make_unique<motion::RobotModel>(xml_file, dmd)};
  ConstraintsAdapter constraints_adapter {};
  constraints_adapter.plan_name = "test";
  constraints_adapter.collision_checker = CollisionCheckerAdapter {};
  auto robot_constraints {
      std::make_shared<RobotConstraints>(*robot_model, constraints_adapter)};
  ThunderParameters thunder_parameters {};
  std::string db_file_path =
      "planning_service/test_data/franka_with_gripper/"
      "thunder.dat";
  auto thunder_planner = std::make_unique<ThunderPlanner>(
      *robot_constraints, thunder_parameters, db_file_path);
  // Let's solve a couple of problems
  Eigen::VectorXd q_start(8), q_goal(8);
  q_start << -1.9, 0.0, -2.3, -0.5, 0.5, 2.1, 0, 0.3;
  q_goal << 1.3, -1.5, 1.6, -1.75, 1.1, 1.9, 1.7, 0.6;
  auto plan_opt = thunder_planner->SolveRecallPlan(q_start, q_goal);
  EXPECT_TRUE(plan_opt.has_value());
  EXPECT_TRUE(plan_opt->front().isApprox(q_start));
  EXPECT_TRUE(plan_opt->back().isApprox(q_goal));
}

// create a unit testing class called ThunderPlannerRoadmapTest that is a friend
// to ThunderPlanner and can test private methods like GetConnectedVertices and
// SparseAdjacencyMatrixOfVertices
class ThunderPlannerRoadmapTest : public ::testing::Test {
 protected:
  ThunderPlannerRoadmapTest() {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    robot_model_ = std::make_unique<motion::RobotModel>(xml_file, dmd);
    ConstraintsAdapter constraints_adapter {};
    constraints_adapter.plan_name = "test";
    constraints_adapter.collision_checker = CollisionCheckerAdapter {};
    const auto n_threads {std::thread::hardware_concurrency()};
    robot_constraints_ = std::make_shared<RobotConstraints>(
        *robot_model_, constraints_adapter, n_threads);
    ThunderParameters thunder_parameters {};
    std::string db_file_path =
        "planning_service/test_data/2d_prismatic_robot/thunder_prm.dat";
    thunder_planner_ = std::make_unique<ThunderPlanner>(
        *robot_constraints_, thunder_parameters, db_file_path);
    num_vertices_ =
        thunder_planner_->experience_database()->getSPARSdb()->getNumVertices()
        - 1;  // SPARSdb has 1 additional vertex than PlannerData
  }

  std::vector<unsigned int> GetConnectedVertices(
      std::vector<unsigned int> included_vertex_indices) {
    return thunder_planner_->GetConnectedVertices(included_vertex_indices);
  }

  Eigen::SparseMatrix<bool> SparseAdjacencyMatrixOfVertices(
      std::vector<unsigned int> vertex_indices) {
    return thunder_planner_->SparseAdjacencyMatrixOfVertices(vertex_indices);
  }

  std::unique_ptr<motion::RobotModel> robot_model_;
  std::shared_ptr<RobotConstraints> robot_constraints_;
  std::unique_ptr<ThunderPlanner> thunder_planner_;
  unsigned int num_vertices_;
};

TEST_F(ThunderPlannerRoadmapTest, TestAdjacencyMatrix) {
  // log the adjacency matrix
  std::vector<unsigned int> vertex_indices;
  for (unsigned int i = 0; i < num_vertices_; i++) {
    vertex_indices.push_back(i);
  }
  const auto adjacency_matrix {SparseAdjacencyMatrixOfVertices(vertex_indices)};
  // make a few assertions about the size and contents of the adjacency matrix
  ASSERT_EQ(adjacency_matrix.rows(), num_vertices_);
  ASSERT_EQ(adjacency_matrix.cols(), num_vertices_);
  // check that the adjacency matrix is symmetric
  for (unsigned int i = 0; i < num_vertices_; i++) {
    for (unsigned int j = i + 1; j < num_vertices_; j++) {
      ASSERT_EQ(adjacency_matrix.coeff(i, j), adjacency_matrix.coeff(j, i));
    }
  }
  // check all the positive entries in the adjacency matrix correspond to
  // connected vertices in planner data
  ob::PlannerData planner_data(thunder_planner_->space_information());
  thunder_planner_->GetPlannerData(planner_data);
  for (unsigned int i = 0; i < num_vertices_; i++) {
    for (unsigned int j = 0; j < num_vertices_; j++) {
      if (adjacency_matrix.coeff(i, j)) {
        EXPECT_TRUE(planner_data.edgeExists(i, j)
                    || planner_data.edgeExists(j, i));
      } else {
        EXPECT_FALSE(planner_data.edgeExists(i, j)
                     || planner_data.edgeExists(j, i));
      }
    }
  }
}

TEST_F(ThunderPlannerRoadmapTest, TestConnectedVertices) {
  // count the total number of connections: expected to be twice the number of
  // edges (we double count connection (i,j) and (j,i))
  int total_connections {};
  for (unsigned int i = 0; i < num_vertices_; i++) {
    const auto connected_vertices {GetConnectedVertices({i})};
    total_connections += connected_vertices.size();
  }
  // get the number of edges
  const auto edges {thunder_planner_->GetRoadmapEdges()};
  const auto num_edges {edges.size()};
  // make a few assertions about the number of connected vertices
  EXPECT_EQ(total_connections,
            2 * num_edges);  // There are two counted connections from each edge
  // assert that the connected vertices to vertex 0 and vertex 1 are the
  // intersection of the connected vertices to vertex 0 and vertex 1
  const auto connected_vertices_0 {GetConnectedVertices({0})};
  const auto connected_vertices_1 {GetConnectedVertices({1})};
  EXPECT_EQ(connected_vertices_0.size(), 9)
      << "This vertex should have 9 connected vertices based on manual "
         "computation";
  EXPECT_EQ(connected_vertices_1.size(), 7)
      << "This vertex should have 7 connected vertices based on manual "
         "computation";
  std::vector<unsigned int> intersection;
  std::set_intersection(
      connected_vertices_0.begin(), connected_vertices_0.end(),
      connected_vertices_1.begin(), connected_vertices_1.end(),
      std::back_inserter(intersection));
  // check that the intersection is the same as the connected vertices to
  // vertices {0,1}
  const auto connected_vertices_0_1 {GetConnectedVertices({0, 1})};
  EXPECT_EQ(connected_vertices_0_1.size(), 5)
      << "This intersection should have 5 elements based on manual computation";
  EXPECT_EQ(intersection, connected_vertices_0_1);
}

TEST_F(ThunderPlannerRoadmapTest, TestLargestCliqueContainingEdge) {
  // get the maximal clique containing edge
  // (source_vertex_index,target_vertex_index)
  unsigned int source_vertex_index {0};
  unsigned int target_vertex_index {1};
  const auto maximal_clique {thunder_planner_->GetLargestCliqueContainingEdge(
      source_vertex_index, target_vertex_index)};
  // check that the maximal clique is a subset of the connected vertices to
  // vertices source_vertex_index and target_vertex_index
  const auto connected_vertices_source_target {
      GetConnectedVertices({source_vertex_index, target_vertex_index})};
  for (const auto vertex : maximal_clique) {
    EXPECT_TRUE(std::find(connected_vertices_source_target.begin(),
                          connected_vertices_source_target.end(), vertex)
                != connected_vertices_source_target.end());
  }
}

}  // namespace ompl
}  // namespace planning
}  // namespace motion
