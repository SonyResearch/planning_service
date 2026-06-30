

#include <gtest/gtest.h>

#include "planning_service/motion/planning/informed_rrt_star.h"
#include "planning_service/motion/splining/cubic_spliner.h"

namespace motion {
namespace splining {

namespace {
void VerifyAllValid(const std::vector<uint8_t>& are_states_valid_vec,
                    const std::string& msg) {
  for (const uint8_t is_valid : are_states_valid_vec) {
    EXPECT_EQ(is_valid, 1) << msg;
  }
}

void VerifySomeInvalid(const std::vector<uint8_t>& are_states_valid_vec,
                       const std::string& msg) {
  const auto is_any_state_invalid {std::any_of(are_states_valid_vec.begin(),
                                               are_states_valid_vec.end(),
                                               [](const uint8_t is_valid) {
                                                 return is_valid == 0;
                                               })};
  EXPECT_TRUE(is_any_state_invalid) << msg;
}
}  // namespace

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

/** Stub class exposing protected methods to the test fixture. */
class CubicSplinerStub : public CubicSpliner {
 public:
  CubicSplinerStub(const RobotConstraints& robot_constraints)
      : CubicSpliner(robot_constraints) {}
  FRIEND_TEST(TestCubicSplinerPrismatic, PathForValidSpline);
  FRIEND_TEST(TestCubicSplinerPrismatic, PathForInvalidSpline);
  FRIEND_TEST(TestCubicSplinerPrismatic, ConstructCubicPath);
  FRIEND_TEST(TestCubicSplinerPrismatic, ConstructCollidingCubicPath);
  FRIEND_TEST(TestCubicSplinerPrismatic, AddNodesToFixPath);
};

/** Test the spliner on a simple example with prismatic joints. This allows us
 * to have a nice equivalence between joint space and workspace, and hence
 * visualize the splining behavior easily.*/
class TestCubicSplinerPrismatic : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    const std::string xml_file {"planning_service/test_data/package.xml"};
    const std::string dmd_file {
        "planning_service/test_data/2d_prismatic_robot/dmd.yaml"};
    const auto dmd {
        drake::yaml::LoadYamlFile<drake::multibody::parsing::ModelDirectives>(
            dmd_file)};
    robot_model_ = std::make_unique<RobotModel>(xml_file, dmd);

    ConstraintsAdapter constraints_adapter;
    constraints_adapter.plan_name = "test";
    constraints_adapter.collision_checker = CollisionCheckerAdapter {};

    const auto n_threads {std::thread::hardware_concurrency()};
    robot_constraints_ = std::make_unique<RobotConstraints>(
        *robot_model_, constraints_adapter, n_threads);

    cubic_spliner_ = std::make_unique<CubicSplinerStub>(*robot_constraints_);
  }

  const std::vector<Eigen::VectorXd>& path_for_invalid_spline() {
    return path_for_invalid_spline_;
  }

  const std::vector<Eigen::VectorXd>& path_for_valid_spline() {
    return path_for_valid_spline_;
  }

  std::vector<Eigen::VectorXd> CreateInvalidPath() {
    Eigen::MatrixXd waypts(2, 4);
    waypts << 1.0, -0.6424, 0.6425, 0.6425, -0.6425, 0.6425, -1.0, -0.6425;
    return MatrixToEigenVectors(waypts);
  }

  std::vector<Eigen::VectorXd> CreateValidPath() {
    Eigen::MatrixXd waypts(2, 4);
    waypts << 1.0, -0.5, 0.5, 0.5, -0.5, 0.5, -1.0, -0.5;
    return MatrixToEigenVectors(waypts);
  }

  std::vector<Eigen::VectorXd> path_for_invalid_spline_ {CreateInvalidPath()};
  std::vector<Eigen::VectorXd> path_for_valid_spline_ {CreateValidPath()};
  std::unique_ptr<RobotModel> robot_model_;
  std::unique_ptr<RobotConstraints> robot_constraints_;
  std::unique_ptr<CubicSplinerStub> cubic_spliner_;
};

TEST_F(TestCubicSplinerPrismatic, PathForValidSpline) {
  const auto& are_states_valid_vec {
      cubic_spliner_->planning_context()->validity_checker()->AreStatesValid(
          EigenVectorsToMatrix(path_for_valid_spline()), true)};
  VerifyAllValid(are_states_valid_vec, "This path itself should be valid.");
}

TEST_F(TestCubicSplinerPrismatic, PathForInvalidSpline) {
  const auto& are_states_valid_vec {
      cubic_spliner_->planning_context()->validity_checker()->AreStatesValid(
          EigenVectorsToMatrix(path_for_invalid_spline()), true)};
  VerifyAllValid(are_states_valid_vec,
                 "This path itself should be valid, even "
                 "though the spline is invalid.");
}

TEST_F(TestCubicSplinerPrismatic, ConstructCubicPath) {
  const auto valid_cubic_spline {cubic_spliner_->ConstructCubicPath(
      EigenVectorsToMatrix(path_for_valid_spline()))};

  // check that the cubic path is valid
  const auto s_sample {
      Eigen::VectorXd::LinSpaced(20, 0., valid_cubic_spline.end_time())};
  const auto q_sample {valid_cubic_spline.vector_values(s_sample)};
  const auto& are_states_valid_vec {
      cubic_spliner_->planning_context()->validity_checker()->AreStatesValid(
          q_sample)};
  VerifyAllValid(are_states_valid_vec, "This spline should be valid.");
}

TEST_F(TestCubicSplinerPrismatic, ConstructCollidingCubicPath) {
  const auto& invalid_cubic_spline {cubic_spliner_->ConstructCubicPath(
      EigenVectorsToMatrix(path_for_invalid_spline()))};

  // check that the cubic path is invalid
  const auto s_sample {
      Eigen::VectorXd::LinSpaced(20, 0., invalid_cubic_spline.end_time())};
  const auto q_sample {invalid_cubic_spline.vector_values(s_sample)};
  const auto& are_states_valid_vec {
      cubic_spliner_->planning_context()->validity_checker()->AreStatesValid(
          q_sample)};
  VerifySomeInvalid(are_states_valid_vec, "This spline should be invalid.");
}

TEST_F(TestCubicSplinerPrismatic, AddNodesToFixPath) {
  const auto& invalid_cubic_spline {cubic_spliner_->ConstructCubicPath(
      EigenVectorsToMatrix(path_for_invalid_spline()))};

  // check that the cubic path is invalid
  const auto s_sample {
      Eigen::VectorXd::LinSpaced(20, 0., invalid_cubic_spline.end_time())};
  const auto q_sample {invalid_cubic_spline.vector_values(s_sample)};
  const auto& are_states_valid_vec {
      cubic_spliner_->planning_context()->validity_checker()->AreStatesValid(
          q_sample)};

  const auto& waypoints_iteration_1 {cubic_spliner_->AddNodesToFixPath(
      EigenVectorsToMatrix(path_for_invalid_spline()), q_sample, s_sample,
      are_states_valid_vec)};

  // check that the path from the new waypoints is valid
  const auto valid_cubic_spline {
      cubic_spliner_->ConstructCubicPath(waypoints_iteration_1)};

  const auto s_sample_valid_spline {
      Eigen::VectorXd::LinSpaced(20, 0., valid_cubic_spline.end_time())};
  const auto q_sample_valid_spline {
      valid_cubic_spline.vector_values(s_sample_valid_spline)};
  const auto& are_states_valid_vec_valid_spline {
      cubic_spliner_->planning_context()->validity_checker()->AreStatesValid(
          q_sample_valid_spline)};
  VerifyAllValid(are_states_valid_vec_valid_spline,
                 "After 1 iteration, the spline should be valid.");
}

TEST_F(TestCubicSplinerPrismatic, WayptsToValidPath) {
  // check that WayptsToValidPath returns a valid path
  CubicSpliningParameters params;
  const auto valid_spline_and_path_opt {cubic_spliner_->WayptsToValidPath(
      EigenVectorsToMatrix(path_for_invalid_spline()), params)};
  EXPECT_TRUE(valid_spline_and_path_opt.has_value())
      << "WayptsToValidPath should not return null";
  const auto& valid_spline_and_path {valid_spline_and_path_opt.value()};
  const auto& valid_spline {valid_spline_and_path.first};

  // s_sample and q_sample
  const auto s_sample_valid_spline {
      Eigen::VectorXd::LinSpaced(20, 0., valid_spline.end_time())};
  const auto q_sample_valid_spline {
      valid_spline.vector_values(s_sample_valid_spline)};
  const auto& are_states_valid_vec_valid_spline {
      cubic_spliner_->planning_context()->validity_checker()->AreStatesValid(
          q_sample_valid_spline)};
  VerifyAllValid(are_states_valid_vec_valid_spline,
                 "The spline should be valid.");
}

}  // namespace splining
}  // namespace motion
