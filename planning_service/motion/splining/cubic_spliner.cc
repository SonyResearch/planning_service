

#include "cubic_spliner.h"

using PPType = drake::trajectories::PiecewisePolynomial<double>;

namespace motion {
namespace splining {

using PPType = drake::trajectories::PiecewisePolynomial<double>;

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

CubicSpliner::CubicSpliner(const RobotConstraints& robot_constraints)
    : planning_context_(
          std::make_unique<planning::ompl::SampleBasedPlanningContext>(
              robot_constraints)) {}

PPType CubicSpliner::ConstructCubicPath(
    const Eigen::MatrixXd& waypts,
    const std::optional<Eigen::VectorXd> forward_tangent_splines) const {
  // Generate CubicSpline of s & waypts
  const double s_max {1.};
  const auto s_waypts {Eigen::VectorXd::LinSpaced(waypts.cols(), 0., s_max)};
  const auto waypt_dot_start {forward_tangent_splines.has_value()
                                  ? forward_tangent_splines.value()
                                  : (waypts.block(0, 1, waypts.rows(), 1)
                                     - waypts.block(0, 0, waypts.rows(), 1))
                                        / s_max * (waypts.cols() - 1)};
  const auto waypt_dot_end {
      (waypts.block(0, waypts.cols() - 1, waypts.rows(), 1)
       - waypts.block(0, waypts.cols() - 2, waypts.rows(), 1))
      / s_max * (waypts.cols() - 1)};
  return PPType::CubicWithContinuousSecondDerivatives(
      s_waypts, waypts, waypt_dot_start, waypt_dot_end);
}

std::optional<std::pair<PPType, std::vector<Eigen::VectorXd>>>
CubicSpliner::WayptsToValidPath(
    const Eigen::MatrixXd& waypts,
    const CubicSpliningParameters splining_parameters) const {
  auto path {
      ConstructCubicPath(waypts, splining_parameters.forward_tangent_splines)};

  Eigen::MatrixXd spline_waypts {waypts};

  std::optional<size_t> successful_iteration {};
  for (int ii {}; ii < splining_parameters.max_resplines; ii++) {
    logging::log()->trace("CubicSpliner:WayptsToValidPath: Resplines {}/{}", ii,
                          splining_parameters.max_resplines);
    // Check if path violates constraints
    const auto s_sample {Eigen::VectorXd::LinSpaced(20, 0., path.end_time())};
    const auto q_sample {path.vector_values(s_sample)};
    static const double limit_tol {1e-5};

    const auto are_states_valid_vec {
        planning_context_->validity_checker()->AreStatesValid(q_sample,
                                                              limit_tol)};
    const bool are_all_states_valid =
        std::all_of(are_states_valid_vec.begin(), are_states_valid_vec.end(),
                    [](uint8_t v) {
                      return v == 1;
                    });
    // Succeed fast: If spline is fully valid
    if (are_all_states_valid) {
      // we're safe
      successful_iteration = ii;
      break;
    }
    logging::log()->debug(
        "CubicSpliner:WayptsToValidPath: Path violates constraints. "
        "Re-estimating.");
    spline_waypts = AddNodesToFixPath(spline_waypts, q_sample, s_sample,
                                      are_states_valid_vec);
    path = ConstructCubicPath(spline_waypts,
                              splining_parameters.forward_tangent_splines);
  }

  if (!successful_iteration) {
    logging::log()->error(
        "CubicSpliner:WayptsToValidPath: failed to construct a cubic spline "
        "which satisfies constraints! Return nullopt.");
    return std::nullopt;
  }
  logging::log()->debug(
      "CubicSpliner:WayptsToValidPath: Returning cubic path on iteration: {}",
      successful_iteration.value());
  const auto modified_state_vec {MatrixToEigenVectors(spline_waypts)};
  logging::log()->debug(
      "CubicSpliner:WayptsToValidPath: returning path: {}, state_vec: {}",
      path.get_segment_times().size(), modified_state_vec.size());
  return std::make_pair(path, modified_state_vec);
}

std::optional<std::pair<drake::trajectories::PiecewisePolynomial<double>,
                        std::vector<Eigen::VectorXd>>>
CubicSpliner::WayptsToValidPath(
    const std::vector<Eigen::VectorXd>& waypts,
    const CubicSpliningParameters splining_parameters) const {
  return WayptsToValidPath(EigenVectorsToMatrix(waypts), splining_parameters);
}

Eigen::MatrixXd CubicSpliner::AddNodesToFixPath(
    const Eigen::MatrixXd& spline_waypts, const Eigen::MatrixXd& q_sample,
    const Eigen::VectorXd& s_sample,
    std::vector<uint8_t> are_states_valid_vec) const {
  std::vector<Eigen::VectorXd> new_waypts {spline_waypts.col(0)};
  size_t s_index {};

  for (size_t knot {1}; knot < static_cast<size_t>(spline_waypts.cols());
       knot++) {
    size_t s_waypt {s_index};
    auto closest_dist {
        (q_sample.col(s_index) - spline_waypts.col(knot)).norm()};
    for (size_t s_test {s_index + 1};
         s_test < static_cast<size_t>(s_sample.size()); s_test++) {
      auto dist {(q_sample.col(s_test) - spline_waypts.col(knot)).norm()};
      if (dist < closest_dist) {
        closest_dist = dist;
        s_waypt = s_test;
      }
    }
    // For each knot point, check if the spline violates constraints between
    // this knot point and the previous one.
    // If it does, add a knot point that is the average of these two knots
    // and insert it in between them.
    size_t check_size {
        std::min(s_waypt - s_index + 2, are_states_valid_vec.size() - s_index)};
    const auto chunk_obeys_constraints {std::all_of(
        are_states_valid_vec.begin() + s_index,
        are_states_valid_vec.begin() + s_index + check_size, [](bool v) {
          return v;
        })};
    if (!chunk_obeys_constraints) {
      new_waypts.push_back(
          (spline_waypts.col(knot - 1) + spline_waypts.col(knot)) / 2.);
    }
    s_index = s_waypt;
    new_waypts.push_back(spline_waypts.col(knot));
  }
  return EigenVectorsToMatrix(new_waypts);
}

}  // namespace splining
}  // namespace motion
