#include "planning_service/motion/splining/internal/path_math.h"

#include <drake/solvers/constraint.h>
#include <drake/solvers/cost.h>
#include <drake/solvers/mathematical_program.h>
#include <drake/solvers/solve.h>

namespace motion {
namespace splining {
namespace internal {

double FindBestMatchTime(const Eigen::VectorXd& q,
                         const drake::trajectories::Trajectory<double>& path,
                         double start_time, double end_time,
                         double search_step_size) {
  DRAKE_THROW_UNLESS(search_step_size > 0.0);
  DRAKE_THROW_UNLESS(q.rows() == path.rows());
  DRAKE_THROW_UNLESS(path.start_time() <= start_time);
  DRAKE_THROW_UNLESS(path.end_time() >= end_time);
  // Check when on path we get closest to q
  double best_time = start_time;
  double best_distance = std::numeric_limits<double>::infinity();
  for (double t = start_time; t <= end_time; t += search_step_size) {
    // Find the closest time on the path
    double distance = (path.value(t) - q).norm();
    if (distance < best_distance) {
      best_distance = distance;
      best_time = t;
    }
  }
  return best_time;
}

std::vector<double> FindBestMatchTimes(
    const std::vector<Eigen::VectorXd>& waypts,
    const drake::trajectories::Trajectory<double>& path, double start_time,
    double end_time, double minimum_time_difference, double search_step_size) {
  // Now we need to find the best time for each waypoint
  std::vector<double> best_times;
  best_times.reserve(std::ssize(waypts));
  for (int i = 0; i < std::ssize(waypts); ++i) {
    double start_time_i =
        i == 0 ? start_time : best_times[i - 1] + minimum_time_difference;
    double end_time_i =
        end_time - (std::ssize(waypts) - i - 1) * minimum_time_difference;
    best_times.push_back(FindBestMatchTime(waypts[i], path, start_time_i,
                                           end_time_i, search_step_size));
  }
  // print the best times
  logging::log()->info(
      "FindBestMatchTimes: path parameter start: {}, end: {}, minimum_spacing: "
      "{}, search_step_size: {}",
      start_time, end_time, minimum_time_difference, search_step_size);
  for (int i = 0; i < std::ssize(best_times); ++i) {
    logging::log()->info(
        "Best path parameter for waypoint {}/{} is {}. "
        "Shifting to \nnew:      ({}) from \noriginal: ({}).",
        i, std::ssize(best_times), best_times[i], waypts[i].transpose(),
        path.value(best_times[i]).transpose());
  }
  return best_times;
}

/** Calculates a path that passes through the given waypoints.
 *
 * @param sample_times The times at which the waypoints are sampled.
 * @param sample_values The values of the waypoints.
 * @param start_tangent The tangent at the start of the path.
 *
 * @return A cubic PiecewisePolynomial trajectory passing through the waypoints.
 */
drake::trajectories::PiecewisePolynomial<double> CalcCubicPath(
    const std::vector<double>& sample_times,
    const std::vector<Eigen::MatrixXd>& sample_values,
    const Eigen::MatrixXd& start_tangent) {
  DRAKE_THROW_UNLESS(sample_times.size() == sample_values.size());
  int n_samples = std::ssize(sample_times);
  Eigen::MatrixXd end_tangent =
      (sample_values[n_samples - 1] - sample_values[n_samples - 2])
      / (sample_times[n_samples - 1] - sample_times[n_samples - 2]);
  return drake::trajectories::PiecewisePolynomial<
      double>::CubicWithContinuousSecondDerivatives(sample_times, sample_values,
                                                    start_tangent, end_tangent);
}

std::vector<Eigen::VectorXd> SmoothWaypoints(
    const std::vector<double>& sample_times,
    const std::vector<Eigen::VectorXd>& sample_values,
    const std::optional<Eigen::VectorXd>& maybe_start_tangent,
    const Eigen::VectorXd& epsilon) {
  DRAKE_THROW_UNLESS(sample_times.size() == sample_values.size());
  int n_samples = std::ssize(sample_times);
  int dim = sample_values[0].size();
  DRAKE_THROW_UNLESS(epsilon.size() == dim);
  logging::log()->info(
      "Smoothing waypoints with {} samples of dimension {} and epsilon: {}, ",
      n_samples, dim, epsilon.transpose());
  // Solve a mathematical program such that minimize the sum of the squared
  // finite-difference approximations of the second derivatives.
  auto prog = drake::solvers::MathematicalProgram();
  std::vector<drake::solvers::VectorXDecisionVariable> e_vars;
  for (int i = 0; i < n_samples; ++i) {
    // Add the decision variables
    e_vars.push_back(
        prog.NewContinuousVariables(dim, "e_" + std::to_string(i)));
    // Bound e between -epsilon and epsilon if i> 0
    if (i > 0) {
      prog.AddBoundingBoxConstraint(-epsilon, epsilon, e_vars[i]);
    } else if (i == 0) {
      // If i == 0, then e_0 = 0
      prog.AddBoundingBoxConstraint(Eigen::VectorXd::Zero(dim),
                                    Eigen::VectorXd::Zero(dim), e_vars[i]);
    }
  }
  // Let's add the quadratic cost: acceleration squared
  std::vector<drake::solvers::Binding<drake::solvers::QuadraticCost>>
      quad_bindings;
  for (int i = 0; i < n_samples - 2; ++i) {
    for (int j = 0; j < dim; ++j) {
      auto diff_2_e = (sample_values[i + 2](j) - sample_values[i + 1](j)
                       + e_vars[i + 2](j) - e_vars[i + 1](j))
                          / (sample_times[i + 2] - sample_times[i + 1])
                      - (sample_values[i + 1](j) - sample_values[i](j)
                         + e_vars[i + 1](j) - e_vars[i](j))
                            / (sample_times[i + 1] - sample_times[i]);
      auto binding = prog.AddQuadraticCost(diff_2_e * diff_2_e);
      quad_bindings.push_back(binding);
    }
  }
  if (maybe_start_tangent.has_value()) {
    const auto& start_tangent = maybe_start_tangent.value();
    DRAKE_THROW_UNLESS(start_tangent.size() == dim);
    logging::log()->info("Smoothing waypoints: considering start_tangent: {}",
                         start_tangent.transpose());
    // Also add some cost to the first derivative: e_1 + sample_values[1] -
    // sample_values[0] must be as close as to the start_tangent as possible
    for (int j = 0; j < dim; ++j) {
      auto diff_1_0 = (sample_values[1](j) + e_vars[1](j) - sample_values[0](j))
                          / (sample_times[1] - sample_times[0])
                      - start_tangent(j);
      const double weight = 1e3;  // ToDO: make this a user-defined parameter
      auto binding = prog.AddQuadraticCost(weight * diff_1_0 * diff_1_0);
      quad_bindings.push_back(binding);
    }
  }
  logging::log()->info("Smoothing program created successfully");
  // Now let's solve the program
  auto result = drake::solvers::Solve(prog);
  if (!result.is_success()) {
    throw std::runtime_error("Failed to solve the smoothing program");
  }
  // Evaluate the binding costs at zero
  double zero_smoothness_cost = 0.0;
  Eigen::VectorXd evaluated_cost = Eigen::VectorXd::Zero(1);
  for (const auto& binding : quad_bindings) {
    binding.evaluator()->Eval(
        Eigen::VectorXd::Zero(binding.evaluator()->num_vars()),
        &evaluated_cost);
    zero_smoothness_cost += evaluated_cost(0);
  }
  logging::log()->info(
      "Smoothing program solved successfully to bring smoothness cost to {} "
      "from {}",
      result.get_optimal_cost(), zero_smoothness_cost);
  // Now let's get the values of the decision variables
  std::vector<Eigen::VectorXd> smoothed_values;
  for (int i = 0; i < n_samples; ++i) {
    smoothed_values.push_back(sample_values[i] + result.GetSolution(e_vars[i]));
    logging::log()->debug(
        "Smoothing waypoint {}: original value: {}, smoothed value: {}", i,
        sample_values[i].transpose(), smoothed_values[i].transpose());
  }
  return smoothed_values;
}

drake::trajectories::PiecewisePolynomial<double> CalcTrailingPathTowardWaypts(
    std::vector<Eigen::VectorXd> waypts,
    const drake::trajectories::Trajectory<double>& path, double time_now,
    double delta_switch, double minimum_spacing, double search_step_size,
    bool smoothing, const Eigen::VectorXd& smoothing_epsilon) {
  auto [tail_sample_times, tail_sample_values, switch_tangent] =
      CalcSamplesAndTangentFromNewWaypts(waypts, path, time_now, delta_switch,
                                         minimum_spacing, search_step_size,
                                         smoothing, smoothing_epsilon);
  std::vector<Eigen::MatrixXd> tail_sample_values_matrix;
  tail_sample_values_matrix.reserve(std::ssize(tail_sample_values));
  for (const auto& value : tail_sample_values) {
    tail_sample_values_matrix.push_back(value);
  }
  return CalcCubicPath(tail_sample_times, tail_sample_values_matrix,
                       switch_tangent);
}

std::tuple<std::vector<double>, std::vector<Eigen::VectorXd>, Eigen::VectorXd>
CalcSamplesAndTangentFromNewWaypts(
    std::vector<Eigen::VectorXd> waypts,
    const drake::trajectories::Trajectory<double>& path, double time_now,
    double delta_switch, double minimum_spacing, double search_step_size,
    bool smoothing, const Eigen::VectorXd& smoothing_epsilon) {
  DRAKE_THROW_UNLESS(delta_switch > 0.0);
  auto best_times = FindBestMatchTimes(waypts, path, time_now, path.end_time(),
                                       minimum_spacing, search_step_size);
  // Now get those times and samples lined up for splining starting at
  // time_now+delta_switch
  std::vector<double> tail_sample_times;
  std::vector<Eigen::VectorXd> tail_sample_values;
  // First, the value at time_now + delta_switch and its time
  tail_sample_times.push_back(time_now + delta_switch);
  tail_sample_values.push_back(path.value(time_now + delta_switch));
  // Next, the values at the best times that are AFTER time_now + delta_switch
  for (int i = 0; i < std::ssize(waypts); ++i) {
    if (best_times[i] > time_now + delta_switch + minimum_spacing / 2.0) {
      tail_sample_times.push_back(best_times[i]);
      tail_sample_values.push_back(waypts[i]);
    } else {
      logging::log()->warn(
          "Ignoring waypoint ({}) at time {} because it is too close to the "
          "switch time {}",
          waypts[i].transpose(), best_times[i], time_now + delta_switch);
    }
  }
  // Now we can spline with the tail
  auto switch_tangent = path.EvalDerivative(time_now + delta_switch, 1);
  if (!smoothing) {
    return std::make_tuple(tail_sample_times, tail_sample_values,
                           switch_tangent);
  }
  auto tail_sample_values_smoothed = SmoothWaypoints(
      tail_sample_times, tail_sample_values, switch_tangent, smoothing_epsilon);
  // log the smoothed trailing path
  for (int i = 0; i < std::ssize(tail_sample_times); ++i) {
    logging::log()->info(
        "Trailing path smoothed: time: {}, original value: {}, smoothed value: "
        "{}",
        tail_sample_times[i], tail_sample_values[i].transpose(),
        tail_sample_values_smoothed[i].transpose());
  }
  return std::make_tuple(tail_sample_times, tail_sample_values_smoothed,
                         switch_tangent);
}

drake::trajectories::PiecewisePolynomial<double> MergeTrajectory(
    const drake::trajectories::PiecewisePolynomial<double>& original_traj,
    double time_now,
    const drake::trajectories::PiecewisePolynomial<double>& other_traj) {
  double time_switch = other_traj.start_time();
  // DRAKE_THROW_UNLESS(time_switch >= time_now);
  if (time_switch < time_now) {
    auto msg = fmt::format(
        "MergeTrajectory: time_switch ({}) is less than time_now ({}). "
        "Cannot merge trajectories.",
        time_switch, time_now);
    throw std::runtime_error(msg);
  }
  auto result = original_traj.SliceByTime(time_now, time_switch);
  result.ConcatenateInTime(other_traj);
  return result;
}

drake::trajectories::PathParameterizedTrajectory<double> MergeTrajectory(
    const drake::trajectories::PathParameterizedTrajectory<double>&
        original_traj,
    double time_now,
    const drake::trajectories::PathParameterizedTrajectory<double>&
        other_traj) {
  const auto& original_path = original_traj.path();
  // Cast the original_path to a piecewise polynomial
  const auto* original_path_poly =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &original_path);
  DRAKE_THROW_UNLESS(original_path_poly != nullptr);
  // Cast the other_path to a piecewise polynomial
  const auto& other_path = other_traj.path();
  const auto* other_path_poly =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &other_path);
  DRAKE_THROW_UNLESS(other_path_poly != nullptr);
  const auto& original_time_scaling = original_traj.time_scaling();
  const auto* original_time_scaling_poly =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &original_time_scaling);
  DRAKE_THROW_UNLESS(original_time_scaling_poly != nullptr);
  const auto& other_time_scaling = other_traj.time_scaling();
  const auto* other_time_scaling_poly =
      dynamic_cast<const drake::trajectories::PiecewisePolynomial<double>*>(
          &other_time_scaling);
  DRAKE_THROW_UNLESS(other_time_scaling_poly != nullptr);
  // Merge the paths
  // Let's first merge the time scaling
  logging::log()->info(
      "Merging trajectories at time_now: {}, original_traj start: {}, "
      "other_traj start: {}",
      time_now, original_time_scaling_poly->start_time(),
      other_time_scaling_poly->start_time());
  auto merged_time_scaling = MergeTrajectory(
      *original_time_scaling_poly, time_now, *other_time_scaling_poly);
  // Now, let's merge the paths
  double s_now = merged_time_scaling.value(time_now)(0, 0);
  logging::log()->info(
      "Merging trajectories at s_now: {}, original_traj start: {}, "
      "other_traj start: {}",
      s_now, original_path_poly->start_time(), other_path_poly->start_time());
  auto merged_path =
      MergeTrajectory(*original_path_poly, s_now, *other_path_poly);
  // Return the merged trajectory
  return drake::trajectories::PathParameterizedTrajectory<double>(
      merged_path, merged_time_scaling);
}

namespace {
int factorial(int n) {
  DRAKE_THROW_UNLESS(n >= 0);
  if (n == 0) {
    return 1;
  }
  return n * factorial(n - 1);
}
}  // namespace

drake::trajectories::PiecewisePolynomial<double>
CompositeBezierCurveToPiecewisePolynomial(
    const drake::trajectories::CompositeTrajectory<double>&
        composite_bezier_curve) {
  // Get the control points of the Bezier curve
  // to the power form.
  std::vector<
      drake::trajectories::PiecewisePolynomial<double>::PolynomialMatrix>
      poly_matrix_vec;
  std::vector<double> breaks;
  double t = composite_bezier_curve.start_time();
  breaks.push_back(t);
  int num_segments = composite_bezier_curve.get_number_of_segments();
  for (int i = 0; i < num_segments; ++i) {
    double duration = composite_bezier_curve.duration(i);
    // Cast trajectory as BezierCurve
    const auto* bezier_curve =
        dynamic_cast<const drake::trajectories::BezierCurve<double>*>(
            &(composite_bezier_curve.segment(i)));
    if (!bezier_curve) {
      auto msg = fmt::format(
          "CompositeBezierCurveToPiecewisePolynomial: segment {} is not a "
          "BezierCurve",
          i);
      throw std::runtime_error(msg);
    }
    int order = bezier_curve->order();
    const auto& control_points = bezier_curve->control_points();
    int n = control_points.rows();
    Eigen::MatrixXd coefficients = Eigen::MatrixXd::Zero(n, order + 1);
    for (int j = 0; j < order + 1; ++j) {
      for (int k = 0; k < j + 1; ++k) {
        double coeff = factorial(order) / factorial(order - j);
        coeff *= std::pow(-1, k + j) / factorial(k) / factorial(j - k);
        coeff /= std::pow(duration, j);
        coefficients.col(j) += control_points.col(k) * coeff;
      }
    }
    Eigen::MatrixX<drake::Polynomiald> poly_matrix(n, 1);
    for (int j = 0; j < n; ++j) {
      poly_matrix(j, 0) = drake::Polynomiald(coefficients.row(j));
    }
    t += duration;
    breaks.push_back(t);
    poly_matrix_vec.push_back(poly_matrix);
  }
  return drake::trajectories::PiecewisePolynomial<double>(poly_matrix_vec,
                                                          breaks);
}

std::pair<drake::trajectories::PiecewisePolynomial<double>,
          drake::trajectories::PiecewisePolynomial<double>>
CombineSequentialSystemTimedTrajectories(
    const std::vector<
        std::pair<drake::trajectories::PiecewisePolynomial<double>,
                  drake::trajectories::PiecewisePolynomial<double>>>&
        system_timed_trajectories,
    double continuity_tolerance) {
  drake::trajectories::PiecewisePolynomial<double> result_pp_path;
  drake::trajectories::PiecewisePolynomial<double> result_time_parameterization;
  for (int i = 0; i < std::ssize(system_timed_trajectories); ++i) {
    auto [pp_path, time_parameterization] = system_timed_trajectories[i];
    DRAKE_THROW_UNLESS(time_parameterization.rows() == 1);
    DRAKE_THROW_UNLESS(time_parameterization.cols() == 1);
    // if result_pp_path is empty, current_s_value = 0
    if (result_pp_path.empty() || pp_path.empty()) {
      result_pp_path = pp_path;
      result_time_parameterization = time_parameterization;
      continue;
    }
    double current_s_value = result_pp_path.end_time();
    double current_t_value = result_time_parameterization.end_time();
    // Check continuity
    if (!result_pp_path.empty() && !pp_path.empty()) {
      auto last_result_conf = result_pp_path.value(result_pp_path.end_time());
      auto first_result_conf = pp_path.value(pp_path.start_time());
      if ((last_result_conf - first_result_conf).norm()
          > continuity_tolerance) {
        auto msg = fmt::format(
            "CombineSequentialSystemTimedTrajectories: The "
            "trajectories are not continuous. The difference between the last "
            "segment of the first trajectory and the first segment of the "
            "second trajectory is greater than the continuity tolerance. "
            "The difference is {}, larger than tolerance {}.",
            (last_result_conf - first_result_conf).norm(),
            continuity_tolerance);
        logging::log()->error(msg);
        throw std::runtime_error(msg);
      }
      double shift_s = current_s_value - pp_path.start_time();
      double shift_t = current_t_value - time_parameterization.start_time();
      pp_path.shiftRight(shift_s);
      time_parameterization.shiftRight(shift_t);
      time_parameterization += Eigen::MatrixXd::Ones(1, 1) * shift_s;
      // Snap times to suppress epsilon error from Drake - until Drake fixes its
      // arbitrary inner epsilon
      double my_epsilon = 1e-12;
      if (std::abs(pp_path.start_time() - current_s_value) < my_epsilon) {
        pp_path.shiftRight(current_s_value - pp_path.start_time());
      }
      if (std::abs(time_parameterization.start_time() - current_t_value)
          < my_epsilon) {
        time_parameterization.shiftRight(current_t_value
                                         - time_parameterization.start_time());
      }
      // And log what is coming
      // Double check if s(t)'s are continuous
      auto s_t_last = result_time_parameterization.value(current_t_value);
      auto s_t_next =
          time_parameterization.value(time_parameterization.start_time());
      // Concatenate the two trajectories
      result_pp_path.ConcatenateInTime(pp_path);
      result_time_parameterization.ConcatenateInTime(time_parameterization);
    }
  }
  return std::make_pair(result_pp_path, result_time_parameterization);
}

drake::trajectories::PiecewisePolynomial<double> MakeUniformTimingForPath(
    const drake::trajectories::PiecewisePolynomial<double>& path,
    double t_start) {
  std::vector<double> times {t_start,
                             path.end_time() - path.start_time() + t_start};
  std::vector<Eigen::MatrixXd> s_samples;
  s_samples.reserve(2);
  Eigen::MatrixXd sample_start(1, 1), sample_end(1, 1);
  sample_start(0, 0) = path.start_time();
  sample_end(0, 0) = path.end_time();
  s_samples.push_back(sample_start);
  s_samples.push_back(sample_end);
  return drake::trajectories::PiecewisePolynomial<double>::FirstOrderHold(
      times, s_samples);
}

drake::trajectories::PiecewisePolynomial<double> RemoveConstantPrepend(
    const drake::trajectories::PiecewisePolynomial<double>& pp) {
  int num_segments = pp.get_number_of_segments();
  for (int i = 0; i < num_segments; ++i) {
    const auto& poly_matrix = pp.getPolynomialMatrix(i);
    bool segment_is_constant = true;
    for (int row = 0; row < poly_matrix.rows() && segment_is_constant; ++row) {
      for (int col = 0; col < poly_matrix.cols() && segment_is_constant;
           ++col) {
        const auto& coeffs = poly_matrix(row, col).GetCoefficients();
        for (int k = 1; k < coeffs.size(); ++k) {
          if (std::abs(coeffs(k)) > 1e-10) {
            segment_is_constant = false;
            break;
          }
        }
      }
    }
    if (!segment_is_constant) {
      return pp.SliceByTime(pp.get_segment_times()[i], pp.end_time());
    }
  }
  // All segments are constant (or pp is empty): return the original.
  return pp;
}

}  // namespace internal
}  // namespace splining
}  // namespace motion
