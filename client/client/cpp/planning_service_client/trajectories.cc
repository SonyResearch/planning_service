
#include "planning_service_client/trajectories.h"

#include <cmath>
#include <iostream>
#include <limits>

#include "planning_service_client/internal/client_throw.h"
#include "planning_service_client/internal/eigen_utils.h"
namespace planning_service_client {

PiecewisePolynomial::PiecewisePolynomial(
    const std::vector<Eigen::MatrixXd>& coefficients_vec,
    const std::vector<double>& breaks)
    : coefficients_vec_ {coefficients_vec},
      breaks_ {breaks},
      dim_ {coefficients_vec_.front().rows()} {
  CLIENT_THROW_UNLESS(coefficients_vec_.size() == breaks.size() - 1);
  degree_ = static_cast<int>(coefficients_vec_.front().cols()) - 1;
  for (size_t i {1}; i < coefficients_vec_.size(); ++i) {
    CLIENT_THROW_UNLESS(coefficients_vec_[i].rows() == dim_);
    degree_ =
        std::max(degree_, static_cast<int>(coefficients_vec_[i].cols()) - 1);
  }
  for (size_t i {1}; i < breaks.size(); ++i) {
    CLIENT_THROW_UNLESS(breaks[i] > breaks[i - 1]);
  }
}

Eigen::VectorXd PiecewisePolynomial::Value(double t) const {
  int segment = FindSegment(t);
  double t_clamped = std::clamp(t, start_time(), end_time());
  double t_segment = t_clamped - breaks_[segment];
  Eigen::VectorXd result = coefficients_vec_[segment].col(0);
  for (int i {1}; i < coefficients_vec_[segment].cols(); ++i) {
    result += coefficients_vec_[segment].col(i) * std::pow(t_segment, i);
  }
  return result;
}

Eigen::VectorXd PiecewisePolynomial::EvalDerivative(double t) const {
  int segment = FindSegment(t);
  double t_clamped = std::clamp(t, start_time(), end_time());
  double t_segment = t_clamped - breaks_[segment];
  if (coefficients_vec_[segment].cols() == 1) {
    return Eigen::VectorXd::Zero(dim_);
  }
  Eigen::VectorXd result = coefficients_vec_[segment].col(1);
  for (int i {2}; i < coefficients_vec_[segment].cols(); ++i) {
    result +=
        coefficients_vec_[segment].col(i) * i * std::pow(t_segment, i - 1);
  }
  return result;
}

PiecewisePolynomial PiecewisePolynomial::Derivative() const {
  std::vector<Eigen::MatrixXd> derivative_coefficients_vec;
  for (size_t i {0}; i < coefficients_vec_.size(); ++i) {
    if (coefficients_vec_[i].cols() == 1) {
      derivative_coefficients_vec.push_back(Eigen::MatrixXd::Zero(dim_, 1));
      continue;
    }
    Eigen::MatrixXd derivative_coefficients =
        Eigen::MatrixXd::Zero(dim_, coefficients_vec_[i].cols() - 1);
    for (long int j {1}; j < coefficients_vec_[i].cols(); j++) {
      derivative_coefficients.col(j - 1) = coefficients_vec_[i].col(j) * j;
    }
    derivative_coefficients_vec.push_back(derivative_coefficients);
  }
  return PiecewisePolynomial(derivative_coefficients_vec, breaks_);
}

bool PiecewisePolynomial::IsCloseTo(const PiecewisePolynomial& other,
                                    double tol) const {
  if (dim_ != other.dim()) {
    return false;
  }
  if (degree_ != other.degree()) {
    return false;
  }
  if (breaks_.size() != other.breaks().size()) {
    return false;
  }
  for (size_t i {0}; i < breaks_.size(); ++i) {
    if (std::abs(breaks_[i] - other.breaks()[i]) > tol) {
      return false;
    }
  }
  for (size_t i {0}; i < coefficients_vec_.size(); ++i) {
    if (!coefficients_vec_[i].isApprox(other.coefficients_vec_[i], tol)) {
      return false;
    }
  }
  return true;
}

double PiecewisePolynomial::duration() const {
  return breaks_.back() - breaks_.front();
}

proto::ClientPiecewisePolynomial PiecewisePolynomial::ToProtoImpl() const {
  proto::ClientPiecewisePolynomial poly_pb;
  std::vector<Eigen::VectorXd> coeffs_vec;
  for (size_t i {0}; i < breaks_.size() - 1; ++i) {
    for (long int j {0}; j < dim_; j++) {
      coeffs_vec.push_back(coefficients_vec_[i].row(j));
    }
  }
  *poly_pb.mutable_breaks() = {breaks_.begin(), breaks_.end()};
  poly_pb.set_dim(dim_);
  // each element is a matrix of coeff vectors
  for (const auto& coeffs : coeffs_vec) {
    // ptr to new vector of coeffs
    proto::Coeffs* coeffs_pb {poly_pb.add_coeffs()};
    auto vec {internal::e_to_v(coeffs)};
    *coeffs_pb->mutable_data() = {vec.begin(), vec.end()};
  }
  return poly_pb;
}

void PiecewisePolynomial::FromProtoImpl(
    const proto::ClientPiecewisePolynomial& msg) {
  dim_ = msg.dim();
  coefficients_vec_.clear();
  breaks_ = std::vector<double>(msg.breaks().begin(), msg.breaks().end());
  const std::vector<proto::Coeffs> coeffs_vec {msg.coeffs().begin(),
                                               msg.coeffs().end()};
  CLIENT_THROW_UNLESS(std::ssize(coeffs_vec)
                      == (std::ssize(breaks_) - 1) * dim_);
  degree_ = 0;
  for (int i {0}; i < std::ssize(breaks_) - 1; i++) {
    const auto& coeffs = coeffs_vec[i * dim_];
    Eigen::VectorXd coeffs_eigen = internal::RepeatedToEigen(coeffs.data());
    int degree = static_cast<int>(coeffs_eigen.size()) - 1;
    degree_ = std::max(degree_, degree);
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(dim_, degree + 1);
    for (long int j {0}; j < dim_; j++) {
      const auto& coeffs = coeffs_vec[i * dim_ + j];
      Eigen::VectorXd coeffs_eigen = internal::RepeatedToEigen(coeffs.data());
      matrix.row(j) = coeffs_eigen;
    }
    coefficients_vec_.push_back(matrix);
  }
}

int PiecewisePolynomial::FindSegment(double t) const {
  if (t < start_time()) {
    return 0;
  }
  if (t > end_time()) {
    return std::ssize(breaks_) - 2;
  }
  auto t_clamped = std::clamp(t, start_time(), end_time());
  return GetRecursiveSegment(t_clamped, 0, std::ssize(breaks_) - 1);
}

int PiecewisePolynomial::GetRecursiveSegment(double t, int start,
                                             int end) const {
  if (end == start + 1) {
    return start;
  }
  int mid = (start + end) / 2;
  if (t < breaks_[mid]) {
    return GetRecursiveSegment(t, start, mid);
  } else if (t > breaks_[mid]) {
    return GetRecursiveSegment(t, mid, end);
  } else
    return mid;
}

PiecewisePolynomial PiecewisePolynomial::Constant(const Eigen::VectorXd& conf,
                                                  double start_time,
                                                  double end_time) {
  Eigen::MatrixXd coeff(conf.size(), 1);
  coeff.col(0) = conf;
  return PiecewisePolynomial({coeff}, {start_time, end_time});
}

void PiecewisePolynomial::ShiftTime(double t_shift) {
  for (auto& t : breaks_) {
    t += t_shift;
  }
}

TimedTrajectory::TimedTrajectory(const PiecewisePolynomial& path,
                                 const PiecewisePolynomial& time_scaling,
                                 double global_time_offset)
    : path_(path),
      time_scaling_(time_scaling),
      global_time_offset_(global_time_offset) {
  if (time_scaling_.dim() != 1) {
    throw std::runtime_error("Time scaling must be a scalar trajectory.");
  }
  // s(time_start) = path_start
  double eps {1e-6};
  auto path_start = path_.start_time();
  auto s_start_value = time_scaling_.Value(time_scaling_.start_time())(0);
  if (!internal::IsApprox(path_start, s_start_value, eps)) {
    std::string msg = "Path start 's' (" + std::to_string(path_start)
                      + ") must match time scaling start 's(start_time)' ("
                      + std::to_string(s_start_value) + ").";
    throw std::runtime_error(msg);
  }
  // s(time_end) = path_end
  auto path_end = path_.end_time();
  auto s_end_value = time_scaling_.Value(time_scaling_.end_time());
  if (!internal::IsApprox(path_end, s_end_value(0), eps)) {
    std::string msg = "Path end 's' (" + std::to_string(path_end)
                      + ") must match time scaling end 's(end_time)' ("
                      + std::to_string(s_end_value(0)) + ").";
    throw std::runtime_error(msg);
  }
}

Eigen::VectorXd TimedTrajectory::GlobalValue(double t_global) const {
  const double t {t_global - global_time_offset_};
  return Value(t);
}

Eigen::VectorXd TimedTrajectory::EvalGlobalDerivative(double t_global) const {
  const double t {t_global - global_time_offset_};
  return EvalDerivative(t);
}

Eigen::VectorXd TimedTrajectory::Value(double t) const {
  double s = time_scaling_.Value(t)(0);
  return path_.Value(s);
}

Eigen::VectorXd TimedTrajectory::EvalDerivative(double t) const {
  double s = time_scaling_.Value(t)(0);
  double ds_dt = time_scaling_.EvalDerivative(t)(0);
  return path_.EvalDerivative(s) * ds_dt;
}

proto::TimedTrajectory TimedTrajectory::ToProtoImpl() const {
  proto::TimedTrajectory timed_traj_pb;
  *timed_traj_pb.mutable_path() = ToProto(path_);
  *timed_traj_pb.mutable_time_scaling() = ToProto(time_scaling_);
  timed_traj_pb.set_global_time_offset(global_time_offset_);
  return timed_traj_pb;
}

void TimedTrajectory::FromProtoImpl(const proto::TimedTrajectory& msg) {
  path_ = FromProto<PiecewisePolynomial>(msg.path());
  time_scaling_ = FromProto<PiecewisePolynomial>(msg.time_scaling());
  global_time_offset_ = msg.global_time_offset();
}

void TimedTrajectory::ShiftTime(double t_shift) {
  time_scaling_.ShiftTime(t_shift);
}

TimedTrajectory TimedTrajectory::Constant(const Eigen::VectorXd& conf,
                                          double start_time, double end_time) {
  auto path = PiecewisePolynomial::Constant(conf, start_time, end_time);
  // want s_t = t;
  std::vector<double> time_breaks {start_time, end_time};
  Eigen::MatrixXd coeff(1, 2);
  coeff << start_time, 1.0;
  auto time_scaling = PiecewisePolynomial({coeff}, time_breaks);
  return TimedTrajectory(path, time_scaling);
}

SystemTimedTrajectory ConstantSystemTimedTrajectory(const SystemConf& conf,
                                                    double start_time,
                                                    double end_time) {
  SystemTimedTrajectory result;
  for (const auto& [key, value] : conf) {
    result[key] = TimedTrajectory::Constant(value.q(), start_time, end_time);
  }
  return result;
}

namespace {
constexpr double kDefaultSliceClampDuration = 1e-3;

// Helper function to compute binomial coefficient C(n, k)
int BinomialCoeff(int n, int k) {
  if (k > n || k < 0) return 0;
  if (k == 0 || k == n) return 1;

  int result = 1;
  for (int i = 1; i <= k; ++i) {
    result = result * (n - i + 1) / i;
  }
  return result;
}

// Shift polynomial coefficients from being relative to old_base to new_base
// Input: coefficients [c0, c1, c2, ...] representing p(t) = sum(c_i * (t -
// old_base)^i) Output: coefficients [d0, d1, d2, ...] representing p(t) =
// sum(d_j * (t - new_base)^j) where new_base > old_base
Eigen::MatrixXd ShiftPolynomialCoefficients(const Eigen::MatrixXd& old_coeffs,
                                            double old_base, double new_base) {
  double delta = new_base - old_base;
  int num_coeffs = old_coeffs.cols();
  int dim = old_coeffs.rows();

  Eigen::MatrixXd new_coeffs = Eigen::MatrixXd::Zero(dim, num_coeffs);

  // For each power j in the new representation
  for (int j = 0; j < num_coeffs; ++j) {
    // Contribution comes from all powers i >= j in the old representation
    // (v + delta)^i = sum_{j=0}^{i} C(i,j) * v^j * delta^(i-j)
    // So coefficient of v^j gets contribution C(i,j) * delta^(i-j) * c_i from
    // each i >= j
    for (int i = j; i < num_coeffs; ++i) {
      double factor = BinomialCoeff(i, j) * std::pow(delta, i - j);
      new_coeffs.col(j) += old_coeffs.col(i) * factor;
    }
  }

  return new_coeffs;
}

PiecewisePolynomial SlicePiecewisePolynomial(const PiecewisePolynomial& poly,
                                             double start, double end) {
  CLIENT_THROW_UNLESS(start < end);
  CLIENT_THROW_UNLESS(start >= poly.start_time());
  CLIENT_THROW_UNLESS(end <= poly.end_time());
  auto PreviousRepresentableValue = [](double value) {
    return std::nextafter(value, -std::numeric_limits<double>::infinity());
  };
  const auto& old_breaks = poly.breaks();
  const auto& old_coeffs = poly.coefficients_vec();
  int start_segment = poly.FindSegment(start);
  // Use the previous representable value before `end` so that if `end` is
  // exactly on a break, we slice up to the segment that ends at that break.
  int end_segment = poly.FindSegment(PreviousRepresentableValue(end));

  std::vector<double> new_breaks;
  std::vector<Eigen::MatrixXd> new_coeffs;
  new_breaks.push_back(start);
  new_coeffs.push_back(ShiftPolynomialCoefficients(
      old_coeffs[start_segment], old_breaks[start_segment], start));
  for (int i = start_segment + 1; i <= end_segment; ++i) {
    new_breaks.push_back(old_breaks[i]);
    new_coeffs.push_back(old_coeffs[i]);
  }
  new_breaks.push_back(end);

  return PiecewisePolynomial(new_coeffs, new_breaks);
}

PiecewisePolynomial MergePiecewisePolynomials(
    const PiecewisePolynomial& left, const PiecewisePolynomial& right) {
  std::vector<double> merged_breaks = left.breaks();
  const auto& right_breaks = right.breaks();
  merged_breaks.insert(merged_breaks.end(), right_breaks.begin() + 1,
                       right_breaks.end());
  std::vector<Eigen::MatrixXd> merged_coeffs = left.coefficients_vec();
  const auto& right_coeffs = right.coefficients_vec();
  merged_coeffs.insert(merged_coeffs.end(), right_coeffs.begin(),
                       right_coeffs.end());
  return PiecewisePolynomial(merged_coeffs, merged_breaks);
}
}  // namespace

TimedTrajectory TimedTrajectory::SliceFromTime(double t, bool clamp_end) const {
  // Validate input
  if (t < time_scaling_.start_time()) {
    throw std::runtime_error(
        "Cannot slice trajectory before its start time. "
        "Requested time: "
        + std::to_string(t) + ", start time: " + std::to_string(start_time()));
  }
  // If t is at or after end time, handle accordingly
  if (t >= time_scaling_.end_time()) {
    if (clamp_end) {
      // Return a constant trajectory at the end value
      Eigen::VectorXd end_value = Value(end_time());
      return TimedTrajectory::Constant(end_value, end_time(),
                                       end_time() + kDefaultSliceClampDuration);
    }
    throw std::runtime_error(
        "Cannot slice trajectory after its end time. "
        "Requested time: "
        + std::to_string(t) + ", end time: " + std::to_string(end_time()));
  }
  // Get the s value at time t
  double s_at_t = time_scaling_.Value(t)(0);
  // Slice the time_scaling polynomial from t to end
  int time_segment = time_scaling_.FindSegment(t);
  std::vector<double> new_time_breaks;
  std::vector<Eigen::MatrixXd> new_time_coeffs;
  // Add the starting time
  new_time_breaks.push_back(t);
  // First segment: from t to the next break
  const auto& old_time_breaks = time_scaling_.breaks();
  const auto& old_time_coeffs = time_scaling_.coefficients_vec();
  Eigen::MatrixXd first_time_coeffs = ShiftPolynomialCoefficients(
      old_time_coeffs[time_segment], old_time_breaks[time_segment], t);
  new_time_coeffs.push_back(first_time_coeffs);
  // Add remaining breaks and coefficients for time_scaling
  for (int i = time_segment + 1; i < std::ssize(old_time_breaks); ++i) {
    new_time_breaks.push_back(old_time_breaks[i]);
  }
  for (int i = time_segment + 1; i < std::ssize(old_time_coeffs); ++i) {
    new_time_coeffs.push_back(old_time_coeffs[i]);
  }
  // Slice the path polynomial from s_at_t to end
  int path_segment = path_.FindSegment(s_at_t);
  std::vector<double> new_path_breaks;
  std::vector<Eigen::MatrixXd> new_path_coeffs;
  // Add the starting s value
  new_path_breaks.push_back(s_at_t);
  // First segment: from s_at_t to the next break
  const auto& old_path_breaks = path_.breaks();
  const auto& old_path_coeffs = path_.coefficients_vec();
  Eigen::MatrixXd first_path_coeffs = ShiftPolynomialCoefficients(
      old_path_coeffs[path_segment], old_path_breaks[path_segment], s_at_t);
  new_path_coeffs.push_back(first_path_coeffs);
  // Add remaining breaks and coefficients for path
  for (int i = path_segment + 1; i < std::ssize(old_path_breaks); ++i) {
    new_path_breaks.push_back(old_path_breaks[i]);
  }
  for (int i = path_segment + 1; i < std::ssize(old_path_coeffs); ++i) {
    new_path_coeffs.push_back(old_path_coeffs[i]);
  }
  // Create new PiecewisePolynomials
  PiecewisePolynomial new_time_scaling(new_time_coeffs, new_time_breaks);
  PiecewisePolynomial new_path(new_path_coeffs, new_path_breaks);
  // Return the new TimedTrajectory
  return TimedTrajectory(new_path, new_time_scaling, global_time_offset_);
}

TimedTrajectory TimedTrajectory::Slice(double t_1, double t_2) const {
  if (t_1 > t_2) {
    throw std::runtime_error(
        "Slice start time must be less than or equal to end time. "
        "Requested slice: ["
        + std::to_string(t_1) + ", " + std::to_string(t_2) + "].");
  }
  if (t_2 - t_1 < kDefaultSliceClampDuration) {
    t_2 = t_1 + kDefaultSliceClampDuration;
  }
  if (t_1 > end_time()) {
    return TimedTrajectory::Constant(Value(end_time()), t_1, t_2);
  }
  // Clamp t_1 and t_2.
  double t_start = std::clamp(t_1, start_time(), end_time());
  double t_end = std::clamp(t_2, start_time(), end_time());
  if (t_start >= t_end) {
    throw std::runtime_error(
        "Slice start time must be before end time after clamping. "
        "Requested slice: ["
        + std::to_string(t_1) + ", " + std::to_string(t_2)
        + "], clamped slice: [" + std::to_string(t_start) + ", "
        + std::to_string(t_end) + "], trajectory time bounds: ["
        + std::to_string(start_time()) + ", " + std::to_string(end_time())
        + "].");
  }

  double s_start = time_scaling_.Value(t_start)(0);
  double s_end = time_scaling_.Value(t_end)(0);
  if (s_start >= s_end) {
    throw std::runtime_error(
        "Slice requires monotonically increasing time scaling over [t_1, "
        "t_2].");
  }

  auto sliced_time_scaling =
      SlicePiecewisePolynomial(time_scaling_, t_start, t_end);
  auto sliced_path = SlicePiecewisePolynomial(path_, s_start, s_end);
  return TimedTrajectory(sliced_path, sliced_time_scaling, global_time_offset_);
}

TimedTrajectory TimedTrajectory::Merge(const TimedTrajectory& traj_1,
                                       const TimedTrajectory& traj_2,
                                       bool check_pos_continuity,
                                       bool check_vel_continuity, double tol) {
  CLIENT_THROW_UNLESS(traj_1.dim() == traj_2.dim());

  if (check_pos_continuity || check_vel_continuity) {
    const auto traj_1_end_pos = traj_1.Value(traj_1.end_time());
    const auto traj_2_start_pos = traj_2.Value(traj_2.start_time());
    if (check_pos_continuity
        && (traj_1_end_pos - traj_2_start_pos).norm() > tol) {
      throw std::runtime_error(
          "Cannot merge trajectories: position discontinuity at merge point.");
    }

    if (check_vel_continuity) {
      const auto traj_1_end_vel = traj_1.EvalDerivative(traj_1.end_time());
      const auto traj_2_start_vel = traj_2.EvalDerivative(traj_2.start_time());
      if ((traj_1_end_vel - traj_2_start_vel).norm() > tol) {
        throw std::runtime_error(
            "Cannot merge trajectories: velocity discontinuity at merge "
            "point.");
      }
    }
  }

  // Shift path domain of traj_2 so it starts where traj_1 path ends.
  double s_shift = traj_1.path().end_time() - traj_2.path().start_time();
  // Shift local time domain of traj_2 so it starts at traj_1 end_time.
  double t_shift = traj_1.end_time() - traj_2.start_time();

  PiecewisePolynomial shifted_path_2 = traj_2.path();
  shifted_path_2.ShiftTime(s_shift);

  PiecewisePolynomial shifted_time_scaling_2 = traj_2.time_scaling();
  shifted_time_scaling_2.ShiftTime(t_shift);
  std::vector<Eigen::MatrixXd> shifted_time_coeffs =
      shifted_time_scaling_2.coefficients_vec();
  for (auto& coeff : shifted_time_coeffs) {
    coeff(0, 0) += s_shift;
  }
  shifted_time_scaling_2 =
      PiecewisePolynomial(shifted_time_coeffs, shifted_time_scaling_2.breaks());

  auto merged_path = MergePiecewisePolynomials(traj_1.path(), shifted_path_2);
  auto merged_time_scaling =
      MergePiecewisePolynomials(traj_1.time_scaling(), shifted_time_scaling_2);

  return TimedTrajectory(merged_path, merged_time_scaling,
                         traj_1.global_time_offset());
}

}  // namespace planning_service_client
