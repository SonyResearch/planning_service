#pragma once

#include <Eigen/Dense>

#include <vector>

#include "planning_service_client/conf.h"
#include "planning_service_client/internal/system_property.h"

namespace planning_service_client {
/** Piecewise Polynomials are defined by a set of polynomials
 *  on different intervals. The polynomials are connected at the
 * breakpoints.
 */
class PiecewisePolynomial
    : public internal::ProtoBase<proto::ClientPiecewisePolynomial> {
 public:
  /**  Default constructor. */
  PiecewisePolynomial() = default;

  /**
   * Constructs a piecewise polynomial from given coefficients and breakpoints.
   * @param coefficients_vec A vector of coefficient matrices for each
   * polynomial segment.
   * @param breaks A vector of breakpoints that define the domain of the
   * piecewise polynomial.
   */
  PiecewisePolynomial(const std::vector<Eigen::MatrixXd>& coefficients_vec,
                      const std::vector<double>& breaks);

  /**
   * Evaluates the spline at a given time.
   * @param t The time at which to evaluate the polynomial.
   * @return The evaluated value as an Eigen::VectorXd.
   */
  Eigen::VectorXd Value(double t) const;

  /**
   * Evaluates the derivative of the spline at a given time.
   * @param t The time at which to evaluate the derivative.
   * @param derivative_order The order of the derivative to compute.
   * @return The evaluated derivative as an Eigen::VectorXd.
   */
  Eigen::VectorXd EvalDerivative(double t) const;

  /**
   * Evaluates the derivative of the spline as a PiecewisePolynomial.
   * @return The evaluated derivative as an Eigen::VectorXd.
   */
  PiecewisePolynomial Derivative() const;

  /**
   * Finds the segment of the polynomial that contains a given time.
   * @param t The time to search for.
   * @return The index of the segment that contains the time.
   */
  int FindSegment(double t) const;

  /**
   * Checks whether this polynomial is close to another within a given
   * tolerance.
   * @param other The other PiecewisePolynomial to compare with.
   * @param tol The tolerance for the comparison.
   * @return True if the polynomials are close within the given tolerance, false
   * otherwise.
   */
  bool IsCloseTo(const PiecewisePolynomial& other, double tol) const;

  /**
   * Gets the dimensionality of the polynomial output.
   * @return The dimension of the polynomial.
   */
  long int dim() const {
    return dim_;
  }

  /**
   * Gets the polynomial degree.
   * @return The degree of the polynomial.
   */
  int degree() const {
    return degree_;
  }

  /* Gets the vector of coefficient matrices.
   * @return A constant reference to the vector of coefficient matrices.
   */
  const std::vector<Eigen::MatrixXd>& coefficients_vec() const {
    return coefficients_vec_;
  }

  /**
   * Gets the breakpoints of the piecewise polynomial.
   * @return A constant reference to the vector of breakpoints.
   */
  const std::vector<double>& breaks() const {
    return breaks_;
  }

  /**
   * Gets the start time of the piecewise polynomial.
   * @return The first breakpoint, representing the start time.
   */
  double start_time() const {
    return breaks_.front();
  }

  /**
   * Gets the end time of the piecewise polynomial.
   * @return The last breakpoint, representing the end time.
   */
  double end_time() const {
    return breaks_.back();
  }

  /**
   * Computes the total duration of the piecewise polynomial.
   * @return The duration, calculated as end_time() - start_time().
   */
  double duration() const;

  /**
   * Shifts all breakpoints by a given time offset.
   * @param t_shift The time offset to shift all breakpoints by.
   */
  void ShiftTime(double t_shift);

  /**
   * Constructs a constant piecewise polynomial with a single value between
   * time 0 and infinity.
   * @param conf The constant configuration.
   */
  static PiecewisePolynomial Constant(
      const Eigen::VectorXd& conf, double start_time = 0.0,
      double end_time = std::numeric_limits<double>::max());

 private:
  /** Converts the PiecewisePolynomial object to a proto message.*/
  proto::ClientPiecewisePolynomial ToProtoImpl() const override;

  /** Replaces the PiecewisePolynomial object with the one in the proto
   * message.*/
  void FromProtoImpl(const proto::ClientPiecewisePolynomial& msg) override;

  int GetRecursiveSegment(double t, int start, int end) const;

  std::vector<Eigen::MatrixXd> coefficients_vec_;
  std::vector<double> breaks_;
  long int dim_ {0};
  int degree_ {0};
};

class TimedTrajectory : public internal::ProtoBase<proto::TimedTrajectory> {
 public:
  TimedTrajectory() = default;

  TimedTrajectory(const PiecewisePolynomial& path,
                  const PiecewisePolynomial& time_scaling,
                  double global_time_offset = 0.0);

  Eigen::VectorXd Value(double t) const;
  Eigen::VectorXd GlobalValue(double t_global) const;

  Eigen::VectorXd EvalDerivative(double t) const;
  Eigen::VectorXd EvalGlobalDerivative(double t_global) const;

  int dim() const {
    return path_.dim();
  }

  double start_time() const {
    return time_scaling_.start_time();
  }

  double global_start_time() const {
    return time_scaling_.start_time() + global_time_offset_;
  }

  double end_time() const {
    return time_scaling_.end_time();
  }

  double global_end_time() const {
    return time_scaling_.end_time() + global_time_offset_;
  }

  double duration() const {
    return time_scaling_.duration();
  }

  bool empty() const {
    return path_.breaks().empty() || time_scaling_.breaks().empty();
  }

  const PiecewisePolynomial& path() const {
    return path_;
  }

  const PiecewisePolynomial& time_scaling() const {
    return time_scaling_;
  }

  double global_time_offset() const {
    return global_time_offset_;
  }

  void SetGlobalTimeOffset(double offset) {
    global_time_offset_ = offset;
  }

  void ShiftGlobalTimeOffset(double offset_shift) {
    global_time_offset_ += offset_shift;
  }

  /** Shifts the trajectory in time by the given amount to the right.
   * It means that start_time() and end_time() will be increased by t_shift.
   */
  void ShiftTime(double t_shift);

  /** Creates a constant timed trajectory with a single value between time 0 and
   * infinity.
   * @param conf The constant configuration.
   */
  static TimedTrajectory Constant(
      const Eigen::VectorXd& conf, double start_time = 0.0,
      double end_time = std::numeric_limits<double>::max());

  /**
   * Creates a new trajectory that is a slice of this trajectory starting from
   * the given time until the end. The global_time_offset_ is preserved.
   * @param t The local time from which to start the slice.
   * @param clamp_end If true, and t >= end_time(), returns a constant
   * trajectory at the end value for duration 1e-4. If false, throws an
   * exception in that case.
   * @return A new TimedTrajectory that starts at time t and ends at the
   * original end_time().
   * @throws std::runtime_error if t < start_time().
   */
  TimedTrajectory SliceFromTime(double t, bool clamp_end = false) const;

  /**
   * Creates a new trajectory that is a slice of this trajectory in [t_1, t_2].
   * If t_1 or t_2 are out of bounds, they are clamped to [start_time(),
   * end_time()].
   */
  TimedTrajectory Slice(double t_1, double t_2) const;

  /**
   * Stitches traj_1 followed by traj_2. The local start_time of traj_2 is moved
   * to traj_1.end_time().
   * If continuity checks are enabled, throws when the continuity condition is
   * violated by more than tol.
   */
  static TimedTrajectory Merge(const TimedTrajectory& traj_1,
                               const TimedTrajectory& traj_2,
                               bool check_pos_continuity = true,
                               bool check_vel_continuity = true,
                               double tol = 1e-6);

 private:
  proto::TimedTrajectory ToProtoImpl() const override;

  void FromProtoImpl(const proto::TimedTrajectory& msg) override;

  PiecewisePolynomial path_;
  PiecewisePolynomial time_scaling_;
  double global_time_offset_;
};

using SystemPiecewisePolynomial =
    internal::SystemProperty<PiecewisePolynomial,
                             proto::SystemPiecewisePolynomial>;

using SystemTimedTrajectory =
    internal::SystemProperty<TimedTrajectory, proto::SystemTimedTrajectory>;

/** Construct a constant SystemTimedTrajectory from a SystemConf.
 * @param conf The constant configuration.
 * @return A SystemTimedTrajectory with the given constant configuration.
 */
SystemTimedTrajectory ConstantSystemTimedTrajectory(
    const SystemConf& conf, double start_time = 0.0,
    double end_time = std::numeric_limits<double>::max());

}  // namespace planning_service_client
