/// @file misc_utils.h
#pragma once

#include <Eigen/Core>

#include <algorithm>
#include <future>
#include <string>
#include <vector>

namespace common {
namespace utils {

template <template <typename> typename T, typename R>
constexpr bool is_future_v = std::is_same_v<std::future<R>, T<R>>
                             || std::is_same_v<std::shared_future<R>, T<R>>;
/**
 * @brief Returns true if the result of a future is ready.
 *
 * @tparam FutureType Type of the future
 * @tparam R Type of the future result
 * @param f Future to check
 * @return true if the result is ready, false otherwise
 */
template <template <typename> typename FutureType, typename R>
std::enable_if_t<is_future_v<FutureType, R>, bool> future_result_ready(
    FutureType<R> const& f) {
  return f.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

/** Check if an iterable contains a value. */
template <template <typename> typename Iterable, typename T>
bool contains(const Iterable<T>& iterable, const T& value) {
  return std::find(iterable.begin(), iterable.end(), value) != iterable.end();
}

/**
 * @brief Retrieve the tag of the planning service. In production, this will
 * be a SemVer tag (X.Y.Z), and in development, this will use the commit
 * hash (dev-abc123).
 *
 * @return std::string containing the tag
 */
std::string GetTag();

/**
 * @brief Returns true when two joint position vectors differ by more than
 * @p tol in any single component (L-infinity norm).
 *
 * Prefer this over `isApprox` for joint positions: `isApprox` uses a relative
 * tolerance that behaves poorly near zero, whereas this checks an absolute
 * dead-band appropriate for revolute/prismatic joints (default 0.1 mrad).
 */
inline bool JointPositionsChanged(const Eigen::VectorXd& q_a,
                                  const Eigen::VectorXd& q_b,
                                  double tol = 1e-4) {
  return (q_b - q_a).lpNorm<Eigen::Infinity>() > tol;
}

}  // namespace utils
}  // namespace common
