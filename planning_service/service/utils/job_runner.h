/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file job_runner.h

#pragma once
#include <future>

#include <expected>

#include "planning_service/common/logging.h"
#include "planning_service/common/misc_utils.h"
#include "planning_service/service/types/error.h"
#include "utils.h"
namespace service {
namespace utils {

/**
 * @brief Class which manages threads of computation returning results of type
 * ResultType (i.e., a "job"), with methods to observe and retrieve results by
 * the unique ID of each job.
 */
template <typename ResultType>
class JobRunner {
 public:
  /** Constructor. */
  explicit JobRunner(const int capacity = 1)
      : capacity_ {static_cast<size_t>(capacity)} {
    if (capacity <= 0)
      throw std::runtime_error(
          "JobRunner must have positive nonzero maximum size!");
  }
  ~JobRunner() = default;

  /**
   * @brief Return the number of jobs which are active but not yet complete.
   */
  inline size_t NumJobsInProgress() const {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    return std::count_if(active_jobs_.cbegin(), active_jobs_.cend(),
                         [](const auto& p) {
                           return !common::utils::future_result_ready(p.second);
                         });
  }

  /**
   * @brief Return the set of IDs whose associated jobs have completed
   * (either by successful termination or error).
   */
  inline const std::set<request_id_t> GetCompletedJobIds() const {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    std::set<request_id_t> completed_job_ids;
    for (const auto& [id, job] : active_jobs_) {
      if (common::utils::future_result_ready(job)) {
        completed_job_ids.insert(id);
      }
    }
    return completed_job_ids;
  }
  /**
   * @brief Check if the result of a job with the given ID is ready.
   *
   * @param id Job ID
   * @return true if ready, false otherwise
   */
  inline bool ResultIsReady(const request_id_t& id) const {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    if (!active_jobs_.count(id)) {
      throw std::runtime_error(fmt::format("No job with ID {} found!", id));
    }
    return common::utils::future_result_ready(active_jobs_.at(id));
  }

  inline const std::expected<ResultType, ServiceError> RetrieveResult(
      const request_id_t& id) {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    if (!active_jobs_.count(id)) {
      throw std::runtime_error(fmt::format("No job with ID {} found!", id));
    }
    if (!common::utils::future_result_ready(active_jobs_.at(id))) {
      throw std::runtime_error(fmt::format("Job {} is not ready!", id));
    }
    if (!active_jobs_.at(id).valid()) {
      return std::unexpected(ServiceError(
          ServiceErrorCode::JOB_THREAD_INVALIDATED,
          fmt::format("JobManager:RetrieveResult: Thread at ID {} has "
                      "been invalidated!",
                      id)));
    }
    try {
      const auto result_opt {active_jobs_.at(id).get()};
      active_jobs_.erase(id);
      return result_opt;
    } catch (const std::exception& e) {
      active_jobs_.erase(id);
      return std::unexpected(ServiceError(
          ServiceErrorCode::JOB_THREAD_EXCEPTION,
          fmt::format("JobManager:RetrieveResult: Thread at ID {} threw an "
                      "exception: {}",
                      id, e.what())));
    }
  }

  /**
   * @brief Wait for the provided timeout in milliseconds for the future at the
   * given ID to terminate, or wait indefinitely if no value is provided.
   *
   * @param id Job ID
   * @param timeout_ms_opt Timeout duration in milliseconds, or nullopt
   */
  inline void WaitForResult(
      const request_id_t& id,
      const std::optional<uint16_t> timeout_ms_opt = std::nullopt) const {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    if (!active_jobs_.count(id)) {
      throw std::runtime_error(fmt::format("No job with ID {} found!", id));
    }
    if (timeout_ms_opt) {
      active_jobs_.at(id).wait_for(chrono_ms(timeout_ms_opt.value()));
    } else {
      active_jobs_.at(id).wait();
    }
  }
  /**
   * @brief Initiate and insert the new job with its ID as a key. Throws if a
   * job with that ID is already present or if the runner is at capacity.
   *
   * @param id Job ID
   * @param job Function whose return type is ResultType
   */
  inline void InsertNewJob(
      const request_id_t& id,
      const std::function<const std::expected<ResultType, ServiceError>()>
          job) {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    if (active_jobs_.count(id)) {
      throw std::runtime_error(
          fmt::format("Job with ID {} has already been started", id));
    }
    if (active_jobs_.size() >= capacity_) {
      throw std::runtime_error("Already at maximum number of active jobs!");
    }
    active_jobs_.emplace(id, std::async(std::launch::async, job));
  }

  /** Return true if job map is empty, false otherwise */
  inline bool Empty() const {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    return active_jobs_.empty();
  }
  /** Return current size. */
  inline size_t Size() const {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    return active_jobs_.size();
  }

  /** Return max length. */
  inline size_t Capacity() const {
    return capacity_;
  }

  /** Check if a request with the given ID is in the job map. */
  inline bool Contains(const request_id_t& id) const {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    return active_jobs_.count(id);
  }
  /** Clear all items from the job map. */
  void Clear() {
    std::scoped_lock<std::mutex> lock {jobs_mtx_};
    active_jobs_.clear();
  }

 private:
  mutable std::mutex jobs_mtx_;
  const size_t capacity_ {1};
  // map of unique IDs to active threads
  std::map<request_id_t,
           std::future<const std::expected<ResultType, ServiceError>>>
      active_jobs_ {};
};
}  // namespace utils
}  // namespace service
