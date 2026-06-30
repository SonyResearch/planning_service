/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file resource_manager.h

#pragma once

#include <stdlib.h>

#include <magic_enum/magic_enum.hpp>

#include "job_runner.h"
#include "planning_service/common/misc_utils.h"
#include "request_queue.h"
#include "resource_registry.h"
namespace fs = std::filesystem;

namespace service {
namespace utils {

/**
 * @brief Class which manages the main compute resource (i.e., the "Draco") for
 * planning and other algorithms, as well as computation of results using that
 * resource. Concretely, owns ID-based lookup access to the resource, a queue to
 * process new requests, and a runner to handle, and store the results of,
 * threads of computation.
 *
 * @tparam RequestType Some derivation of RequestAdapter
 * @tparam ResultType The result of the "jobs" mnaged by the JobRunner
 */
template <typename RequestType, typename ResultType>
class ResourceManager {
 public:
  /**
   * @brief Constructor. Create the Draco instances at the data path and
   * initialize the queue and runner.
   * @param system_name Enabled system name
   * @param options Options handling the loading of resources from disk
   * @param max_queue_size maximum number of requests which may wait in the
   * queue at a given instant
   * @param max_active_jobs maximum number of jobs which may be running at
   * a given instant
   */
  ResourceManager(const std::string& system_name,
                  const ResourceOptions& options, const int max_queue_size = 1,
                  const int max_active_jobs = 1)
      : registry_ {std::make_shared<ResourceRegistry>(system_name, options)},
        request_queue_ {
            std::make_unique<RequestQueue<RequestType>>(max_queue_size)},
        job_runner_ {std::make_unique<JobRunner<ResultType>>(max_active_jobs)} {
  }

  // At most a single manager is scoped to an active service
  ResourceManager(ResourceManager& other) = delete;
  ResourceManager(ResourceManager&& other) = delete;
  ~ResourceManager() = default;

  /**
   * @brief Return true if the job request with the given ID is either
   * waiting in the queue or has already been initiated.
   *
   * @param id desired job ID
   *
   * @return true if the job is either in the queue or in the job runner
   */
  inline bool HasJob(const request_id_t& id) const {
    return request_queue_->Contains(id) || job_runner_->Contains(id);
  }

  /**
   * @brief Add a request to the queue. This does not start a job.
   *
   * @param request Request to be processed
   *
   * @return true on success, one of ServiceError on failure
   *
   */
  std::expected<bool, ServiceError> QueueRequest(const RequestType& request) {
    if (HasJob(request.id)) {
      logging::log()->warn(
          "RM:QueueRequest: Request {} has already been added to queue",
          request.id);
      // if we have already received the request, we don't need to fail, just
      // ignore it
      return true;
    }
    if (request_queue_->Size() >= request_queue_->Capacity()) {
      return std::unexpected(
          ServiceError(ServiceErrorCode::QUEUE_CAPACITY_REACHED,
                       fmt::format("Queue is at capacity ({})!",
                                   request_queue_->Capacity())));
    }
    request_queue_->Enqueue(request);
    logging::log()->info("RM:QueueRequest: Request {} added to queue",
                         request.id);
    return true;
  }
  /** Clear the queue. */
  void ClearQueue() {
    request_queue_->Clear();
  }

 protected:
  /**
   * @brief Wrapper around StartJobImpl that checks for correctness in the queue
   * and the job_runner.
   *
   * @param request Request to be processed
   * @return true on success, one of ServiceError on failure
   */
  const std::expected<bool, ServiceError> StartJob(const RequestType& request) {
    if (HasJob(request.id)) {
      return std::unexpected(ServiceError(
          ServiceErrorCode::DUPLICATE_ID,
          fmt::format("Job with ID {} is already in progress!", request.id)));
    }
    if (job_runner_->Size() >= job_runner_->Capacity()) {
      return std::unexpected(
          ServiceError(ServiceErrorCode::JOB_CAPACITY_REACHED,
                       "Already at maximum number of active jobs"));
    }
    return StartJobImpl(request);
  }

  /**
   * @brief Virtual. The specific implementation to start a given job. In
   * general, this should unpack data from the request, and load it into a
   * thread of execution in the JobRunner.
   *
   * @param request Request to be processed
   * @returntrue on success, one of ServiceError on failure
   */
  virtual const std::expected<bool, ServiceError> StartJobImpl(
      const RequestType& request) = 0;

  /** Perform any cleanup to the underlying resources. */
  virtual void Cleanup() {}

 public:
  /**
   * @brief Move a job from the queue and start an active thread of execution
   * to solve that job
   *
   * @return true on success, one of ServiceError on failure
   */
  const std::expected<bool, ServiceError> RunOnce() {
    try {
      const auto request {request_queue_->PopNextRequest()};
      logging::log()->info(
          "ResourceManager:RunOnce: Moving next queued request "
          "into active thread");
      return StartJob(request);
    } catch (const queue_empty_error& e) {
      return std::unexpected(
          ServiceError(ServiceErrorCode::QUEUE_EMPTY, "Queue is empty!"));
    } catch (const std::exception& e) {
      return std::unexpected(ServiceError(
          ServiceErrorCode::UNKNOWN,
          fmt::format("Failed due to unexpected exception: {}", e.what())));
    }
  }

  /**
   * @brief Start the internal polling for the manager, which will move job
   * requests from the queue to an active thread of execution as space becomes
   * available
   */
  void Run() {
    logging::log()->info("ResourceManager:Run: Polling queue for new requests");
    while (!stop_triggered_) {
      Cleanup();
      if (request_queue_->Empty()
          || job_runner_->Size() == job_runner_->Capacity()) {
        continue;
      }
      auto result {RunOnce()};
      if (!result) {
        logging::log()->error(
            "ResourceManager:Run: Internal error attempting to start "
            "next job: {}",
            result.error());
      }
      std::this_thread::sleep_for(
          chrono_ms(static_cast<uint16_t>(1e3 / poll_freq_hz_)));
    }
    logging::log()->info("ResourceManager:Run: Stopping resource manager");
  }

  bool Running() const {
    return !stop_triggered_;
  }
  /** Stop the manager. */
  void Stop() {
    stop_triggered_ = true;
  }
  /** Return pointer to the registry. */
  std::shared_ptr<ResourceRegistry> registry() {
    return registry_;
  }

 protected:
  // frequency at which the queue is polled for new requests
  uint16_t poll_freq_hz_ {100};
  // registry of all Draco instances - shared to expose to the registry service
  std::shared_ptr<ResourceRegistry> registry_;
  // ordered queue of jobs that have not yet started
  std::unique_ptr<RequestQueue<RequestType>> request_queue_;
  // map of request IDs to jobs in progress
  std::unique_ptr<JobRunner<ResultType>> job_runner_;
  // atomic bool used to shutdown the main loop
  std::atomic<bool> stop_triggered_ {false};
};

}  // namespace utils
}  // namespace service
