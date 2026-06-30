/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

/// @file request_queue.h

#pragma once
#include <future>

#include "planning_service/common/logging.h"
#include "planning_service/service/types/types.h"
namespace service {
namespace utils {

/** Error thrown when trying to access an element from the queue, but there are
 * none. */
class queue_empty_error : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
  using std::runtime_error::operator=;
  explicit queue_empty_error()
      : runtime_error {"Queue was queried for element, but is empty"} {}
};
/** Error thrown when trying to add another element to the queue, but it is at
 * its capacity. */
class queue_at_capacity_error : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
  using std::runtime_error::operator=;
  explicit queue_at_capacity_error()
      : runtime_error {"Queue is at maximum capacity"} {}
};

/**
 * @brief Queue class which receives requests to be processed in the order in
 * which they were received.
 */
template <typename RequestType>
class RequestQueue {
  static_assert(
      std::is_base_of_v<service::RequestAdapter, RequestType>,
      "A RequestQueue must be instantiated only for a RequestAdapter");

 public:
  /** Constructor taking maximum possible number of requests in the queue */
  explicit RequestQueue(const int capacity = 1)
      : capacity_ {static_cast<size_t>(capacity)} {
    if (capacity <= 0)
      throw std::runtime_error(
          "Queue must have positive nonzero maximum size!");
  }
  ~RequestQueue() = default;

  /** Check if a request with the given ID is in the queue. */
  inline bool Contains(const request_id_t& id) const {
    std::scoped_lock<std::mutex> lock {queue_mtx_};
    return queued_ids_.count(id);
  }
  /** Clear all items from the queue. */
  inline void Clear() {
    std::scoped_lock<std::mutex> lock {queue_mtx_};
    queued_requests_.clear();
    queued_ids_.clear();
  }

  /**
   * @brief Add a request to the queue if the max length has not been
   * exceeded, throw otherwise
   * @param req request
   *
   */
  inline void Enqueue(const RequestType& request) {
    std::scoped_lock<std::mutex> lock {queue_mtx_};
    if (queued_requests_.size() >= capacity_) {
      throw queue_at_capacity_error();
    }
    if (queued_ids_.count(request.id)) {
      logging::log()->debug(
          "RequestQueue:Enqueue: Request with ID {} already in queue. Ignoring",
          request.id);
    }
    // add request to the queue
    queued_requests_.push_back(request);
    // add ID to the set of queued IDs to track duplicate ID requests
    queued_ids_.insert(request.id);
  }
  /** Peek the next request in the queue, if one exists. Throw otherwise. */
  inline const RequestType& PeekNextRequest() const {
    std::scoped_lock<std::mutex> lock {queue_mtx_};
    if (queued_requests_.size() == 0) {
      throw queue_empty_error();
    }
    return queued_requests_.front();
  }
  /** Return and remove the next reqwuest in the queue, if one exists. Throw
   * otherwise. */
  inline const RequestType PopNextRequest() {
    std::scoped_lock<std::mutex> lock {queue_mtx_};
    if (queued_requests_.size() == 0) {
      throw queue_empty_error();
    }
    const auto req {queued_requests_.front()};
    // destroy first element
    queued_requests_.pop_front();
    // remove ID from list
    queued_ids_.erase(req.id);
    return req;
  }

  /** Return true if queue is empty, false otherwise */
  inline bool Empty() const {
    std::scoped_lock<std::mutex> lock {queue_mtx_};
    return queued_requests_.empty();
  }
  /** Return current size. */
  inline size_t Size() const {
    std::scoped_lock<std::mutex> lock {queue_mtx_};
    return queued_requests_.size();
  }

  /** Return max length. */
  inline size_t Capacity() const {
    return capacity_;
  }

 private:
  mutable std::mutex queue_mtx_;
  const size_t capacity_ {1};
  // double-ended queue to store and retrieve requests
  std::deque<RequestType> queued_requests_ {};
  // set of ids in the queue for quick lookup
  std::set<request_id_t> queued_ids_ {};
};
}  // namespace utils
}  // namespace service
