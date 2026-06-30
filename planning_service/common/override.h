#include <condition_variable>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>

#pragma once

namespace {
const inline std::string to_string(const std::thread::id& id) {
  std::ostringstream oss;
  oss << id;
  return oss.str();
}
}  // namespace
namespace common {

// Forward declaration
template <typename T>
class ScopedOverride;

/**
 * @brief Class template to hold a value that can be overridden and restored.
 * Useful for managing parameters or settings which a user may want to change
 * temporarily and then revert back to the original value.
 *
 * Intended to be used with ScopedOverride (see below) - without it, the
 * value is effectively read-only. Users should be familiar with its
 * `std::optional`-like semantics.
 *
 * @tparam T
 */
template <typename T>
class Overrideable {
 public:
  Overrideable(const T& value)
      : curr_value_(std::make_unique<T>(value)), initialized_(true) {}
  Overrideable() = default;

  /** Get the current value. */
  T& value() const {
    check_has_value();
    return *curr_value_;
  }
  /** Pointer access operator. */
  T* operator->() const {
    check_has_value();
    return curr_value_.get();
  }
  /** Dereference operator. */
  T& operator*() const {
    check_has_value();
    return *curr_value_;
  }

  /** Returns true if the current value is set. */
  bool has_value() const {
    try {
      check_has_value();
    } catch (const std::runtime_error&) {
      return false;
    }
    return true;
  }

  operator bool() const {
    return has_value();
  }
  /**
   * @brief Initialize the value exactly once. Intended for default-constructed
   * instances that need a default value set before any ScopedOverride is used.
   * @throws std::runtime_error if the value has already been initialized.
   */
  void initialize(const T& value) {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (initialized_) {
      throw std::runtime_error(
          "Overrideable: Value has already been initialized and cannot be "
          "initialized again.");
    }
    curr_value_ = std::make_unique<T>(value);
    initialized_ = true;
  }
  /** Returns true if the value is currently owned by a ScopedOverride. */
  bool owned() const {
    return owner_token_ != 0;
  }

  /** Returns true if the current value has been overridden. */
  bool overridden() const {
    return overridden_;
  }

 protected:
  /** Clone and return the current value. */
  T clone() const {
    check_has_value();
    return *curr_value_;
  }

  /** Set the current value, saving the previous one if it exists. */
  void set(uint64_t token, const T& value) {
    check_mutable(token);
    check_overridden();
    std::scoped_lock<std::mutex> lock(mutex_);
    if (curr_value_ != nullptr) {
      last_value_ = std::move(curr_value_);
      overridden_ = true;
    }
    curr_value_ = std::make_unique<T>(value);
  }

  /**
   * @brief Restore the previous value, if it exists.
   * @return True if the value was restored, false otherwise.
   */
  void restore(uint64_t token) {
    check_mutable(token);
    std::scoped_lock<std::mutex> lock(mutex_);
    if (last_value_ != nullptr) {
      curr_value_ = std::move(last_value_);
      last_value_.reset();
    }
    overridden_ = false;
  }

 private:
  /** Check if the value is mutable or throw. An instance is mutable if it is
   * not owned or is owned by the current thread. */
  void check_mutable(uint64_t token) const {
    if (owner_token_ != 0) {
      if (owner_token_ != token) {
        throw std::runtime_error(
            "Overrideable: Value is already owned by another ScopedOverride "
            "and may not be modified.");
      }
    }
  }
  /** Check if the value has already been overridden; throw otherwise. */
  void check_overridden() const {
    if (overridden_) {
      throw std::runtime_error(
          "Overrideable: Value has been set and may not be changed.");
    }
  }
  /** Check if the value has been set; throw otherwise. */
  void check_has_value() const {
    std::scoped_lock<std::mutex> lock(mutex_);
    if (curr_value_ == nullptr) {
      throw std::runtime_error("Overrideable: No current value set.");
    }
  }
  /** The following methods are used by ScopedOverride. */
  friend class ScopedOverride<T>;

  /** Set the owner of this overrideable. */
  void set_owner(
      uint64_t token, bool wait,
      std::optional<std::chrono::milliseconds> timeout = std::nullopt) {
    // We want to check if any thread currently owns the instance.
    if (owned()) {
      if (owner_token_ == token) {
        return;
      }
      if (!wait) {
        throw std::runtime_error(
            "Overrideable:set_owner: Value is already owned by a "
            "ScopedOverride.");
      }
      std::unique_lock<std::mutex> lock(mutex_);
      const auto pred {[this]() {
        return !owned();
      }};
      const auto now {std::chrono::steady_clock::now()};
      if (timeout.has_value()) {
        if (!owner_cv_.wait_for(lock, *timeout, pred)) {
          throw std::runtime_error(
              "Overrideable:set_owner: Timeout waiting for "
              "ownership after "
              + std::to_string(timeout->count() * 1e-3) + "s.");
        }
      } else {
        owner_cv_.wait(lock, pred);
      }
      const auto elapsed {std::chrono::steady_clock::now() - now};
    }
    owner_token_ = token;
  }

  /** Release the owner of this overrideable. */
  void release(uint64_t token) {
    check_mutable(token);
    // If unowned, nothing to do.
    if (owner_token_ == 0) {
      return;
    }
    owner_token_ = 0;
    owner_cv_.notify_all();
  }

  // Token signifies ownership
  std::atomic<uint64_t> owner_token_ {0};
  // Condition variable to notify waiting threads when ownership is released.
  std::condition_variable owner_cv_;
  // Whether the current value has been overridden.
  std::atomic_bool overridden_ {false};
  // Mutex to protect access to the current and last values.
  mutable std::mutex mutex_;
  // Current and last values.
  std::unique_ptr<T> curr_value_;
  std::unique_ptr<T> last_value_;
  // Whether the value has been initialized.
  bool initialized_ {false};
};

/**
 * @brief Class template to manage the lifetime of an override on an
 * Overrideable instance. On construction, the ScopedOverride takes ownership of
 * the Overrideable instance and optionally sets a new value. On destruction,
 * the ScopedOverride restores the previous value and releases ownership.
 *
 * @tparam T
 */
template <typename T>
class ScopedOverride {
 protected:
  using callback_t = std::function<void()>;

 public:
  ScopedOverride() = default;
  /** Constructor. */
  ScopedOverride(Overrideable<T>& target) {
    acquire(target);
  }
  /** Convenience constructor to acquire the instance and set a new value. */
  ScopedOverride(Overrideable<T>& target, const T& value) {
    acquire(target);
    set(value);
  }
  /** Prevent copy and assignment. */
  ScopedOverride(const ScopedOverride&) = delete;
  ScopedOverride& operator=(const ScopedOverride&) = delete;

  /** Move semantics (allowed on same thread only). */
  ScopedOverride(ScopedOverride&& other) {
    if (this_thread_id_ != other.this_thread_id_) {
      throw std::runtime_error(
          "ScopedOverride: Cannot move ownership across threads.");
    }
    token_ = other.token_;
    acquire(*other.target_);
    on_set_ = std::move(other.on_set_);
    on_restore_ = std::move(other.on_restore_);
    other.target_ = nullptr;
  }

  ScopedOverride& operator=(ScopedOverride&& other) {
    if (this_thread_id_ != other.this_thread_id_) {
      throw std::runtime_error(
          "ScopedOverride: Cannot move ownership across threads.");
    }
    if (this != &other) {
      release();
      token_ = other.token_;
      acquire(*other.target_);
      on_set_ = std::move(other.on_set_);
      on_restore_ = std::move(other.on_restore_);
      other.target_ = nullptr;
    }
    return *this;
  }

  /** Destructor. Restore to original value and release ownership. */
  ~ScopedOverride() {
    release();
  }

  /** Release the target Overrideable from ownership. */
  void release() {
    if (target_ == nullptr) {
      return;
    }
    // If the target was never set, we don't want to
    // run the event action
    if (target_->overridden()) {
      target_->restore(token_);
      if (on_restore_) on_restore_();
    }
    // Release from ownership
    target_->release(token_);
    target_ = nullptr;
  }

  /**
   * @brief Assign a new Overrideable target to manage.
   *
   * @param target
   */
  void acquire(Overrideable<T>& target) {
    if (target_ != nullptr) {
      throw std::runtime_error(
          "ScopedOverride: Already managing an Overrideable instance.");
    }
    if (token_ == 0) {
      token_ = generate_token();
    }
    target_ = &target;
    target_->set_owner(token_, wait_, timeout_);
  }

  /**
   * @brief Set a callback function which will be triggered when the underlying
   * target value is set.
   * @throws std::runtime_error if an on_set callback has already been
   * registered.
   */
  void on_set(callback_t callback) {
    if (on_set_) {
      throw std::runtime_error(
          "ScopedOverride: on_set callback has already been set.");
    }
    on_set_ = std::move(callback);
  }

  /**
   * @brief Set a callback function which will be triggered when the underlying
   * target value is restored.
   * @throws std::runtime_error if an on_restore callback has already been
   * registered.
   */
  void on_restore(callback_t callback) {
    if (on_restore_) {
      throw std::runtime_error(
          "ScopedOverride: on_restore callback has already been set.");
    }
    on_restore_ = std::move(callback);
  }

  /**
   * @brief Set a callback function which will be triggered on both set and
   * restore events.
   * @throws std::runtime_error if either callback has already been registered.
   */
  void on_event(callback_t callback) {
    if (on_set_ || on_restore_) {
      throw std::runtime_error(
          "ScopedOverride: on_set or on_restore callback has already been "
          "set.");
    }
    on_set_ = callback;
    on_restore_ = callback;
  }

  /** Clone the current value of the target override. */
  T clone() const {
    if (target_ == nullptr) {
      throw std::runtime_error(
          "ScopedOverride: Not managing any Overrideable instance.");
    }
    return target_->clone();
  }

  /** Set the value of the target override. */
  void set(const T& value) {
    if (target_ == nullptr) {
      throw std::runtime_error(
          "ScopedOverride: Not managing any Overrideable instance.");
    }
    target_->set(token_, value);
    if (on_set_) on_set_();
  }

  bool has_target() const {
    return target_ != nullptr;
  }

 private:
  static uint64_t generate_token() {
    static std::atomic<uint64_t> counter {1};
    return counter.fetch_add(1, std::memory_order_relaxed);
  }
  /** Whether to wait for ownership when assigning the target. */
  const bool wait_ {true};
  const std::thread::id this_thread_id_ {std::this_thread::get_id()};
  /** Timeout for waiting for ownership. */
  const std::optional<std::chrono::milliseconds> timeout_ {
      std::chrono::milliseconds(1000)};  // 1 second
  /** Pointer to the target Overrideable instance. */
  Overrideable<T>* target_ {nullptr};
  uint64_t token_ {0};
  /** Optional event callback function. */
  std::function<void()> on_set_;
  std::function<void()> on_restore_;
};

}  // namespace common
