#pragma once
#include <functional>
#include <initializer_list>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <variant>
#include <vector>

#include "planning_service_client/internal/proto_base.h"

namespace planning_service_client {
// Helper trait to detect vectors
template <typename T>
struct is_vector : std::false_type {};

template <typename T, typename A>
struct is_vector<std::vector<T, A>> : std::true_type {};

template <typename T>
inline constexpr bool is_vector_v = is_vector<T>::value;

// Forward declaration
class Value;
/**
 * @brief Joint configuration of a system.
 *
 */
class State : public internal::ProtoBase<proto::State> {
 public:
  State() = default;

  State(const State& other) = default;

  /** Initializer list constructor. */
  State(std::initializer_list<std::pair<std::string, Value>> init);
  State& operator=(const State& other) = default;

  /**
   * @brief Add a key-value pair to the state. If the key already exists, it
   * will be updated.
   *
   * @param key The key to add.
   * @param value The value to add.
   */
  void Add(const std::string& key, const Value& value);

  /** Perfect forwarding version of Add. */
  template <typename T, typename = std::enable_if_t<
                            !std::is_same_v<std::decay_t<T>, Value>>>
  void Add(const std::string& key, T&& value) {
    Add(key, Value(std::forward<T>(value)));
  }

  /** Remove a key-value pair from the state. */
  void Erase(const std::string& key) {
    state_.erase(key);
  }

  // Const access
  const Value& operator[](const std::string& key) const;
  // Mutable access
  Value& operator[](const std::string& key);

  /** Check if the state contains a key. */
  bool Contains(const std::string& key) const {
    return state_.contains(key);
  }
  // Iterators
  using iterator = typename std::map<std::string, Value>::iterator;
  using const_iterator = typename std::map<std::string, Value>::const_iterator;
  iterator begin() {
    return state_.begin();
  }
  iterator end() {
    return state_.end();
  }
  const_iterator begin() const {
    return state_.begin();
  }
  const_iterator end() const {
    return state_.end();
  }
  /**
   * @brief Compute the hash of the state.
   *
   * This hash is based on a canonical, implementation-invariant representation
   * of the state.
   */
  uint64_t CalcHash() const;

  /**
   * @brief Convert the state to a string representation that fits in one line.
   * The string is a compact representation of the state, suitable for logging
   * or display in a UI.
   *
   * @param include_hash If true, include the hash of the state in the string.
   */
  std::string ToCompactString(bool include_hash = false) const;

  /**
   * @brief Check if this state is a subset of another state.
   * A state is a subset of another state if all key-value pairs in this
   state are also present in the other state with the same values.
   *
   * @param other The other state to compare against.
   * @return true if this state is a subset of the other state, false otherwise.
   */
  bool IsSubsetOf(const State& other) const;

 private:
  proto::State ToProtoImpl() const override;

  void FromProtoImpl(const proto::State& msg) override;

  std::map<std::string, Value> state_;
};

/**
 * @brief A single value stored by key in a State object.
 *
 */
class Value : public internal::ProtoBase<proto::Value> {
  friend class State;

 public:
  using value_type = std::variant<std::monostate, std::string, double, bool,
                                  std::vector<Value>, State>;

  Value() = default;
  /** Numeric constructor. */
  template <typename T, typename = std::enable_if_t<
                            std::is_arithmetic_v<std::decay_t<T>>
                            && !std::is_same_v<std::decay_t<T>, bool>>>
  Value(T value) : value_(static_cast<double>(value)) {}
  Value(bool value) : value_(value) {}
  /** Const char constructor. */
  Value(const char* value) : value_(std::string(value)) {}

  /** Forwarding constructor. */
  template <typename T, typename = std::enable_if_t<
                            !std::is_same_v<std::decay_t<T>, Value>
                            && !is_vector_v<std::decay_t<T>>
                            && !std::is_arithmetic_v<std::decay_t<T>>>>
  Value(T&& value) : value_(std::forward<T>(value)) {}
  /** Vector constructors. */
  template <typename T>
  Value(const std::vector<T>& vec) {
    std::vector<Value> value_vec;
    value_vec.reserve(vec.size());
    for (const auto& v : vec) {
      value_vec.emplace_back(v);
    }
    value_ = std::move(value_vec);
  }
  template <typename T>
  Value(std::vector<T>&& vec) {
    std::vector<Value> value_vec;
    value_vec.reserve(vec.size());
    for (auto&& v : vec) {
      value_vec.emplace_back(std::move(v));
    }
    value_ = std::move(value_vec);
  }
  /** Initializer list constructors. */
  template <typename T>
  Value(std::initializer_list<T> list) {
    std::vector<Value> value_vec;
    value_vec.reserve(list.size());
    for (const auto& v : list) {
      value_vec.emplace_back(v);
    }
    value_ = std::move(value_vec);
  }
  Value(std::initializer_list<std::pair<std::string, Value>> list) {
    value_ = State(list);
  }

  template <typename T>
  operator T() const {
    return std::get<T>(value_);
  }

  Value(const Value& other) = default;
  Value& operator=(const Value& other) = default;

  /** Const getter. */
  template <typename T>
  std::enable_if_t<!std::is_same_v<std::decay_t<T>, Value>, const T&> get()
      const {
    return std::get<T>(value_);
  }
  /** Mutable getter. */
  template <typename T>
  std::enable_if_t<!std::is_same_v<std::decay_t<T>, Value>, T&> get() {
    return std::get<T>(value_);
  }

  constexpr size_t index() const {
    return value_.index();
  }

  template <typename T>
  constexpr bool Is() const {
    static_assert(
        std::disjunction_v<
            std::is_same<T, std::monostate>, std::is_same<T, std::string>,
            std::is_same<T, double>, std::is_same<T, bool>,
            std::is_same<T, std::vector<Value>>, std::is_same<T, State>>,
        "T is not a valid alternative of Value::value_type");
    return std::holds_alternative<T>(value_);
  }

  /** Convert the state to a string representation that fits in one line.
   * The string is a compact representation of the state, suitable for
   * logging or display in a UI. */
  std::string ToCompactString() const;

  /**
   * Visit wrapper over the underlying variant. See Shape::Visit for the
   * analogous pattern.
   */
  template <typename Visitor>
  decltype(auto) Visit(Visitor&& visitor) const {
    return std::visit(std::forward<Visitor>(visitor), value_);
  }

 private:
  /** Get the underlying variant. Used by State only. */
  const value_type& variant() const {
    return value_;
  }
  proto::Value ToProtoImpl() const override;

  void FromProtoImpl(const proto::Value& msg) override;

  value_type value_;
};

}  // namespace planning_service_client
