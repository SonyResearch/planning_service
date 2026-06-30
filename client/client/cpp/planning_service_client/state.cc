#include "planning_service_client/state.h"

#include "planning_service_client/internal/overloaded.h"

namespace planning_service_client {

namespace {

// Forward declaration so CanonicalString(Value) can call CanonicalString(State)
std::string CanonicalString(const State& state);

// Produces a canonical, fully-precise, non-truncated string for a Value.
// Iteration over nested States uses std::map order (sorted by key), making
// the output fully deterministic regardless of proto serialization.
std::string CanonicalString(const Value& value) {
  return value.Visit(
      overloaded {[](const std::monostate&) -> std::string {
                    return "null";
                  },
                  // bool must come before the generic arithmetic case to avoid
                  // the implicit bool->double conversion.
                  [](bool arg) -> std::string {
                    return arg ? "true" : "false";
                  },
                  [](double arg) -> std::string {
                    std::ostringstream oss;
                    oss << std::fixed << std::setprecision(6) << arg;
                    return oss.str();
                  },
                  [](const std::string& arg) -> std::string {
                    return "\"" + arg + "\"";
                  },
                  [](const std::vector<Value>& arg) -> std::string {
                    std::ostringstream oss;
                    oss << "[";
                    bool first {true};
                    for (const auto& v : arg) {
                      if (!first) oss << ",";
                      oss << CanonicalString(v);
                      first = false;
                    }
                    oss << "]";
                    return oss.str();
                  },
                  [](const State& arg) -> std::string {
                    return CanonicalString(arg);
                  }});
}

// Iterates the underlying std::map in sorted-key order, guaranteeing a stable
// canonical string independent of proto map serialization behaviour.
std::string CanonicalString(const State& state) {
  std::ostringstream oss;
  oss << "{";
  bool first {true};
  for (const auto& [key, val] : state) {
    if (!first) oss << ",";
    oss << key << ":" << CanonicalString(val);
    first = false;
  }
  oss << "}";
  return oss.str();
}

}  // namespace

// Value class implementations

proto::Value Value::ToProtoImpl() const {
  proto::Value msg;
  std::visit(overloaded {[&](const std::monostate&) {
                           msg.mutable_null_value();
                         },
                         [&](const std::string& arg) {
                           msg.set_string(arg);
                         },
                         [&](double arg) {
                           msg.set_number(arg);
                         },
                         [&](bool arg) {
                           msg.set_bool_(arg);
                         },
                         [&](const std::vector<Value>& arg) {
                           for (const auto& value : arg) {
                             msg.mutable_list()->add_values()->CopyFrom(
                                 ToProto(value));
                           }
                         },
                         [&](const State& arg) {
                           msg.mutable_state()->CopyFrom(ToProto(arg));
                         }},
             value_);
  return msg;
}

void Value::FromProtoImpl(const proto::Value& msg) {
  switch (msg.value_case()) {
    case proto::Value::kString: {
      value_ = msg.string();
      break;
    }
    case proto::Value::kNumber: {
      value_ = msg.number();
      break;
    }
    case proto::Value::kBool: {
      value_ = msg.bool_();
      break;
    }
    case proto::Value::kList: {
      std::vector<Value> vec;
      for (const auto& value : msg.list().values()) {
        vec.push_back(FromProto<Value>(value));
      }
      value_ = vec;
      break;
    }
    case proto::Value::kState: {
      value_ = FromProto<State>(msg.state());
      break;
    }
    case proto::Value::kNullValue:
      value_ = std::monostate {};
      break;
    case proto::Value::VALUE_NOT_SET:
    default:
      throw std::runtime_error("Unsupported type in Value proto");
  }
}

std::string Value::ToCompactString() const {
  std::ostringstream result;
  std::visit(overloaded {[&result](const std::monostate&) {
                           result << "null";
                         },
                         [&result](const std::string& arg) {
                           result << arg.substr(0, 10);
                         },
                         [&result](double arg) {
                           result << std::fixed << std::setprecision(2) << arg;
                         },
                         [&result](bool arg) {
                           result << (arg ? "T" : "F");
                         },
                         [&result](const std::vector<Value>& arg) {
                           result << "[";
                           bool first {true};
                           for (const auto& value : arg) {
                             if (!first) result << ",";
                             result << value.ToCompactString();
                             first = false;
                           }
                           result << "]";
                         },
                         [&result](const State& arg) {
                           result << arg.ToCompactString();
                         }},
             value_);
  return result.str();
}

// State class implementations

State::State(std::initializer_list<std::pair<std::string, Value>> init) {
  for (const auto& [key, value] : init) {
    state_[key] = value;
  }
}

void State::Add(const std::string& key, const Value& value) {
  state_[key] = value;
}

Value& State::operator[](const std::string& key) {
  return state_.at(key);
}
const Value& State::operator[](const std::string& key) const {
  return state_.at(key);
}

std::string State::ToCompactString(bool include_hash) const {
  std::ostringstream result;
  result << "{";
  bool first {true};
  for (const auto& [key, val] : state_) {
    if (!first) result << ",";
    result << key.substr(0, 10) << ":" << val.ToCompactString();
    first = false;
  }
  result << "}";
  if (include_hash) {
    result << "-h" << CalcHash();
  }
  return result.str();
}

uint64_t State::CalcHash() const {
  return std::hash<std::string> {}(CanonicalString(*this));
}
bool State::IsSubsetOf(const State& other) const {
  for (const auto& [key, val] : state_) {
    // Check if the other state contains the key
    if (!other.Contains(key)) {
      return false;
    }
    bool is_subset {std::visit(
        overloaded {
            [&](const State& s1, const State& s2) {
              return s1.IsSubsetOf(s2);
            },
            [&](const auto& v1, const auto& v2) {
              if constexpr (std::is_same_v<std::decay_t<decltype(v1)>,
                                           std::decay_t<decltype(v2)>>) {
                return v1 == v2;
              }
              return false;
            }},
        val.variant(), other.state_.at(key).variant())};
    if (!is_subset) {
      return false;
    }
  }
  return true;
}

proto::State State::ToProtoImpl() const {
  proto::State msg;
  for (const auto& [key, val] : state_) {
    msg.mutable_entries()->insert({key, ToProto(val)});
  }
  return msg;
}

void State::FromProtoImpl(const proto::State& msg) {
  state_.clear();
  for (const auto& [key, value] : msg.entries()) {
    state_[key] = FromProto<Value>(value);
  }
}

}  // namespace planning_service_client
