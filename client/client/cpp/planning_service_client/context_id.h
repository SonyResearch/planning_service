#pragma once

#include <Eigen/Dense>

#include "planning_service_client/internal/proto_base.h"
#include "proto/basic_types.pb.h"

namespace planning_service_client {

class ContextId : public internal::ProtoBase<proto::ContextId> {
 public:
  ContextId() = default;

  ContextId(uint64_t value, const std::string& system)
      : value_(value), system_(system) {}

  operator bool() const {
    return value_ != 0;
  }

  uint64_t value() const {
    return value_;
  }

  std::string system() const {
    return system_;
  }

  /** Overload the equality operator */
  bool operator==(const ContextId& other) const {
    // Don't need to compare the system name.
    return value_ == other.value_;
  }

 private:
  proto::ContextId ToProtoImpl() const override {
    proto::ContextId msg;
    msg.set_value(value_);
    msg.set_system(system_);
    return msg;
  }

  void FromProtoImpl(const proto::ContextId& msg) override {
    value_ = msg.value();
    system_ = msg.system();
  }

  uint64_t value_ {0};
  std::string system_ {""};
};

}  // namespace planning_service_client
