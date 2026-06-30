#pragma once

#include <Eigen/Dense>

#include "planning_service_client/internal/system_property.h"

namespace planning_service_client {
/**
 * @brief Joint configuration of a system.
 *
 */
class Conf : public internal::ProtoBase<proto::Conf> {
 public:
  Conf() = default;

  Conf(const Eigen::VectorXd& q) : q_(q) {}

  // overload = operator
  Conf& operator=(const Eigen::VectorXd& q) {
    q_ = q;
    return *this;
  }

  bool operator==(const Conf& other) const {
    // False if the sizes are different
    if (q_.size() != other.q_.size()) {
      return false;
    }
    return q_ == other.q_;
  }

  const Eigen::VectorXd& q() const {
    return q_;
  }

 private:
  proto::Conf ToProtoImpl() const override;

  void FromProtoImpl(const proto::Conf& msg) override;

  Eigen::VectorXd q_ = Eigen::VectorXd::Zero(0);
};

using SystemConf = internal::SystemProperty<Conf, proto::SystemConf>;
}  // namespace planning_service_client
