
#pragma once

#include <Eigen/Dense>

#include <vector>

#include <google/protobuf/repeated_field.h>

#include "proto/basic_types.pb.h"

namespace planning_service_client {
namespace internal {

std::vector<double> e_to_v(Eigen::VectorXd e);

Eigen::VectorXd v_to_e(std::vector<double> v);

const google::protobuf::RepeatedField<double> EigenToRepeated(
    const Eigen::VectorXd& ev);

Eigen::VectorXd RepeatedToEigen(
    const google::protobuf::RepeatedField<double>& field);

bool IsApprox(double a, double b, double tol = 1e-6);

proto::Vector3 EigenVector3dToProto(const Eigen::Vector3d& vec);

Eigen::Vector3d ProtoToEigenVector3d(const proto::Vector3& msg);

}  // namespace internal
}  // namespace planning_service_client
