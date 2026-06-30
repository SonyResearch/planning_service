
#include "planning_service_client/internal/eigen_utils.h"

namespace planning_service_client {
namespace internal {
std::vector<double> e_to_v(Eigen::VectorXd e) {
  std::vector<double> v;
  v.resize(e.size());
  Eigen::VectorXd::Map(&v[0], e.size()) = e;
  return v;
}

Eigen::VectorXd v_to_e(std::vector<double> v) {
  return Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(v.data(), v.size());
}

const google::protobuf::RepeatedField<double> EigenToRepeated(
    const Eigen::VectorXd& ev) {
  const auto vec {e_to_v(ev)};
  google::protobuf::RepeatedField<double> field {vec.begin(), vec.end()};
  return field;
}

Eigen::VectorXd RepeatedToEigen(
    const google::protobuf::RepeatedField<double>& field) {
  return v_to_e(std::vector<double>(field.begin(), field.end()));
}

bool IsApprox(double a, double b, double tol) {
  return std::abs(a - b) < tol;
}

proto::Vector3 EigenVector3dToProto(const Eigen::Vector3d& vec) {
  proto::Vector3 msg;
  msg.set_x(vec[0]);
  msg.set_y(vec[1]);
  msg.set_z(vec[2]);
  return msg;
}

Eigen::Vector3d ProtoToEigenVector3d(const proto::Vector3& msg) {
  return Eigen::Vector3d(msg.x(), msg.y(), msg.z());
}

}  // namespace internal
}  // namespace planning_service_client
