
#include "conf.h"

#include "planning_service_client/internal/eigen_utils.h"
namespace planning_service_client {
proto::Conf Conf::ToProtoImpl() const {
  proto::Conf conf_pb;
  *conf_pb.mutable_data() = internal::EigenToRepeated(q_);
  return conf_pb;
}

void Conf::FromProtoImpl(const proto::Conf& msg) {
  q_ = internal::v_to_e(
      std::vector<double>(msg.data().begin(), msg.data().end()));
}
}  // namespace planning_service_client
