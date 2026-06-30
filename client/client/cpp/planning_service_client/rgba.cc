#include "rgba.h"

namespace planning_service_client {

proto::Rgba Rgba::ToProtoImpl() const {
  proto::Rgba msg;
  msg.set_r(r_);
  msg.set_g(g_);
  msg.set_b(b_);
  msg.set_a(a_);
  return msg;
}

void Rgba::FromProtoImpl(const proto::Rgba& msg) {
  r_ = msg.r();
  g_ = msg.g();
  b_ = msg.b();
  a_ = msg.a();
}

}  // namespace planning_service_client
