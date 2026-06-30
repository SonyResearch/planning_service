#include <gtest/gtest.h>

#include "planning_service_client/conf.h"

namespace planning_service_client {

TEST(Conf, ToProtoFromProto) {
  Eigen::VectorXd q(3);
  q << 1.0, 2.0, 3.0;
  Conf dut(q);
  auto msg = ToProto(dut);
  auto dut_deserialized = FromProto<Conf>(msg);
  EXPECT_TRUE(dut.q().isApprox(dut_deserialized.q()));
}

TEST(SystemConf, ToProtoFromProto) {
  Eigen::VectorXd q(3);
  q << 1.0, 2.0, 3.0;
  Eigen::VectorXd q2(3);
  q2 << 4.0, 5.0, 6.0;
  SystemConf dut;
  dut["left"] = Conf(q);
  dut["right"] = Conf(q2);
  auto msg = ToProto(dut);
  auto dut_deserialized = FromProto<SystemConf>(msg);
  EXPECT_TRUE(dut["left"].q().isApprox(dut_deserialized["left"].q()));
  EXPECT_TRUE(dut["right"].q().isApprox(dut_deserialized["right"].q()));
}
}  // namespace planning_service_client
