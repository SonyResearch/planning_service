#include <gtest/gtest.h>

#include <google/protobuf/util/message_differencer.h>

#include "client/cpp/planning_service_client/internal/eigen_utils.h"

namespace planning_service_client {
namespace internal {

TEST(EigenVector3dToProto, Basic) {
  Eigen::Vector3d vec;
  vec << 1.0, 2.0, 3.0;
  proto::Vector3 msg = EigenVector3dToProto(vec);
  EXPECT_DOUBLE_EQ(msg.x(), 1.0);
  EXPECT_DOUBLE_EQ(msg.y(), 2.0);
  EXPECT_DOUBLE_EQ(msg.z(), 3.0);
}

TEST(ProtoToEigenVector3d, Basic) {
  proto::Vector3 msg;
  msg.set_x(1.0);
  msg.set_y(2.0);
  msg.set_z(3.0);
  Eigen::Vector3d vec = ProtoToEigenVector3d(msg);
  EXPECT_DOUBLE_EQ(vec[0], 1.0);
  EXPECT_DOUBLE_EQ(vec[1], 2.0);
  EXPECT_DOUBLE_EQ(vec[2], 3.0);
}

}  // namespace internal
}  // namespace planning_service_client
