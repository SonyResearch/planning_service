#include <gtest/gtest.h>

#include "planning_service_client/planner/fixed_frames_motion.h"
#include "test_utils.h"

namespace planning_service_client {
namespace planner {

TEST(FixedFramesMotion, ToProtoFromProto) {
  auto system_conf = test::RandomSystemConf(2);
  std::string frame_A = "frame_A";
  std::string frame_B = "frame_B";
  FixedFramesMotion fixed_frames_motion_problem(system_conf, frame_A, frame_B);
  auto proto = ToProto(fixed_frames_motion_problem);
  auto dut = FromProto<FixedFramesMotion>(proto);
  EXPECT_TRUE(test::NearEqualSystemConf(dut.system_conf(), system_conf));
  EXPECT_EQ(dut.frame_A(), frame_A);
  EXPECT_EQ(dut.frame_B(), frame_B);
}

}  // namespace planner
}  // namespace planning_service_client
