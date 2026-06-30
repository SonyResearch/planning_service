#include <gtest/gtest.h>

#include "planning_service_client/planner/anchor.h"

namespace planning_service_client {
namespace planner {

TEST(Anchor, ToProtoFromProto) {
  SystemConf system_conf;
  system_conf["robot_1"] = Conf(Eigen::VectorXd::Ones(3));
  std::vector<FrameRelativePose> wayposes;
  wayposes.push_back(FrameRelativePose("frame_A", "frame_B",
                                       Eigen::Vector3d(1, 2, 3),
                                       Eigen::Quaterniond(1, 0, 0, 0)));
  wayposes.push_back(FrameRelativePose("frame_B", "frame_C",
                                       Eigen::Vector3d(4, 5, 6),
                                       Eigen::Quaterniond(1, 0, 0, 0)));
  Anchor anchor(system_conf, wayposes);
  auto msg = ToProto(anchor);
  auto anchor_back = FromProto<Anchor>(msg);
  EXPECT_EQ(anchor_back.system_conf().at("robot_1").q(),
            system_conf["robot_1"].q());
  EXPECT_EQ(anchor_back.poses().size(), wayposes.size());
}

}  // namespace planner
}  // namespace planning_service_client
