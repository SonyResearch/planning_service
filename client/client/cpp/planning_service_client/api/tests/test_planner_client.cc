#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include "client/cpp/planning_service_client/api/planner_client.h"
#include "planning_service_client/planner/global_ik_problem.h"
#include "proto/planner_mock.grpc.pb.h"

namespace planning_service_client {

using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SaveArg;
using ::testing::SetArgPointee;

class MockMotionPlannerClient : public client::MotionPlannerClient {
 public:
  MockMotionPlannerClient()
      : MotionPlannerClient("nonexistent:1234", "test_client") {
    auto mock_stub = std::make_unique<proto::MockMotionPlannerStub>();
    stub_ptr_ = mock_stub.get();
    SetStub(std::move(mock_stub));
  }

  proto::MockMotionPlannerStub* stub_ptr_;
};

class TestPlannerClient : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    client_ = std::make_unique<MockMotionPlannerClient>();
  }

  std::unique_ptr<MockMotionPlannerClient> client_;
};

TEST_F(TestPlannerClient, SetPoseInFrame) {
  EXPECT_CALL(*client_->stub_ptr_, SetPoseInParentFrame(_, _, _))
      .Times(1)
      .WillOnce(Return(grpc::Status::OK));
  FrameRelativePose frp("frame_A", "frame_B", Eigen::Vector3d(1.0, 2.0, 3.0),
                        Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(client_->SetPoseInFrame(frp));
}

TEST_F(TestPlannerClient, SetPoseInFrameWithTransactionId) {
  proto::SetPoseInParentFrameRequest request;
  EXPECT_CALL(*client_->stub_ptr_, SetPoseInParentFrame(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), Return(grpc::Status::OK)));
  FrameRelativePose frp("frame_A", "frame_B", Eigen::Vector3d(1.0, 2.0, 3.0),
                        Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  EXPECT_TRUE(client_->SetPoseInFrame(frp, "txn-123"));
  EXPECT_EQ(request.frame_relative_pose().frame_a(), "frame_A");
  EXPECT_EQ(request.frame_relative_pose().frame_b(), "frame_B");
}

TEST_F(TestPlannerClient, CalcRelativePoseWithTransactionId) {
  proto::CalcRelativePoseRequest request;
  proto::CalcRelativePoseResponse response;
  EXPECT_CALL(*client_->stub_ptr_, CalcRelativePose(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), SetArgPointee<2>(response),
                      Return(grpc::Status::OK)));
  ContextId context_id(42, "test_system");
  proto::SystemConf system_conf;
  auto [resp, status] = client_->CalcRelativePose(
      context_id, system_conf, "frame_B", "world", "txn-456");
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(request.frame_b(), "frame_B");
  EXPECT_EQ(request.frame_a(), "world");
}

TEST_F(TestPlannerClient, CheckSatisfiedWithTransactionId) {
  proto::CheckSatisfiedRequest request;
  proto::CheckSatisfiedResponse response;
  response.set_satisfied(true);
  EXPECT_CALL(*client_->stub_ptr_, CheckSatisfied(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), SetArgPointee<2>(response),
                      Return(grpc::Status::OK)));
  ContextId context_id(42, "test_system");
  proto::SystemConf system_conf;
  EXPECT_NO_THROW(client_->CheckSatisfied(context_id, {system_conf},
                                          /*collision_options=*/std::nullopt,
                                          /*options=*/std::nullopt, "txn-789"));
}

TEST_F(TestPlannerClient, SolvePlanWithTransactionId) {
  proto::SolvePlanRequest request;
  proto::SolvePlanResponse response;
  response.mutable_result()->set_is_success(false);
  response.mutable_result()->set_failure_status(proto::FAILURE_STATUS_GENERAL);
  EXPECT_CALL(*client_->stub_ptr_, SolvePlan(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), SetArgPointee<2>(response),
                      Return(grpc::Status::OK)));
  ContextId context_id(42, "test_system");
  planner::GlobalIKProblem problem;
  SystemConf start_conf;
  planner::MotionProblemDefinition def("test_plan", context_id, problem,
                                       start_conf);
  EXPECT_NO_THROW(client_->SolvePlan(def, "txn-abc"));
}

}  // namespace planning_service_client
