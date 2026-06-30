#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <grpcpp/test/mock_stream.h>

#include "client/cpp/planning_service_client/api/visualizer_client.h"
#include "proto/visualizer.grpc.pb.h"
#include "proto/visualizer_mock.grpc.pb.h"

namespace planning_service_client {

using ::testing::_;
using ::testing::DoAll;
using ::testing::Return;
using ::testing::SaveArg;
using ::testing::SetArgPointee;

class MockVisualizerClient : public client::VisualizerClient {
 public:
  MockVisualizerClient() : VisualizerClient("nonexistent:1234", "test_client") {
    auto mock_stub = std::make_unique<proto::MockVisualizerStub>();
    stub_ptr_ = mock_stub.get();
    SetStub(std::move(mock_stub));
  }

  proto::MockVisualizerStub* stub_ptr_;
};

class TestVisualizerClient : public ::testing::Test {
 protected:
  virtual void SetUp() override {
    client_ = std::make_unique<MockVisualizerClient>();
  }
  std::unique_ptr<MockVisualizerClient> client_;
};

TEST_F(TestVisualizerClient, GetVisualizerStatus) {
  proto::GetVisualizerStatusRequest request;
  proto::GetVisualizerStatusResponse response;
  EXPECT_CALL(*client_->stub_ptr_, GetVisualizerStatus(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), SetArgPointee<2>(response),
                      Return(grpc::Status::OK)));
  client_->GetVisualizerStatus();
  EXPECT_EQ(request.SerializeAsString(),
            proto::GetVisualizerStatusRequest().SerializeAsString());
}

TEST_F(TestVisualizerClient, GetVisualizerStatusError) {
  EXPECT_CALL(*client_->stub_ptr_, GetVisualizerStatus(_, _, _))
      .Times(1)
      .WillOnce(Return(grpc::Status(grpc::StatusCode::INTERNAL, "Error")));
  EXPECT_THROW(client_->GetVisualizerStatus(), std::runtime_error);
}

TEST_F(TestVisualizerClient, StreamConfigurations) {
  auto writer =
      new grpc::testing::MockClientWriter<proto::StreamConfigurationsRequest>();
  SystemConf sys_conf;
  sys_conf["robot1"] = Conf(Eigen::VectorXd::Constant(3, 1.0));
  proto::StreamConfigurationsRequest expected_msg;
  expected_msg.mutable_system_conf()->CopyFrom(ToProto(sys_conf));
  proto::StreamConfigurationsRequest streamed_msg;
  EXPECT_CALL(*writer, Write(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&streamed_msg), Return(true)));
  client_->QueueStreamedConfiguration(sys_conf);
  EXPECT_CALL(*client_->stub_ptr_, StreamConfigurationsRaw(_, _))
      .Times(1)
      .WillOnce(Return(writer));
  client_->StreamConfigurationsAsync();
  // Give some time for the streaming thread to process the queued config.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  client_->StopStreamConfigurations();
  EXPECT_EQ(streamed_msg.SerializeAsString(), expected_msg.SerializeAsString());
}

TEST_F(TestVisualizerClient, ToggleObject) {
  proto::ToggleObjectRequest request;
  EXPECT_CALL(*client_->stub_ptr_, ToggleObject(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), Return(grpc::Status::OK)));
  client_->ToggleObject("path/to/obj", false);
  EXPECT_EQ(request.path(), "path/to/obj");
  EXPECT_FALSE(request.visible());
}

TEST_F(TestVisualizerClient, ToggleObjectError) {
  EXPECT_CALL(*client_->stub_ptr_, ToggleObject(_, _, _))
      .Times(1)
      .WillOnce(Return(grpc::Status(grpc::StatusCode::INTERNAL, "Error")));

  EXPECT_THROW(client_->ToggleObject("path/to/obj", false), std::runtime_error);
}

TEST_F(TestVisualizerClient, ToggleFrame) {
  proto::ToggleFrameRequest request;
  EXPECT_CALL(*client_->stub_ptr_, ToggleFrame(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), Return(grpc::Status::OK)));
  client_->ToggleFrame("world", true);
  EXPECT_EQ(request.frame(), "world");
  EXPECT_TRUE(request.visible());
}

TEST_F(TestVisualizerClient, ToggleFrameVisibleFalse) {
  proto::ToggleFrameRequest request;
  EXPECT_CALL(*client_->stub_ptr_, ToggleFrame(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), Return(grpc::Status::OK)));
  client_->ToggleFrame("base_link", false);
  EXPECT_EQ(request.frame(), "base_link");
  EXPECT_FALSE(request.visible());
}

TEST_F(TestVisualizerClient, ToggleFrameError) {
  EXPECT_CALL(*client_->stub_ptr_, ToggleFrame(_, _, _))
      .Times(1)
      .WillOnce(Return(grpc::Status(grpc::StatusCode::INTERNAL, "Error")));
  EXPECT_THROW(client_->ToggleFrame("world", true), std::runtime_error);
}

TEST_F(TestVisualizerClient, SetObject) {
  const std::string object_path = "path/to/object";
  ShapeInFrame shape_in_frame;
  shape_in_frame.set_frame("world");
  shape_in_frame.set_translation(Eigen::Vector3d(1.0, 2.0, 3.0));
  shape_in_frame.set_quaternion(Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  shape_in_frame.set_shape(Sphere(0.5));
  const proto::ShapeInFrame expected_shape_in_frame = ToProto(shape_in_frame);

  proto::SetObjectRequest request;
  EXPECT_CALL(*client_->stub_ptr_, SetObject(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), Return(grpc::Status::OK)));

  client_->SetObject(object_path, shape_in_frame);

  EXPECT_EQ(request.path(), object_path);
  EXPECT_EQ(request.shape_in_frame().SerializeAsString(),
            expected_shape_in_frame.SerializeAsString());
}

TEST_F(TestVisualizerClient, SetObjectError) {
  ShapeInFrame shape_in_frame;
  shape_in_frame.set_shape(Sphere(0.5));

  EXPECT_CALL(*client_->stub_ptr_, SetObject(_, _, _))
      .Times(1)
      .WillOnce(Return(grpc::Status(grpc::StatusCode::INTERNAL, "Error")));

  EXPECT_THROW(client_->SetObject("path/to/object", shape_in_frame),
               std::runtime_error);
}

TEST_F(TestVisualizerClient, DeleteObject) {
  proto::DeleteObjectRequest request;
  EXPECT_CALL(*client_->stub_ptr_, DeleteObject(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), Return(grpc::Status::OK)));

  client_->DeleteObject("path/to/object");

  EXPECT_EQ(request.path(), "path/to/object");
}

TEST_F(TestVisualizerClient, DeleteObjectError) {
  EXPECT_CALL(*client_->stub_ptr_, DeleteObject(_, _, _))
      .Times(1)
      .WillOnce(Return(grpc::Status(grpc::StatusCode::INTERNAL, "Error")));

  EXPECT_THROW(client_->DeleteObject("path/to/object"), std::runtime_error);
}

TEST_F(TestVisualizerClient, CalcPose) {
  proto::CalcPoseRequest request;
  proto::CalcPoseResponse response;
  auto& X_AB = *response.mutable_pose()->mutable_x_ab();
  X_AB.mutable_translation()->set_x(1.0);
  X_AB.mutable_quat()->set_w(1.0);
  EXPECT_CALL(*client_->stub_ptr_, CalcPose(_, _, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<1>(&request), SetArgPointee<2>(response),
                      Return(grpc::Status::OK)));
  auto pose = client_->CalcPose("frame_a", "frame_b");
  EXPECT_EQ(pose.X_AB_translation().x(), 1.0);
  EXPECT_EQ(pose.X_AB_quaternion().w(), 1.0);
}
}  // namespace planning_service_client
