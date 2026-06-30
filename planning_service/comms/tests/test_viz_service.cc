#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <atomic>
#include <thread>

#include <grpcpp/test/mock_stream.h>

#include "planning_service/comms/viz_service.h"

namespace comms {
namespace test {

using ::testing::_;
using ::testing::DoAll;
using ::testing::Invoke;
using ::testing::Return;
using ::testing::SaveArg;
using ::testing::SetArgPointee;
using ::testing::StrictMock;
using ::testing::Throw;

class MockVisualizerHub
    : public service::visualization::VisualizerHubInterface {
 public:
  MOCK_METHOD(bool, LoadAndSetModelData,
              (const service::visualization::VisualizerData& visualizer_data,
               bool force_reload),
              (override));
  MOCK_METHOD((std::expected<bool, std::string>), SetConfiguration,
              (const psc::SystemConf& system_conf), (override));
  MOCK_METHOD((std::expected<bool, std::string>), SetConfiguration,
              (const Eigen::VectorXd& q), (override));
  MOCK_METHOD(const drake::math::RigidTransformd, CalcPose,
              (std::string_view frame_B_name, std::string_view frame_A_name,
               const std::optional<psc::SystemConf>& system_conf),
              (const, override));
  MOCK_METHOD(bool, HasDraco, (), (const, override));
  MOCK_METHOD(size_t, ActiveHash, (), (const, override));
  MOCK_METHOD(void, RequestStop, (), (override));
  MOCK_METHOD(void, WaitOnStatus,
              (const service::visualization::VisualizerStatus& status),
              (const, override));
  MOCK_METHOD(service::visualization::VisualizerStatus, GetStatus, (),
              (const, override));
  MOCK_METHOD(void, SetVisualizerOptions,
              (const service::visualization::VisualizerOptions& options),
              (override));
  MOCK_METHOD(service::visualization::VisualizerOptions, GetVisualizerOptions,
              (), (const, override));
  MOCK_METHOD(void, QueueMeshcatTask,
              (const service::visualization::meshcat_task_t& f), (override));
  MOCK_METHOD(void, ClearStreamingState, (), (override));
  MOCK_METHOD(std::optional<std::string>, ResolveFrameMeshcatPath,
              (std::string_view frame_name), (const, override));
  MOCK_METHOD(bool, ToggleFrame, (std::string_view path, bool visible),
              (override));
  MOCK_METHOD(
      (std::vector<std::pair<std::string, drake::math::RigidTransformd>>),
      ToggleFramesByPath, (std::string_view path, bool active), (override));
};

class TestVisualizerService;

/** Thin subclass that exposes DoStreamConfigurations for testing. */
class VisualizerServiceStub : public comms::VisualizerService {
 public:
  using comms::VisualizerService::DoSetConfiguration;
  using comms::VisualizerService::DoStreamConfigurations;
  using comms::VisualizerService::VisualizerService;

 private:
  friend class TestVisualizerService;
};

class TestVisualizerService : public ::testing::Test {
 protected:
  void SetUp() override {
    hub = std::make_shared<StrictMock<MockVisualizerHub>>();
    service = std::make_unique<VisualizerServiceStub>(hub);
    meshcat = std::make_unique<drake::geometry::Meshcat>();
    box_in_frame = psc::ShapeInFrame(psc::Box(1.0, 2.0, 3.0));
    box_in_frame.set_frame("parent_frame");
  }
  psc::ShapeInFrame box_in_frame;
  std::shared_ptr<StrictMock<MockVisualizerHub>> hub;
  std::unique_ptr<VisualizerServiceStub> service;
  std::unique_ptr<drake::geometry::Meshcat> meshcat;
};

TEST_F(TestVisualizerService, StartVisualizer_ContextId_Success) {
  proto::StartVisualizerRequest req;
  req.mutable_context_id()->set_value(123);
  req.set_force_reload(false);
  req.mutable_options()->set_enable_sliders(true);
  req.mutable_options()->set_show_iris_regions(false);
  req.mutable_options()->set_show_prm(true);

  service::visualization::VisualizerOptions captured_options;
  service::visualization::VisualizerData captured_data;

  EXPECT_CALL(*hub, GetVisualizerOptions())
      .WillOnce(Return(service::visualization::VisualizerOptions {}));
  EXPECT_CALL(*hub, SetVisualizerOptions(_))
      .WillOnce(SaveArg<0>(&captured_options));
  EXPECT_CALL(*hub, LoadAndSetModelData(_, false))
      .WillOnce(DoAll(SaveArg<0>(&captured_data), Return(true)));
  EXPECT_CALL(*hub,
              WaitOnStatus(service::visualization::VisualizerStatus::Starting))
      .Times(1);
  EXPECT_CALL(*hub, ActiveHash()).WillOnce(Return(123));

  proto::StartVisualizerResponse resp;
  const auto status = service->StartVisualizer(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_TRUE(captured_options.enable_sliders);
  EXPECT_FALSE(captured_options.show_iris_regions);
  EXPECT_TRUE(captured_options.show_prm);
  ASSERT_TRUE(captured_data.context_id.has_value());
  EXPECT_EQ(captured_data.context_id->value, 123);
  EXPECT_FALSE(captured_data.dmd_filename.has_value());
}

TEST_F(TestVisualizerService, StartVisualizer_DmdFilename_Success) {
  proto::StartVisualizerRequest req;
  req.set_dmd_filename("foo.yaml");
  req.set_force_reload(true);

  service::visualization::VisualizerData captured_data;
  EXPECT_CALL(*hub, GetVisualizerOptions())
      .WillOnce(Return(service::visualization::VisualizerOptions {}));
  EXPECT_CALL(*hub, SetVisualizerOptions(_)).Times(1);
  EXPECT_CALL(*hub, LoadAndSetModelData(_, true))
      .WillOnce(DoAll(SaveArg<0>(&captured_data), Return(true)));
  EXPECT_CALL(*hub,
              WaitOnStatus(service::visualization::VisualizerStatus::Starting))
      .Times(1);
  EXPECT_CALL(*hub, ActiveHash()).WillOnce(Return(1));

  proto::StartVisualizerResponse resp;
  const auto status = service->StartVisualizer(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  ASSERT_TRUE(captured_data.dmd_filename.has_value());
  EXPECT_EQ(*captured_data.dmd_filename, "foo.yaml");
  EXPECT_FALSE(captured_data.context_id.has_value());
}

TEST_F(TestVisualizerService, StartVisualizer_LoadFailure_ResourceExhausted) {
  proto::StartVisualizerRequest req;
  req.mutable_context_id()->set_value(123);

  EXPECT_CALL(*hub, GetVisualizerOptions())
      .WillOnce(Return(service::visualization::VisualizerOptions {}));
  EXPECT_CALL(*hub, SetVisualizerOptions(_)).Times(1);
  EXPECT_CALL(*hub, LoadAndSetModelData(_, false)).WillOnce(Return(false));
  EXPECT_CALL(*hub, WaitOnStatus(_)).Times(0);
  EXPECT_CALL(*hub, ActiveHash()).Times(0);

  proto::StartVisualizerResponse resp;
  const auto status = service->StartVisualizer(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::RESOURCE_EXHAUSTED);
}

TEST_F(TestVisualizerService, StartVisualizer_Exception_Internal) {
  proto::StartVisualizerRequest req;
  req.mutable_context_id()->set_value(123);

  EXPECT_CALL(*hub, GetVisualizerOptions())
      .WillOnce(Return(service::visualization::VisualizerOptions {}));
  EXPECT_CALL(*hub, SetVisualizerOptions(_)).Times(1);
  EXPECT_CALL(*hub, LoadAndSetModelData(_, false))
      .WillOnce(Throw(std::runtime_error("mock error")));
  EXPECT_CALL(*hub, WaitOnStatus(_)).Times(0);
  EXPECT_CALL(*hub, ActiveHash()).Times(0);

  proto::StartVisualizerResponse resp;
  const auto status = service->StartVisualizer(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::INTERNAL);
}

TEST_F(TestVisualizerService, StopVisualizer_Success) {
  EXPECT_CALL(*hub, RequestStop()).Times(1);
  EXPECT_CALL(*hub,
              WaitOnStatus(service::visualization::VisualizerStatus::Stopping))
      .Times(1);

  proto::StopVisualizerRequest req;
  proto::StopVisualizerResponse resp;
  const auto status = service->StopVisualizer(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
}

TEST_F(TestVisualizerService, GetVisualizerStatus_Success) {
  EXPECT_CALL(*hub, GetStatus())
      .WillOnce(Return(service::visualization::VisualizerStatus::Idle));

  proto::GetVisualizerStatusRequest req;
  proto::GetVisualizerStatusResponse resp;
  const auto status = service->GetVisualizerStatus(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(resp.status(), proto::VisualizerStatus::VISUALIZER_STATUS_IDLE);
}

TEST_F(TestVisualizerService, GetVisualizerStatus_ParseFailure_Internal) {
  EXPECT_CALL(*hub, GetStatus())
      .WillOnce(Return(service::visualization::VisualizerStatus::None));

  proto::GetVisualizerStatusRequest req;
  proto::GetVisualizerStatusResponse resp;
  const auto status = service->GetVisualizerStatus(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::INTERNAL);
}

TEST_F(TestVisualizerService, SetConfiguration_Conf_Success) {
  EXPECT_CALL(*hub, SetConfiguration(::testing::A<const Eigen::VectorXd&>()))
      .WillOnce(Invoke(
          [](const Eigen::VectorXd& q) -> std::expected<bool, std::string> {
            EXPECT_EQ(q.size(), 3);
            EXPECT_DOUBLE_EQ(q[0], 1.0);
            EXPECT_DOUBLE_EQ(q[1], 2.0);
            EXPECT_DOUBLE_EQ(q[2], 3.0);
            return true;
          }));

  proto::SetConfigurationRequest req;
  req.mutable_conf()->add_data(1.0);
  req.mutable_conf()->add_data(2.0);
  req.mutable_conf()->add_data(3.0);
  proto::SetConfigurationResponse resp;
  const auto status = service->SetConfiguration(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
}

TEST_F(TestVisualizerService, SetConfiguration_SystemConf_Success) {
  EXPECT_CALL(*hub, SetConfiguration(::testing::A<const psc::SystemConf&>()))
      .WillOnce(Return(true));

  proto::SetConfigurationRequest req;
  (*req.mutable_system_conf()->mutable_data())["robot1"].add_data(0.5);
  proto::SetConfigurationResponse resp;
  const auto status = service->SetConfiguration(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
}

TEST_F(TestVisualizerService, SetConfiguration_MissingData_Internal) {
  proto::SetConfigurationRequest req;
  proto::SetConfigurationResponse resp;
  const auto status = service->SetConfiguration(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::INTERNAL);
}

TEST_F(TestVisualizerService, CalcPose_Success) {
  EXPECT_CALL(*hub, CalcPose("tool", "world", _))
      .WillOnce(Return(drake::math::RigidTransformd()));

  proto::CalcPoseRequest req;
  req.set_frame_a("world");
  req.set_frame_b("tool");
  proto::CalcPoseResponse resp;
  const auto status = service->CalcPose(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(resp.pose().frame_a(), "world");
  EXPECT_EQ(resp.pose().frame_b(), "tool");
}

TEST_F(TestVisualizerService, CalcPose_Exception_Internal) {
  EXPECT_CALL(*hub, CalcPose(_, _, _))
      .WillOnce(Throw(std::runtime_error("bad")));

  proto::CalcPoseRequest req;
  req.set_frame_a("world");
  req.set_frame_b("tool");
  proto::CalcPoseResponse resp;
  const auto status = service->CalcPose(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::INTERNAL);
}

TEST_F(TestVisualizerService, CalcPose_EmptyFrameA_DefaultsToWorld) {
  EXPECT_CALL(*hub, CalcPose("tool", "world", _))
      .WillOnce(Return(drake::math::RigidTransformd()));

  proto::CalcPoseRequest req;
  req.set_frame_b("tool");
  proto::CalcPoseResponse resp;
  const auto status = service->CalcPose(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(resp.pose().frame_a(), "world");
  EXPECT_EQ(resp.pose().frame_b(), "tool");
}

TEST_F(TestVisualizerService, CalcPose_NoSystemConfOverride_PassesNullopt) {
  std::optional<psc::SystemConf> captured_override {psc::SystemConf {}};
  EXPECT_CALL(*hub, CalcPose("tool", "world", _))
      .WillOnce(DoAll(SaveArg<2>(&captured_override),
                      Return(drake::math::RigidTransformd())));

  proto::CalcPoseRequest req;
  req.set_frame_a("world");
  req.set_frame_b("tool");
  proto::CalcPoseResponse resp;
  const auto status = service->CalcPose(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_FALSE(captured_override.has_value());
}

TEST_F(TestVisualizerService, CalcPose_WithSystemConfOverride_PassedToHub) {
  std::optional<psc::SystemConf> captured_override;
  EXPECT_CALL(*hub, CalcPose("tool", "world", _))
      .WillOnce(DoAll(SaveArg<2>(&captured_override),
                      Return(drake::math::RigidTransformd())));

  proto::CalcPoseRequest req;
  req.set_frame_a("world");
  req.set_frame_b("tool");
  (*req.mutable_system_conf_override()->mutable_data())["robot1"].add_data(0.5);
  proto::CalcPoseResponse resp;
  const auto status = service->CalcPose(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  ASSERT_TRUE(captured_override.has_value());
  EXPECT_TRUE(captured_override->contains("robot1"));
  EXPECT_EQ(captured_override->at("robot1").q().size(), 1);
  EXPECT_DOUBLE_EQ(captured_override->at("robot1").q()[0], 0.5);
}

TEST_F(TestVisualizerService, CalcPose_PoseValuesPopulated) {
  const Eigen::Vector3d expected_t(1.0, 2.0, 3.0);
  EXPECT_CALL(*hub, CalcPose("tool", "world", _))
      .WillOnce(Return(drake::math::RigidTransformd(expected_t)));

  proto::CalcPoseRequest req;
  req.set_frame_a("world");
  req.set_frame_b("tool");
  proto::CalcPoseResponse resp;
  const auto status = service->CalcPose(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_DOUBLE_EQ(resp.pose().x_ab().translation().x(), 1.0);
  EXPECT_DOUBLE_EQ(resp.pose().x_ab().translation().y(), 2.0);
  EXPECT_DOUBLE_EQ(resp.pose().x_ab().translation().z(), 3.0);
}

TEST_F(TestVisualizerService, DisplayTrajectory_Unimplemented) {
  proto::DisplayTrajectoryRequest req;
  proto::DisplayTrajectoryResponse resp;
  const auto status = service->DisplayTrajectory(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::UNIMPLEMENTED);
}

TEST_F(TestVisualizerService, SetObject_RelativePath) {
  proto::SetObjectRequest req;
  proto::SetObjectResponse resp;
  req.set_path("my_object");
  req.mutable_shape_in_frame()->CopyFrom(psc::ToProto(box_in_frame));
  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, CalcPose("parent_frame", "world", _))
      .WillOnce(Return(drake::math::RigidTransformd()));
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));
  const auto status = service->SetObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_NO_THROW(captured_task(*meshcat));
  EXPECT_TRUE(meshcat->HasPath("/drake/objects/my_object"));
}

TEST_F(TestVisualizerService, SetObject_AbsolutePath) {
  proto::SetObjectRequest req;
  proto::SetObjectResponse resp;
  req.set_path("/drake/objects/my_object");
  req.mutable_shape_in_frame()->CopyFrom(psc::ToProto(box_in_frame));
  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, CalcPose("parent_frame", "world", _))
      .WillOnce(Return(drake::math::RigidTransformd()));
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));
  const auto status = service->SetObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_NO_THROW(captured_task(*meshcat));
  EXPECT_TRUE(meshcat->HasPath("/drake/objects/my_object"));
}

TEST_F(TestVisualizerService, SetObject_ModelGeometry_Disallowed) {
  proto::SetObjectRequest req;
  proto::SetObjectResponse resp;
  req.set_path("/drake/visual/thing");
  req.mutable_shape_in_frame()->CopyFrom(psc::ToProto(box_in_frame));
  const auto status = service->SetObject(nullptr, &req, &resp);
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.error_code(), grpc::StatusCode::PERMISSION_DENIED);
  req.set_path("/drake/collision/thing");
  const auto status2 = service->SetObject(nullptr, &req, &resp);
  EXPECT_FALSE(status2.ok());
  EXPECT_EQ(status2.error_code(), grpc::StatusCode::PERMISSION_DENIED);
}

TEST_F(TestVisualizerService, SetObject_EmptyFrame) {
  proto::SetObjectRequest req;
  proto::SetObjectResponse resp;
  req.set_path("thing");
  req.mutable_shape_in_frame()->CopyFrom(psc::ToProto(box_in_frame));
  req.mutable_shape_in_frame()->clear_frame();
  // Empty frame should be defaulted to "world"
  EXPECT_CALL(*hub, CalcPose("world", "world", _))
      .WillOnce(Return(drake::math::RigidTransformd()));
  EXPECT_CALL(*hub, QueueMeshcatTask(_)).Times(1);
  const auto status = service->SetObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
}

TEST_F(TestVisualizerService, DeleteObject_RelativePath) {
  proto::DeleteObjectRequest req;
  proto::DeleteObjectResponse resp;
  req.set_path("my_object");
  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));
  const auto status = service->DeleteObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  // Set any object first, then apply, then make sure it doesn't exist anymore.
  meshcat->SetObject("/drake/objects/my_object",
                     drake::geometry::Box(1.0, 1.0, 1.0));
  EXPECT_NO_THROW(captured_task(*meshcat));
  EXPECT_FALSE(meshcat->HasPath("/drake/objects/my_object"));
}

TEST_F(TestVisualizerService, DeleteObject_AbsolutePath) {
  proto::DeleteObjectRequest req;
  proto::DeleteObjectResponse resp;
  req.set_path("/drake/objects/my_object");
  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));
  const auto status = service->DeleteObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  // Set any object first, then apply, then make sure it doesn't exist anymore.
  meshcat->SetObject("/drake/objects/my_object",
                     drake::geometry::Box(1.0, 1.0, 1.0));
  EXPECT_NO_THROW(captured_task(*meshcat));
  EXPECT_FALSE(meshcat->HasPath("/drake/objects/my_object"));
}

TEST_F(TestVisualizerService, DeleteObject_ModelGeometry_Disallowed) {
  proto::DeleteObjectRequest req;
  proto::DeleteObjectResponse resp;
  req.set_path("/drake/visual/thing");
  const auto status = service->DeleteObject(nullptr, &req, &resp);
  EXPECT_FALSE(status.ok());
  EXPECT_EQ(status.error_code(), grpc::StatusCode::PERMISSION_DENIED);
  req.set_path("/drake/collision/thing");
  const auto status2 = service->DeleteObject(nullptr, &req, &resp);
  EXPECT_FALSE(status2.ok());
  EXPECT_EQ(status2.error_code(), grpc::StatusCode::PERMISSION_DENIED);
}

TEST_F(TestVisualizerService, ToggleObject_QueuesTwoTasksWhenShorthandPath) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("thing");
  req.set_visible(true);
  // No frames match — frame task must NOT be queued.
  EXPECT_CALL(*hub, ToggleFramesByPath("thing", true))
      .WillOnce(
          Return(std::vector<
                 std::pair<std::string, drake::math::RigidTransformd>> {}));
  // Collision goes first.
  service::visualization::meshcat_task_t captured_collision_task;
  service::visualization::meshcat_task_t captured_visual_task;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(2)
      .WillOnce(SaveArg<0>(&captured_collision_task))
      .WillOnce(SaveArg<0>(&captured_visual_task));
  const auto status = service->ToggleObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  // Pre-register the paths so the HasPath guard in the task doesn't bail out.
  meshcat->SetObject("/drake/collision/thing", drake::geometry::Box(1, 1, 1));
  meshcat->SetObject("/drake/visual/thing", drake::geometry::Box(1, 1, 1));
  EXPECT_NO_THROW(captured_collision_task(*meshcat));
  EXPECT_NO_THROW(captured_visual_task(*meshcat));
  EXPECT_TRUE(meshcat->HasPath("/drake/collision/thing"));
  EXPECT_TRUE(meshcat->HasPath("/drake/visual/thing"));
}

TEST_F(TestVisualizerService, ToggleObject_QueuesOneTaskWhenExplicitSubtree) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("/drake/visual/thing");
  req.set_visible(true);
  // visual/ prefix is stripped before calling ToggleFramesByPath.
  EXPECT_CALL(*hub, ToggleFramesByPath("thing", true))
      .WillOnce(
          Return(std::vector<
                 std::pair<std::string, drake::math::RigidTransformd>> {}));
  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));
  const auto status = service->ToggleObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());
  EXPECT_NO_THROW(captured_task(*meshcat));
}

TEST_F(TestVisualizerService, StreamConfigurations_Success) {
  using MockReader =
      grpc::testing::MockServerReader<proto::StreamConfigurationsRequest>;
  auto reader = std::make_unique<MockReader>();

  proto::StreamConfigurationsRequest req;
  (*req.mutable_system_conf()->mutable_data())["r1"].add_data(0.1);

  EXPECT_CALL(*hub, GetStatus())
      .WillRepeatedly(Return(service::visualization::VisualizerStatus::Active));
  EXPECT_CALL(*reader, Read(_))
      .WillOnce(DoAll(SetArgPointee<0>(req), Return(true)))
      .WillOnce(Return(false));
  EXPECT_CALL(*hub, SetConfiguration(::testing::A<const psc::SystemConf&>()))
      .WillOnce(Return(true));
  EXPECT_CALL(*hub, ClearStreamingState()).Times(1);

  grpc::ServerContext ctx;
  proto::StreamConfigurationsResponse resp;
  const auto status =
      service->DoStreamConfigurations(&ctx, reader.get(), &resp);
  EXPECT_TRUE(status.ok());
}

// Hub returns an error mid-stream → INTERNAL status.
TEST_F(TestVisualizerService, StreamConfigurations_HubError_Internal) {
  using MockReader =
      grpc::testing::MockServerReader<proto::StreamConfigurationsRequest>;
  auto reader = std::make_unique<MockReader>();

  proto::StreamConfigurationsRequest req;
  (*req.mutable_system_conf()->mutable_data())["r1"].add_data(0.1);

  EXPECT_CALL(*hub, GetStatus())
      .WillRepeatedly(Return(service::visualization::VisualizerStatus::Active));
  EXPECT_CALL(*reader, Read(_))
      .WillOnce(DoAll(SetArgPointee<0>(req), Return(true)));
  EXPECT_CALL(*hub, SetConfiguration(::testing::A<const psc::SystemConf&>()))
      .WillOnce(Return(std::unexpected<std::string>("mock hub error")));
  EXPECT_CALL(*hub, ClearStreamingState()).Times(1);

  grpc::ServerContext ctx;
  proto::StreamConfigurationsResponse resp;
  const auto status =
      service->DoStreamConfigurations(&ctx, reader.get(), &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::INTERNAL);
}

TEST_F(TestVisualizerService, StreamConfigurations_NewClientDropsIncumbent) {
  using MockReader =
      grpc::testing::MockServerReader<proto::StreamConfigurationsRequest>;
  auto reader1 = std::make_unique<MockReader>();
  auto reader2 = std::make_unique<MockReader>();

  std::atomic<bool> stream1_exited {false};
  std::mutex sync_mtx;
  std::condition_variable sync_cv;
  bool stream1_in_read = false;

  // reader1: signal that it has been entered, linger briefly, then finish.
  EXPECT_CALL(*reader1, Read(_))
      .WillOnce(Invoke([&](proto::StreamConfigurationsRequest*) -> bool {
        {
          std::lock_guard lock(sync_mtx);
          stream1_in_read = true;
        }
        sync_cv.notify_all();
        // Stay alive long enough for stream2 to arrive and call TryCancel().
        // In a real server the cancel would propagate through the transport;
        // here we just wait the fixed window and then exit normally.
        std::this_thread::sleep_for(std::chrono::milliseconds(150));
        return false;
      }));

  // reader2: nothing to read — it just connects and disconnects cleanly.
  EXPECT_CALL(*reader2, Read(_)).WillOnce(Return(false));
  EXPECT_CALL(*hub, ClearStreamingState()).Times(2);

  grpc::ServerContext ctx1, ctx2;
  proto::StreamConfigurationsResponse resp1, resp2;

  // Launch the incumbent.
  std::thread t1([&] {
    service->DoStreamConfigurations(&ctx1, reader1.get(), &resp1);
    stream1_exited = true;
  });

  // Wait until reader1 is registered as the active streamer.
  {
    std::unique_lock lock(sync_mtx);
    sync_cv.wait(lock, [&] {
      return stream1_in_read;
    });
  }

  // Connect the new client; it must wait for the incumbent to exit.
  const auto status2 =
      service->DoStreamConfigurations(&ctx2, reader2.get(), &resp2);

  // If StreamConfigurations returned it means stream1 cleared the slot
  // (active_streamer_cv_ guarantee).  Let t1 finish setting stream1_exited.
  t1.join();

  EXPECT_TRUE(status2.ok());
  EXPECT_TRUE(stream1_exited.load());
}

TEST_F(TestVisualizerService, DoSetConfiguration_Conf_Success) {
  EXPECT_CALL(*hub, SetConfiguration(::testing::A<const Eigen::VectorXd&>()))
      .WillOnce(Invoke(
          [](const Eigen::VectorXd& q) -> std::expected<bool, std::string> {
            EXPECT_EQ(q.size(), 2);
            EXPECT_DOUBLE_EQ(q[0], 1.0);
            EXPECT_DOUBLE_EQ(q[1], 2.0);
            return true;
          }));

  proto::SetConfigurationRequest req;
  req.mutable_conf()->add_data(1.0);
  req.mutable_conf()->add_data(2.0);
  const auto result = service->DoSetConfiguration(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(*result);
}

TEST_F(TestVisualizerService, DoSetConfiguration_SystemConf_Success) {
  EXPECT_CALL(*hub, SetConfiguration(::testing::A<const psc::SystemConf&>()))
      .WillOnce(Return(true));

  proto::SetConfigurationRequest req;
  (*req.mutable_system_conf()->mutable_data())["robot1"].add_data(0.5);
  const auto result = service->DoSetConfiguration(req);
  ASSERT_TRUE(result.has_value());
  EXPECT_TRUE(*result);
}

TEST_F(TestVisualizerService, DoSetConfiguration_NoData_ReturnsUnexpected) {
  proto::SetConfigurationRequest req;
  const auto result = service->DoSetConfiguration(req);
  EXPECT_FALSE(result.has_value());
}
// ---------------------------------------------------------------------------
// ToggleObject × frame-axes interactions
// ---------------------------------------------------------------------------

// Bare path: ToggleFramesByPath is called with the bare path (no subtree
// prefix), and the subpath matches /drake/frames/{bare_path}/....
TEST_F(TestVisualizerService,
       ToggleObject_BarePath_CallsFramesWithBareSubpath) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("panda_east");
  req.set_visible(false);

  std::string captured_subpath;
  bool captured_active;
  EXPECT_CALL(*hub, ToggleFramesByPath(_, _))
      .WillOnce(DoAll(
          SaveArg<0>(&captured_subpath), SaveArg<1>(&captured_active),
          Return(std::vector<
                 std::pair<std::string, drake::math::RigidTransformd>> {})));
  EXPECT_CALL(*hub, QueueMeshcatTask(_)).Times(2);  // collision + visual

  EXPECT_TRUE(service->ToggleObject(nullptr, &req, &resp).ok());
  EXPECT_EQ(captured_subpath, "panda_east");
  EXPECT_FALSE(captured_active);
}

// visual/ prefix is stripped before ToggleFramesByPath is called.
TEST_F(TestVisualizerService,
       ToggleObject_VisualPath_StripsVisualPrefix_BeforeCallingFrames) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("visual/panda_east");
  req.set_visible(true);

  std::string captured_subpath;
  EXPECT_CALL(*hub, ToggleFramesByPath(_, _))
      .WillOnce(DoAll(
          SaveArg<0>(&captured_subpath),
          Return(std::vector<
                 std::pair<std::string, drake::math::RigidTransformd>> {})));
  EXPECT_CALL(*hub, QueueMeshcatTask(_)).Times(1);  // visual only

  EXPECT_TRUE(service->ToggleObject(nullptr, &req, &resp).ok());
  // Must NOT include the "visual/" prefix.
  EXPECT_EQ(captured_subpath, "panda_east");
}

// collision/ path must NOT call ToggleFramesByPath at all.
TEST_F(TestVisualizerService, ToggleObject_CollisionPath_DoesNotAffectFrames) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("collision/panda_east");
  req.set_visible(false);

  EXPECT_CALL(*hub, ToggleFramesByPath(_, _)).Times(0);
  EXPECT_CALL(*hub, QueueMeshcatTask(_)).Times(1);  // collision only

  EXPECT_TRUE(service->ToggleObject(nullptr, &req, &resp).ok());
}

// When ToggleFramesByPath returns transforms, a third task is queued that
// applies each one via SetTransform.
TEST_F(TestVisualizerService,
       ToggleObject_NonEmptyFrameUpdates_QueuesFrameTransformTask) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("panda_east");
  req.set_visible(false);

  // Two frame axes to teleport.
  const std::string path_a {"/drake/frames/panda_east/link1/"};
  const std::string path_b {"/drake/frames/panda_east/link2/"};
  drake::math::RigidTransformd X_far;
  X_far.set_translation(Eigen::Vector3d(0, 0, -1e6));
  std::vector<std::pair<std::string, drake::math::RigidTransformd>>
      frame_updates {{path_a, X_far}, {path_b, X_far}};

  EXPECT_CALL(*hub, ToggleFramesByPath("panda_east", false))
      .WillOnce(Return(frame_updates));

  // 2 geometry tasks + 1 frame task.
  std::vector<service::visualization::meshcat_task_t> captured_tasks;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(3)
      .WillRepeatedly(Invoke(
          [&captured_tasks](const service::visualization::meshcat_task_t& t) {
            captured_tasks.push_back(t);
          }));

  EXPECT_TRUE(service->ToggleObject(nullptr, &req, &resp).ok());
  ASSERT_EQ(captured_tasks.size(), 3u);

  // Register the paths so SetTransform doesn't silently skip them.
  meshcat->SetObject(path_a, drake::geometry::Sphere(0.01));
  meshcat->SetObject(path_b, drake::geometry::Sphere(0.01));

  // Execute the frame task (last one queued).
  EXPECT_NO_THROW(captured_tasks[2](*meshcat));
}

// When ToggleFramesByPath returns an empty vector, no extra task is queued.
TEST_F(TestVisualizerService,
       ToggleObject_EmptyFrameUpdates_NoExtraQueuedTask) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("panda_east");
  req.set_visible(true);

  EXPECT_CALL(*hub, ToggleFramesByPath("panda_east", true))
      .WillOnce(
          Return(std::vector<
                 std::pair<std::string, drake::math::RigidTransformd>> {}));
  // Only the 2 geometry tasks; no frame task.
  EXPECT_CALL(*hub, QueueMeshcatTask(_)).Times(2);

  EXPECT_TRUE(service->ToggleObject(nullptr, &req, &resp).ok());
}

// ---------------------------------------------------------------------------
// ToggleObject × objects/ subtree
// ---------------------------------------------------------------------------

// Hiding an objects/ path teleports it; no collision/visual fan-out and no
// ToggleFramesByPath call.
TEST_F(TestVisualizerService, ToggleObject_ObjectsPath_Hide_TeleportsAway) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("objects/my_box");
  req.set_visible(false);

  // ToggleFramesByPath must NOT be called for objects/ paths.
  EXPECT_CALL(*hub, ToggleFramesByPath(_, _)).Times(0);

  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));

  const auto status = service->ToggleObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());

  // Validate the task acts on the correct Meshcat path.
  meshcat->SetObject("objects/my_box", drake::geometry::Box(1, 1, 1));
  EXPECT_NO_THROW(captured_task(*meshcat));
}

// visible=true with no prior SetObject: no task queued (can't restore unknown
// transform).
TEST_F(TestVisualizerService,
       ToggleObject_ObjectsPath_ShowNoStoredTransform_NoTaskQueued) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("objects/my_box");
  req.set_visible(true);

  EXPECT_CALL(*hub, ToggleFramesByPath(_, _)).Times(0);
  EXPECT_CALL(*hub, QueueMeshcatTask(_)).Times(0);

  EXPECT_TRUE(service->ToggleObject(nullptr, &req, &resp).ok());
}

// visible=true after a SetObject: exactly one restore task is queued.
TEST_F(TestVisualizerService,
       ToggleObject_ObjectsPath_ShowAfterSet_QueuesRestoreTask) {
  // Populate the transform store via a real SetObject call.
  {
    proto::SetObjectRequest set_req;
    proto::SetObjectResponse set_resp;
    set_req.set_path("my_box");
    psc::ShapeInFrame sif(psc::Box(1.0, 1.0, 1.0));
    set_req.mutable_shape_in_frame()->CopyFrom(psc::ToProto(sif));
    drake::math::RigidTransformd X_WO;
    X_WO.set_translation(Eigen::Vector3d(1.0, 2.0, 3.0));
    EXPECT_CALL(*hub, CalcPose("world", "world", _)).WillOnce(Return(X_WO));
    EXPECT_CALL(*hub, QueueMeshcatTask(_)).Times(1);
    ASSERT_TRUE(service->SetObject(nullptr, &set_req, &set_resp).ok());
  }
  // Now toggling visible=true should queue one restore task.
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("objects/my_box");
  req.set_visible(true);

  EXPECT_CALL(*hub, ToggleFramesByPath(_, _)).Times(0);

  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));

  EXPECT_TRUE(service->ToggleObject(nullptr, &req, &resp).ok());
  meshcat->SetObject("objects/my_box", drake::geometry::Box(1, 1, 1));
  EXPECT_NO_THROW(captured_task(*meshcat));
}

// Absolute /drake/ prefix is stripped before objects/ check.
TEST_F(TestVisualizerService,
       ToggleObject_AbsoluteObjectsPath_QueuesOneTaskNoFrames) {
  proto::ToggleObjectRequest req;
  proto::ToggleObjectResponse resp;
  req.set_path("/drake/objects/my_box");
  req.set_visible(false);

  EXPECT_CALL(*hub, ToggleFramesByPath(_, _)).Times(0);

  service::visualization::meshcat_task_t captured_task;
  EXPECT_CALL(*hub, QueueMeshcatTask(_))
      .Times(1)
      .WillOnce(SaveArg<0>(&captured_task));

  const auto status = service->ToggleObject(nullptr, &req, &resp);
  EXPECT_TRUE(status.ok());

  meshcat->SetObject("objects/my_box", drake::geometry::Box(1, 1, 1));
  EXPECT_NO_THROW(captured_task(*meshcat));
}

// ---------------------------------------------------------------------------
// ToggleFrame
// ---------------------------------------------------------------------------

TEST_F(TestVisualizerService, ToggleFrame_EmptyFrame_InvalidArgument) {
  proto::ToggleFrameRequest req;  // frame() is empty by default
  proto::ToggleFrameResponse resp;
  const auto status = service->ToggleFrame(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::INVALID_ARGUMENT);
}

// Case 1 — absolute path that IS under /drake/frames/: used verbatim.
TEST_F(TestVisualizerService,
       ToggleFrame_Case1_AbsolutePathUnderFrames_Success) {
  proto::ToggleFrameRequest req;
  req.set_frame("/drake/frames/mymodel/wrist_link");
  req.set_visible(true);

  std::string captured_path;
  bool captured_visible;
  EXPECT_CALL(*hub, ToggleFrame(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&captured_path), SaveArg<1>(&captured_visible),
                      Return(true)));

  proto::ToggleFrameResponse resp;
  EXPECT_TRUE(service->ToggleFrame(nullptr, &req, &resp).ok());
  EXPECT_EQ(captured_path, "/drake/frames/mymodel/wrist_link/");
  EXPECT_TRUE(captured_visible);
}

TEST_F(TestVisualizerService,
       ToggleFrame_Case1_AbsolutePathAlreadyHasTrailingSlash_Success) {
  proto::ToggleFrameRequest req;
  req.set_frame("/drake/frames/mymodel/wrist_link/");
  req.set_visible(false);

  std::string captured_path;
  EXPECT_CALL(*hub, ToggleFrame(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&captured_path), Return(true)));

  proto::ToggleFrameResponse resp;
  EXPECT_TRUE(service->ToggleFrame(nullptr, &req, &resp).ok());
  // Exactly one trailing slash — no double slash.
  EXPECT_EQ(captured_path, "/drake/frames/mymodel/wrist_link/");
}

TEST_F(TestVisualizerService,
       ToggleFrame_Case1_AbsolutePathWrongSubtree_InvalidArgument) {
  for (const std::string bad_path :
       {"/drake/objects/thing", "/drake/visual/thing", "/unrelated/path"}) {
    proto::ToggleFrameRequest req;
    req.set_frame(bad_path);
    proto::ToggleFrameResponse resp;
    // These are bad Meshcat paths and should not require model query.
    EXPECT_CALL(*hub, ResolveFrameMeshcatPath(_)).Times(0);
    // Invalid paths should be rejected before any attempt to toggle.
    EXPECT_CALL(*hub, ToggleFrame(_, _)).Times(0);
    const auto status = service->ToggleFrame(nullptr, &req, &resp);
    EXPECT_EQ(status.error_code(), grpc::StatusCode::INVALID_ARGUMENT)
        << "Expected INVALID_ARGUMENT for path: " << bad_path;
    EXPECT_FALSE(meshcat->HasPath(bad_path + "/"));
  }
}

TEST_F(TestVisualizerService,
       ToggleFrame_Case2_RelativeWithFramesPrefix_PrependsDrake) {
  proto::ToggleFrameRequest req;
  req.set_frame("frames/mymodel/wrist_link");
  req.set_visible(true);

  std::string captured_path;
  EXPECT_CALL(*hub, ToggleFrame(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&captured_path), Return(true)));

  proto::ToggleFrameResponse resp;
  EXPECT_TRUE(service->ToggleFrame(nullptr, &req, &resp).ok());
  EXPECT_EQ(captured_path, "/drake/frames/mymodel/wrist_link/");
  EXPECT_FALSE(meshcat->HasPath("/drake/frames/frames/mymodel/wrist_link/"));
}

TEST_F(TestVisualizerService,
       ToggleFrame_Case2_RelativeWithoutFramesPrefix_PrependsDrakeFrames) {
  proto::ToggleFrameRequest req;
  req.set_frame("mymodel/wrist_link");
  req.set_visible(false);

  std::string captured_path;
  EXPECT_CALL(*hub, ToggleFrame(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&captured_path), Return(true)));

  proto::ToggleFrameResponse resp;
  EXPECT_TRUE(service->ToggleFrame(nullptr, &req, &resp).ok());
  EXPECT_EQ(captured_path, "/drake/frames/mymodel/wrist_link/");
  EXPECT_FALSE(meshcat->HasPath("/drake/mymodel/wrist_link/"));
}

TEST_F(TestVisualizerService, ToggleFrame_Case3_FrameNotFound_InvalidArgument) {
  proto::ToggleFrameRequest req;
  req.set_frame("unknown_frame");
  proto::ToggleFrameResponse resp;

  EXPECT_CALL(*hub, ResolveFrameMeshcatPath("unknown_frame"))
      .WillOnce(Return(std::nullopt));

  const auto status = service->ToggleFrame(nullptr, &req, &resp);
  EXPECT_EQ(status.error_code(), grpc::StatusCode::INVALID_ARGUMENT);
}

TEST_F(TestVisualizerService, ToggleFrame_Case3_FrameResolved_UsesModelPath) {
  proto::ToggleFrameRequest req;
  req.set_frame("wrist_link");
  req.set_visible(true);
  proto::ToggleFrameResponse resp;

  // The resolved path already ends with '/' so no extra slash should be
  // added.
  EXPECT_CALL(*hub, ResolveFrameMeshcatPath("wrist_link"))
      .WillOnce(Return("/drake/frames/iiwa/wrist_link/"));

  std::string captured_path;
  EXPECT_CALL(*hub, ToggleFrame(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&captured_path), Return(true)));

  EXPECT_TRUE(service->ToggleFrame(nullptr, &req, &resp).ok());
  EXPECT_EQ(captured_path, "/drake/frames/iiwa/wrist_link/");
}

TEST_F(TestVisualizerService,
       ToggleFrame_Case3_FrameResolvedNoTrailingSlash_SlashAppended) {
  proto::ToggleFrameRequest req;
  req.set_frame("wrist_link");
  req.set_visible(false);
  proto::ToggleFrameResponse resp;

  // Hub returns path without trailing slash — normalisation must add one.
  EXPECT_CALL(*hub, ResolveFrameMeshcatPath("wrist_link"))
      .WillOnce(Return("/drake/frames/iiwa/wrist_link"));

  std::string captured_path;
  EXPECT_CALL(*hub, ToggleFrame(_, _))
      .Times(1)
      .WillOnce(DoAll(SaveArg<0>(&captured_path), Return(true)));

  EXPECT_TRUE(service->ToggleFrame(nullptr, &req, &resp).ok());
  EXPECT_EQ(captured_path, "/drake/frames/iiwa/wrist_link/");
}

}  // namespace test
}  // namespace comms
