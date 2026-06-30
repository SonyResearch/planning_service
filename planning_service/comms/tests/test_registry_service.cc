/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include "planning_service/comms/registry_service.h"
#include "testing_utils.h"

namespace comms {
namespace test {

class ContextRegistryServiceTest
    : public ServiceTest<ContextRegistryService,
                         proto::PlanContextRegistry::Stub> {
  void SetService() {
    service_ = std::make_unique<ContextRegistryService>(
        std::make_shared<service::utils::ResourceRegistry>(system_, options_));
  }
  void SetStub() {
    stub_ = proto::PlanContextRegistry::NewStub(channel_);
  }
};

TEST_F(ContextRegistryServiceTest, HandleRegisterPlanContextRequest_Empty) {
  proto::RegisterPlanContextRequest req;
  proto::RegisterPlanContextResponse resp;
  grpc::ClientContext context;
  // Default to 5 sec timeout.
  std::chrono::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::milliseconds(5000);
  context.set_deadline(deadline);
  const auto status {
      stub_->HandleRegisterPlanContextRequest(&context, req, &resp)};
  EXPECT_FALSE(status.ok()) << "Error: " << status.error_message();
  EXPECT_EQ(resp.context_id().value(), 0);
}

TEST_F(ContextRegistryServiceTest, HandleRegisterPlanContextRequest) {
  proto::RegisterPlanContextRequest req;
  *req.mutable_context() = MakePlanContext();
  req.set_system("iiwa");
  proto::RegisterPlanContextResponse resp;
  grpc::ClientContext context;
  const auto status {
      stub_->HandleRegisterPlanContextRequest(&context, req, &resp)};
  EXPECT_TRUE(status.ok()) << "Error: " << status.error_message();
  EXPECT_EQ(resp.context_id().value(), kIiwaBoxesHash);
}
}  // namespace test
}  // namespace comms
