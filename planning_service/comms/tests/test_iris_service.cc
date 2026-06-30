/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <gtest/gtest.h>

#include "planning_service/comms/iris_service.h"
#include "testing_utils.h"

namespace comms {
namespace test {

class IrisBuildServiceTest
    : public ServiceTest<IrisBuilderService, proto::IrisBuilder::Stub> {
  void SetService() {
    auto mgr {
        std::make_shared<service::iris::IrisBuildManager>(system_, options_)};
    service_ = std::make_unique<IrisBuilderService>(mgr);
  }
  void SetStub() {
    stub_ = proto::IrisBuilder::NewStub(channel_);
  }
};

TEST_F(IrisBuildServiceTest, HandleBuildRequest) {}

}  // namespace test
}  // namespace comms
