/*
 * Copyright © 2023 Dexai Robotics. All rights reserved.
 */

#include <drake/math/rigid_transform.h>
#include <gtest/gtest.h>

#include <fstream>

#include "planning_service/draco/draco.h"
#include "planning_service/draco/tests/test_utils.h"

namespace draco {

TEST(Draco, Basics) {
  EXPECT_NO_THROW(Draco(test::Wallflower()));
  EXPECT_NO_THROW(Draco(test::DualWallflowers()));
  EXPECT_NO_THROW(Draco(test::DualPandas()));
}

TEST(Draco, Options) {
  auto dut = Draco(test::Wallflower());
  EXPECT_NO_THROW(dut.options());
  // Tip: print them and copy them from terminal log into test files.
  logging::log()->info("Draco options: {}",
                       drake::yaml::SaveYamlString(dut.options()));
}

}  // namespace draco
