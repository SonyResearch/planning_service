/*
 * System test for the visualizer service – C++ client (GoogleTest).
 *
 * Frame combinations are loaded from system_tests/visualizer_frames.yaml.
 * Every source frame is tested against every target frame.
 *
 * Configuration via environment variables:
 *   VIZ_ADDR    host:port of the visualizer service (default:
 * visualizer-dev:5550) TEST_SRCDIR set by Bazel; used to locate the YAML data
 * file
 */

#include <drake/common/name_value.h>
#include <drake/common/yaml/yaml_io.h>
#include <gtest/gtest.h>

#include <chrono>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include "planning_service_client/api/visualizer_client.h"
#include "planning_service_client/conf.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/visualizer_status.h"

namespace psc = planning_service_client;

// ---------------------------------------------------------------------------
// YAML config
// ---------------------------------------------------------------------------

struct VisualizerTestConfig {
  std::vector<std::string> source_frames;
  std::vector<std::string> target_frames;
  template <typename Archive>
  void Serialize(Archive* a) {
    a->Visit(DRAKE_NVP(source_frames));
    a->Visit(DRAKE_NVP(target_frames));
  }
};

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::string EnvOr(const char* name, const char* fallback) {
  const char* v = std::getenv(name);
  return v ? std::string(v) : std::string(fallback);
}

static std::string FramesFilePath() {
  const char* srcdir = std::getenv("TEST_SRCDIR");
  if (srcdir) {
    return std::string(srcdir) + "/_main/system_tests/visualizer_frames.yaml";
  }
  return "system_tests/visualizer_frames.yaml";
}

// Returns the cartesian product of source_frames × target_frames.
static std::vector<std::tuple<std::string, std::string>> LoadFramePairs() {
  auto cfg = drake::yaml::LoadYamlFile<VisualizerTestConfig>(FramesFilePath());
  std::vector<std::tuple<std::string, std::string>> pairs;
  for (const auto& src : cfg.source_frames) {
    for (const auto& tgt : cfg.target_frames) {
      pairs.emplace_back(src, tgt);
    }
  }
  return pairs;
}

// ---------------------------------------------------------------------------
// Helper: build a SystemConf override for the two lifty robot instances.
//
// Lifty joint order within each Conf vector: [rev, piston]
//   rev    ∈ [-1.57, 1.57]  (revolute)
//   piston ∈ [0.00,  0.30]  (prismatic)
// ---------------------------------------------------------------------------
static psc::SystemConf MakeOverrideConf() {
  psc::SystemConf sc;
  Eigen::VectorXd qa(2);
  qa << 1.0, 0.25;  // a: rev=1.0, piston=0.25
  sc["a"] = psc::Conf(qa);
  Eigen::VectorXd qb(2);
  qb << -1.0, 0.05;  // b: rev=-1.0, piston=0.05
  sc["b"] = psc::Conf(qb);
  return sc;
}

static psc::VisualizerStatus WaitForActive(
    const psc::client::VisualizerClient& client, int timeout_s = 60,
    int poll_ms = 1000) {
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::seconds(timeout_s);
  while (std::chrono::steady_clock::now() < deadline) {
    auto s = client.GetVisualizerStatus();
    if (s.status() == psc::VisualizerStatus::Status::kActive) return s;
    std::this_thread::sleep_for(std::chrono::milliseconds(poll_ms));
  }
  throw std::runtime_error("Timed out waiting for ACTIVE status");
}

// ---------------------------------------------------------------------------
// Shared client – created once for the whole process
// ---------------------------------------------------------------------------

static std::unique_ptr<psc::client::VisualizerClient> g_client;

// ---------------------------------------------------------------------------
// Lifecycle test (runs first – alphabetically before "Lifty/...")
// ---------------------------------------------------------------------------

class VisualizerLifecycleTest : public ::testing::Test {
 protected:
  static void SetUpTestSuite() {
    g_client = std::make_unique<psc::client::VisualizerClient>(
        EnvOr("VIZ_ADDR", "visualizer-dev:5550"), "cpp_viz_system_test");
    ASSERT_TRUE(
        g_client->Connect(/*num_attempts=*/60, /*attempt_interval_ms=*/1000))
        << "Could not connect to visualizer service";
  }
};

TEST_F(VisualizerLifecycleTest, DefaultContextIsActive) {
  ASSERT_NO_THROW(WaitForActive(*g_client, 60));
}

// ---------------------------------------------------------------------------
// Parameterised CalcPose tests – one per (source, target) combination
// ---------------------------------------------------------------------------

class VisualizerCalcPoseTest
    : public ::testing::TestWithParam<std::tuple<std::string, std::string>> {};

TEST_P(VisualizerCalcPoseTest, CalcPose) {
  const auto& [frame_a, frame_b] = GetParam();
  psc::FrameRelativePose pose;
  ASSERT_NO_THROW(pose = g_client->CalcPose(frame_a, frame_b));
  EXPECT_EQ(pose.frame_A(), frame_a);
  EXPECT_EQ(pose.frame_B(), frame_b);
}

INSTANTIATE_TEST_SUITE_P(
    Lifty, VisualizerCalcPoseTest, ::testing::ValuesIn(LoadFramePairs()),
    [](const ::testing::TestParamInfo<std::tuple<std::string, std::string>>&
           info) {
      // Replace non-alphanumeric chars to get a valid name
      auto sanitise = [](std::string s) {
        for (char& c : s)
          if (!std::isalnum(c)) c = '_';
        return s;
      };
      return sanitise(std::get<0>(info.param)) + "__"
             + sanitise(std::get<1>(info.param));
    });

// ---------------------------------------------------------------------------
// CalcPose joint-override tests
//
// VisualizerCalcPoseOverrideBase  – shared fixture; holds MakeOverrideConf()
//   ├─ TEST_F  PoseDiffersFromDefault   (single non-parameterised test)
//   └─ VisualizerCalcPoseOverrideTest   (parameterised over all frame pairs)
//        └─ TEST_P  CalcPoseWithOverride
// ---------------------------------------------------------------------------

class VisualizerCalcPoseOverrideBase : public ::testing::Test {
 protected:
  static psc::SystemConf MakeOverrideConf() {
    return ::MakeOverrideConf();
  }
};

// Single test: the override must actually change the returned pose.
TEST_F(VisualizerCalcPoseOverrideBase, PoseDiffersFromDefault) {
  const psc::FrameRelativePose default_pose =
      g_client->CalcPose("a::stamp", "world");
  const psc::FrameRelativePose overridden_pose =
      g_client->CalcPose("a::stamp", "world", MakeOverrideConf());
  EXPECT_FALSE(default_pose.X_AB_translation().isApprox(
      overridden_pose.X_AB_translation(), 1e-3))
      << "Pose did not change when a joint-position override was applied";
}

// Parameterised tests: well-formed pose for every (source, target) pair.
class VisualizerCalcPoseOverrideTest
    : public VisualizerCalcPoseOverrideBase,
      public ::testing::WithParamInterface<
          std::tuple<std::string, std::string>> {};

TEST_P(VisualizerCalcPoseOverrideTest, CalcPoseWithOverride) {
  const auto& [frame_a, frame_b] = GetParam();
  psc::FrameRelativePose pose;
  ASSERT_NO_THROW(pose =
                      g_client->CalcPose(frame_a, frame_b, MakeOverrideConf()));
  EXPECT_EQ(pose.frame_A(), frame_a);
  EXPECT_EQ(pose.frame_B(), frame_b);
  EXPECT_TRUE(pose.X_AB_translation().allFinite());
  EXPECT_TRUE(pose.X_AB_quaternion().coeffs().allFinite());
  EXPECT_NEAR(pose.X_AB_quaternion().norm(), 1.0, 1e-6);
}

INSTANTIATE_TEST_SUITE_P(Lifty, VisualizerCalcPoseOverrideTest,
                         ::testing::ValuesIn(LoadFramePairs()),
                         [](const ::testing::TestParamInfo<
                             std::tuple<std::string, std::string>>& info) {
                           auto sanitise = [](std::string s) {
                             for (char& c : s)
                               if (!std::isalnum(c)) c = '_';
                             return s;
                           };
                           return sanitise(std::get<0>(info.param)) + "__"
                                  + sanitise(std::get<1>(info.param));
                         });

// ---------------------------------------------------------------------------
// ToggleFrame – path resolution integration tests
// ---------------------------------------------------------------------------

// A frame guaranteed to exist in the loaded lifty model.
static constexpr std::string_view kKnownFrame {"a::stamp"};
// Its canonical Meshcat path.
static constexpr std::string_view kKnownFrameAbsPath {"/drake/frames/a/stamp"};

class VisualizerToggleFrameTest : public ::testing::Test {};

// Case 3 — bare name resolved via the loaded model.
TEST_F(VisualizerToggleFrameTest, BareNameHide) {
  EXPECT_NO_THROW(g_client->ToggleFrame(kKnownFrame, false));
}
TEST_F(VisualizerToggleFrameTest, BareNameShow) {
  EXPECT_NO_THROW(g_client->ToggleFrame(kKnownFrame, true));
}

// Case 1 — absolute path under /drake/frames/ used verbatim.
TEST_F(VisualizerToggleFrameTest, AbsolutePathHide) {
  EXPECT_NO_THROW(g_client->ToggleFrame(kKnownFrameAbsPath, false));
}
TEST_F(VisualizerToggleFrameTest, AbsolutePathShow) {
  EXPECT_NO_THROW(g_client->ToggleFrame(kKnownFrameAbsPath, true));
}

// Case 2a — relative path with "frames/" prefix: "/drake/" is prepended.
TEST_F(VisualizerToggleFrameTest, RelativeFramesPrefixHide) {
  EXPECT_NO_THROW(g_client->ToggleFrame("frames/a/stamp", false));
}
TEST_F(VisualizerToggleFrameTest, RelativeFramesPrefixShow) {
  EXPECT_NO_THROW(g_client->ToggleFrame("frames/a/stamp", true));
}

// Case 2b — relative path without "frames/" prefix: "/drake/frames/" prepended.
TEST_F(VisualizerToggleFrameTest, RelativeNoFramesPrefixHide) {
  EXPECT_NO_THROW(g_client->ToggleFrame("a/stamp", false));
}
TEST_F(VisualizerToggleFrameTest, RelativeNoFramesPrefixShow) {
  EXPECT_NO_THROW(g_client->ToggleFrame("a/stamp", true));
}

// Roundtrip — all four input forms: hide then show.
TEST_F(VisualizerToggleFrameTest, Roundtrip) {
  for (std::string_view frame :
       {kKnownFrame, kKnownFrameAbsPath, std::string_view {"frames/a/stamp"},
        std::string_view {"a/stamp"}}) {
    EXPECT_NO_THROW(g_client->ToggleFrame(frame, false)) << "frame=" << frame;
    EXPECT_NO_THROW(g_client->ToggleFrame(frame, true)) << "frame=" << frame;
  }
}

// Error cases — server must reject these.
TEST_F(VisualizerToggleFrameTest, EmptyNameThrows) {
  EXPECT_THROW(g_client->ToggleFrame("", true), std::runtime_error);
}

TEST_F(VisualizerToggleFrameTest, WrongSubtreeThrows) {
  EXPECT_THROW(g_client->ToggleFrame("/drake/objects/some_object", true),
               std::runtime_error);
}

TEST_F(VisualizerToggleFrameTest, UnknownBareNameThrows) {
  EXPECT_THROW(g_client->ToggleFrame("this_frame_does_not_exist_xyz", true),
               std::runtime_error);
}
