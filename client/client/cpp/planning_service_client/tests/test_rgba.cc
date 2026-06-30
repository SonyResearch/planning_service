#include <gtest/gtest.h>

#include "planning_service_client/rgba.h"

namespace planning_service_client {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

TEST(Rgba, DefaultConstruct) {
  Rgba dut;
  EXPECT_EQ(dut.r(), 0.0);
  EXPECT_EQ(dut.g(), 0.0);
  EXPECT_EQ(dut.b(), 0.0);
  EXPECT_EQ(dut.a(), 1.0);
}

TEST(Rgba, ExplicitConstruct) {
  Rgba dut(0.1, 0.2, 0.3, 0.4);
  EXPECT_EQ(dut.r(), 0.1);
  EXPECT_EQ(dut.g(), 0.2);
  EXPECT_EQ(dut.b(), 0.3);
  EXPECT_EQ(dut.a(), 0.4);
}

TEST(Rgba, DefaultAlphaIsOne) {
  Rgba dut(0.5, 0.5, 0.5);
  EXPECT_EQ(dut.a(), 1.0);
}

// ---------------------------------------------------------------------------
// Equality
// ---------------------------------------------------------------------------

TEST(Rgba, EqualityTrue) {
  Rgba a(0.1, 0.2, 0.3, 0.4);
  Rgba b(0.1, 0.2, 0.3, 0.4);
  EXPECT_EQ(a, b);
}

TEST(Rgba, EqualityFalse) {
  Rgba a(0.1, 0.2, 0.3, 0.4);
  Rgba b(0.9, 0.2, 0.3, 0.4);
  EXPECT_NE(a, b);
}

// ---------------------------------------------------------------------------
// Proto round-trip
// ---------------------------------------------------------------------------

TEST(Rgba, ToProtoFromProto) {
  Rgba dut(0.25, 0.5, 0.75, 0.9);
  auto msg = ToProto(dut);

  EXPECT_EQ(msg.r(), 0.25);
  EXPECT_EQ(msg.g(), 0.5);
  EXPECT_EQ(msg.b(), 0.75);
  EXPECT_EQ(msg.a(), 0.9);

  auto roundtripped = FromProto<Rgba>(msg);
  EXPECT_EQ(roundtripped, dut);
}

TEST(Rgba, RoundTripPreservesDefaultAlpha) {
  Rgba dut(0.1, 0.2, 0.3);
  auto roundtripped = FromProto<Rgba>(ToProto(dut));
  EXPECT_EQ(roundtripped, dut);
  EXPECT_EQ(roundtripped.a(), 1.0);
}

// ---------------------------------------------------------------------------
// Named color factories
// ---------------------------------------------------------------------------

TEST(Rgba, Red) {
  auto c = Rgba::Red();
  EXPECT_EQ(c.r(), 1.0);
  EXPECT_EQ(c.g(), 0.0);
  EXPECT_EQ(c.b(), 0.0);
  EXPECT_EQ(c.a(), 1.0);
}

TEST(Rgba, Green) {
  auto c = Rgba::Green();
  EXPECT_EQ(c.r(), 0.0);
  EXPECT_EQ(c.g(), 1.0);
  EXPECT_EQ(c.b(), 0.0);
}

TEST(Rgba, Blue) {
  auto c = Rgba::Blue();
  EXPECT_EQ(c.r(), 0.0);
  EXPECT_EQ(c.g(), 0.0);
  EXPECT_EQ(c.b(), 1.0);
}

TEST(Rgba, White) {
  auto c = Rgba::White();
  EXPECT_EQ(c.r(), 1.0);
  EXPECT_EQ(c.g(), 1.0);
  EXPECT_EQ(c.b(), 1.0);
}

TEST(Rgba, Black) {
  auto c = Rgba::Black();
  EXPECT_EQ(c.r(), 0.0);
  EXPECT_EQ(c.g(), 0.0);
  EXPECT_EQ(c.b(), 0.0);
}

TEST(Rgba, NamedColorCustomAlpha) {
  auto c = Rgba::Red(0.5);
  EXPECT_EQ(c.a(), 0.5);
}

}  // namespace planning_service_client
