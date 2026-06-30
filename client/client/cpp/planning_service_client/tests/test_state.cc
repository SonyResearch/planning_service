#include <gtest/gtest.h>

#include <iostream>

#include "planning_service_client/state.h"

namespace planning_service_client {

TEST(Value, Ctor) {
  EXPECT_NO_THROW(Value());
  EXPECT_NO_THROW(Value(3.14));
  EXPECT_NO_THROW(Value(true));
  EXPECT_NO_THROW(Value("test string"));
  EXPECT_NO_THROW(Value(std::vector<Value> {Value(1.0), Value(2.0)}));
  EXPECT_NO_THROW(Value(std::vector<float> {1.0, 2.0, 3.0}));
  State state;
  state.Add("key1", 42);
  state.Add("key2", "value2");
  state.Add("key3", std::vector<double> {1.0, 2.0, 3.0});
  EXPECT_NO_THROW(auto value = Value(state));
  EXPECT_NO_THROW(Value(std::vector<State> {state, state}));
}

TEST(State, Basics) {
  State state;
  state.Add("mode", "automatic");
  state.Add("speed", 3.5);
  state.Add("enabled", true);
  state.Add("end_effector", "gripper");
  auto hash_1 = state.CalcHash();
  // Again to check consistency
  auto hash_2 = state.CalcHash();
  EXPECT_EQ(hash_1, hash_2);
  // Now, make another state with the same values, but in different order
  State state2;
  state2.Add("end_effector", std::string("gripper"));
  state2.Add("enabled", true);
  state2.Add("speed", 3.5);
  state2.Add("mode", "automatic");
  auto hash_3 = state2.CalcHash();
  // Check that the hashes are the same
  EXPECT_EQ(hash_1, hash_3);
  // Now, change one value
  state2.Add("mode", "manual");
  auto hash_4 = state2.CalcHash();
  // Check that the hashes are different
  EXPECT_NE(hash_1, hash_4);
  // Compare the string representations
  auto str1 = state.ToString();
  auto str2 = state2.ToString();
  EXPECT_NE(str1, str2);
  // Check short string
  auto short_str1 = state.ToCompactString();
  auto short_str2 = state2.ToCompactString();
  EXPECT_NE(short_str1, short_str2);
}

TEST(State, HashOrderInvariant) {
  State state1;
  state1.Add("a", 1);
  state1.Add("b", 2);
  state1.Add("c", 3);
  State state2;
  state2.Add("c", 3);
  state2.Add("b", 2);
  state2.Add("a", 1);
  EXPECT_EQ(state1.CalcHash(), state2.CalcHash());
}

TEST(State, HashTypeInvariant) {
  State state1;
  state1.Add("value", 42.0);  // double
  State state2;
  state2.Add("value", 42);  // int
  EXPECT_EQ(state1.CalcHash(), state2.CalcHash());
}

TEST(State, HashNullVsEmpty) {
  State state1;
  state1.Add("value", Value());  // null
  State state2;
  state2.Add("value", std::string(""));  // empty string
  EXPECT_NE(state1.CalcHash(), state2.CalcHash());
}

TEST(State, IsSubsetOf) {
  State state1;
  state1.Add("mode", "automatic");
  state1.Add("speed", 3.5);
  state1.Add("enabled", true);
  state1.Add("end_effector", "gripper");
  State state2 = state1;  // copy
  // state2 is identical to state1, so it is a subset
  EXPECT_TRUE(state2.IsSubsetOf(state1));
  // Same hashes
  EXPECT_EQ(state1.CalcHash(), state2.CalcHash());
  // Add one more entry to state2
  state2.Add("extra", 42.0);
  EXPECT_FALSE(state2.IsSubsetOf(state1));
  EXPECT_TRUE(state1.IsSubsetOf(state2));
  // The hash of state1 should be different from state2
  EXPECT_NE(state1.CalcHash(), state2.CalcHash());
}

TEST(State, ToProtoFromProto) {
  State state;
  state.Add("mode", "automatic");
  state.Add("speed", 3.5);
  state.Add("enabled", true);
  state.Add("end_effector", "gripper");
  auto proto = ToProto(state);
  State state2 = FromProto<State>(proto);
  auto hash_1 = state.CalcHash();
  auto hash_2 = state2.CalcHash();
  EXPECT_EQ(hash_1, hash_2);
  // The strings should be the same
  EXPECT_EQ(state.ToString(), state2.ToString());
}

TEST(Value, NullRoundtrip) {
  State state;
  state.Add("key", Value());
  auto proto = ToProto(state);
  State state2 = FromProto<State>(proto);
  EXPECT_EQ(state.CalcHash(), state2.CalcHash());
  EXPECT_TRUE(state2["key"].Is<std::monostate>());
}

TEST(State, Nested) {
  State state;
  state.Add("mode", "automatic");
  state.Add("speed", 3.5);
  // nested state
  State end_effector;
  end_effector.Add("type", "gripper");
  end_effector.Add("force", 10.0);
  end_effector.Add("mechanical", true);
  state.Add("end_effector", end_effector);
  // Generate hash and string
  auto hash_1 = state.CalcHash();
  // If the end_effector state changes, the hash should change
  State& end_effector_state = state["end_effector"].get<State>();
  end_effector_state["force"] = 15.0;  // change force
  auto hash_2 = state.CalcHash();
  EXPECT_NE(hash_1, hash_2);
  // Now, test proto conversion
  auto proto = ToProto(state);
  State state2 = FromProto<State>(proto);
  auto hash_3 = state2.CalcHash();
  EXPECT_EQ(hash_2, hash_3);
}

TEST(State, List) {
  State state;
  std::vector<float> numbers {1.0, 2.0, 3.0, 4.0, 5.0};
  state.Add("numbers", numbers);
  auto& mutable_vec {state["numbers"].get<std::vector<Value>>()};
  mutable_vec.emplace_back(6.0);
  const auto vec = state["numbers"].get<std::vector<Value>>();
  EXPECT_EQ(vec.size(), 6);
}

TEST(State, InitializerList) {
  Value value {{1.0, 2.0, 3.0}};
  EXPECT_TRUE(value.Is<std::vector<Value>>());
  State state {{"mode", "automatic"},
               {"speed", 3.5},
               {"enabled", true},
               {"end_effector", {{"type", "gripper"}, {"force", 10.0}}}};
  std::cout << state.ToCompactString(true) << std::endl;
  EXPECT_EQ(state["mode"].get<std::string>(), "automatic");
  EXPECT_EQ(state["speed"].get<double>(), 3.5);
  EXPECT_EQ(state["enabled"].get<bool>(), true);
  const auto& ee_state = state["end_effector"].get<State>();
  EXPECT_EQ(ee_state["type"].get<std::string>(), "gripper");
  EXPECT_EQ(ee_state["force"].get<double>(), 10.0);
}

// --- CalcHash-specific tests ---

// Two doubles that differ only beyond the 6th decimal place should hash the
// same; a difference at the 6th decimal place should hash differently.
TEST(State, HashDoublePrecision) {
  State same1;
  same1.Add("x", 1.0000001);  // differs beyond 6th decimal place
  State same2;
  same2.Add("x", 1.0000002);
  EXPECT_EQ(same1.CalcHash(), same2.CalcHash());

  State different;
  different.Add("x", 1.000001);  // differs at 6th decimal place
  EXPECT_NE(same1.CalcHash(), different.CalcHash());
}

// List order should matter: [1,2,3] != [3,2,1].
TEST(State, HashListOrderSensitive) {
  State state1;
  state1.Add("vals", std::vector<double> {1.0, 2.0, 3.0});
  State state2;
  state2.Add("vals", std::vector<double> {3.0, 2.0, 1.0});
  EXPECT_NE(state1.CalcHash(), state2.CalcHash());
}

// An empty list and a list with one element should not collide.
TEST(State, HashEmptyVsNonEmptyList) {
  State state1;
  state1.Add("vals", std::vector<double> {});
  State state2;
  state2.Add("vals", std::vector<double> {0.0});
  EXPECT_NE(state1.CalcHash(), state2.CalcHash());
}

// bool true/false should hash differently from each other and from numeric 1/0.
TEST(State, HashBoolDistinct) {
  State bool_true, bool_false, num_one, num_zero;
  bool_true.Add("v", true);
  bool_false.Add("v", false);
  num_one.Add("v", 1.0);
  num_zero.Add("v", 0.0);
  EXPECT_NE(bool_true.CalcHash(), bool_false.CalcHash());
  EXPECT_NE(bool_true.CalcHash(), num_one.CalcHash());
  EXPECT_NE(bool_false.CalcHash(), num_zero.CalcHash());
}

// null should not collide with false, 0.0, or empty string.
TEST(State, HashNullDistinct) {
  State null_state, false_state, zero_state, empty_str_state;
  null_state.Add("v", Value());
  false_state.Add("v", false);
  zero_state.Add("v", 0.0);
  empty_str_state.Add("v", std::string(""));
  EXPECT_NE(null_state.CalcHash(), false_state.CalcHash());
  EXPECT_NE(null_state.CalcHash(), zero_state.CalcHash());
  EXPECT_NE(null_state.CalcHash(), empty_str_state.CalcHash());
}

// Key names should affect the hash.
TEST(State, HashKeyNameSensitive) {
  State state1, state2;
  state1.Add("foo", 1.0);
  state2.Add("bar", 1.0);
  EXPECT_NE(state1.CalcHash(), state2.CalcHash());
}

// Deeply nested states: order-invariance holds at every level.
TEST(State, HashDeeplyNestedOrderInvariant) {
  State inner1;
  inner1.Add("z", 3.0);
  inner1.Add("a", 1.0);

  State inner2;
  inner2.Add("a", 1.0);
  inner2.Add("z", 3.0);

  State outer1, outer2;
  outer1.Add("inner", inner1);
  outer2.Add("inner", inner2);

  EXPECT_EQ(outer1.CalcHash(), outer2.CalcHash());
}

// Changing a character inside a string value must change the hash.
TEST(State, HashStringContentSensitive) {
  State state1, state2;
  state1.Add("s", std::string("hello"));
  state2.Add("s", std::string("hellp"));
  EXPECT_NE(state1.CalcHash(), state2.CalcHash());
}

// A state with one key should not collide with a state with two keys where one
// is the prefix of the other (guards against naive concatenation bugs).
TEST(State, HashNoPrefixCollision) {
  State state1, state2;
  state1.Add("ab", 1.0);
  state2.Add("a", 1.0);
  state2.Add("b", 1.0);  // "a:1b:1" vs "ab:1" — distinct
  EXPECT_NE(state1.CalcHash(), state2.CalcHash());
}

}  // namespace planning_service_client
