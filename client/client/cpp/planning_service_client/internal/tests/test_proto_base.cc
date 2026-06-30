#include <gtest/gtest.h>

#include <google/protobuf/util/message_differencer.h>

#include "client/cpp/planning_service_client/internal/proto_base.h"
#include "client/cpp/planning_service_client/test_data/dummy.h"

namespace planning_service_client {
namespace internal {

class ProtoBaseTest : public ::testing::Test {
 protected:
  void SetUp() override {
    obj_.set_field("test_field");
    msg_.set_field("test_field");
    int length {3};
    for (int i = 0; i < length; ++i) {
      msg_.add_repeated_fields("repeated_field_" + std::to_string(i));
      obj_.mutable_repeated_fields()->emplace_back("repeated_field_"
                                                   + std::to_string(i));
    }
  }

  DummyMessage obj_;
  dummy_protos::DummyMessage msg_;
};

TEST_F(ProtoBaseTest, ToProto) {
  auto proto = ToProto(obj_);
  EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(proto, msg_));
}

TEST_F(ProtoBaseTest, FromProto) {
  auto obj = FromProto<DummyMessage>(msg_);
  EXPECT_EQ(obj, obj_);
}

TEST_F(ProtoBaseTest, ToStringText) {
  auto text = obj_.ToString(false);
  std::string loaded_text;
  EXPECT_NO_THROW(loaded_text = msg_.DebugString());
  EXPECT_TRUE(loaded_text.find(text) != std::string::npos);
}
TEST_F(ProtoBaseTest, ToStringBinary) {
  auto binary = obj_.ToString(true);
  std::string loaded_binary;
  EXPECT_NO_THROW(msg_.SerializeToString(&loaded_binary));
  EXPECT_EQ(binary, loaded_binary);
}
TEST_F(ProtoBaseTest, ToJson) {
  auto json = obj_.ToJson();
  std::string loaded_json;
  google::protobuf::util::JsonPrintOptions options;
  options.preserve_proto_field_names = true;
  absl::Status status;
  EXPECT_NO_THROW(status = google::protobuf::util::MessageToJsonString(
                      msg_, &loaded_json, options));
  EXPECT_TRUE(status.ok());
  EXPECT_EQ(json, loaded_json);
}

TEST_F(ProtoBaseTest, FromStringText) {
  const auto text = obj_.ToString(false);
  const auto obj = FromString<DummyMessage>(text, false);
  EXPECT_EQ(obj, obj_);
}
TEST_F(ProtoBaseTest, FromStringBinary) {
  const auto binary = obj_.ToString(true);
  const auto obj = FromString<DummyMessage>(binary, true);
  EXPECT_EQ(obj, obj_);
}
TEST_F(ProtoBaseTest, FromJson) {
  const auto json = obj_.ToJson();
  const auto obj = FromJson<DummyMessage>(json);
  EXPECT_EQ(obj, obj_);
}

TEST(Vector, ScalarTypes) {
  std::vector<int> int_vec {1, 2, 3, 4, 5};
  auto repeated_field = ToProto(int_vec);
  EXPECT_EQ(repeated_field.size(), int_vec.size());
  for (size_t i = 0; i < int_vec.size(); ++i) {
    EXPECT_EQ(repeated_field.Get(i), int_vec[i]);
  }
  auto int_vec_converted = FromProto(repeated_field);
  EXPECT_EQ(int_vec_converted, int_vec);

  std::vector<double> double_vec {1.1, 2.2, 3.3};
  auto repeated_field_double = ToProto(double_vec);
  EXPECT_EQ(repeated_field_double.size(), double_vec.size());
  for (size_t i = 0; i < double_vec.size(); ++i) {
    EXPECT_EQ(repeated_field_double.Get(i), double_vec[i]);
  }
  auto double_vec_converted = FromProto(repeated_field_double);
  EXPECT_EQ(double_vec_converted, double_vec);

  std::vector<std::string> string_vec {"one", "two", "three"};
  auto repeated_field_string = ToProto(string_vec);
  EXPECT_EQ(repeated_field_string.size(), string_vec.size());
  for (size_t i = 0; i < string_vec.size(); ++i) {
    EXPECT_EQ(repeated_field_string.Get(i), string_vec[i]);
  }
  auto string_vec_converted = FromProto(repeated_field_string);
  EXPECT_EQ(string_vec_converted, string_vec);
}

TEST(Vector, ProtoBaseTypes) {
  dummy_protos::DummyMessage msg1;
  msg1.set_field("msg1");
  dummy_protos::DummyMessage msg2;
  msg2.set_field("msg2");
  dummy_protos::DummyMessage msg3;
  msg3.set_field("msg3");

  DummyMessage obj1;
  obj1.set_field("msg1");
  DummyMessage obj2;
  obj2.set_field("msg2");
  DummyMessage obj3;
  obj3.set_field("msg3");

  std::vector<DummyMessage> obj_vec {obj1, obj2, obj3};
  auto repeated_field = ToProto(obj_vec);
  EXPECT_EQ(repeated_field.size(), obj_vec.size());
  for (size_t i = 0; i < obj_vec.size(); ++i) {
    EXPECT_TRUE(google::protobuf::util::MessageDifferencer::Equals(
        repeated_field.Get(i), ToProto(obj_vec[i])));
  }
  auto obj_vec_converted = FromProto<DummyMessage>(repeated_field);
  EXPECT_EQ(obj_vec_converted, obj_vec);
}
}  // namespace internal
}  // namespace planning_service_client
