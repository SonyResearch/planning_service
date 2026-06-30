#include <gtest/gtest.h>

#include <google/protobuf/util/message_differencer.h>

#include "client/cpp/planning_service_client/test_data/dummy.h"
#include "planning_service_client/common/io_utils.h"
namespace planning_service_client {
namespace common {

/**
 * @brief Test fixture for testing IO utils against various protobuf message
 * types. This fixture will create a temporary directory for each test, write
 * the protobuf message to both JSON and text formats, and destroy the temporary
 * directory after each test.
 *
 * @tparam T message type.
 */
template <typename T>
class ProtoIoTest : public ::testing::Test {
 protected:
  void SetUp() override {
    std::string path_template_str {"planning_service_client_test_XXXXXX"};
    const char* dtemp = ::mkdtemp(&path_template_str[0]);
    temp_dir_ = std::filesystem::temp_directory_path() / dtemp;
    std::filesystem::create_directories(temp_dir_);
    json_path_ = temp_dir_ / "msg.json";
    txt_path_ = temp_dir_ / "msg.txtpb";
    bin_path_ = temp_dir_ / "msg.binpb";
    SetObject();
  }

  void TearDown() override {
    // Clean up the temporary directory after each test
    if (std::filesystem::exists(temp_dir_)) {
      std::filesystem::remove_all(temp_dir_);
    }
  }
  // Pure virtual function to set the object, to be implemented by derived
  // classes
  virtual void SetObject() = 0;

  std::filesystem::path temp_dir_;
  std::filesystem::path json_path_;
  std::filesystem::path txt_path_;
  std::filesystem::path bin_path_;
  T obj_;
};

/** String message test implementation. */
class DummyMessageTest : public ProtoIoTest<DummyMessage> {
 protected:
  void SetObject() override {
    obj_ = DummyMessage();
    obj_.set_field("Test message");
  }
  DummyMessage obj_;
};

TEST_F(DummyMessageTest, ToFromFile) {
  const std::string contents {"Test string to be written to file."};
  std::string loaded;
  EXPECT_NO_THROW(WriteToFile(txt_path_, contents, false));
  EXPECT_NO_THROW(LoadFromFile(txt_path_, loaded, false));
  EXPECT_EQ(contents, loaded);
  // Binary mode
  EXPECT_NO_THROW(WriteToFile(bin_path_, contents, true));
  EXPECT_NO_THROW(LoadFromFile(bin_path_, loaded, true));
  EXPECT_EQ(contents, loaded);
}

TEST_F(DummyMessageTest, Json) {
  const std::string contents {obj_.ToJson()};
  EXPECT_NO_THROW(SaveToJson(json_path_, obj_));
  // Eval contents
  std::string in;
  LoadFromFile(json_path_, in, false);
  EXPECT_EQ(contents, in);
  // Eval loaded object
  DummyMessage loaded;
  EXPECT_NO_THROW(loaded = LoadFromJsonFile<DummyMessage>(json_path_));
  EXPECT_EQ(obj_, loaded);
}

TEST_F(DummyMessageTest, Text) {
  bool binary {false};
  const std::string contents {obj_.ToString(binary)};
  EXPECT_NO_THROW(SaveToFile(txt_path_, obj_, binary));
  // Eval contents
  std::string in;
  LoadFromFile(txt_path_, in, binary);
  EXPECT_EQ(contents, in);
  // Eval loaded object
  DummyMessage loaded;
  EXPECT_NO_THROW(loaded = LoadFromFile<DummyMessage>(txt_path_, binary));
  EXPECT_EQ(obj_, loaded);
}

TEST_F(DummyMessageTest, Binary) {
  bool binary {true};
  const std::string contents {obj_.ToString(binary)};
  EXPECT_NO_THROW(SaveToFile(bin_path_, obj_, binary));
  // Eval contents
  std::string in;
  LoadFromFile(bin_path_, in, binary);
  EXPECT_EQ(contents, in);
  // Eval loaded object
  DummyMessage loaded;
  EXPECT_NO_THROW(loaded = LoadFromFile<DummyMessage>(bin_path_, binary));
  EXPECT_EQ(obj_, loaded);
}

TEST(JsonTest, SnakeCase) {
  const std::filesystem::path json_path {
      "client/cpp/planning_service_client/test_data/dummy_message_snake.json"};
  DummyMessage obj;
  EXPECT_NO_THROW(obj = LoadFromJsonFile<DummyMessage>(json_path));
  dummy_protos::DummyMessage msg;
  EXPECT_NO_THROW(msg = ToProto(obj));
  DummyMessage new_obj;
  EXPECT_NO_THROW(new_obj = FromProto<DummyMessage>(msg));
  EXPECT_EQ(obj, new_obj);
}

TEST(JsonTest, CamelCase) {
  const std::filesystem::path json_path {
      "client/cpp/planning_service_client/test_data/dummyMessageCamel.json"};
  DummyMessage obj;
  EXPECT_NO_THROW(obj = LoadFromJsonFile<DummyMessage>(json_path));
  dummy_protos::DummyMessage msg;
  EXPECT_NO_THROW(msg = ToProto(obj));
  DummyMessage new_obj;
  EXPECT_NO_THROW(new_obj = FromProto<DummyMessage>(msg));
  EXPECT_EQ(obj, new_obj);
}

}  // namespace common
}  // namespace planning_service_client
