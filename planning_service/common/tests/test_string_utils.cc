#include <gtest/gtest.h>

#include "planning_service/common/string_utils.h"
namespace common {
namespace utils {
TEST(TestStringUtils, ToLower) {
  std::string input = "HeLLo WoRLd!";
  std::string expected = "hello world!";
  EXPECT_EQ(to_lower(input), expected);
}

TEST(TestStringUtils, JoinStringsSet) {
  std::set<std::string> input = {"apple", "banana", "cherry"};
  std::string expected = "apple, banana, cherry";
  EXPECT_EQ(join_strings(input, ", "), expected);
}

TEST(TestStringUtils, JoinStringsVector) {
  std::vector<std::string> input = {"dog", "cat", "mouse"};
  std::string expected = "dog | cat | mouse";
  EXPECT_EQ(join_strings(input, " | "), expected);
}

TEST(TestStringUtils, SplitString) {
  std::string input = "red,green,blue,yellow";
  char delimiter = ',';
  std::vector<std::string> expected = {"red", "green", "blue", "yellow"};
  EXPECT_EQ(split_string(input, delimiter), expected);
  // Also do a test with one string
  input = "one";
  expected = {"one"};
  EXPECT_EQ(split_string(input, delimiter), expected);
}

TEST(TestStringUtils, IncludesString) {
  std::string str = "The quick brown fox jumps over the lazy dog";
  std::string substr1 = "brown fox";
  std::string substr2 = "purple cat";
  EXPECT_TRUE(string_includes(str, substr1));
  EXPECT_FALSE(string_includes(str, substr2));
}

}  // namespace utils
}  // namespace common
