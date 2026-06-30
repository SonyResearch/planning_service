/// @file misc_utils.cc

#include "misc_utils.h"

#include <array>
namespace common {
namespace utils {

std::string GetTag() {
  std::string result;
  const char* version = std::getenv("SEMVER_TAG");
  if (version != nullptr && version[0] != '\0') {
    result = std::string(version);
  } else {
    result += "dev";
    // read short ref from git
    std::array<char, 128> buffer;
    std::unique_ptr<FILE, int (*)(FILE*)> pipe(
        popen("git rev-parse --short HEAD", "r"), pclose);
    if (!pipe) {
      throw std::runtime_error("GetTag: popen() failed!");
    }
    std::string ref;
    while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
      ref += buffer.data();
    }
    // Remove any trailing newline character
    if (!ref.empty() && ref.back() == '\n') {
      ref.pop_back();
    }
    result += "-" + ref;
  }
  return result;
}
}  // namespace utils
}  // namespace common
