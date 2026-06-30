/// @file file_utils.cc

#include "file_utils.h"

#include "planning_service/common/logging.h"

namespace common {
namespace utils {

fs::path temp_dir() {
  std::string path_template_str {"planning_service_XXXXXX"};
  const char* dtemp = ::mkdtemp(&path_template_str[0]);
  return fs::temp_directory_path() / dtemp;
}

std::string FileToString(const fs::path& file_path) {
  if (!fs::exists(file_path)) {
    throw std::runtime_error("FileToString: File " + file_path.string()
                             + " does not exist!");
  }
  std::ifstream input {file_path, std::fstream::binary | std::fstream::in};
  std::stringstream buffer;
  buffer << input.rdbuf();
  const auto string {std::string(buffer.str())};
  return string.c_str();
}

bool SaveToFile(const fs::path& file_path, const std::string& contents,
                bool overwrite_existing, bool binary) {
  if (!fs::is_directory(file_path.parent_path())) {
    return false;
  }
  if (fs::is_regular_file(file_path) && !overwrite_existing) {
    logging::log()->warn("SaveToFile: File {} already exists", file_path);
    return true;
  }
  logging::log()->debug("SaveToFile: Saving to {}", file_path);
  try {
    std::ofstream out {
        file_path, binary ? std::ios::binary | std::ios::out : std::ios::out};
    out << contents;
    out.close();
    if (!out.good()) {
      logging::log()->error("SaveToFile: I/O exception: {}", strerror(errno));
      return false;
    }
  } catch (const std::exception& e) {
    logging::log()->error("SaveToFile: Miscellaneous exception: {}", e.what());
    return false;
  }
  return true;
}

bool SaveJsonToFile(const fs::path& file_path, const json& j, int indent,
                    bool overwrite_existing) {
  if (file_path.extension() != ".json") {
    logging::log()->error("SaveJsonToFile: File {} is not a JSON file",
                          file_path);
    return false;
  }
  bool binary {true};
  return SaveToFile(file_path, j.dump(indent), overwrite_existing, binary);
}

bool LoadJsonFromFile(const fs::path& file_path, json& j) {
  if (!fs::exists(file_path)) {
    logging::log()->error("LoadJsonFromFile: File {} does not exist",
                          file_path);
    return false;
  }
  if (file_path.extension() != ".json") {
    logging::log()->error("LoadJsonFromFile: File {} is not a JSON file",
                          file_path);
    return false;
  }
  try {
    std::ifstream file(file_path);
    j = json::parse(file);
  } catch (const std::exception& e) {
    logging::log()->error("LoadJsonFromFile: Exception: {}", e.what());
    return false;
  }
  return true;
}

bool StringInFile(const fs::path& file_path, const std::string& str) {
  if (!fs::is_regular_file(file_path)) {
    throw std::runtime_error(
        fmt::format("File '{}' does not exist", file_path));
  }
  // Open the file
  std::ifstream file {file_path, std::fstream::binary | std::fstream::in};
  if (!file.is_open()) {
    throw std::runtime_error(
        fmt::format("Failed to open file '{}'", file_path));
  }
  // Read line-by-line
  bool found {false};
  std::string line;
  int cnt {0};
  while (std::getline(file, line)) {
    ++cnt;
    if (line.find(str) != std::string::npos) {
      logging::log()->debug("StringInFile: Found: '{}' in line {}: '{}'", str,
                            cnt, line);
      found = true;
      break;
    }
  }
  file.close();
  return found;
}

}  // namespace utils
}  // namespace common
