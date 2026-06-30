

#include "io_utils.h"

namespace planning_service_client {
namespace common {

void WriteToFile(const std::filesystem::path& path, const std::string& contents,
                 bool binary) {
  const auto canonical_path {std::filesystem::weakly_canonical(path)};
  if (!std::filesystem::is_directory(canonical_path.parent_path())) {
    throw std::runtime_error("Parent directory `"
                             + canonical_path.parent_path().string()
                             + "` does not exist.");
  }
  auto options {std::ios::out};
  if (binary) {
    options |= std::ios::binary;
  }
  std::ofstream out {canonical_path, options};
  out << contents;
  out.close();
  if (!out.good()) throw;
}

void LoadFromFile(const std::filesystem::path& path, std::string& contents,
                  bool binary) {
  const auto canonical_path {std::filesystem::weakly_canonical(path)};
  if (!std::filesystem::is_regular_file(canonical_path)) {
    throw std::runtime_error("File `" + canonical_path.string()
                             + "` does not exist.");
  }
  auto options {std::ios::in};
  if (binary) {
    options |= std::ios::binary;
  }
  std::ifstream in {canonical_path, options};
  if (!in.is_open())
    throw std::runtime_error("Failed to open file `" + canonical_path.string()
                             + "`.");
  std::stringstream buffer;
  buffer << in.rdbuf();
  in.close();
  if (!in.good()) throw;
  contents = buffer.str();
}

}  // namespace common
}  // namespace planning_service_client
