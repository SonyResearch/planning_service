

#include <filesystem>
#include <fstream>
#include <sstream>

#include <google/protobuf/message.h>
#include <google/protobuf/text_format.h>
#include <google/protobuf/util/json_util.h>

#include "planning_service_client/internal/proto_base.h"
namespace planning_service_client {
namespace common {

/** Write string contents to a file. */
void WriteToFile(const std::filesystem::path& path, const std::string& contents,
                 bool binary = false);
/** Load file contents into a string. */
void LoadFromFile(const std::filesystem::path& path, std::string& contents,
                  bool binary = false);

/** Save a ProtoBase object to a JSON file. */
template <typename DerivedProto>
void SaveToJson(const std::filesystem::path& path,
                const internal::ProtoBase<DerivedProto>& obj) {
  WriteToFile(path, obj.ToJson());
}
/** Save a ProtoBase object to a file. */
template <typename DerivedProto>
void SaveToFile(const std::filesystem::path& path,
                const internal::ProtoBase<DerivedProto>& obj,
                bool binary = false) {
  WriteToFile(path, obj.ToString(binary), binary);
}

template <typename Derived>
Derived LoadFromJsonFile(const std::filesystem::path& path) {
  std::string in;
  LoadFromFile(path, in);
  return FromJson<Derived>(in);
}

template <typename Derived>
Derived LoadFromFile(const std::filesystem::path& path, bool binary) {
  std::string in;
  LoadFromFile(path, in, binary);
  return FromString<Derived>(in, binary);
}

}  // namespace common
}  // namespace planning_service_client
