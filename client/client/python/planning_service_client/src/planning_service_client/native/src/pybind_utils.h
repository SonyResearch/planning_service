#pragma once

#include <Eigen/Dense>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>

#include <string>
#include <type_traits>

#include "planning_service_client/common/io_utils.h"
#include "planning_service_client/internal/proto_base.h"

namespace planning_service_client::native_pybind {

namespace py = pybind11;

/**
 * @brief Serialize an object to protobuf binary form and return as Python
 * `bytes`.
 *
 * Enabled only for types deriving from
 * `planning_service_client::internal::ProtoBase`.
 */
template <typename T,
          typename = std::enable_if_t<
              planning_service_client::internal::is_proto_base_v<T>>>
py::bytes ToProtoBytes(const T& obj) {
  return py::bytes(obj.ToString(/*binary=*/true));
}

/**
 * @brief Parse an object from protobuf binary form provided as Python `bytes`.
 *
 * Enabled only for types deriving from
 * `planning_service_client::internal::ProtoBase`.
 *
 * @tparam T Type to deserialize.
 * @param b Python `bytes` containing the binary-serialized message.
 * @return Deserialized instance of `T`.
 */
template <typename T,
          typename = std::enable_if_t<
              planning_service_client::internal::is_proto_base_v<T>>>
T FromProtoBytes(const py::bytes& b) {
  const std::string s = py::cast<std::string>(b);
  return planning_service_client::FromString<T>(s, /*binary=*/true);
}

/**
 * @brief Add `to_proto_bytes()` and `from_proto_bytes()` methods to a bound
 * class.
 *
 * Enabled only for types deriving from
 * `planning_service_client::internal::ProtoBase`.
 */
template <typename T>
py::class_<T>& AddProtoBytesMethods(
    py::class_<T>& cls, const char* to_name = "to_proto_bytes",
    const char* from_name = "from_proto_bytes") {
  static_assert(planning_service_client::internal::is_proto_base_v<T>,
                "AddProtoBytesMethods requires T to derive from ProtoBase");
  cls.def(to_name, &ToProtoBytes<T>);
  cls.def_static(from_name, &FromProtoBytes<T>);
  return cls;
}

/**
 * @brief Add a `to_json()` method to a bound class.
 *
 * This assumes `T` provides a `ToJson()` member function.
 *
 * @tparam T Bound C++ type.
 * @param cls pybind11 class wrapper to add the method to.
 * @param name Python method name.
 * @return Reference to `cls` for call chaining.
 */
template <typename T>
py::class_<T>& AddToJsonMethod(py::class_<T>& cls,
                               const char* name = "to_json") {
  cls.def(name, &T::ToJson);
  return cls;
}

/**
 * @brief Add `__str__` and `__repr__` methods to a bound class.
 *
 * Both methods use the proto text-format string produced by `ToString()`,
 * mirroring the C++ base-class string representation. `__repr__` additionally
 * wraps the output with the Python class name.
 *
 * Enabled only for types deriving from
 * `planning_service_client::internal::ProtoBase`.
 */
template <typename T>
py::class_<T>& AddStringMethods(py::class_<T>& cls) {
  static_assert(planning_service_client::internal::is_proto_base_v<T>,
                "AddStringMethods requires T to derive from ProtoBase");
  cls.def("__str__", [](const T& self) {
    return self.ToString();
  });
  cls.def("__repr__", [](const T& self) {
    const std::string class_name = py::type::of(py::cast(self))
                                       .attr("__name__")
                                       .template cast<std::string>();
    return class_name + "(" + self.ToString() + ")";
  });
  return cls;
}

/**
 * @brief Convenience helper adding both proto-bytes and JSON methods.
 *
 * Adds: `to_proto_bytes`, `from_proto_bytes`, `to_json`, `__str__`, and
 * `__repr__`.
 *
 * @tparam T Bound C++ type.
 * @param cls pybind11 class wrapper.
 * @return Reference to `cls` for call chaining.
 */
template <typename T>
py::class_<T>& AddProtoAndJsonMethods(py::class_<T>& cls) {
  static_assert(planning_service_client::internal::is_proto_base_v<T>,
                "AddProtoAndJsonMethods requires T to derive from ProtoBase");
  AddProtoBytesMethods(cls);
  AddToJsonMethod(cls);
  AddStringMethods(cls);
  return cls;
}

/**
 * @brief Add `save_to_json`, `save_to_file`, `load_from_json_file`, and
 * `load_from_file` methods to a bound class.
 *
 * Enabled only for types deriving from
 * `planning_service_client::internal::ProtoBase`.
 */
template <typename T>
py::class_<T>& AddFileIOMethods(py::class_<T>& cls) {
  static_assert(planning_service_client::internal::is_proto_base_v<T>,
                "AddFileIOMethods requires T to derive from ProtoBase");
  cls.def(
         "save_to_json",
         [](const T& self, const std::string& path) {
           planning_service_client::common::SaveToJson(path, self);
         },
         py::arg("path"))
      .def(
          "save_to_file",
          [](const T& self, const std::string& path, bool binary) {
            planning_service_client::common::SaveToFile(path, self, binary);
          },
          py::arg("path"), py::arg("binary") = false)
      .def_static(
          "load_from_json_file",
          [](const std::string& path) {
            return planning_service_client::common::LoadFromJsonFile<T>(path);
          },
          py::arg("path"))
      .def_static(
          "load_from_file",
          [](const std::string& path, bool binary) {
            return planning_service_client::common::LoadFromFile<T>(path,
                                                                    binary);
          },
          py::arg("path"), py::arg("binary") = false);
  return cls;
}

/**
 * @brief Convert quaternion stored as (w, x, y, z) into an Eigen quaternion.
 *
 * @param wxyz Vector containing quaternion coefficients in (w, x, y, z) order.
 * @return Eigen quaternion with the corresponding coefficients.
 */
inline Eigen::Quaterniond QuaternionFromWxyz(const Eigen::Vector4d& wxyz) {
  return Eigen::Quaterniond(wxyz[0], wxyz[1], wxyz[2], wxyz[3]);
}

/**
 * @brief Convert an Eigen quaternion into a (w, x, y, z) coefficient vector.
 *
 * @param q Eigen quaternion.
 * @return Vector of coefficients in (w, x, y, z) order.
 */
inline Eigen::Vector4d QuaternionToWxyz(const Eigen::Quaterniond& q) {
  return Eigen::Vector4d(q.w(), q.x(), q.y(), q.z());
}

}  // namespace planning_service_client::native_pybind
