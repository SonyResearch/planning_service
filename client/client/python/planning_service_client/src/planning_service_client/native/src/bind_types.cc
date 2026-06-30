#include <pybind11/eigen.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "bindings.h"
#include "planning_service_client/conf.h"
#include "planning_service_client/context_id.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/rgba.h"
#include "planning_service_client/visualizer_status.h"
#include "pybind_utils.h"

namespace planning_service_client::native_pybind {

void BindTypes(py::module_& m) {
  using planning_service_client::Conf;
  using planning_service_client::ContextId;
  using planning_service_client::FrameRelativePose;
  using planning_service_client::Rgba;
  using planning_service_client::SystemConf;
  using planning_service_client::VisualizerStatus;

  // --- Rgba ---
  auto rgba = py::class_<Rgba>(m, "Rgba", py::module_local());
  rgba.def(py::init<>())
      .def(py::init<double, double, double, double>(), py::arg("r"),
           py::arg("g"), py::arg("b"), py::arg("a") = 1.0)
      .def_property_readonly("r", &Rgba::r)
      .def_property_readonly("g", &Rgba::g)
      .def_property_readonly("b", &Rgba::b)
      .def_property_readonly("a", &Rgba::a)
      .def(py::self == py::self)
      .def_static("red", &Rgba::Red, py::arg("alpha") = 1.0)
      .def_static("green", &Rgba::Green, py::arg("alpha") = 1.0)
      .def_static("blue", &Rgba::Blue, py::arg("alpha") = 1.0)
      .def_static("white", &Rgba::White, py::arg("alpha") = 1.0)
      .def_static("black", &Rgba::Black, py::arg("alpha") = 1.0);
  AddProtoAndJsonMethods(rgba);

  auto context_id = py::class_<ContextId>(m, "ContextId", py::module_local());
  context_id
      .def(py::init<uint64_t, const std::string&>(), py::arg("value"),
           py::arg("system") = "")
      .def_property_readonly("value", &ContextId::value)
      .def_property_readonly("system", &ContextId::system)
      .def("__bool__", [](const ContextId& id) {
        return static_cast<bool>(id);
      });
  AddProtoAndJsonMethods(context_id);
  AddFileIOMethods(context_id);

  auto conf = py::class_<Conf>(m, "Conf", py::module_local());
  conf.def(py::init<>())
      .def(py::init<const Eigen::VectorXd&>(), py::arg("q"))
      .def_property_readonly("q", &Conf::q)
      .def(py::self == py::self);
  AddProtoAndJsonMethods(conf);
  AddFileIOMethods(conf);

  auto system_conf =
      py::class_<SystemConf>(m, "SystemConf", py::module_local());
  system_conf.def(py::init<>())
      .def(py::init<const std::map<std::string, Conf>&>(), py::arg("data"))
      .def("contains", &SystemConf::contains, py::arg("key"))
      .def(
          "at",
          [](const SystemConf& s, const std::string& key) {
            return s.at(key);
          },
          py::arg("key"))
      .def(
          "set",
          [](SystemConf& s, const std::string& key, const Conf& value) {
            s[key] = value;
          },
          py::arg("key"), py::arg("value"))
      .def("keys",
           [](const SystemConf& s) {
             std::vector<std::string> out;
             out.reserve(static_cast<size_t>(s.size()));
             for (const auto& kv : s.data()) {
               out.push_back(kv.first);
             }
             return out;
           })
      .def("values",
           [](const SystemConf& s) {
             std::vector<Conf> out;
             out.reserve(static_cast<size_t>(s.size()));
             for (const auto& kv : s.data()) {
               out.push_back(kv.second);
             }
             return out;
           })
      .def("items",
           [](const SystemConf& s) {
             std::vector<std::pair<std::string, Conf>> out;
             out.reserve(static_cast<size_t>(s.size()));
             for (const auto& kv : s.data()) {
               out.push_back(kv);
             }
             return out;
           })
      .def("size", &SystemConf::size)
      .def("empty", &SystemConf::empty)
      .def(py::self == py::self);
  AddProtoAndJsonMethods(system_conf);
  AddFileIOMethods(system_conf);

  auto frame_relative_pose =
      py::class_<FrameRelativePose>(m, "FrameRelativePose", py::module_local());
  frame_relative_pose.def(py::init<>())
      .def(py::init([](const std::string& frame_A, const std::string& frame_B,
                       const Eigen::Vector3d& translation,
                       const Eigen::Vector4d& quaternion_wxyz) {
             return FrameRelativePose(frame_A, frame_B, translation,
                                      QuaternionFromWxyz(quaternion_wxyz));
           }),
           py::arg("frame_A"), py::arg("frame_B"), py::arg("translation"),
           py::arg("quaternion_wxyz"))
      .def_property_readonly("frame_A", &FrameRelativePose::frame_A)
      .def_property_readonly("frame_B", &FrameRelativePose::frame_B)
      .def_property_readonly("translation",
                             &FrameRelativePose::X_AB_translation)
      .def_property_readonly("quaternion_wxyz",
                             [](const FrameRelativePose& p) {
                               return QuaternionToWxyz(p.X_AB_quaternion());
                             })
      .def(py::self == py::self);
  AddProtoAndJsonMethods(frame_relative_pose);
  AddFileIOMethods(frame_relative_pose);

  // --- VisualizerStatus ---
  py::class_<VisualizerStatus> viz_status(m, "VisualizerStatus",
                                          py::module_local());
  py::enum_<VisualizerStatus::Status>(viz_status, "Status")
      .value("UNSPECIFIED", VisualizerStatus::Status::kUnspecified)
      .value("IDLE", VisualizerStatus::Status::kIdle)
      .value("ACTIVE", VisualizerStatus::Status::kActive)
      .value("STARTING", VisualizerStatus::Status::kStarting)
      .value("STOPPING", VisualizerStatus::Status::kStopping)
      .value("ERROR", VisualizerStatus::Status::kError)
      .export_values();
  viz_status.def_property_readonly("status", &VisualizerStatus::status)
      .def_property_readonly("details", &VisualizerStatus::details);
}

}  // namespace planning_service_client::native_pybind
