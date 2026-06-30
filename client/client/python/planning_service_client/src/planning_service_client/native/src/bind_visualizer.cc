#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>

#include "bindings.h"
#include "planning_service_client/api/visualizer_client.h"
#include "planning_service_client/frame_relative_pose.h"
#include "planning_service_client/rgba.h"
#include "planning_service_client/shape.h"
#include "pybind_utils.h"

namespace planning_service_client::native_pybind {

void BindVisualizer(py::module_& m) {
  using planning_service_client::ContextId;
  using planning_service_client::FrameRelativePose;
  using planning_service_client::Rgba;
  using planning_service_client::ShapeInFrame;
  using planning_service_client::SystemConf;
  using planning_service_client::client::VisualizerClient;

  py::class_<VisualizerClient>(m, "VisualizerClient", py::module_local())
      .def(py::init<const std::string&, const std::string&,
                    const std::string&>(),
           py::arg("addr"), py::arg("client_id"), py::arg("config_json") = "{}")
      .def(
          "connect",
          [](VisualizerClient& c, int num_attempts, int attempt_interval_ms) {
            py::gil_scoped_release release;
            return c.Connect(num_attempts, attempt_interval_ms);
          },
          py::arg("num_attempts") = 5, py::arg("attempt_interval_ms") = 1000)
      .def(
          "start_visualizer",
          [](VisualizerClient& c, const ContextId& context_id,
             bool force_reload) {
            py::gil_scoped_release release;
            c.StartVisualizer(context_id, force_reload);
          },
          py::arg("context_id"), py::arg("force_reload") = false)
      .def("stop_visualizer",
           [](VisualizerClient& c) {
             py::gil_scoped_release release;
             c.StopVisualizer();
           })
      .def("get_visualizer_status",
           [](VisualizerClient& c) {
             py::gil_scoped_release release;
             return c.GetVisualizerStatus();
           })
      .def(
          "toggle_object",
          [](VisualizerClient& c, const std::string& path, bool visible) {
            py::gil_scoped_release release;
            c.ToggleObject(path, visible);
          },
          py::arg("path"), py::arg("visible"))
      .def(
          "set_object",
          [](VisualizerClient& c, const std::string& path,
             const ShapeInFrame& shape_in_frame, const Rgba& color) {
            py::gil_scoped_release release;
            c.SetObject(path, shape_in_frame, color);
          },
          py::arg("path"), py::arg("shape_in_frame"),
          py::arg("color") = Rgba::White())
      .def(
          "delete_object",
          [](VisualizerClient& c, const std::string& path) {
            py::gil_scoped_release release;
            c.DeleteObject(path);
          },
          py::arg("path"))
      .def("queue_streamed_configuration",
           &VisualizerClient::QueueStreamedConfiguration,
           py::arg("system_conf"))
      .def("stream_configurations",
           [](VisualizerClient& c) {
             py::gil_scoped_release release;
             c.StreamConfigurations();
           })
      .def("stream_configurations_async",
           &VisualizerClient::StreamConfigurationsAsync)
      .def("stop_stream_configurations",
           &VisualizerClient::StopStreamConfigurations)
      .def(
          "calc_pose",
          [](VisualizerClient& c, const std::string& frame_a,
             const std::string& frame_b,
             const std::optional<SystemConf>& system_conf_override) {
            py::gil_scoped_release release;
            return c.CalcPose(frame_a, frame_b, system_conf_override);
          },
          py::arg("frame_a"), py::arg("frame_b"),
          py::arg("system_conf_override") = py::none())
      .def(
          "toggle_frame",
          [](VisualizerClient& c, const std::string& frame, bool visible) {
            py::gil_scoped_release release;
            c.ToggleFrame(frame, visible);
          },
          py::arg("frame"), py::arg("visible"));
}

}  // namespace planning_service_client::native_pybind
