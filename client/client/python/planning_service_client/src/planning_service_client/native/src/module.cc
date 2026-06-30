#include <pybind11/pybind11.h>

#include "bindings.h"

namespace py = pybind11;

PYBIND11_MODULE(_planning_service_client, m) {
  m.doc() = "C++ (pybind11) bindings for planning_service_client";

  auto types = m.def_submodule("types", "Core proto-backed value types.");
  auto visualizer =
      m.def_submodule("visualizer", "Visualizer client bindings.");

  planning_service_client::native_pybind::BindTypes(types);
  planning_service_client::native_pybind::BindShape(types);
  planning_service_client::native_pybind::BindState(types);
  planning_service_client::native_pybind::BindVisualizer(visualizer);
}
