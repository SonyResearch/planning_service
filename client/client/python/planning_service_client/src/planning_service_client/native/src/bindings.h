#pragma once

#include <pybind11/pybind11.h>

namespace planning_service_client::native_pybind {

namespace py = pybind11;

/** Apply pybind implementations for some basic types to the base module. */
void BindTypes(py::module_& m);
/** Apply pybind implementations for Shape and ShapeInFrame. */
void BindShape(py::module_& m);
/** Apply pybind implementations for Value and State. */
void BindState(py::module_& m);
/**
 * @brief Apply pybind implementations for the visualizer client to the given
 * module.
 */
void BindVisualizer(py::module_& m);

}  // namespace planning_service_client::native_pybind
