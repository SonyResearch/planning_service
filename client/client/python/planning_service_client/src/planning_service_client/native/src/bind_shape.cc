#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>

#include "bindings.h"
#include "planning_service_client/shape.h"
#include "pybind_utils.h"

namespace planning_service_client::native_pybind {

void BindShape(py::module_& m) {
  using planning_service_client::Box;
  using planning_service_client::Capsule;
  using planning_service_client::Cylinder;
  using planning_service_client::Shape;
  using planning_service_client::ShapeInFrame;
  using planning_service_client::Sphere;

  // --- Shape (abstract base) ---
  py::class_<Shape>(m, "Shape", py::module_local())
      .def_property_readonly("dims", &Shape::dims)
      .def_property_readonly("type", &Shape::type);

  // --- Sphere ---
  auto sphere = py::class_<Sphere>(m, "Sphere", py::module_local());
  sphere.def(py::init<>())
      .def(py::init<double>(), py::arg("radius"))
      .def_property_readonly("radius", &Sphere::radius);
  AddProtoAndJsonMethods(sphere);

  // --- Cylinder ---
  auto cylinder = py::class_<Cylinder>(m, "Cylinder", py::module_local());
  cylinder.def(py::init<>())
      .def(py::init<double, double>(), py::arg("radius"), py::arg("height"))
      .def_property_readonly("radius", &Cylinder::radius)
      .def_property_readonly("height", &Cylinder::height);
  AddProtoAndJsonMethods(cylinder);

  // --- Capsule ---
  auto capsule = py::class_<Capsule>(m, "Capsule", py::module_local());
  capsule.def(py::init<>())
      .def(py::init<double, double>(), py::arg("radius"), py::arg("height"))
      .def_property_readonly("radius", &Capsule::radius)
      .def_property_readonly("height", &Capsule::height);
  AddProtoAndJsonMethods(capsule);

  // --- Box ---
  auto box = py::class_<Box>(m, "Box", py::module_local());
  box.def(py::init<>())
      .def(py::init<double, double, double>(), py::arg("width"),
           py::arg("depth"), py::arg("height"))
      .def_property_readonly("width", &Box::width)
      .def_property_readonly("depth", &Box::depth)
      .def_property_readonly("height", &Box::height)
      .def_static("cube", &Box::Cube, py::arg("side_length"));
  AddProtoAndJsonMethods(box);

  // --- ShapeInFrame ---
  auto shape_in_frame =
      py::class_<ShapeInFrame>(m, "ShapeInFrame", py::module_local());
  shape_in_frame.def(py::init<>())
      .def(py::init<const Sphere&>(), py::arg("shape"))
      .def(py::init<const Cylinder&>(), py::arg("shape"))
      .def(py::init<const Capsule&>(), py::arg("shape"))
      .def(py::init<const Box&>(), py::arg("shape"))
      .def_property_readonly("frame", &ShapeInFrame::frame)
      .def_property_readonly("translation", &ShapeInFrame::translation)
      .def_property_readonly("quaternion_wxyz",
                             [](const ShapeInFrame& s) {
                               return QuaternionToWxyz(s.quaternion());
                             })
      .def_property_readonly("color", &ShapeInFrame::color)
      .def_property_readonly("name", &ShapeInFrame::name)
      .def("set_frame", &ShapeInFrame::set_frame, py::arg("frame"))
      .def("set_translation", &ShapeInFrame::set_translation,
           py::arg("translation"))
      .def(
          "set_quaternion",
          [](ShapeInFrame& s, const Eigen::Vector4d& wxyz) {
            s.set_quaternion(QuaternionFromWxyz(wxyz));
          },
          py::arg("quaternion_wxyz"))
      .def("set_color", &ShapeInFrame::set_color, py::arg("color"))
      .def("set_name", &ShapeInFrame::set_name, py::arg("name"))
      .def(
          "set_shape",
          [](ShapeInFrame& s, const Sphere& shape) {
            s.set_shape(shape);
          },
          py::arg("shape"))
      .def(
          "set_shape",
          [](ShapeInFrame& s, const Cylinder& shape) {
            s.set_shape(shape);
          },
          py::arg("shape"))
      .def(
          "set_shape",
          [](ShapeInFrame& s, const Capsule& shape) {
            s.set_shape(shape);
          },
          py::arg("shape"))
      .def(
          "set_shape",
          [](ShapeInFrame& s, const Box& shape) {
            s.set_shape(shape);
          },
          py::arg("shape"))
      .def_property_readonly("is_sphere", &ShapeInFrame::Is<Sphere>)
      .def_property_readonly("is_cylinder", &ShapeInFrame::Is<Cylinder>)
      .def_property_readonly("is_capsule", &ShapeInFrame::Is<Capsule>)
      .def_property_readonly("is_box", &ShapeInFrame::Is<Box>)
      .def_property_readonly(
          "shape",
          [](const ShapeInFrame& s) -> py::object {
            if (s.Is<Sphere>()) return py::cast(s.sphere());
            if (s.Is<Cylinder>()) return py::cast(s.cylinder());
            if (s.Is<Capsule>()) return py::cast(s.capsule());
            if (s.Is<Box>()) return py::cast(s.box());
            throw std::runtime_error(
                "ShapeInFrame::shape: unknown or unset shape type");
          })
      .def_property_readonly("sphere", &ShapeInFrame::sphere)
      .def_property_readonly("cylinder", &ShapeInFrame::cylinder)
      .def_property_readonly("capsule", &ShapeInFrame::capsule)
      .def_property_readonly("box", &ShapeInFrame::box)
      .def_static("make_sphere", &ShapeInFrame::MakeSphere, py::arg("radius"))
      .def_static("make_cylinder", &ShapeInFrame::MakeCylinder,
                  py::arg("radius"), py::arg("height"))
      .def_static("make_capsule", &ShapeInFrame::MakeCapsule, py::arg("radius"),
                  py::arg("height"))
      .def_static("make_box", &ShapeInFrame::MakeBox, py::arg("width"),
                  py::arg("depth"), py::arg("height"));
  AddProtoAndJsonMethods(shape_in_frame);
}

}  // namespace planning_service_client::native_pybind
