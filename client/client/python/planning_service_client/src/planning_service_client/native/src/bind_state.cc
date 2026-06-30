#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <string>
#include <vector>

#include "bindings.h"
#include "planning_service_client/state.h"
#include "pybind_utils.h"

namespace planning_service_client::native_pybind {

namespace {

// Convert an arbitrary Python object to a Value, mirroring the implicit
// constructors available in C++.  Checked in this order:
//   None  -> monostate
//   bool  -> bool   (must come before int: bool is a subclass of int)
//   int   -> double
//   float -> double
//   str   -> string
//   list  -> vector<Value>  (elements converted recursively)
//   Value -> Value  (pass-through)
//   State -> Value(State)
Value ValueFromPy(const py::object& obj) {
  using planning_service_client::State;
  using planning_service_client::Value;
  if (obj.is_none()) return Value();
  if (py::isinstance<py::bool_>(obj)) return Value(obj.cast<bool>());
  if (py::isinstance<py::int_>(obj)) return Value(obj.cast<double>());
  if (py::isinstance<py::float_>(obj)) return Value(obj.cast<double>());
  if (py::isinstance<py::str>(obj)) return Value(obj.cast<std::string>());
  if (py::isinstance<py::list>(obj)) {
    std::vector<Value> vec;
    for (const auto& item : obj) {
      vec.push_back(ValueFromPy(py::reinterpret_borrow<py::object>(item)));
    }
    return Value(std::move(vec));
  }
  if (py::isinstance<Value>(obj)) return obj.cast<Value>();
  if (py::isinstance<State>(obj)) return Value(obj.cast<State>());
  throw py::type_error(
      "Cannot convert Python object to Value: unsupported type '"
      + std::string(py::str(py::type::of(obj))) + "'");
}

}  // namespace

void BindState(py::module_& m) {
  using planning_service_client::State;
  using planning_service_client::Value;

  // Forward-declare both before defining methods: Value can hold a State and
  // State maps string -> Value, so both type registrations must exist first.
  auto value = py::class_<Value>(m, "Value", py::module_local());
  auto state = py::class_<State>(m, "State", py::module_local());

  value.def(py::init<>())
      .def(py::init<const std::string&>(), py::arg("v"))
      .def(py::init<double>(), py::arg("v"))
      .def(py::init<bool>(), py::arg("v"))
      .def(py::init<const State&>(), py::arg("v"))
      .def(py::init<const std::vector<Value>&>(), py::arg("v"))
      .def_property_readonly("is_none",
                             [](const Value& v) {
                               return v.Is<std::monostate>();
                             })
      .def_property_readonly("is_string",
                             [](const Value& v) {
                               return v.Is<std::string>();
                             })
      .def_property_readonly("is_double",
                             [](const Value& v) {
                               return v.Is<double>();
                             })
      .def_property_readonly("is_bool",
                             [](const Value& v) {
                               return v.Is<bool>();
                             })
      .def_property_readonly("is_list",
                             [](const Value& v) {
                               return v.Is<std::vector<Value>>();
                             })
      .def_property_readonly("is_state",
                             [](const Value& v) {
                               return v.Is<State>();
                             })
      .def_property_readonly("as_string",
                             [](const Value& v) {
                               return v.get<std::string>();
                             })
      .def_property_readonly("as_double",
                             [](const Value& v) {
                               return v.get<double>();
                             })
      .def_property_readonly("as_bool",
                             [](const Value& v) {
                               return v.get<bool>();
                             })
      .def_property_readonly("as_list",
                             [](const Value& v) {
                               return v.get<std::vector<Value>>();
                             })
      .def_property_readonly("as_state",
                             [](const Value& v) {
                               return v.get<State>();
                             })
      .def_property_readonly("index", &Value::index)
      .def("to_compact_string", &Value::ToCompactString);
  AddProtoAndJsonMethods(value);
  AddFileIOMethods(value);

  state.def(py::init<>())
      .def(
          "add",
          [](State& s, const std::string& key, const py::object& value) {
            s.Add(key, ValueFromPy(value));
          },
          py::arg("key"), py::arg("value"))
      .def("erase", &State::Erase, py::arg("key"))
      .def("contains", &State::Contains, py::arg("key"))
      .def(
          "__getitem__",
          [](const State& s, const std::string& key) {
            return s[key];
          },
          py::arg("key"))
      .def(
          "__setitem__",
          [](State& s, const std::string& key, const py::object& value) {
            s.Add(key, ValueFromPy(value));
          },
          py::arg("key"), py::arg("value"))
      .def("__contains__", &State::Contains, py::arg("key"))
      .def("calc_hash", &State::CalcHash)
      .def("to_compact_string", &State::ToCompactString,
           py::arg("include_hash") = false)
      .def("is_subset_of", &State::IsSubsetOf, py::arg("other"))
      .def("keys",
           [](const State& s) {
             std::vector<std::string> out;
             for (const auto& kv : s) out.push_back(kv.first);
             return out;
           })
      .def("values",
           [](const State& s) {
             std::vector<Value> out;
             for (const auto& kv : s) out.push_back(kv.second);
             return out;
           })
      .def("items", [](const State& s) {
        std::vector<std::pair<std::string, Value>> out;
        for (const auto& kv : s) out.push_back(kv);
        return out;
      });
  AddProtoAndJsonMethods(state);
  AddFileIOMethods(state);
}

}  // namespace planning_service_client::native_pybind
