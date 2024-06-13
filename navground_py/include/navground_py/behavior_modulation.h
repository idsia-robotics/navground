#ifndef NAVGROUND_CORE_PY_BEHAVIOR_MODULATION_H
#define NAVGROUND_CORE_PY_BEHAVIOR_MODULATION_H

#include <pybind11/pybind11.h>

#include <memory>

#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"

using navground::core::Behavior;
using navground::core::BehaviorModulation;

namespace py = pybind11;

inline bool has_modulations_py(const py::object &obj) {
  return py::hasattr(obj, "__py_modulations");
}

inline py::list get_modulations_py(const py::object &obj) {
  if (!py::hasattr(obj, "__py_modulations")) {
    obj.attr("__py_modulations") = py::list();
  }
  return static_cast<py::list>(py::getattr(obj, "__py_modulations"));
}

inline void add_modulation_py(const py::object &obj, const py::object value) {
  get_modulations_py(obj).append(value);
  obj.cast<Behavior &>().add_modulation(
      value.cast<std::shared_ptr<BehaviorModulation>>());
}

inline void remove_modulation_py(const py::object &obj,
                                 const py::object value) {
  obj.cast<Behavior &>().remove_modulation(
      value.cast<std::shared_ptr<BehaviorModulation>>());
  get_modulations_py(obj).attr("remove")(value);
}

inline void clear_modulations_py(const py::object &obj) {
  obj.cast<Behavior &>().clear_modulations();
  if (has_modulations_py(obj)) {
    get_modulations_py(obj).attr("clear")();
  }
}

#endif  // NAVGROUND_CORE_PY_YAML_H
