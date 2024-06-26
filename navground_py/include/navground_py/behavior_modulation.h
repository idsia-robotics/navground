#ifndef NAVGROUND_CORE_PY_BEHAVIOR_MODULATION_H
#define NAVGROUND_CORE_PY_BEHAVIOR_MODULATION_H

#include <pybind11/pybind11.h>

#include <memory>

#include "collection.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"

using navground::core::Behavior;
using navground::core::BehaviorModulation;

namespace py = pybind11;

inline void add_modulation_py(const py::object &obj, const py::object value) {
  add_py_item(obj, value, "modulations");
  obj.cast<Behavior &>().add_modulation(
      value.cast<std::shared_ptr<BehaviorModulation>>());
}

inline void remove_modulation_py(const py::object &obj,
                                 const py::object value) {
  obj.cast<Behavior &>().remove_modulation(
      value.cast<std::shared_ptr<BehaviorModulation>>());
  remove_py_item(obj, value, "modulations");
}

inline void clear_modulations_py(const py::object &obj) {
  obj.cast<Behavior &>().clear_modulations();
  clear_collection_py(obj, "modulations");
}

#endif  // NAVGROUND_CORE_PY_BEHAVIOR_MODULATION_H
