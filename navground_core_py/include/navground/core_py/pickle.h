#ifndef NAVGROUND_CORE_PY_PICKLE_H
#define NAVGROUND_CORE_PY_PICKLE_H

#include <pybind11/pybind11.h>

#include "yaml.h"

namespace py = pybind11;
//
template <typename S, typename C> void pickle_via_yaml(C &cls) {
  cls.def(py::pickle(
      [](typename C::type *obj) {
        // __getstate__
        auto py_obj = py::cast(obj);
        if (py::hasattr(py_obj, "__dict__")) {
          return py::make_tuple(YAML::dump<typename S::Native>(obj),
                                py_obj.attr("__dict__"));
        }
        return py::make_tuple(YAML::dump<typename S::Native>(obj));
      },
      [](const py::tuple &t) {
        // __setstate__
        if (t.size() > 0) {
          const YAML::Node node = YAML::Load(t[0].cast<std::string>());
          auto obj = YAML::load_node_py<S>(node);
          auto cpp_obj = obj.template cast<std::shared_ptr<typename C::type>>();
          py::dict py_state;
          if (t.size() > 1) {
            py_state = t[1].cast<py::dict>();
          }
          return std::make_pair(cpp_obj, py_state);
        }
        throw std::runtime_error("Invalid state!");
      }));
}
template <typename S, typename C> void pickle_via_yaml_native(C &cls) {
  cls.def(py::pickle(
      [](typename C::type *obj) {
        // __getstate__
        return py::make_tuple(YAML::dump<S>(obj));
      },
      [](py::tuple t) {
        // __setstate__
        if (t.size() != 1)
          throw std::runtime_error("Invalid state!");
        const YAML::Node node = YAML::Load(t[0].cast<std::string>());
        return node.as<S>();
      }));
}

#endif // NAVGROUND_CORE_PY_PICKLE_H
