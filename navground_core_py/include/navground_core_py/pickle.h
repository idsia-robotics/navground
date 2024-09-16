#ifndef NAVGROUND_CORE_PY_PICKLE_H
#define NAVGROUND_CORE_PY_PICKLE_H

#include <pybind11/pybind11.h>

#include "yaml.h"

namespace py = pybind11;
// 
template <typename S, typename C>
void pickle_via_yaml(C &cls) {
  cls.def(py::pickle(
      [](typename C::type *obj) {
        // __getstate__
        return py::make_tuple(YAML::dump<typename S::Native>(obj));
      },
      [](py::tuple t) {
        // __setstate__
        if (t.size() != 1) throw std::runtime_error("Invalid state!");
        const YAML::Node node = YAML::Load(t[0].cast<std::string>());
        auto obj = YAML::load_node_py<S>(node);
        return obj.template cast<std::shared_ptr<typename C::type>>();
      }));
}
template <typename S, typename C>
void pickle_via_yaml_native(C &cls) {
  cls.def(py::pickle(
      [](typename C::type *obj) {
        // __getstate__
        return py::make_tuple(YAML::dump<S>(obj));
      },
      [](py::tuple t) {
        // __setstate__
        if (t.size() != 1) throw std::runtime_error("Invalid state!");
        const YAML::Node node = YAML::Load(t[0].cast<std::string>());
        return node.as<S>();
      }));
}

#endif  // NAVGROUND_CORE_PY_PICKLE_H
