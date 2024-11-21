#ifndef NAVGROUND_CORE_PY_PROPERTY_H
#define NAVGROUND_CORE_PY_PROPERTY_H

#include <pybind11/pybind11.h>

#include "navground/core/property.h"

namespace py = pybind11;
using navground::core::Property;
using navground::core::Vector2;
using navground::core::HasProperties;

template <typename> struct is_std_vector : std::false_type {};

template <typename T, typename A>
struct is_std_vector<std::vector<T, A>> : std::true_type {};

template <typename T>
std::optional<Property::Field> convert_py(const Property::Field &value) {
  return std::visit(
      [](auto &&arg) -> std::optional<Property::Field> {
        using V = std::decay_t<decltype(arg)>;
        if constexpr (std::is_convertible<V, T>::value) {
          return static_cast<T>(arg);
        }
        // accept list[integer] as a list[float]
        // if constexpr (std::is_same_v<T, std::vector<ng_float_t>> &&
        //               std::is_same_v<V, std::vector<int>>) {
        //   return arg;
        // }
        // nice idea but does not work as Python may convert [1, 1.1] to a
        // list[bool]

        // accept empty lists as list[X]
        if constexpr (is_std_vector<V>::value && is_std_vector<T>::value) {
          if (arg.size() == 0) {
            return arg;
          }
        }
        // accept list of 2 floats or integers as vector2
        if constexpr ((std::is_same_v<V, std::vector<ng_float_t>> ||
                       std::is_same_v<V, std::vector<int>>) &&
                      std::is_same_v<T, Vector2>) {
          if (arg.size() == 2) {
            return Vector2(arg[0], arg[1]);
          }
        }
        // accept vector2 as list of (2) floats
        if constexpr (std::is_same_v<V, Vector2> &&
                      std::is_same_v<T, std::vector<ng_float_t>>) {
          return std::vector<ng_float_t>{arg[0], arg[1]};
        }
        // we reject everything else, just like in C++
        return std::nullopt;
      },
      value);
}

inline std::optional<Property::Field>
convert_py(const Property::Field &value, const Property::Field &default_value) {
  return std::visit(
      [&value](auto &&arg) {
        using T = std::decay_t<decltype(arg)>;
        return convert_py<T>(value);
      },
      default_value);
}

inline Property
make_property_py(const py::object &py_getter, const py::object &py_setter,
                 const Property::Field &default_value,
                 const std::string &description = "",
                 const std::vector<std::string> &deprecated_names = {}) {
  Property p;
  if (!py_getter.is_none()) {
    p.getter = [py_getter](const HasProperties *obj) {
      auto py_value = py_getter.attr("__call__")(py::cast(obj));
      return py_value.cast<Property::Field>();
    };
  } else {
    p.getter = nullptr;
  }
  if (!py_setter.is_none()) {
    p.setter = [py_setter, default_value](HasProperties *obj,
                                          const Property::Field &value) {
      // py::print("set value to", value, "default is", default_value);
      if (auto v = convert_py(value, default_value)) {
        // py::print("converted to", *v);
        py_setter.attr("__call__")(py::cast(obj), py::cast(*v));
      } else {
        // py::print("not compatible!");
      }
    };
  } else {
    p.setter = nullptr;
  }
  p.readonly = py_setter.is_none();
  p.default_value = default_value;
  p.type_name = std::string(Property::friendly_type_name(default_value));
  p.description = description;
  p.deprecated_names = deprecated_names;
  return p;
}

inline Property make_property_with_py_property(
    const py::object &py_property, const Property::Field &default_value,
    const std::string &description = "",
    const std::vector<std::string> &deprecated_names = {}) {
  return make_property_py(py_property.attr("fget"), py_property.attr("fset"),
                          default_value, description, deprecated_names);
}

#endif // NAVGROUND_CORE_PY_YAML_H