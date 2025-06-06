#ifndef NAVGROUND_CORE_PY_PROPERTY_H
#define NAVGROUND_CORE_PY_PROPERTY_H

#include <pybind11/pybind11.h>

#include "navground/core/property.h"
#include "navground/core_py/yaml.h"

namespace py = pybind11;
using navground::core::get_scalar_type_name;
using navground::core::HasProperties;
using navground::core::Property;
using navground::core::Vector2;

using Converter =
    std::function<std::optional<Property::Field>(const Property::Field &)>;

bool is_empty_vector(const Property::Field &value) {
  return std::visit(
      [](auto &&arg) -> bool {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (is_std_vector_v<T>) {
          return arg.size() == 0;
        }
        return false;
      },
      value);
}

template <typename T>
std::optional<Property::Field> convert_py(const Property::Field &value) {
  return std::visit(
      [](auto &&arg) -> std::optional<Property::Field> {
        using V = std::decay_t<decltype(arg)>;
        if constexpr (std::is_convertible<V, T>::value) {
          return static_cast<T>(arg);
        }
        if constexpr (is_std_vector_v<T>) {
          using TS = typename T::value_type;
          if constexpr (is_std_vector_v<V>) {
            // Empty vectors are always accepted.
            if (arg.size() == 0) {
              return std::vector<TS>();
            }
            using VS = std::decay_t<decltype(arg[0])>;

            if constexpr (std::is_convertible<VS, TS>::value) {
              std::vector<TS> rs(arg.size());
              std::transform(arg.begin(), arg.end(), rs.begin(),
                             [](auto &&item) { return static_cast<TS>(item); });
              return rs;
            }
          } else if constexpr (std::is_same_v<V, Vector2>) {
            // accept a vector2 as a list of (2) numbers
            if constexpr (std::is_convertible<ng_float_t, TS>::value) {
              return std::vector<TS>{static_cast<TS>(arg[0]),
                                     static_cast<TS>(arg[1])};
            }
          }
          return std::nullopt;
        }
        // accept list of 2 numbers as vector2
        if constexpr (is_std_vector_v<V> && std::is_same_v<T, Vector2>) {
          using VS = std::decay_t<decltype(arg[0])>;
          if constexpr (std::is_convertible<VS, ng_float_t>::value) {
            if (arg.size() == 2) {
              return Vector2(static_cast<ng_float_t>(arg[0]),
                             static_cast<ng_float_t>(arg[1]));
            }
          }
        }
        // we reject everything else, just like in C++
        return std::nullopt;
      },
      value);
}

// Pay attention that Python may pass
// a value like ``[3.2, 1, 4] as [True, True, True]

inline std::optional<Converter>
get_converter(const std::string &scalar_type_name, bool is_vector) {
  if (is_vector) {
    if (scalar_type_name == "int")
      return convert_py<std::vector<int>>;
    if (scalar_type_name == "float")
      return convert_py<std::vector<ng_float_t>>;
    if (scalar_type_name == "str")
      return convert_py<std::vector<std::string>>;
    if (scalar_type_name == "bool")
      return convert_py<std::vector<bool>>;
    if (scalar_type_name == "vector")
      return convert_py<std::vector<Vector2>>;
    return std::nullopt;
  }
  if (scalar_type_name == "int")
    return convert_py<int>;
  if (scalar_type_name == "float")
    return convert_py<ng_float_t>;
  if (scalar_type_name == "str")
    return convert_py<std::string>;
  if (scalar_type_name == "bool")
    return convert_py<bool>;
  if (scalar_type_name == "vector")
    return convert_py<Vector2>;
  return std::nullopt;
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
                 const Converter &converter,
                 const std::string &description = "",
                 const std::optional<py::function> &py_schema = std::nullopt,
                 const std::vector<std::string> &deprecated_names = {}) {
  Property p;
  if (!py_getter.is_none()) {
    p.getter = [py_getter, converter, default_value](const HasProperties *obj) {
      // CHANGED(28/5/2025): Added layer of safety here because we
      // are not wrapping python getters, which they may return arbitrary types.
      auto py_value = py_getter.attr("__call__")(py::cast(obj));
      if (auto v = converter(py_value.cast<Property::Field>())) {
        return *v;
      }
      return default_value;
    };
  } else {
    p.getter = nullptr;
  }
  if (!py_setter.is_none()) {
    p.setter = [py_setter](HasProperties *obj, const Property::Field &value) {
      // CHANGED(28/5/2025): I don't need a layer of safety here because we
      // are already wrapping python setters as safe

      py_setter(py::cast(obj), py::cast(value));

      // // py::print("set value to", value, "default is", default_value);
      // if (auto v = convert_py(value, default_value)) {
      //   // py::print("converted to", *v);
      //   py_setter.attr("__call__")(py::cast(obj), py::cast(*v));
      // } else {
      //   // py::print("not compatible!");
      // }
    };
  } else {
    p.setter = nullptr;
  }
  if (py_schema) {
    p.schema = YAML::from_schema_py(*py_schema);
  } else {
    p.schema = nullptr;
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
    const Converter &converter, const std::string &description = "",
    const std::optional<py::function> &py_schema = std::nullopt,
    const std::vector<std::string> &deprecated_names = {}) {
  return make_property_py(py_property.attr("fget"), py_property.attr("fset"),
                          default_value, converter, description, py_schema,
                          deprecated_names);
}

inline Property make_property_py_with_type(
    const py::object &py_getter, const py::object &py_setter,
    const Property::Field &default_value, const std::string &type_name,
    const std::string &description = "",
    const std::optional<py::function> &py_schema = std::nullopt,
    const std::vector<std::string> &deprecated_names = {}) {
  const auto [scalar_type, is_vector] = get_scalar_type_name(type_name);
  const auto converter = get_converter(scalar_type, is_vector);
  if (!converter) {
    throw std::runtime_error("Invalid type name" + type_name);
  }
  const auto safe_default_value = (*converter)(default_value);
  if (!safe_default_value) {
    throw std::runtime_error("Invalid default value");
  }
  return make_property_py(py_getter, py_setter, *safe_default_value, *converter,
                          description, py_schema, deprecated_names);
}

inline Property make_property_with_py_property_with_type(
    const py::object &py_property, const Property::Field &default_value,
    const std::string &type_name, const std::string &description = "",
    const std::optional<py::function> &py_schema = std::nullopt,
    const std::vector<std::string> &deprecated_names = {}) {
  const auto [scalar_type, is_vector] = get_scalar_type_name(type_name);
  const auto converter = get_converter(scalar_type, is_vector);
  if (!converter) {
    throw std::runtime_error("Invalid type name" + type_name);
  }
  const auto safe_default_value = (*converter)(default_value);
  if (!safe_default_value) {
    throw std::runtime_error("Invalid default value");
  }
  return make_property_py(py_property.attr("fget"), py_property.attr("fset"),
                          *safe_default_value, *converter, description,
                          py_schema, deprecated_names);
}

inline py::cpp_function safe_setter(py::function setter,
                                    const Converter &converter) {
  return py::cpp_function(
      [setter, converter](py::object self, py::object value) {
        auto safe_value = converter(value.cast<Property::Field>());
        if (safe_value) {
          setter(self, py::cast(*safe_value));
        }
      });
}

#endif // NAVGROUND_CORE_PY_YAML_H