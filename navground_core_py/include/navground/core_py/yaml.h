#ifndef NAVGROUND_CORE_PY_YAML_H
#define NAVGROUND_CORE_PY_YAML_H

#include <pybind11/pybind11.h>

#include "navground/core/yaml/schema.h"
#include "yaml-cpp/yaml.h"

namespace py = pybind11;

namespace YAML {

inline const std::string load_string_py_doc(const std::string &instance,
                                            const std::string &type) {
  std::string doc =
      "Load a " + instance + " from a YAML string.\n\n";
  doc += ":param value: the YAML string.\n";
  doc += ":return: The loaded " + instance + " or ``None`` if loading fails.\n";
  doc += ":rtype: " + type + "| None";
  return doc;
}

inline const char *dump_doc() {
  return R"doc(
Dumps the object to a YAML-string.

:return: The YAML representation
:rtype: str
)doc";
}

inline const char *component_schema_py_doc() {
  return R"doc(
Returns the json-schema of a component

Returns an empty dictionary if a not registered type is requested.

:param reference_register_schema: Whether to reference registered components schema in the base class schema.
:type with_registers: bool
:param type: An optional registered type. If not specified, it returns the schema of the base class.
:type type: str
:return: A json-schema of the registered class
:rtype: :py:type:`dict[str, typing.Any]`
           )doc";
}

inline const char *register_schema_py_doc() {
  return R"doc(
Returns the json-schema that includes registered components.

:return: "anyOf" json-schema of all registered components.
:rtype: :py:type:`dict[str, typing.Any]`
           )doc";
}

inline const char *schema_py_doc() {
  return R"doc(
Returns the json-schema

:return: json-schema
:rtype: :py:type:`dict[str, typing.Any]`
)doc";
}

inline py::object to_py(const Node &node) {
  py::module_ yaml = py::module_::import("yaml");
  Emitter e;
  e << node;
  return yaml.attr("safe_load")(std::string(e.c_str()));
}

inline Node from_py(const py::object &obj) {
  py::module_ yaml = py::module_::import("yaml");
  std::string value = yaml.attr("safe_dump")(obj).cast<std::string>();
  return Load(value);
}

template <typename T> py::object schema_py() {
  const auto node = schema::schema<T>();
  return to_py(node);
}

template <typename T> py::object register_schema_py() {
  const auto node = schema::register_schema<T>();
  return to_py(node);
}

template <typename T>
py::object
component_schema_py(bool reference_register_schema = true,
                    const std::optional<std::string> &type = std::nullopt) {
  const auto node = schema::schema<T>(reference_register_schema, type);
  return to_py(node);
}

template <typename T> py::object make_type_from_yaml_py(const Node &node) {
  if (node.IsMap()) {
    std::string type = node["type"].as<std::string>("");
    auto v = T::make_type(type);
    return v;
  }
  return py::none();
}

template <typename T> py::object load_node_py(const Node &node) {
  auto obj = make_type_from_yaml_py<T>(node);
  if (!obj.is_none()) {
    convert<typename T::Native>::decode(node, obj.template cast<T &>());
  }
  return obj;
}

template <typename T> py::object load_string_py(const std::string &value) {
  try {
    Node node = Load(value);
    return load_node_py<T>(node);
  } catch (const ParserException &) {
    return py::none();
  }
}

template <typename T, typename S> py::object make_subtype_from_yaml_py(const Node &node) {
  if (node.IsMap()) {
    std::string type = node["type"].as<std::string>("");
    auto v = T::template make_subtype<S>(type);
    return v;
  }
  return py::none();
}

template <typename T, typename S> py::object load_node_py(const Node &node) {
  auto obj = make_subtype_from_yaml_py<T, S>(node);
  if (!obj.is_none()) {
    convert<typename T::Native>::decode(node, obj.template cast<T &>());
  }
  return obj;
}

template <typename T, typename S> py::object load_string_py(const std::string &value) {
  try {
    Node node = Load(value);
    return load_node_py<T, S>(node);
  } catch (const ParserException &) {
    return py::none();
  }
}


template <typename T>
py::object load_string_unique_py(const std::string &value) {
  try {
    Node node = Load(value);
    auto obj = std::make_unique<T>();
    convert<T>::decode(node, *obj);
    return py::cast(std::move(obj));
  } catch (const ParserException &) {
    return py::none();
  }
}

using Schema = std::function<void(YAML::Node &)>;

inline Schema from_schema_py(const py::function &schema_fn) {
  return [schema_fn](YAML::Node &node) {
    auto obj = YAML::to_py(node);
    schema_fn(obj);
    node = YAML::from_py(obj);
  };
}

} // namespace YAML

#endif // NAVGROUND_CORE_PY_YAML_H
