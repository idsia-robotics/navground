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
      "Load a " + instance + " modulation from a YAML string.\n\n";
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

inline const char *base_schema_py_doc() {
  return R"doc(
Returns the json-schema of the base class

:param with_registers: Whether to reference registered components schema
:type with_registers: bool
:return: The json-schema of the base class
:rtype: ``typing.Dict[str, Any]``
           )doc";
}

inline const char *schema_of_type_py_doc() {
  return R"doc(
Returns the json-schema of a registered component

Returns an empty dictionary if the type is not registered.

:param type: The name of the component
:type type: str
:return: A json-schema of the registered class
:rtype: ``typing.Dict[str, Any]``
           )doc";
}

inline const char *register_schema_py_doc() {
  return R"doc(
Returns the json-schema that includes registered components

:return: "anyOf" json-schema of all registered components.
:rtype: ``typing.Dict[str, Any]``
           )doc";
}

inline const char *schema_py_doc() {
  return R"doc(
Returns the json-schema

return: json-schema
:rtype: ``typing.Dict[str, Any]
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
  const auto node = schema::registered<T>();
  return to_py(node);
}

template <typename T>
py::object base_schema_py(bool reference_register = true) {
  const auto node = schema::base<T>(reference_register);
  return to_py(node);
}

template <typename T>
py::object schema_of_type_py(const std::string & type) {
  const auto node = schema::schema_of_type<T>(type);
  if (node.IsNull()) {
    return py::dict();
  }
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
  } catch (const ParserException &ex) {
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
  } catch (const ParserException &ex) {
    return py::none();
  }
}

} // namespace YAML

#endif // NAVGROUND_CORE_PY_YAML_H
