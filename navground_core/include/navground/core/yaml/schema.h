#ifndef NAVGROUND_CORE_YAML_SCHEMA_H
#define NAVGROUND_CORE_YAML_SCHEMA_H

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>

namespace YAML {

namespace schema {

inline const std::string BASE_URL = "http://navground/";
inline const std::string SCHEMA_PREFIX = "";
inline const std::string SCHEMA =
    "https://json-schema.org/draft/2020-12/schema";

inline std::string ref_name(const std::string &name) {
  return SCHEMA_PREFIX + name;
}

inline std::string id_name(const std::string &name) {
  return BASE_URL + SCHEMA_PREFIX + name;
}

template <typename T> struct type_t {
  static std::string name() { return std::string(convert<T>::name); }
  static Node schema() { return convert<T>::schema(); }
};

template <typename T> Node type() { return type_t<T>::schema(); }

template <> struct type_t<bool> {
  static std::string name() { return "boolean"; }
  static Node schema() {
    Node node;
    node["type"] = "boolean";
    return node;
  }
};

template <> struct type_t<int> {
  static std::string name() { return "integer"; }
  static Node schema() {
    Node node;
    node["type"] = "integer";
    return node;
  }
};

template <> struct type_t<ng_float_t> {
  static std::string name() { return "number"; }
  static Node schema() {
    Node node;
    node["type"] = "number";
    return node;
  }
};

template <> struct type_t<std::string> {
  static std::string name() { return "string"; }
  static Node schema() {
    Node node;
    node["type"] = "string";
    return node;
  }
};

template <> struct type_t<navground::core::Vector2> {
  static std::string name() { return "vector2"; }
  static Node schema() {
    Node node;
    node["type"] = "array";
    node["items"] = type<ng_float_t>();
    node["minItems"] = 2;
    node["maxItems"] = 2;
    return node;
  }
};

template <typename T> struct type_t<std::vector<T>> {
  static std::string name() { return type_t<T>::name() + "_array"; }
  static Node schema() {
    Node node;
    node["type"] = "array";
    node["items"] = type<T>();
    return node;
  }
};

inline Node schema(const std::string &name) {
  Node node;
  node["$id"] = id_name(name);
  node["$schema"] = SCHEMA;
  return node;
}

/**
 * @brief      Returns the json-schema for a type
 *
 * @tparam     T  The type
 *
 * @return     A json-schema encoded as a \ref YAML::Node.
 */
template <typename T> Node schema() {
  // Node node = schema(type_t<T>::name());
  // for (const auto &[k, v] : type_t<T>::schema()) {
  //   node[k] = v;
  // }
  Node node = type_t<T>::schema();
  node["$id"] = id_name(type_t<T>::name());
  node["$schema"] = SCHEMA;
  return node;
}

template <typename T> Node ref() {
  Node node;
  node["$ref"] = ref_name(type_t<T>::name());
  return node;
}

inline Node property_schema(const navground::core::Property &property) {
  Node node = std::visit(
      [](auto &&arg) -> Node {
        using T = std::decay_t<decltype(arg)>;
        return type<T>();
      },
      property.default_value);
  if (property.readonly) {
    node["readOnly"] = true;
  } else {
    node["default"] = property.default_value;
  }
  node["description"] = property.description;
  return node;
}

inline Node
registered_component_schema(const std::string &type,
                            const navground::core::Properties &properties) {
  Node node;
  node["properties"]["type"]["const"] = type;
  for (const auto &[name, property] : properties) {
    node["properties"][name] = property_schema(property);
  }
  return node;
}

/**
 * @brief     Returns the json-schema that includes registered components
 *
 * @tparam     T  The component type (should be a sub-class of \ref
 * navground::core::HasRegister<T>)
 *
 * @return    "anyOf" json-schema of all registered components encoded as a
 * \ref YAML::Node.
 */
template <typename T> Node registered() {
  Node node;
  node["$schema"] = SCHEMA;
  node["$id"] = id_name(type_t<T>::name() + "_register");
  const auto &t_schema = T::type_schema();
  for (const auto &[name, properties] : T::type_properties()) {
    auto tnode = registered_component_schema(name, properties);
    if (t_schema.count(name)) {
      t_schema.at(name)(tnode);
    }
    node["anyOf"].push_back(tnode);
  }
  return node;
}

/**
 * @brief     Returns the json-schema of a component base-class
 *
 * @param[in]  reference_register  Whether to reference registered components
 * schema
 *
 * @tparam     T  The component type (should be a sub-class of \ref
 * navground::core::HasRegister<T>)
 *
 * @return    A json-schema encoded as a \ref YAML::Node.
 */
template <typename T> Node base(bool reference_register) {
  Node node = schema<T>();
  if (reference_register) {
    node["$ref"] = ref_name(type_t<T>::name() + "_register");
    node["unevaluatedProperties"] = false;
  } else {
    node["unevaluatedProperties"] = true;
  }
  return node;
}

template <typename T> Node base_with_ref() { return base<T>(true); }

} // namespace schema

} // namespace YAML

#endif // NAVGROUND_CORE_YAML_SCHEMA_H