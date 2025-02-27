#ifndef NAVGROUND_CORE_YAML_SCHEMA_H
#define NAVGROUND_CORE_YAML_SCHEMA_H

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/core/yaml/property.h"
#include "yaml-cpp/yaml.h"
#include <string>
#include <vector>

namespace YAML {

namespace schema {

using Modifier = std::function<void(YAML::Node &)>;

/**
 * @brief      Set enum options
 *
 * @param      node    The node
 * @param      values  The possible values
 */
template<typename T>
inline void set_enum(Node &node, const std::vector<T> & values) { 
  node["enum"] = values;
}

/**
 * @brief      Constrains to >= 0
 *
 * @param      node  The node
 */
inline void positive(Node &node) { node["minimum"] = 0; }

/**
 * @brief      Constrains to > 0
 *
 * @param      node  The node
 */
inline void strict_positive(Node &node) { node["exclusiveMinimum"] = 0; }

/**
 * @brief      Constrains to contain at least one item.
 *
 * @param      node  The node
 */
inline void not_empty(Node &node) { node["minItems"] = 1; }

inline const std::string BASE_URL = "http://navground/";
inline const std::string SCHEMA_PREFIX = "";
inline const std::string SCHEMA =
    "https://json-schema.org/draft/2020-12/schema";

/** @private */
inline std::string ref_name(const std::string &name) {
  return SCHEMA_PREFIX + name;
}

/** @private */
inline std::string id_name(const std::string &name) {
  return BASE_URL + SCHEMA_PREFIX + name;
}

/** @private */
inline Node schema_prefix(const std::string &name) {
  Node node;
  node["$id"] = id_name(name);
  node["$schema"] = SCHEMA;
  return node;
}

/** @private */
template <typename T> struct type_t {
  static std::string name() { return std::string(convert<T>::name); }
  static Node schema() { return convert<T>::schema(); }
};

/**
 * @brief      Reference to a C++ type
 *
 * @tparam     T     The type
 *
 * @return     A json-schema encoded as a \ref YAML::Node.
 */
template <typename T> Node ref() {
  Node node;
  node["$ref"] = ref_name(type_t<T>::name());
  return node;
}

/** @private */
template <typename T> Node type() { return type_t<T>::schema(); }

/** @private */
template <> struct type_t<bool> {
  static std::string name() { return "boolean"; }
  static Node schema() {
    Node node;
    node["type"] = "boolean";
    return node;
  }
};

/** @private */
template <> struct type_t<int> {
  static std::string name() { return "integer"; }
  static Node schema() {
    Node node;
    node["type"] = "integer";
    return node;
  }
};

/** @private */
template <> struct type_t<unsigned> {
  static std::string name() { return "positive_integer"; }
  static Node schema() {
    Node node;
    node["type"] = "integer";
    node["minimum"] = 0;
    return node;
  }
};

/** @private */
template <> struct type_t<ng_float_t> {
  static std::string name() { return "number"; }
  static Node schema() {
    Node node;
    node["type"] = "number";
    return node;
  }
};

/**
 * @brief A positive float
 */
struct positive_float;

/** @private */
template <> struct type_t<positive_float> {
  static std::string name() { return "positive_number"; }
  static Node schema() {
    Node node;
    node["type"] = "number";
    node["minimum"] = 0;
    return node;
  }
};

/** @private */
template <> struct type_t<std::string> {
  static std::string name() { return "string"; }
  static Node schema() {
    Node node;
    node["type"] = "string";
    return node;
  }
};

/** @private */
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

/** @private */
template <typename T> struct type_t<std::vector<T>> {
  static std::string name() { return type_t<T>::name() + "_array"; }
  static Node schema() {
    Node node;
    node["type"] = "array";
    node["items"] = type<T>();
    return node;
  }
};

/** @private */
template <> struct type_t<std::vector<Vector2>> {
  static std::string name() { return type_t<Vector2>::name() + "_array"; }
  static Node schema() {
    Node node;
    node["type"] = "array";
    node["items"] = ref<Vector2>();
    return node;
  }
};

/**
 * @brief      Returns the json-schema for a C++ type
 *
 * @tparam     T  The type
 *
 * @return     A json-schema encoded as a \ref YAML::Node.
 */
template <typename T> Node schema() {
  Node node = type_t<T>::schema();
  node["$id"] = id_name(type_t<T>::name());
  node["$schema"] = SCHEMA;
  return node;
}

/** @private */
inline Node property_schema(const navground::core::Property &property) {
  Node node = std::visit(
      [](auto &&arg) -> Node {
        using T = std::decay_t<decltype(arg)>;
        if constexpr (std::is_same_v<T, Vector2>) {
          return ref<Vector2>();
        }
        return type<T>();
      },
      property.default_value);
  if (property.readonly) {
    node["readOnly"] = true;
  } else {
    node["default"] = property.default_value;
  }
  node["description"] = property.description;
  if (property.schema) {
    property.schema(node);
  }
  return node;
}

/** @private */
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
template <typename T> Node register_schema() {
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

/** @private */
template <typename T> Node schema_of_type(const std::string &type) {
  const auto &ps = T::type_properties();
  if (!ps.count(type)) {
    return Node();
  }
  Node node = schema<T>();
  for (const auto &[name, property] : ps.at(type)) {
    node["properties"][name] = property_schema(property);
  }
  const auto &t_schema = T::type_schema();
  if (t_schema.count(type)) {
    t_schema.at(type)(node);
  }
  node["unevaluatedProperties"] = false;
  return node;
}

/**
 * @brief     Returns the json-schema of a component
 *
 * @param[in]  reference_register_schema  Whether to reference registered
 * components schema in the base class schema.
 * @param[in]  type  An optional registered type. If not specified, it returns the schema of the base class.
 *
 * @tparam     T  The component type (should be a sub-class of \ref
 * navground::core::HasRegister<T>)
 *
 * @return    A json-schema encoded as a \ref YAML::Node.
 */
template <typename T>
Node schema(bool reference_register_schema,
            const std::optional<std::string> &type = std::nullopt) {
  if (type) {
    return schema_of_type<T>(*type);
  }
  Node node = schema<T>();
  if (reference_register_schema) {
    node["$ref"] = ref_name(type_t<T>::name() + "_register");
    node["unevaluatedProperties"] = false;
  } else {
    node["unevaluatedProperties"] = true;
  }
  return node;
}

} // namespace schema

} // namespace YAML

namespace std {

template <> struct is_floating_point<YAML::schema::positive_float> {
  static constexpr bool value = true;
};
} // namespace std

#endif // NAVGROUND_CORE_YAML_SCHEMA_H
