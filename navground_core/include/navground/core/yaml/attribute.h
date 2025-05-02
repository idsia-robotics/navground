#ifndef NAVGROUND_CORE_YAML_ATTRIBUTE_H
#define NAVGROUND_CORE_YAML_ATTRIBUTE_H

#include "navground/core/attribute.h"
#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/core/types.h"
#include "navground/core/yaml/schema.h"
#include "yaml-cpp/yaml.h"
#include <type_traits>

namespace YAML {

template <std::size_t I = 0>
std::optional<navground::core::Attribute>
decode_attribute_value(const Node &node, const std::string &type) {
  if constexpr (I < std::variant_size_v<navground::core::Attribute>) {
    using T = std::variant_alternative_t<I, navground::core::Attribute>;
    if (navground::core::Property::field_type_name<T>() == type) {
      return node.as<T>();
    }
    return decode_attribute_value<I + 1>(node, type);
  }
  return std::nullopt;
}

inline std::optional<navground::core::Attribute>
decode_attribute(const Node &node) {
  const auto type = node["type"].as<std::string>();
  return decode_attribute_value(node["value"], type);
}

template <typename T> Node attribute_schema() {
  Node node;
  node["type"] = "object";
  if constexpr (std::is_same_v<T, navground::core::Vector2>) {
    node["properties"]["value"] = schema::ref<T>();
  } else {
    node["properties"]["value"] = schema::type<T>();
  }
  node["properties"]["type"]["const"] =
      navground::core::Property::field_type_name<T>();
  node["unevaluatedProperties"] = false;
  node["required"].push_back("value");
  node["required"].push_back("type");
  return node;
}

template <> struct convert<navground::core::Attributes> {
  static Node encode(const navground::core::Attributes &rhs) {
    Node node;
    for (const auto &[k, v] : rhs) {
      node[k]["value"] = v;
      node[k]["type"] = navground::core::Property::friendly_type_name(v);
    }
    return node;
  }
  static bool decode(const Node &node, navground::core::Attributes &rhs) {
    if (!node.IsMap()) {
      return false;
    }
    for (const auto &item : node) {
      const auto value = decode_attribute(item.second);
      if (value) {
        rhs[item.first.as<std::string>()] = *value;
      }
    }
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    Node attribute_types;
    attribute_types["anyOf"].push_back(attribute_schema<int>());
    attribute_types["anyOf"].push_back(attribute_schema<ng_float_t>());
    attribute_types["anyOf"].push_back(attribute_schema<bool>());
    attribute_types["anyOf"].push_back(attribute_schema<std::string>());
    attribute_types["anyOf"].push_back(attribute_schema<Vector2>());
    attribute_types["anyOf"].push_back(attribute_schema<std::vector<int>>());
    attribute_types["anyOf"].push_back(
        attribute_schema<std::vector<ng_float_t>>());
    attribute_types["anyOf"].push_back(attribute_schema<std::vector<bool>>());
    attribute_types["anyOf"].push_back(
        attribute_schema<std::vector<std::string>>());
    attribute_types["anyOf"].push_back(
        attribute_schema<std::vector<Vector2>>());
    node["additionalProperties"] = attribute_types;
    return node;
  }
  static constexpr const char name[] = "attributes";
};

inline void encode_attributes(Node &node,
                              const navground::core::HasAttributes &rhs) {
  const auto attrs = rhs.get_attributes();
  if (attrs.size()) {
    node["attributes"] = attrs;
  }
}
inline bool decode_attributes(const Node &node,
                              navground::core::HasAttributes &rhs) {
  if (!node.IsMap()) {
    return false;
  }
  if (node["attributes"]) {
    const auto attributes =
        node["attributes"].as<navground::core::Attributes>();
    rhs.set_attributes(attributes);
  }
  return true;
}

} // namespace YAML

#endif // NAVGROUND_CORE_YAML_ATTRIBUTE_H