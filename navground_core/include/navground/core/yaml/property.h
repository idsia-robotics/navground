#ifndef NAVGROUND_CORE_YAML_PROPERTY_H
#define NAVGROUND_CORE_YAML_PROPERTY_H

#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "yaml-cpp/yaml.h"

using navground::core::Property;
using navground::core::Vector2;

namespace YAML {

template <>
struct convert<Vector2> {
  static Node encode(const Vector2& rhs) {
    Node node;
    node.push_back(rhs[0]);
    node.push_back(rhs[1]);
    return node;
  }
  static bool decode(const Node& node, Vector2& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    // std::cout << "convert to Vector2 from " << node << std::endl;
    rhs[0] = node[0].as<float>();
    rhs[1] = node[1].as<float>();
    return true;
  }
};

// Note(Jerome): I don't know why I have to [re]define
// `convert<std::vector<bool>>`, as it is already defined in yaml-cpp; but
// without this definition the compiler complains.

template <>
struct convert<std::vector<bool>> {
  static Node encode(const std::vector<bool>& rhs) {
    Node node;
    for (const bool& item : rhs) {
      node.push_back(item);
    }
    return node;
  }
  static bool decode(const Node& node, std::vector<bool>& rhs) {
    if (node.Type() != YAML::NodeType::Sequence) return false;
    for (const auto& c : node) {
      rhs.push_back(c.as<bool>());
    }
    return true;
  }
};

inline Property::Field decode_property(const Property& property, const Node& node) {
  return std::visit(
      [&node](auto&& arg) -> Property::Field {
        using T = std::decay_t<decltype(arg)>;
        return node.as<T>();
      },
      property.default_value);
}

inline Node encode_property(const Property::Field& value) {
  return std::visit([](auto&& arg) { return Node(arg); }, value);
}

template <>
struct convert<Property::Field> {
  static Node encode(const Property::Field& rhs) {
    return encode_property(rhs);
  }
};

}  // namespace YAML

#endif  // NAVGROUND_CORE_YAML_PROPERTY_H