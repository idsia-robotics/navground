#ifndef NAVGROUND_CORE_YAML_REGISTER_H
#define NAVGROUND_CORE_YAML_REGISTER_H

#include <memory>

#include "navground/core/register.h"
#include "navground/core/yaml/property.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

template <typename T>
void encode_type_and_properties(Node& node, const T& obj) {
  node["type"] = obj.get_type();
  for (const auto& [name, _] : obj.get_properties()) {
    node[name] = encode_property(obj.get(name));
  }
  obj.encode(node);
}

template <typename T>
void decode_properties(const Node& node, T& obj) {
  for (const auto& [name, property] : obj.get_properties()) {
    if (node[name]) {
      obj.set(name, decode_property(property, node[name]));
    } else {
      for (const auto& alt_name : property.deprecated_names) {
        if (node[alt_name]) {
          std::cerr << "Property name " << alt_name << " is deprecated for "
                    << obj.get_type() << ", use " << name << " instead"
                    << std::endl;
          obj.set(name, decode_property(property, node[alt_name]));
        }
      }
    }
  }
  obj.decode(node);
}

template <typename T>
std::shared_ptr<T> make_type_from_yaml(const Node& node) {
  if (node.IsMap() && node["type"]) {
    std::string type = node["type"].as<std::string>();
    auto obj = T::make_type(type);
    if (!obj) return nullptr;
    decode_properties(node, *obj);
    return obj;
  }
  return nullptr;
}

}  // namespace YAML

#endif  // NAVGROUND_CORE_YAML_REGISTER_H