#ifndef NAVGROUND_SIM_YAML_SCHEMA_H
#define NAVGROUND_SIM_YAML_SCHEMA_H

#include "navground/core/schema.h"
#include "navground/sim/sampling/sampler.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

namespace schema {

using namespace navground::sim;

inline Node property_sampler_schema(const navground::core::Property &property) {
  return std::visit(
      [](auto &&arg) -> Node {
        using T = std::decay_t<decltype(arg)>;
        return ref<Sampler<T>>();
      },
      property.default_value);
}

inline Node registered_component_sampler_schema(
    const std::string &type, const navground::core::Properties &properties) {
  Node node;
  node["properties"]["type"]["const"] = type;
  for (const auto &[name, property] : properties) {
    node["properties"][name] = property_sampler_schema(property);
  }
  return node;
}

/**
 * @brief     Returns the json-schema that includes samplers for registered
 * components
 *
 * @tparam     T  The component type (should be a sub-class of \ref
 * navground::core::HasRegister<T>)
 *
 * @return    "anyOf" json-schema of all registered components samplers encoded
 * as a
 * \ref YAML::Node.
 */
template <typename T> Node registered_sampler() {
  Node node;
  node["$schema"] = "https://json-schema.org/draft/2020-12/schema";
  node["$id"] = id_name(type_t<T>::name() + "_register");
  for (const auto &[name, properties] : T::Type::type_properties()) {
    auto tnode = registered_component_sampler_schema(name, properties);
    node["anyOf"].push_back(tnode);
  }
  return node;
}

} // namespace schema

} // namespace YAML

#endif // NAVGROUND_CORE_YAML_SCHEMA_H
