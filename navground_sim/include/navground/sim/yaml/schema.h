#ifndef NAVGROUND_SIM_YAML_SCHEMA_H
#define NAVGROUND_SIM_YAML_SCHEMA_H

#include "navground/core/schema.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/sampling/sampler.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

namespace schema {

using namespace navground::sim;

/** @private */
inline Node property_sampler_schema(const std::string &name,
                                    const navground::core::Property &property) {
  Node node = property_schema(property);
  return std::visit(
      [&node, &name](auto &&arg) -> Node {
        using T = std::decay_t<decltype(arg)>;
        return sampler<T>(node, name);
      },
      property.default_value);
}

/** @private */
inline Node registered_component_sampler_schema(
    const std::string &type, const navground::core::Properties &properties) {
  Node node;
  node["properties"]["type"]["const"] = type;
  for (const auto &[name, property] : properties) {
    node["properties"][name] = property_sampler_schema(name, property);
  }
  return node;
}

/**
 * @private
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
template <typename T> Node register_sampler_schema() {
  Node node;
  node["$schema"] = "https://json-schema.org/draft/2020-12/schema";
  node["$id"] = id_name(type_t<T>::name() + "_register");
  const auto &t_schema = T::Type::type_schema();
  for (const auto &[name, properties] : T::Type::type_properties()) {
    auto tnode = registered_component_sampler_schema(name, properties);
    if (t_schema.count(name)) {
      t_schema.at(name)(tnode);
    }
    node["anyOf"].push_back(tnode);
  }
  return node;
}

/** @private */
template <typename T> Node sampler_schema_of_type(const std::string &type) {
  const auto &ps = T::Type::type_properties();
  if (!ps.count(type)) {
    return Node();
  }
  Node node = schema<T>();

  for (const auto &[name, property] : ps.at(type)) {
    node["properties"][name] = property_sampler_schema(name, property);
  }
  const auto &t_schema = T::Type::type_schema();
  if (t_schema.count(type)) {
    t_schema.at(type)(node);
  }
  node["unevaluatedProperties"] = false;
  return node;
}

/**
 * @private
 * @brief     Returns the json-schema of a sampler of a component
 *
 * @param[in]  reference_register_schema  Whether to reference registered
 * components schema in the base class schema.
 * @param[in]  type  An optional registered type. If not specified, it returns
 * the schema of the base class.
 *
 * @tparam     T  The component type (should be a sub-class of \ref
 * navground::core::HasRegister<T>)
 *
 * @return    A json-schema encoded as a \ref YAML::Node.
 */
template <typename T>
Node sampler_schema(bool reference_register_schema,
                    const std::optional<std::string> &type = std::nullopt) {
  if (type) {
    return sampler_schema_of_type<T>(*type);
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

/** @private */
template <> Node register_schema<Scenario>() {
  return register_sampler_schema<Scenario>();
}

/** @private */
template <>
Node schema<Scenario>(bool reference_register_schema,
                      const std::optional<std::string> &type) {
  return sampler_schema<Scenario>(reference_register_schema, type);
}

} // namespace schema

} // namespace YAML

#endif // NAVGROUND_CORE_YAML_SCHEMA_H
