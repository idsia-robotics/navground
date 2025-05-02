#ifndef NAVGROUND_CORE_YAML_SCHEMA_CORE_H
#define NAVGROUND_CORE_YAML_SCHEMA_CORE_H

#include "navground/core/yaml/attribute.h"
#include "navground/core/yaml/core.h"
#include "navground/core/yaml/schema.h"
#include "yaml-cpp/yaml.h"

namespace navground::core {
/**
 * @brief      Returns the bundle json-schema for \ref navground::core
 *
 * @return     A json-schema encoded as a \ref YAML::Node.
 */
inline YAML::Node bundle_schema() {
  using namespace YAML::schema;

  auto node = schema_prefix("core");
  node["$defs"]["vector2"] = schema<Vector2>();
  node["$defs"]["line_segment"] = schema<LineSegment>();
  node["$defs"]["disc"] = schema<Disc>();
  node["$defs"]["neighbor"] = schema<Neighbor>();
  node["$defs"]["behavior"] = schema<Behavior>(true);
  node["$defs"]["behavior_register"] = register_schema<Behavior>();
  node["$defs"]["behavior_modulation"] = schema<BehaviorModulation>(true);
  node["$defs"]["behavior_modulation_register"] =
      register_schema<BehaviorModulation>();
  node["$defs"]["kinematics"] = schema<Kinematics>(true);
  node["$defs"]["kinematics_register"] = register_schema<Kinematics>();
  node["$defs"]["attributes"] = schema<Attributes>();
  return node;
}

} // namespace navground::core

#endif // NAVGROUND_CORE_YAML_SCHEMA_CORE_H
