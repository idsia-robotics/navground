#ifndef NAVGROUND_CORE_YAML_SCHEMA_CORE_H
#define NAVGROUND_CORE_YAML_SCHEMA_CORE_H

#include "navground/core/yaml/core.h"
#include "navground/core/yaml/schema.h"
#include "yaml-cpp/yaml.h"

namespace YAML {

namespace schema {

/**
 * @brief      
 *
 * @tparam     T  The type
 *
 * @return     A json-schema encoded as a \ref YAML::Node.
 */

/**
 * @brief      Returns the bundle json-schema for \ref navground::core
 *
 * @return     A json-schema encoded as a \ref YAML::Node.
 */
inline Node core() {
  using namespace navground::core;

  Node node = schema("core");
  node["$defs"]["vector2"] = schema<Vector2>();
  node["$defs"]["line_segment"] = schema<LineSegment>();
  node["$defs"]["disc"] = schema<Disc>();
  node["$defs"]["neighbor"] = schema<Neighbor>();
  node["$defs"]["behavior"] = base<Behavior>(true);
  node["$defs"]["behavior_register"] = registered<Behavior>();
  node["$defs"]["behavior_modulation"] = base<BehaviorModulation>(true);
  node["$defs"]["behavior_modulation_register"] =
      registered<BehaviorModulation>();
  node["$defs"]["kinematics"] = base<Kinematics>(true);
  node["$defs"]["kinematics_register"] = registered<Kinematics>();
  return node;
}

} // namespace schema

} // namespace YAML

#endif // NAVGROUND_CORE_YAML_SCHEMA_CORE_H
