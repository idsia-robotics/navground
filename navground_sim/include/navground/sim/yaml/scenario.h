#ifndef NAVGROUND_SIM_YAML_SCENARIO_H
#define NAVGROUND_SIM_YAML_SCENARIO_H

#include <iostream>
#include <memory>

#include "navground/sim/sampling/agent.h"
#include "navground/sim/scenario.h"
#include "navground/sim/yaml/sampling.h"
#include "navground/sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

using navground::sim::AgentSampler;
using navground::sim::Scenario;
using navground::sim::World;

// #define _UNSAFE_GROUP_CASTING

namespace YAML {

template <typename W = World>
struct convert_scenario {
  using AS = AgentSampler<W>;

  static Node encode(const Scenario& rhs) {
    Node node;
    encode_type_and_properties<Scenario>(node, rhs);
    node["obstacles"] = rhs.obstacles;
    node["walls"] = rhs.walls;
    for (const auto& group : rhs.groups) {
#ifndef _UNSAFE_GROUP_CASTING
      if (AS* g = dynamic_cast<AS*>(group.get()))
#else
      AS* g = static_cast<AS*>(group.get());
#endif  // _UNSAFE_GROUP_CASTING
      {
        node["groups"].push_back(*g);
      }
    }
    return node;
  }
  static bool decode(const Node& node, Scenario& rhs) {
    decode_properties(node, rhs);
    if (node["groups"]) {
      if (node["groups"].IsSequence()) {
        for (const auto& c : node["groups"]) {
          auto group = std::make_unique<AS>();
          convert<AS>::decode(c, *group);
          rhs.groups.push_back(std::move(group));
        }
      }
    }
    if (node["obstacles"]) {
      for (const auto& c : node["obstacles"]) {
        rhs.obstacles.push_back(c.as<Disc>());
      }
    }
    if (node["walls"]) {
      for (const auto& c : node["walls"]) {
        rhs.walls.push_back(c.as<Wall>());
      }
    }
    return true;
  }
};

template <>
struct convert<Scenario> {
  static Node encode(const Scenario& rhs) {
    return convert_scenario<>::encode(rhs);
  }
  static bool decode(const Node& node, Scenario& rhs) {
    return convert_scenario<>::decode(node, rhs);
  }
};

template <>
struct convert<std::shared_ptr<Scenario>> {
  static Node encode(const std::shared_ptr<Scenario>& rhs) {
    if (rhs) {
      return convert_scenario<>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node& node, std::shared_ptr<Scenario>& rhs) {
    rhs = make_type_from_yaml<Scenario>(node);
    if (!rhs) {
      rhs = std::make_shared<Scenario>();
    }
    convert_scenario<>::decode(node, *rhs);
    return true;
  }
};

}  // namespace YAML

#endif  // NAVGROUND_SIM_YAML_SCENARIO_H
