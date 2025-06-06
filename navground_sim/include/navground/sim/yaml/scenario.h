#ifndef NAVGROUND_SIM_YAML_SCENARIO_H
#define NAVGROUND_SIM_YAML_SCENARIO_H

#include <iostream>
#include <memory>

#include "navground/core/yaml/schema.h"
#include "navground/sim/sampling/agent.h"
#include "navground/sim/scenario.h"
#include "navground/sim/yaml/sampling.h"
#include "navground/sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

using navground::sim::AgentSampler;
using navground::sim::BoundingBox;
using navground::sim::Scenario;
using navground::sim::World;

// #define _UNSAFE_GROUP_CASTING

namespace YAML {

template <typename W = World> struct convert_scenario {
  using AS = AgentSampler<W>;

  static Node encode(const Scenario &rhs) {
    Node node;
    encode_type_and_properties<Scenario>(node, rhs);
    for (const auto &[name, sampler] : rhs.get_property_samplers()) {
      if (sampler) {
        node[name] = sampler;
      }
    }
    if (rhs.bounding_box) {
      node["bounding_box"] = *(rhs.bounding_box);
    }
    node["obstacles"] = rhs.obstacles;
    for (const auto &line : rhs.walls) {
      Node wn;
      wn["line"] = line;
      node["walls"].push_back(wn);
    }
    for (const auto &group : rhs.get_groups()) {
#ifndef _UNSAFE_GROUP_CASTING
      if (AS *g = dynamic_cast<AS *>(group.get()))
#else
      AS *g = static_cast<AS *>(group.get());
#endif // _UNSAFE_GROUP_CASTING
      {
        node["groups"].push_back(*g);
      }
    }
    return node;
  }
  static bool decode(const Node &node, Scenario &rhs) {
    decode_properties(node, rhs);
    if (node["groups"]) {
      if (node["groups"].IsSequence()) {
        for (const auto &c : node["groups"]) {
          auto group = std::make_unique<AS>();
          convert<AS>::decode(c, *group);
          rhs.add_group(std::move(group));
        }
      }
    }
    if (node["obstacles"]) {
      for (const auto &c : node["obstacles"]) {
        rhs.obstacles.push_back(c.as<Disc>());
      }
    }
    if (node["walls"]) {
      for (const auto &c : node["walls"]) {
        rhs.walls.push_back(c.as<Wall>());
      }
    }
    if (node["bounding_box"]) {
      rhs.bounding_box = node["bounding_box"].as<BoundingBox>();
    }
    return true;
  }
};

template <> void decode_properties(const Node &node, Scenario &obj) {
  for (const auto &[name, property] : obj.get_properties()) {
    if (node[name]) {
      try {
        obj.set(name, decode_property(property, node[name]));
      } catch (const std::runtime_error &) {
        obj.add_property_sampler(name, property_sampler(node[name], property));
      }
    } else {
      for (const auto &alt_name : property.deprecated_names) {
        if (node[alt_name]) {
          try {
            obj.set(name, decode_property(property, node[alt_name]));
          } catch (const std::runtime_error &) {
            obj.add_property_sampler(
                name, property_sampler(node[alt_name], property));
          }
        }
      }
    }
  }
  obj.decode(node);
}

template <> struct convert<Scenario> {
  static Node encode(const Scenario &rhs) {
    return convert_scenario<>::encode(rhs);
  }
  static bool decode(const Node &node, Scenario &rhs) {
    return convert_scenario<>::decode(node, rhs);
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["bounding_box"] = schema::ref<BoundingBox>();
    node["properties"]["obstacles"]["type"] = "array";
    node["properties"]["obstacles"]["items"] = schema::ref<Obstacle>();
    node["properties"]["walls"]["type"] = "array";
    node["properties"]["walls"]["items"] = schema::ref<Wall>();
    node["properties"]["groups"]["type"] = "array";
    node["properties"]["groups"]["items"] = schema::ref<AgentSampler<World>>();
    return node;
  }
  static constexpr const char name[] = "scenario";
};

template <> struct convert<std::shared_ptr<Scenario>> {
  static Node encode(const std::shared_ptr<Scenario> &rhs) {
    if (rhs) {
      return convert_scenario<>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node &node, std::shared_ptr<Scenario> &rhs) {
    rhs = make_type_from_yaml<Scenario>(node);
    if (!rhs) {
      rhs = std::make_shared<Scenario>();
    }
    convert_scenario<>::decode(node, *rhs);
    return true;
  }
};

} // namespace YAML

#endif // NAVGROUND_SIM_YAML_SCENARIO_H
