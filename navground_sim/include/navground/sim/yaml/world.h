#ifndef NAVGROUND_SIM_YAML_WORLD_H
#define NAVGROUND_SIM_YAML_WORLD_H

#include <memory>

#include "navground/core/yaml/core.h"
#include "navground/core/yaml/property.h"
#include "navground/core/yaml/register.h"
#include "navground/sim/world.h"
#include "yaml-cpp/yaml.h"

using navground::core::Behavior;
using navground::core::Disc;
using navground::core::Kinematics;
using navground::core::LineSegment;
using navground::core::Vector2;
using navground::sim::Agent;
using navground::sim::StateEstimation;
using navground::sim::Task;
using navground::sim::World;
using navground::sim::Obstacle;
using navground::sim::Wall;

namespace YAML {

template <>
struct convert<Obstacle> {
  static Node encode(const Obstacle& rhs) {
    Node node = convert<Disc>::encode(rhs.disc);
    node["uid"] = rhs.uid;
    return node;
  }
  static bool decode(const Node& node, Obstacle& rhs) {
    return convert<Disc>::decode(node, rhs.disc);
  }
};

template <>
struct convert<Wall> {
  static Node encode(const Wall& rhs) {
    Node node = convert<LineSegment>::encode(rhs.line);
    node["uid"] = rhs.uid;
    return node;
  }
  static bool decode(const Node& node, Wall& rhs) {
    return convert<LineSegment>::decode(node, rhs.line);
  }
};

template <>
struct convert<Task> {
  static Node encode(const Task& rhs) {
    Node node;
    encode_type_and_properties<Task>(node, rhs);
    return node;
  }
  static bool decode(const Node& node, Task& rhs) {
    decode_properties(node, rhs);
    return true;
  }
};

template <>
struct convert<std::shared_ptr<Task>> {
  static Node encode(const std::shared_ptr<Task>& rhs) {
    return convert<Task>::encode(*rhs);
  }
  static bool decode(const Node& node, std::shared_ptr<Task>& rhs) {
    rhs = make_type_from_yaml<Task>(node);
    if (rhs) {
      convert<Task>::decode(node, *rhs);
    }
    return true;
  }
};

template <>
struct convert<StateEstimation> {
  static Node encode(const StateEstimation& rhs) {
    Node node;
    encode_type_and_properties<StateEstimation>(node, rhs);
    return node;
  }
  static bool decode(const Node& node, StateEstimation& rhs) {
    decode_properties(node, rhs);
    return true;
  }
};

template <>
struct convert<std::shared_ptr<StateEstimation>> {
  static Node encode(const std::shared_ptr<StateEstimation>& rhs) {
    return convert<StateEstimation>::encode(*rhs);
  }
  static bool decode(const Node& node, std::shared_ptr<StateEstimation>& rhs) {
    rhs = make_type_from_yaml<StateEstimation>(node);
    if (rhs) {
      convert<StateEstimation>::decode(node, *rhs);
    }
    return true;
  }
};

template <>
struct convert<Agent> {
  static Node encode(const Agent& rhs) {
    Node node;
    if (const auto b = rhs.get_behavior()) {
      node["behavior"] = *b;
    }
    if (const auto k = rhs.get_kinematics()) {
      node["kinematics"] = *k;
    }
    if (const auto t = rhs.get_task()) {
      node["task"] = *t;
    }
    if (const auto s = rhs.get_state_estimation()) {
      node["state_estimation"] = *s;
    }
    node["position"] = rhs.pose.position;
    node["orientation"] = rhs.pose.orientation;
    node["velocity"] = rhs.twist.velocity;
    node["angular_speed"] = rhs.twist.angular_speed;
    node["radius"] = rhs.radius;
    node["control_period"] = rhs.control_period;
    node["type"] = rhs.type;
    node["id"] = rhs.id;
    node["uid"] = rhs.uid;
    if (rhs.tags.size()) {
      for(const auto & tag : rhs.tags) {
        node["tags"].push_back(tag);
      }
    }
    return node;
  }
  static bool decode(const Node& node, Agent& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["behavior"]) {
      rhs.set_behavior(node["behavior"].as<std::shared_ptr<Behavior>>());
    }
    if (node["kinematics"]) {
      rhs.set_kinematics(node["kinematics"].as<std::shared_ptr<Kinematics>>());
    }
    if (node["task"]) {
      rhs.set_task(node["task"].as<std::shared_ptr<Task>>());
    }
    if (node["state_estimation"]) {
      rhs.set_state_estimation(node["state_estimation"].as<std::shared_ptr<StateEstimation>>());
    }
    if (node["position"]) {
      rhs.pose.position = node["position"].as<Vector2>();
    }
    if (node["orientation"]) {
      rhs.pose.orientation = node["orientation"].as<float>();
    }
    if (node["velocity"]) {
      rhs.twist.velocity = node["velocity"].as<Vector2>();
    }
    if (node["angular_speed"]) {
      rhs.twist.angular_speed = node["angular_speed"].as<float>();
    }
    if (node["radius"]) {
      rhs.radius = node["radius"].as<float>();
    }
    if (node["control_period"]) {
      rhs.control_period = node["control_period"].as<float>();
    }
    if (node["type"]) {
      rhs.type = node["type"].as<std::string>();
    }
    if (node["id"]) {
      rhs.id = node["id"].as<unsigned>();
    }
    if (node["tags"]) {
      const auto vs = node["tags"].as<std::vector<std::string>>();
      rhs.tags = std::set<std::string>(vs.begin(), vs.end());
    }
    return true;
  }
};

template <>
struct convert<std::shared_ptr<Agent>> {
  static Node encode(const std::shared_ptr<Agent>& rhs) {
    if (rhs) {
      return convert<Agent>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node& node, std::shared_ptr<Agent>& rhs) {
    rhs = std::make_shared<Agent>();
    if (convert<Agent>::decode(node, *rhs)) {
      return true;
    }
    rhs = nullptr;
    return false;
  }
};

template <typename T = Agent>
struct convert_world {
  static Node encode(const World& rhs) {
    Node node;
    node["obstacles"] = rhs.get_obstacles();
    node["walls"] = rhs.get_walls();
    node["agents"] = rhs.get_agents();
    return node;
  }
  static bool decode(const Node& node, World& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["agents"]) {
      if (node["agents"].IsSequence()) {
        for (const auto& c : node["agents"]) {
          // TODO
          rhs.add_agent(c.as<std::shared_ptr<T>>());
        }
      }
    }
    if (node["obstacles"]) {
      for (const auto& c : node["obstacles"]) {
        rhs.add_obstacle(c.as<Disc>());
      }
    }
    if (node["walls"]) {
      for (const auto& c : node["walls"]) {
        rhs.add_wall(c.as<LineSegment>());
      }
    }
    return true;
  }
};

template <>
struct convert<World> {
  static Node encode(const World& rhs) { return convert_world<>::encode(rhs); }
  static bool decode(const Node& node, World& rhs) {
    return convert_world<>::decode(node, rhs);
  }
};

}  // namespace YAML

#endif  // NAVGROUND_SIM_YAML_WORLD_H
