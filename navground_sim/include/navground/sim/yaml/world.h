#ifndef NAVGROUND_SIM_YAML_WORLD_H
#define NAVGROUND_SIM_YAML_WORLD_H

#include <memory>

#include "navground/core/types.h"
#include "navground/core/yaml/core.h"
#include "navground/core/yaml/property.h"
#include "navground/core/yaml/register.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/world.h"
#include "yaml-cpp/yaml.h"

using navground::core::Behavior;
using navground::core::Disc;
using navground::core::Kinematics;
using navground::core::LineSegment;
using navground::core::Vector2;
using navground::sim::Agent;
using navground::sim::BoundingBox;
using navground::sim::Obstacle;
using navground::sim::StateEstimation;
using navground::sim::Task;
using navground::sim::Wall;
using navground::sim::World;

namespace YAML {

template <>
struct convert<Obstacle> {
  static Node encode(const Obstacle& rhs) {
    Node node = convert<Disc>::encode(rhs.disc);
    node["uid"] = rhs.uid;
    return node;
  }
  static bool decode(const Node& node, Obstacle& rhs) {
    if (node["uid"]) {
      rhs.uid = node["uid"].as<unsigned>();
    }
    return convert<Disc>::decode(node, rhs.disc);
  }
  static Node schema() {
    Node node = schema::type<Disc>();
    node["properties"]["uid"] = schema::type<int>();
    return node;
  }
  static constexpr const char name[] = "obstacle";
};

template <>
struct convert<std::shared_ptr<Obstacle>> {
  static Node encode(const std::shared_ptr<Obstacle>& rhs) {
    if (rhs) {
      return convert<Obstacle>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node& node, std::shared_ptr<Obstacle>& rhs) {
    rhs = std::make_shared<Obstacle>();
    if (convert<Obstacle>::decode(node, *rhs)) {
      return true;
    }
    rhs = nullptr;
    return false;
  }
};

template <>
struct convert<Wall> {
  static Node encode(const Wall& rhs) {
    Node node;
    node["line"] = convert<LineSegment>::encode(rhs.line);
    node["uid"] = rhs.uid;
    return node;
  }
  static bool decode(const Node& node, Wall& rhs) {
    if (node["uid"]) {
      rhs.uid = node["uid"].as<unsigned>();
    }
    if (node["line"]) {
      rhs.line = node["line"].as<LineSegment>();
      return true;
    }
    return false;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["line"] = schema::type<LineSegment>();
    node["properties"]["uid"] = schema::type<int>();
    return node;
  }
  static constexpr const char name[] = "wall";
};

template <>
struct convert<std::shared_ptr<Wall>> {
  static Node encode(const std::shared_ptr<Wall>& rhs) {
    if (rhs) {
      return convert<Wall>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node& node, std::shared_ptr<Wall>& rhs) {
    rhs = std::make_shared<Wall>();
    if (convert<Wall>::decode(node, *rhs)) {
      return true;
    }
    rhs = nullptr;
    return false;
  }
};

template <>
struct convert<BoundingBox> {
  static Node encode(const BoundingBox& rhs) {
    Node node;
    if (!rhs.isNull()) {
      node["min_x"] = rhs.getMinX();
      node["min_y"] = rhs.getMinY();
      node["max_x"] = rhs.getMaxX();
      node["max_y"] = rhs.getMaxY();
    }
    return node;
  }
  static bool decode(const Node& node, BoundingBox& rhs) {
    ng_float_t x, y, X, Y;
    if (node["min_x"]) {
      x = node["min_x"].as<ng_float_t>();
    } else {
      x = -std::numeric_limits<ng_float_t>::infinity();
    }
    if (node["max_x"]) {
      X = node["max_x"].as<ng_float_t>();
    } else {
      X = std::numeric_limits<ng_float_t>::infinity();
    }
    if (node["min_y"]) {
      y = node["min_y"].as<ng_float_t>();
    } else {
      y = -std::numeric_limits<ng_float_t>::infinity();
    }
    if (node["max_y"]) {
      Y = node["max_y"].as<ng_float_t>();
    } else {
      Y = std::numeric_limits<ng_float_t>::infinity();
    }
    rhs.init(x, X, y, Y);
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["min_x"] = schema::type<ng_float_t>();
    node["properties"]["min_y"] = schema::type<ng_float_t>();
    node["properties"]["max_x"] = schema::type<ng_float_t>();
    node["properties"]["max_y"] = schema::type<ng_float_t>();
    return node;
  }
  static constexpr const char name[] = "bounding_box";
};

template <>
struct convert<World::Lattice> {
  static Node encode(const World::Lattice& rhs) {
    Node node;
    if (rhs.has_value()) {
      auto [x, y] = *rhs;
      node.push_back(x);
      node.push_back(y);
    }
    return node;
  }
  static bool decode(const Node& node, World::Lattice& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return true;
    }
    rhs = std::make_tuple<ng_float_t, ng_float_t>(node[0].as<ng_float_t>(),
                                                  node[1].as<ng_float_t>());
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "array";
    node["items"] = schema::type<ng_float_t>();
    node["minItems"] = 2;
    node["maxItems"] = 2;
    return node;
  }
  static constexpr const char name[] = "lattice";
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
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["type"] = schema::type<std::string>();
    return node;
  }
  static constexpr const char name[] = "task";
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
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["type"] = schema::type<std::string>();
    return node;
  }
  static constexpr const char name[] = "state_estimation";
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
    node["speed_tolerance"] = rhs.get_controller()->get_speed_tolerance();
    node["type"] = rhs.type;
    node["color"] = rhs.color;
    node["id"] = rhs.id;
    node["uid"] = rhs.uid;
    if (rhs.external) {
      node["external"] = true;
    }
    if (rhs.tags.size()) {
      for (const auto& tag : rhs.tags) {
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
      rhs.set_state_estimation(
          node["state_estimation"].as<std::shared_ptr<StateEstimation>>());
    }
    if (node["position"]) {
      rhs.pose.position = node["position"].as<Vector2>();
    }
    if (node["orientation"]) {
      rhs.pose.orientation = node["orientation"].as<ng_float_t>();
    }
    if (node["velocity"]) {
      rhs.twist.velocity = node["velocity"].as<Vector2>();
    }
    if (node["angular_speed"]) {
      rhs.twist.angular_speed = node["angular_speed"].as<ng_float_t>();
    }
    if (node["radius"]) {
      rhs.radius = node["radius"].as<ng_float_t>();
    }
    if (node["control_period"]) {
      rhs.control_period = node["control_period"].as<ng_float_t>();
    }
    if (node["speed_tolerance"]) {
      rhs.get_controller()->set_speed_tolerance(node["speed_tolerance"].as<ng_float_t>());
    }
    if (node["type"]) {
      rhs.type = node["type"].as<std::string>();
    }
    if (node["color"]) {
      rhs.color = node["color"].as<std::string>();
    }
    if (node["id"]) {
      rhs.id = node["id"].as<unsigned>();
    }
    if (node["tags"]) {
      const auto vs = node["tags"].as<std::vector<std::string>>();
      rhs.tags = std::set<std::string>(vs.begin(), vs.end());
    }
    if (node["uid"]) {
      rhs.uid = node["uid"].as<unsigned>();
    }
    if (node["external"]) {
      rhs.external = node["external"].as<bool>();
    }
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["behavior"] = schema::ref<Behavior>();
    node["properties"]["kinematics"] = schema::ref<Kinematics>();
    node["properties"]["task"] = schema::ref<Task>();
    node["properties"]["state_estimation"] = schema::ref<StateEstimation>();
    node["properties"]["position"] = schema::ref<Vector2>();
    node["properties"]["orientation"] = schema::type<ng_float_t>();
    node["properties"]["velocity"] = schema::ref<Vector2>();
    node["properties"]["angular_speed"] = schema::type<ng_float_t>();
    node["properties"]["radius"] = schema::type<ng_float_t>();
    node["properties"]["control_period"] = schema::type<ng_float_t>();
    node["properties"]["speed_tolerance"] = schema::type<ng_float_t>();
    node["properties"]["type"] = schema::type<std::string>();
    node["properties"]["color"] = schema::type<std::string>();
    node["properties"]["id"] = schema::type<int>();
    node["properties"]["uid"] = schema::type<int>();
    node["properties"]["external"] = schema::type<bool>();
    node["properties"]["tags"]["type"] = "array";
    node["properties"]["tags"]["items"] = schema::type<std::string>();
    return node;
  }
  static constexpr const char name[] = "agent";
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
    // HACK(Jerome): To avoid issues with serializing the world
    // when there are very small components
    rhs.snap_twists_to_zero();
    Node node;
    node["obstacles"] = rhs.get_obstacles();
    node["walls"] = rhs.get_walls();
    node["agents"] = rhs.get_agents();
    if (rhs.has_bounding_box()) {
      node["bounding_box"] = rhs.get_bounding_box();
    }
    if (rhs.has_lattice()) {
      for (int i = 0; i < 2; ++i) {
        auto lattice = rhs.get_lattice(i);
        if (lattice) {
          std::string axis = i == 0 ? "x" : "y";
          node["lattice"][axis] = lattice;
        }
      }
    }
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
        rhs.add_obstacle(c.as<Obstacle>());
      }
    }
    if (node["walls"]) {
      for (const auto& c : node["walls"]) {
        rhs.add_wall(c.as<Wall>());
      }
    }
    if (node["bounding_box"]) {
      rhs.set_bounding_box(node["bounding_box"].as<BoundingBox>());
    }
    if (node["lattice"]) {
      std::array<std::string, 2> axes{"x", "y"};
      for (unsigned i = 0; i < axes.size(); ++i) {
        if (node["lattice"][axes[i]]) {
          auto value = node["lattice"][axes[i]].as<World::Lattice>();
          rhs.set_lattice(i, value);
        }
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
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["agents"]["type"] = "array";
    node["properties"]["agents"]["items"] = schema::ref<Agent>();
    node["properties"]["obstacles"]["type"] = "array";
    node["properties"]["obstacles"]["items"] = schema::ref<Obstacle>();
    node["properties"]["walls"]["type"] = "array";
    node["properties"]["walls"]["items"] = schema::ref<Wall>();    
    node["properties"]["bounding_box"] = schema::type<BoundingBox>(); 
    node["properties"]["lattice"]["type"] = "object";
    node["properties"]["lattice"]["properties"]["x"] = schema::type<World::Lattice>();
    node["properties"]["lattice"]["properties"]["y"] = schema::type<World::Lattice>();
    return node;
  }
  static constexpr const char name[] = "world";
};

}  // namespace YAML

#endif  // NAVGROUND_SIM_YAML_WORLD_H
