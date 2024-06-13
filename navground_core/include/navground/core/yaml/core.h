#ifndef NAVGROUND_CORE_YAML_CORE_H
#define NAVGROUND_CORE_YAML_CORE_H

#include "navground/core/behavior.h"
#include "navground/core/common.h"
#include "navground/core/kinematics.h"
#include "navground/core/states/geometric.h"
#include "navground/core/types.h"
#include "navground/core/yaml/register.h"
#include "yaml-cpp/yaml.h"

using navground::core::Behavior;
using navground::core::BehaviorModulation;
using navground::core::Disc;
using navground::core::GeometricState;
using navground::core::Kinematics;
using navground::core::LineSegment;
using navground::core::Neighbor;
using navground::core::SocialMargin;
using navground::core::Vector2;

namespace YAML {

template <>
struct convert<LineSegment> {
  static Node encode(const LineSegment& rhs) {
    Node node;
    node.push_back(rhs.p1);
    node.push_back(rhs.p2);
    return node;
  }
  static bool decode(const Node& node, LineSegment& rhs) {
    if (!node.IsSequence() || node.size() != 2) {
      return false;
    }
    rhs.p1 = node[0].as<Vector2>();
    rhs.p2 = node[1].as<Vector2>();
    rhs.update();
    return true;
  }
};

template <>
struct convert<Disc> {
  static Node encode(const Disc& rhs) {
    Node node;
    node["position"] = rhs.position;
    node["radius"] = rhs.radius;
    return node;
  }
  static bool decode(const Node& node, Disc& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    rhs.position = node["position"].as<Vector2>();
    rhs.radius = node["radius"].as<ng_float_t>();
    return true;
  }
};

template <>
struct convert<Neighbor> {
  static Node encode(const Neighbor& rhs) {
    Node node = convert<Disc>::encode(static_cast<const Disc&>(rhs));
    node["velocity"] = rhs.velocity;
    node["id"] = rhs.id;
    return node;
  }
  static bool decode(const Node& node, Neighbor& rhs) {
    if (!convert<Disc>::decode(node, static_cast<Disc&>(rhs))) {
      return false;
    }
    rhs.velocity = node["velocity"].as<Vector2>();
    rhs.id = node["id"].as<unsigned>();
    return true;
  }
};

template <>
struct convert<Behavior::Heading> {
  static Node encode(const Behavior::Heading& rhs) {
    return Node(Behavior::heading_to_string(rhs));
  }
  static bool decode(const Node& node, Behavior::Heading& rhs) {
    rhs = Behavior::heading_from_string(node.as<std::string>());
    return true;
  }
};

template <>
struct convert<Behavior> {
  static Node encode(const Behavior& rhs) {
    Node node;
    encode_type_and_properties<Behavior>(node, rhs);
    node["optimal_speed"] = rhs.get_optimal_speed();
    node["optimal_angular_speed"] = rhs.get_optimal_angular_speed();
    node["rotation_tau"] = rhs.get_rotation_tau();
    node["safety_margin"] = rhs.get_safety_margin();
    node["horizon"] = rhs.get_horizon();
    node["radius"] = rhs.get_radius();
    node["heading"] = rhs.get_heading_behavior();
    auto k = rhs.get_kinematics();
    if (k) {
      node["kinematics"] = *k;
    }
    node["social_margin"] = rhs.social_margin;
    if (rhs.get_modulations().size()) {
      node["modulations"] = rhs.get_modulations();
    }
    return node;
  }
  static bool decode(const Node& node, Behavior& rhs) {
    decode_properties(node, rhs);
    if (node["optimal_speed"]) {
      rhs.set_optimal_speed(node["optimal_speed"].as<ng_float_t>());
    }
    if (node["optimal_angular_speed"]) {
      rhs.set_optimal_angular_speed(
          node["optimal_angular_speed"].as<ng_float_t>());
    }
    if (node["rotation_tau"]) {
      rhs.set_rotation_tau(node["rotation_tau"].as<ng_float_t>());
    }
    if (node["safety_margin"]) {
      rhs.set_safety_margin(node["safety_margin"].as<ng_float_t>());
    }
    if (node["horizon"]) {
      rhs.set_horizon(node["horizon"].as<ng_float_t>());
    }
    if (node["radius"]) {
      rhs.set_radius(node["radius"].as<ng_float_t>());
    }
    if (node["heading"]) {
      rhs.set_heading_behavior(node["heading"].as<Behavior::Heading>());
    }
    if (node["social_margin"]) {
      rhs.social_margin = node["social_margin"].as<SocialMargin>();
    }
    if (node["modulations"]) {
      auto mods = node["modulations"]
                      .as<std::vector<std::shared_ptr<BehaviorModulation>>>();
      // rhs.get_modulations().merge(mods);
      std::copy(mods.begin(), mods.end(),
                std::back_inserter(rhs.get_modulations()));
    }
    return true;
  }
};

template <>
struct convert<std::shared_ptr<Behavior>> {
  static Node encode(const std::shared_ptr<Behavior>& rhs) {
    if (rhs) {
      return convert<Behavior>::encode(*rhs);
    }
    return Node();
  }
  static bool decode(const Node& node, std::shared_ptr<Behavior>& rhs) {
    rhs = make_type_from_yaml<Behavior>(node);
    if (rhs) {
      convert<Behavior>::decode(node, *rhs);
    }
    return true;
  }
};

template <>
struct convert<Kinematics> {
  static Node encode(const Kinematics& rhs) {
    Node node;
    encode_type_and_properties<Kinematics>(node, rhs);
    node["max_speed"] = rhs.get_max_speed();
    node["max_angular_speed"] = rhs.get_max_angular_speed();
    return node;
  }
  static bool decode(const Node& node, Kinematics& rhs) {
    decode_properties(node, rhs);
    if (node["max_speed"]) {
      rhs.set_max_speed(node["max_speed"].as<ng_float_t>());
    }
    if (node["max_angular_speed"]) {
      rhs.set_max_angular_speed(node["max_angular_speed"].as<ng_float_t>());
    }
    return true;
  }
};

template <>
struct convert<std::shared_ptr<Kinematics>> {
  static Node encode(const std::shared_ptr<Kinematics>& rhs) {
    return convert<Kinematics>::encode(*rhs);
  }
  static bool decode(const Node& node, std::shared_ptr<Kinematics>& rhs) {
    rhs = make_type_from_yaml<Kinematics>(node);
    if (rhs) {
      convert<Kinematics>::decode(node, *rhs);
    }
    return true;
  }
};

template <>
struct convert<BehaviorModulation> {
  static Node encode(const BehaviorModulation& rhs) {
    Node node;
    encode_type_and_properties<BehaviorModulation>(node, rhs);
    node["enabled"] = rhs.get_enabled();
    return node;
  }
  static bool decode(const Node& node, BehaviorModulation& rhs) {
    decode_properties(node, rhs);
    rhs.set_enabled(node["enabled"].as<bool>(true));
    return true;
  }
};

template <>
struct convert<std::shared_ptr<BehaviorModulation>> {
  static Node encode(const std::shared_ptr<BehaviorModulation>& rhs) {
    return convert<BehaviorModulation>::encode(*rhs);
  }
  static bool decode(const Node& node,
                     std::shared_ptr<BehaviorModulation>& rhs) {
    rhs = make_type_from_yaml<BehaviorModulation>(node);
    if (rhs) {
      convert<BehaviorModulation>::decode(node, *rhs);
    }
    return true;
  }
};

template <>
struct convert<SocialMargin> {
  static Node encode(const SocialMargin& rhs) {
    Node node;
    auto m = rhs.get_modulation();
    if (m) {
      node["modulation"] = m;
    }
    node["default"] = rhs.get_default_value();
    for (const auto& [k, v] : rhs.get_values()) {
      if (v) {
        node["values"][k] = v;
      }
    }
    return node;
  }
  static bool decode(const Node& node, SocialMargin& rhs) {
    if (node["modulation"]) {
      rhs.set_modulation(
          node["modulation"].as<std::shared_ptr<SocialMargin::Modulation>>());
    }
    if (node["values"] && node["values"].IsMap()) {
      for (const auto& p : node["values"]) {
        rhs.set(p.first.as<unsigned>(), p.second.as<ng_float_t>());
      }
    }
    if (node["default"]) {
      rhs.set(node["default"].as<ng_float_t>());
    }
    return true;
  }
};

template <>
struct convert<std::shared_ptr<SocialMargin::Modulation>> {
  static Node encode(const std::shared_ptr<SocialMargin::Modulation>& rhs) {
    Node node;
    if (auto m = dynamic_cast<SocialMargin::ZeroModulation*>(rhs.get())) {
      node["type"] = "zero";
    } else if (auto m =
                   dynamic_cast<SocialMargin::ConstantModulation*>(rhs.get())) {
      node["type"] = "constant";
    } else if (auto m =
                   dynamic_cast<SocialMargin::LinearModulation*>(rhs.get())) {
      node["type"] = "linear";
      node["upper"] = m->get_upper_distance();
    } else if (auto m = dynamic_cast<SocialMargin::QuadraticModulation*>(
                   rhs.get())) {
      node["type"] = "quadratic";
      node["upper"] = m->get_upper_distance();
    } else if (auto m =
                   dynamic_cast<SocialMargin::LogisticModulation*>(rhs.get())) {
      node["type"] = "logistic";
    }
    return node;
  }

  static bool decode(const Node& node,
                     std::shared_ptr<SocialMargin::Modulation>& rhs) {
    rhs = nullptr;
    if (node["type"]) {
      std::string type = node["type"].as<std::string>();
      if (type == "zero") {
        rhs = std::make_shared<SocialMargin::ZeroModulation>();
      } else if (type == "constant") {
        rhs = std::make_shared<SocialMargin::ConstantModulation>();
      } else if (type == "linear") {
        rhs = std::make_shared<SocialMargin::LinearModulation>(
            node["upper"].as<ng_float_t>(10.0));
      } else if (type == "quadratic") {
        rhs = std::make_shared<SocialMargin::QuadraticModulation>(
            node["upper"].as<ng_float_t>(10.0));
      } else if (type == "logistic") {
        rhs = std::make_shared<SocialMargin::LogisticModulation>();
      }
    }
    return rhs != nullptr;
  }
};

}  // namespace YAML

#endif  // NAVGROUND_CORE_YAML_CORE_H
