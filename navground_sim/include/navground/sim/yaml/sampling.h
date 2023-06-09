#ifndef NAVGROUND_SIM_YAML_SAMPLER_H
#define NAVGROUND_SIM_YAML_SAMPLER_H

#include <iostream>
#include <memory>

#include "navground/core/yaml/property.h"
#include "navground/sim/sampling/agent.h"
#include "navground/sim/sampling/register.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

using navground::core::Property;
using navground::sim::AgentSampler;
using navground::sim::BehaviorSampler;
using navground::sim::ChoiceSampler;
using navground::sim::ConstantSampler;
using navground::sim::GridSampler;
using navground::sim::is_algebra;
using navground::sim::is_number;
using navground::sim::KinematicsSampler;
using navground::sim::NormalSampler;
using navground::sim::PropertySampler;
using navground::sim::RegularSampler;
using navground::sim::Sampler;
using navground::sim::SamplerFromRegister;
using navground::sim::SequenceSampler;
using navground::sim::StateEstimationSampler;
using navground::sim::TaskSampler;
using navground::sim::uniform_distribution;
using navground::sim::UniformSampler;
using navground::sim::World;
using navground::sim::Wrap;
using navground::sim::wrap_from_string;
using navground::sim::wrap_to_string;

namespace YAML {

#if 0
std::unique_ptr<PropertySampler> property_sampler(const Node& node,
                                                  const Property& property) {
  try {
    const auto value = decode_property(property, node);
    return std::make_unique<ConstantSampler<Property::Field>>(value);
  } catch (const std::exception& e) {
    std::cerr << e.what() << std::endl;
  }
  if (node.Type() == YAML::NodeType::Sequence) {
    std::vector<Property::Field> values;
    for (const YAML::Node& c : node) {
      const auto value = decode_property(property, c);
      values.push_back(value);
    }
    if (values.size()) {
      return std::make_unique<SequenceSampler<Property::Field>>(values);
    }
  }
  return nullptr;
}
#endif

template <typename T>
std::unique_ptr<Sampler<T>> read_sampler(const Node& node) {
  // implicit const
  try {
    return std::make_unique<ConstantSampler<T>>(node.as<T>());
  } catch (YAML::BadConversion) {
  }
  // implicit sequence
  if (node.Type() == YAML::NodeType::Sequence) {
    try {
      return std::make_unique<SequenceSampler<T>>(node.as<std::vector<T>>());
    } catch (YAML::BadConversion) {
    }
  }
  if (node.Type() != YAML::NodeType::Map || !node["sampler"]) {
    return nullptr;
  }
  const auto sampler = node["sampler"].as<std::string>();
  if (sampler == "constant") {
    if (node["value"]) {
      return std::make_unique<ConstantSampler<T>>(node["value"].as<T>());
    }
  }
  if (sampler == "sequence") {
    if (node["values"]) {
      Wrap wrap(Wrap::loop);
      if (node["wrap"]) {
        wrap = wrap_from_string(node["wrap"].as<std::string>());
      }
      return std::make_unique<SequenceSampler<T>>(
          node["values"].as<std::vector<T>>(), wrap);
    }
    return nullptr;
  }
  if (sampler == "choice") {
    if (node["values"]) {
      return std::make_unique<ChoiceSampler<T>>(
          node["values"].as<std::vector<T>>());
    }
    return nullptr;
  }
  if constexpr (is_algebra<T>) {
    if (sampler == "regular") {
      if (node["from"]) {
        const auto start = node["from"].as<T>();
        Wrap wrap(Wrap::loop);
        if (node["wrap"]) {
          wrap = wrap_from_string(node["wrap"].as<std::string>());
        }
        if (node["to"] && node["number"]) {
          const auto end = node["to"].as<T>();
          const auto number = node["number"].as<unsigned>();
          return std::make_unique<RegularSampler<T>>(
              RegularSampler<T>::make_with_interval(start, end, number, wrap));
        }
        if (node["step"]) {
          const auto step = node["step"].as<T>();
          std::optional<unsigned> number = std::nullopt;
          if (node["number"]) {
            number = node["number"].as<unsigned>();
          }
          return std::make_unique<RegularSampler<T>>(
              RegularSampler<T>::make_with_step(start, step, number, wrap));
        }
      }
      return nullptr;
    }
  }
  if constexpr (std::is_same_v<T, Vector2>) {
    if (sampler == "grid") {
      if (node["from"] && node["to"] && node["numbers"]) {
        const auto start = node["from"].as<Vector2>();
        const auto end = node["to"].as<Vector2>();
        const auto numbers = node["numbers"].as<std::array<unsigned, 2>>();
        Wrap wrap(Wrap::loop);
        if (node["wrap"]) {
          wrap = wrap_from_string(node["wrap"].as<std::string>());
        }
        return std::make_unique<GridSampler>(start, end, numbers, wrap);
      }
      return nullptr;
    }
  }
  // !std::is_void_v<uniform_distribution<T>>
  if constexpr (is_number<T>) {
    if (sampler == "uniform") {
      if (node["from"] && node["to"]) {
        const auto min = node["from"].as<T>();
        const auto max = node["to"].as<T>();
        return std::make_unique<UniformSampler<T>>(min, max);
      }
      return nullptr;
    }
  }
  if constexpr (is_number<T>) {
    if (sampler == "normal") {
      if (node["mean"] && node["std_dev"]) {
        std::optional<T> min;
        std::optional<T> max;
        if (node["min"]) {
          min = node["min"].as<T>();
        }
        if (node["max"]) {
          max = node["max"].as<T>();
        }
        const auto mean = node["mean"].as<float>();
        const auto std_dev = node["std_dev"].as<float>();
        return std::make_unique<NormalSampler<T>>(mean, std_dev, min, max);
      }
      return nullptr;
    }
  }
  return nullptr;
}

template <typename T>
struct convert<ConstantSampler<T>> {
  static Node encode(const ConstantSampler<T>& rhs) {
    Node node;
    node["sampler"] = "constant";
    node["value"] = rhs.value;
    return node;
  }
};

template <typename T>
struct convert<SequenceSampler<T>> {
  static Node encode(const SequenceSampler<T>& rhs) {
    Node node;
    node["sampler"] = "sequence";
    node["values"] = rhs.values;
    node["wrap"] = wrap_to_string(rhs.wrap);
    return node;
  }
};

template <typename T>
struct convert<ChoiceSampler<T>> {
  static Node encode(const ChoiceSampler<T>& rhs) {
    Node node;
    node["sampler"] = "choice";
    node["values"] = rhs.values;
    return node;
  }
};

template <typename T>
struct convert<RegularSampler<T>> {
  static Node encode(const RegularSampler<T>& rhs) {
    Node node;
    node["from"] = rhs.from;
    if (rhs.to) {
      node["to"] = *rhs.to;
    }
    node["step"] = rhs.step;
    if (rhs.number) {
      node["number"] = *rhs.number;
    }
    node["sampler"] = "regular";
    node["wrap"] = wrap_to_string(rhs.wrap);
    return node;
  }
};

template <>
struct convert<GridSampler> {
  static Node encode(const GridSampler& rhs) {
    Node node;
    node["from"] = rhs.from;
    node["to"] = rhs.to;
    node["numbers"] = rhs.numbers;
    node["sampler"] = "grid";
    node["wrap"] = wrap_to_string(rhs.wrap);
    return node;
  }
};

template <typename T>
struct convert<UniformSampler<T>> {
  static Node encode(const UniformSampler<T>& rhs) {
    Node node;
    node["from"] = rhs.min;
    node["to"] = rhs.max;
    node["sampler"] = "uniform";
    return node;
  }
};

template <typename T>
struct convert<NormalSampler<T>> {
  static Node encode(const NormalSampler<T>& rhs) {
    Node node;
    if (rhs.min) {
      node["min"] = *rhs.min;
    }
    if (rhs.max) {
      node["max"] = *rhs.max;
    }
    node["mean"] = rhs.mean;
    node["std_dev"] = rhs.std_dev;
    node["sampler"] = "normal";
    return node;
  }
};

inline std::unique_ptr<PropertySampler> property_sampler(
    const Node& node, const Property& property) {
  return std::visit(
      [&node](auto&& arg) -> std::unique_ptr<PropertySampler> {
        using V = std::decay_t<decltype(arg)>;
        return std::make_unique<PropertySampler>(
            PropertySampler(read_sampler<V>(node)));
      },
      property.default_value);
}

template <typename T>
struct convert<Sampler<T>*> {
  static Node encode(const Sampler<T>* rhs) {
    if (!rhs) return Node();
    if (const ConstantSampler<T>* sampler =
            dynamic_cast<const ConstantSampler<T>*>(rhs)) {
      return Node(*sampler);
    }
    if (const SequenceSampler<T>* sampler =
            dynamic_cast<const SequenceSampler<T>*>(rhs)) {
      return Node(*sampler);
    }
    if (const ChoiceSampler<T>* sampler =
            dynamic_cast<const ChoiceSampler<T>*>(rhs)) {
      return Node(*sampler);
    }
    if constexpr (is_algebra<T>) {
      if (const RegularSampler<T>* sampler =
              dynamic_cast<const RegularSampler<T>*>(rhs)) {
        return Node(*sampler);
      }
    }
    if constexpr (std::is_same_v<T, Vector2>) {
      if (const GridSampler* sampler = dynamic_cast<const GridSampler*>(rhs)) {
        return Node(*sampler);
      }
    }
    if constexpr (is_number<T>) {
      if (const UniformSampler<T>* sampler =
              dynamic_cast<const UniformSampler<T>*>(rhs)) {
        return Node(*sampler);
      }
      if (const NormalSampler<T>* sampler =
              dynamic_cast<const NormalSampler<T>*>(rhs)) {
        return Node(*sampler);
      }
    }
    return Node();
  }
};

template <typename T>
struct convert<std::shared_ptr<Sampler<T>>> {
  static Node encode(const std::shared_ptr<Sampler<T>>& rhs) {
    return convert<Sampler<T>*>::encode(rhs.get());
  }
};

template <>
struct convert<std::shared_ptr<PropertySampler>> {
  static Node encode(const std::shared_ptr<PropertySampler>& rhs) {
    return std::visit([](auto&& arg) { return Node(arg.get()); }, rhs->sampler);
  }
};

template <typename T>
bool decode_sr(const Node& node, SamplerFromRegister<T>* rhs) {
  if (!node.IsMap() || !node["type"]) {
    return false;
  }
  rhs->type = node["type"].as<std::string>();
  const auto& properties = T::type_properties();
  if (!properties.count(rhs->type)) {
    std::cerr << "No type " << rhs->type << " in register "
              << get_type_name<T>() << std::endl;
    for (const auto& s : T::types()) {
      std::cout << s << std::endl;
    }
    return false;
  }
  for (const auto& [name, property] : properties.at(rhs->type)) {
    if (node[name]) {
      rhs->properties[name] = property_sampler(node[name], property);
    }
  }
  return true;
}

template <typename T>
Node encode_sr(const SamplerFromRegister<T>& rhs) {
  Node node;
  if (rhs.type.empty()) {
    return node;
  }
  node["type"] = rhs.type;
  for (const auto& [name, property_sampler] : rhs.properties) {
    if (property_sampler) {
      node[name] = property_sampler;
    }
  }
  return node;
}

template <typename T>
struct convert<BehaviorSampler<T>> {
  static Node encode(const BehaviorSampler<T>& rhs) {
    Node node = encode_sr<T>(rhs);
    // std::cout << "W" << rhs.type << std::endl;
    if (rhs.optimal_speed) {
      node["optimal_speed"] = rhs.optimal_speed;
    }
    if (rhs.optimal_angular_speed) {
      node["optimal_angular_speed"] = rhs.optimal_angular_speed;
    }
    if (rhs.rotation_tau) {
      node["rotation_tau"] = rhs.rotation_tau;
    }
    if (rhs.safety_margin) {
      node["safety_margin"] = rhs.safety_margin;
    }
    if (rhs.horizon) {
      node["horizon"] = rhs.horizon;
    }
    if (rhs.heading) {
      node["heading"] = rhs.heading;
    }
    return node;
  }
  static bool decode(const Node& node, BehaviorSampler<T>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r) return false;
    if (node["optimal_speed"]) {
      rhs.optimal_speed = read_sampler<float>(node["optimal_speed"]);
    }
    if (node["optimal_angular_speed"]) {
      rhs.optimal_angular_speed =
          read_sampler<float>(node["optimal_angular_speed"]);
    }
    if (node["rotation_tau"]) {
      rhs.rotation_tau = read_sampler<float>(node["rotation_tau"]);
    }
    if (node["safety_margin"]) {
      rhs.safety_margin = read_sampler<float>(node["safety_margin"]);
    }
    if (node["horizon"]) {
      rhs.horizon = read_sampler<float>(node["horizon"]);
    }
    if (node["heading"]) {
      rhs.heading = read_sampler<std::string>(node["heading"]);
    }
    return true;
  }
};

template <typename T>
struct convert<KinematicsSampler<T>> {
  static Node encode(const KinematicsSampler<T>& rhs) {
    Node node = encode_sr<T>(rhs);
    if (rhs.max_speed) {
      node["max_speed"] = rhs.max_speed;
    }
    if (rhs.max_angular_speed) {
      node["max_angular_speed"] = rhs.max_angular_speed;
    }
    return node;
  }
  static bool decode(const Node& node, KinematicsSampler<T>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r) return false;
    if (node["max_speed"]) {
      rhs.max_speed = read_sampler<float>(node["max_speed"]);
    }
    if (node["max_angular_speed"]) {
      rhs.max_angular_speed = read_sampler<float>(node["max_angular_speed"]);
    }
    return true;
  }
};

template <typename T>
struct convert<SamplerFromRegister<T>> {
  static Node encode(const SamplerFromRegister<T>& rhs) {
    return encode_sr<T>(rhs);
  }
  static bool decode(const Node& node, SamplerFromRegister<T>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    return r;
  }
};

template <typename W>
struct convert<AgentSampler<W>> {
  using B = typename W::A::B;
  using K = typename W::A::K;
  using T = typename W::A::T;
  using S = typename W::A::S;

  static Node encode(const AgentSampler<W>& rhs) {
    Node node;
    if (!rhs.behavior.type.empty()) {
      node["behavior"] = rhs.behavior;
    }
    if (!rhs.kinematics.type.empty()) {
      node["kinematics"] = rhs.kinematics;
    }
    if (!rhs.task.type.empty()) {
      node["task"] = rhs.task;
    }
    if (!rhs.state_estimation.type.empty()) {
      node["state_estimation"] = rhs.state_estimation;
    }
    if (rhs.position) {
      node["position"] = rhs.position;
    }
    if (rhs.orientation) {
      node["orientation"] = rhs.orientation;
    }
    if (rhs.radius) {
      node["radius"] = rhs.radius;
    }
    if (rhs.control_period) {
      node["control_period"] = rhs.control_period;
    }
    if (rhs.number) {
      node["number"] = rhs.number;
    }
    if (rhs.type) {
      node["type"] = rhs.type;
    }
    if (rhs.id) {
      node["id"] = rhs.id;
    }
    if (!rhs.name.empty()) {
      node["name"] = rhs.name;
    }
    return node;
  }
  static bool decode(const Node& node, AgentSampler<W>& rhs) {
    if (!node.IsMap()) {
      return false;
    }
    if (node["behavior"]) {
      rhs.behavior = node["behavior"].as<BehaviorSampler<B>>();
    }
    if (node["kinematics"]) {
      rhs.kinematics = node["kinematics"].as<KinematicsSampler<K>>();
    }
    if (node["task"]) {
      rhs.task = node["task"].as<TaskSampler<T>>();
    }
    if (node["state_estimation"]) {
      rhs.state_estimation =
          node["state_estimation"].as<StateEstimationSampler<S>>();
    }
    if (node["position"]) {
      rhs.position = read_sampler<Vector2>(node["position"]);
    }
    if (node["orientation"]) {
      rhs.orientation = read_sampler<float>(node["orientation"]);
    }
    if (node["radius"]) {
      rhs.radius = read_sampler<float>(node["radius"]);
    }
    if (node["control_period"]) {
      rhs.control_period = read_sampler<float>(node["control_period"]);
    }
    if (node["number"]) {
      rhs.number = node["number"].as<unsigned>(0);
    }
    if (node["type"]) {
      rhs.type = read_sampler<std::string>(node["type"]);
    }
    if (node["id"]) {
      rhs.id = read_sampler<int>(node["id"]);
    }
    if (node["name"]) {
      rhs.name = node["name"].as<std::string>();
    }
    return true;
  }
};

}  // namespace YAML

#endif  // NAVGROUND_SIM_YAML_SAMPLER_H
