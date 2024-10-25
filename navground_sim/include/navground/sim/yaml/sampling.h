#ifndef NAVGROUND_SIM_YAML_SAMPLER_H
#define NAVGROUND_SIM_YAML_SAMPLER_H

#include <iostream>
#include <memory>

#include "navground/core/types.h"
#include "navground/core/yaml/property.h"
#include "navground/sim/sampling/agent.h"
#include "navground/sim/sampling/register.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/yaml/world.h"
#include "navground/sim/export.h"
#include "yaml-cpp/yaml.h"

using navground::core::Property;
using navground::sim::AgentSampler;
using navground::sim::BehaviorModulationSampler;
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

bool NAVGROUND_SIM_EXPORT get_use_compact_samplers();

void NAVGROUND_SIM_EXPORT set_use_compact_samplers(bool value);

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
  bool once = false;
  // implicit const
  try {
    return std::make_unique<ConstantSampler<T>>(node.as<T>(), once);
  } catch (YAML::BadConversion &) {
  }
  // implicit sequence
  if (node.Type() == YAML::NodeType::Sequence) {
    try {
      return std::make_unique<SequenceSampler<T>>(node.as<std::vector<T>>(),
                                                  Wrap::loop, once);
    } catch (YAML::BadConversion &) {
    }
  }
  if (node.Type() != YAML::NodeType::Map || !node["sampler"]) {
    return nullptr;
  }
  if (node["once"]) {
    once = node["once"].as<bool>();
  }
  const auto sampler = node["sampler"].as<std::string>();
  if (sampler == "constant") {
    if (node["value"]) {
      return std::make_unique<ConstantSampler<T>>(node["value"].as<T>(), once);
    }
  }
  if (sampler == "sequence") {
    if (node["values"]) {
      Wrap wrap(Wrap::loop);
      if (node["wrap"]) {
        wrap = wrap_from_string(node["wrap"].as<std::string>());
      }
      return std::make_unique<SequenceSampler<T>>(
          node["values"].as<std::vector<T>>(), wrap, once);
    }
    return nullptr;
  }
  if (sampler == "choice") {
    if (node["values"]) {
      return std::make_unique<ChoiceSampler<T>>(
          node["values"].as<std::vector<T>>(), once);
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
              RegularSampler<T>::make_with_interval(start, end, number, wrap,
                                                    once));
        }
        if (node["step"]) {
          const auto step = node["step"].as<T>();
          std::optional<unsigned> number = std::nullopt;
          if (node["number"]) {
            number = node["number"].as<unsigned>();
          }
          return std::make_unique<RegularSampler<T>>(
              RegularSampler<T>::make_with_step(start, step, number, wrap,
                                                once));
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
        return std::make_unique<GridSampler>(start, end, numbers, wrap, once);
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
        return std::make_unique<UniformSampler<T>>(min, max, once);
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
        bool clamp = node["clamp"].as<bool>(true);
        const auto mean = node["mean"].as<ng_float_t>();
        const auto std_dev = node["std_dev"].as<ng_float_t>();
        return std::make_unique<NormalSampler<T>>(mean, std_dev, min, max, once,
                                                  clamp);
      }
      return nullptr;
    }
  }
  return nullptr;
}

template <typename T>
struct convert<ConstantSampler<T>> {
  static Node encode(const ConstantSampler<T>& rhs) {
    if (get_use_compact_samplers() && !rhs.once) {
      return Node(rhs.value);
    }
    Node node;
    node["sampler"] = "constant";
    node["value"] = rhs.value;
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
};

template <typename T>
struct convert<SequenceSampler<T>> {
  static Node encode(const SequenceSampler<T>& rhs) {
    if (get_use_compact_samplers() && !rhs.once && rhs.wrap == Wrap::loop) {
      return Node(rhs.values);
    }
    Node node;
    node["sampler"] = "sequence";
    node["values"] = rhs.values;
    node["wrap"] = wrap_to_string(rhs.wrap);
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
};

template <typename T>
struct convert<ChoiceSampler<T>> {
  static Node encode(const ChoiceSampler<T>& rhs) {
    Node node;
    node["sampler"] = "choice";
    node["values"] = rhs.values;
    if (rhs.once) {
      node["once"] = rhs.once;
    }
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
    if (rhs.once) {
      node["once"] = rhs.once;
    }
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
    if (rhs.once) {
      node["once"] = rhs.once;
    }
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
    if (rhs.once) {
      node["once"] = rhs.once;
    }
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
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    node["clamp"] = rhs.clamp;
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
  if (!node.IsMap()) {
    return false;
  }
  rhs->type = node["type"].as<std::string>("");
  const auto& properties = T::type_properties();
  if (!properties.count(rhs->type)) {
    std::cerr << "No type " << rhs->type << " in register "
              << get_type_name<T>() << std::endl;
    for (const auto& s : T::types()) {
      std::cout << s << std::endl;
    }
    return false;
  }
  rhs->node = node;
  for (const auto& [name, property] : properties.at(rhs->type)) {
    if (node[name]) {
      rhs->properties[name] = property_sampler(node[name], property);
    } else {
      for (const auto& alt_name : property.deprecated_names) {
        if (node[alt_name]) {
          std::cerr << "Property name " << alt_name << " is deprecated for "
                    << rhs->type << ", use " << name << " instead" << std::endl;
          rhs->properties[name] = property_sampler(node[alt_name], property);
        }
      }
    }
  }
  return true;
}

template <typename T>
Node encode_sr(const SamplerFromRegister<T>& rhs) {
  Node node;
  if (!T::has_type(rhs.type)) {
    return node;
  }
  if (rhs.node && rhs.node.IsMap()) {
    for (const auto& c : rhs.node) {
      node[c.first] = c.second;
    }
  }
  if (!rhs.type.empty()) {
    node["type"] = rhs.type;
  }
  for (const auto& [name, sampler] : rhs.properties) {
    if (sampler) {
      node[name] = sampler;
    }
  }
  return node;
}

template <typename T, typename M>
struct convert<BehaviorSampler<T, M>> {
  static Node encode(const BehaviorSampler<T, M>& rhs) {
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
    if (rhs.path_look_ahead) {
      node["path_look_ahead"] = rhs.path_look_ahead;
    }
    if (rhs.path_tau) {
      node["path_tau"] = rhs.path_tau;
    }
    if (rhs.heading) {
      node["heading"] = rhs.heading;
    }
    if (rhs.modulations.size()) {
      node["modulations"] = rhs.modulations;
    }
    return node;
  }
  static bool decode(const Node& node, BehaviorSampler<T, M>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r) return false;
    if (node["optimal_speed"]) {
      rhs.optimal_speed = read_sampler<ng_float_t>(node["optimal_speed"]);
    }
    if (node["optimal_angular_speed"]) {
      rhs.optimal_angular_speed =
          read_sampler<ng_float_t>(node["optimal_angular_speed"]);
    }
    if (node["rotation_tau"]) {
      rhs.rotation_tau = read_sampler<ng_float_t>(node["rotation_tau"]);
    }
    if (node["safety_margin"]) {
      rhs.safety_margin = read_sampler<ng_float_t>(node["safety_margin"]);
    }
    if (node["horizon"]) {
      rhs.horizon = read_sampler<ng_float_t>(node["horizon"]);
    }
    if (node["path_look_ahead"]) {
      rhs.path_look_ahead = read_sampler<ng_float_t>(node["path_look_ahead"]);
    }
    if (node["path_tau"]) {
      rhs.path_tau = read_sampler<ng_float_t>(node["path_tau"]);
    }
    if (node["heading"]) {
      rhs.heading = read_sampler<std::string>(node["heading"]);
    }
    if (node["modulations"]) {
      rhs.modulations =
          node["modulations"].as<std::vector<BehaviorModulationSampler<M>>>();
    }
    return true;
  }
};

template <typename T>
struct convert<BehaviorModulationSampler<T>> {
  static Node encode(const BehaviorModulationSampler<T>& rhs) {
    Node node = encode_sr<T>(rhs);
    if (rhs.enabled) {
      node["enabled"] = rhs.enabled;
    }
    return node;
  }
  static bool decode(const Node& node, BehaviorModulationSampler<T>& rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r) return false;
    if (node["enabled"]) {
      rhs.enabled = read_sampler<bool>(node["enabled"]);
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
      rhs.max_speed = read_sampler<ng_float_t>(node["max_speed"]);
    }
    if (node["max_angular_speed"]) {
      rhs.max_angular_speed =
          read_sampler<ng_float_t>(node["max_angular_speed"]);
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
  using M = typename W::A::M;
  using K = typename W::A::K;
  using T = typename W::A::T;
  using S = typename W::A::S;

  static Node encode(const AgentSampler<W>& rhs) {
    Node node;
    // if (!rhs.behavior.type.empty()) {
    node["behavior"] = rhs.behavior;
    // }
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
    if (rhs.color) {
      node["color"] = rhs.color;
    }
    if (rhs.tags) {
      node["tags"] = rhs.tags;
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
      rhs.behavior = node["behavior"].as<BehaviorSampler<B, M>>();
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
      rhs.orientation = read_sampler<ng_float_t>(node["orientation"]);
    }
    if (node["radius"]) {
      rhs.radius = read_sampler<ng_float_t>(node["radius"]);
    }
    if (node["control_period"]) {
      rhs.control_period = read_sampler<ng_float_t>(node["control_period"]);
    }
    if (node["number"]) {
      rhs.number = read_sampler<unsigned>(node["number"]);
    }
    if (node["type"]) {
      rhs.type = read_sampler<std::string>(node["type"]);
    }
    if (node["color"]) {
      rhs.color = read_sampler<std::string>(node["color"]);
    }
    if (node["id"]) {
      rhs.id = read_sampler<int>(node["id"]);
    }
    if (node["name"]) {
      rhs.name = node["name"].as<std::string>();
    }
    if (node["tags"]) {
      rhs.tags = read_sampler<std::vector<std::string>>(node["tags"]);
    }
    return true;
  }
};

}  // namespace YAML

#endif  // NAVGROUND_SIM_YAML_SAMPLER_H
