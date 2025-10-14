#ifndef NAVGROUND_SIM_YAML_SAMPLER_H
#define NAVGROUND_SIM_YAML_SAMPLER_H

#include <iostream>
#include <memory>

#include "navground/core/types.h"
#include "navground/core/yaml/property.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/export.h"
#include "navground/sim/sampling/agent.h"
#include "navground/sim/sampling/register.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

using navground::core::Property;
using navground::sim::AgentSampler;
using navground::sim::BehaviorModulationSampler;
using navground::sim::BehaviorSampler;
using navground::sim::BinarySampler;
using navground::sim::ChoiceSampler;
using navground::sim::ConstantSampler;
using navground::sim::GridSampler;
using navground::sim::is_algebra;
using navground::sim::is_number;
using navground::sim::KinematicsSampler;
using navground::sim::NormalSampler;
using navground::sim::NormalSampler2D;
using navground::sim::PermutationSampler;
using navground::sim::PropertySampler;
using navground::sim::RegularSampler;
using navground::sim::Sampler;
using navground::sim::SamplerFromRegister;
using navground::sim::SequenceSampler;
using navground::sim::StateEstimationSampler;
using navground::sim::TaskSampler;
using navground::sim::uniform_distribution;
using navground::sim::UniformSampler;
using navground::sim::UniformSizeSampler;
using navground::sim::World;
using navground::sim::Wrap;
using navground::sim::wrap_from_string;
using navground::sim::wrap_to_string;

namespace YAML {

namespace schema {

inline Node generic_type() {
  Node node;
  node["$dynamicRef"] = "#T";
  return node;
}

inline Node generic() {
  Node node;
  node["$defs"]["of"]["$dynamicAnchor"] = "T";
  node["$defs"]["of"]["not"] = true;
  return node;
}

template <typename T> Node sampler_for_type();

} // namespace schema

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
std::unique_ptr<Sampler<T>> read_sampler(const Node &node) {
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
          node["values"].as<std::vector<T>>(),
          node["probabilities"].as<std::vector<double>>(std::vector<double>()),
          once);
    }
    return nullptr;
  }
  if constexpr (is_std_vector_v<T>) {
    using S = typename T::value_type;
    if (sampler == "uniform_size") {
      if (node["max_size"] && node["value"]) {
        auto scalar_sampler = read_sampler<S>(node["value"]);
        if (scalar_sampler) {
          return std::make_unique<UniformSizeSampler<S>>(
              std::move(scalar_sampler), node["min_size"].as<size_t>(0),
              node["max_size"].as<size_t>(), once);
        }
      }
      return nullptr;
    }
    if (sampler == "permutation") {
      if (node["value"]) {
        return std::make_unique<PermutationSampler<S>>(
            node["value"].as<std::vector<S>>(), node["random"].as<bool>(true),
            node["forward"].as<bool>(true), once);
      }
      return nullptr;
    }
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
    if (sampler == "uniform") {
      if (node["from"] && node["to"]) {
        const auto min = node["from"].as<T>();
        const auto max = node["to"].as<T>();
        return std::make_unique<UniformSampler<T>>(min, max, once);
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
    if (sampler == "normal") {
      if (node["mean"] && node["std_dev"]) {
        const auto mean = node["mean"].as<Vector2>();
        const auto std_dev = node["std_dev"].as<std::array<ng_float_t, 2>>();
        const auto angle = node["angle"].as<ng_float_t>(0);
        return std::make_unique<NormalSampler2D>(mean, std_dev, angle, once);
      }
      return nullptr;
    }
  }
  if constexpr (std::is_convertible<bool, T>::value) {
    if (sampler == "binary") {
      return std::make_unique<BinarySampler<T>>(
          node["probability"].as<double>(0.5), once);
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

template <typename T> struct convert<ConstantSampler<T>> {
  static Node encode(const ConstantSampler<T> &rhs) {
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
  static Node schema() {
    Node node = schema::generic();
    Node extended;
    extended["type"] = "object";
    extended["properties"]["value"] = schema::generic_type();
    extended["properties"]["once"] = schema::type<bool>();
    extended["properties"]["sampler"]["const"] = "const";
    extended["additionalProperties"] = false;
    extended["required"] = std::vector<std::string>({"sampler", "value"});
    node["anyOf"].push_back(schema::generic_type());
    node["anyOf"].push_back(extended);
    return node;
  }
  static constexpr const char name[] = "const";
};

template <typename T> struct convert<SequenceSampler<T>> {
  static Node encode(const SequenceSampler<T> &rhs) {
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
  static Node schema() {
    Node node = schema::generic();
    Node compact;
    compact["type"] = "array";
    compact["items"] = schema::generic_type();
    compact["minItems"] = 1;
    Node extended;
    extended["type"] = "object";
    extended["properties"]["values"] = YAML::Clone(compact);
    extended["properties"]["once"] = schema::type<bool>();
    extended["properties"]["wrap"]["enum"] =
        std::vector<std::string>{"terminate", "repeat", "loop"};
    extended["properties"]["sampler"]["const"] = "sequence";
    extended["required"] = std::vector<std::string>({"sampler", "values"});
    extended["additionalProperties"] = false;
    node["anyOf"].push_back(compact);
    node["anyOf"].push_back(extended);
    return node;
  }
  static constexpr const char name[] = "sequence";
};

template <typename T> struct convert<BinarySampler<T>> {
  static Node encode(const BinarySampler<T> &rhs) {
    Node node;
    node["sampler"] = "binary";
    node["probability"] = rhs.get_probability();
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
  static Node schema() {
    Node node = schema::generic();
    node["type"] = "object";
    node["properties"]["probability"] = schema::type<schema::positive_float>();
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["sampler"]["const"] = "binary";
    node["required"] = std::vector<std::string>({"sampler"});
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "binary";
};

template <typename T> struct convert<ChoiceSampler<T>> {
  static Node encode(const ChoiceSampler<T> &rhs) {
    Node node;
    node["sampler"] = "choice";
    node["values"] = rhs._values;
    node["probabilities"] = rhs._probabilities;
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
  static Node schema() {
    Node node = schema::generic();
    Node values;
    values["type"] = "array";
    values["items"] = schema::generic_type();
    values["minItems"] = 1;
    Node probabilities;
    probabilities["type"] = "array";
    probabilities["items"] = schema::type<schema::positive_float>();
    node["type"] = "object";
    node["properties"]["values"] = values;
    node["properties"]["probabilities"] = probabilities;
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["sampler"]["const"] = "choice";
    node["required"] = std::vector<std::string>({"sampler", "values"});
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "choice";
};

template <typename T> struct convert<RegularSampler<T>> {
  static Node encode(const RegularSampler<T> &rhs) {
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
  static Node schema() {
    Node node = schema::generic();
    node["type"] = "object";
    node["properties"]["from"] = schema::generic_type();
    node["properties"]["to"] = schema::generic_type();
    node["properties"]["step"] = schema::generic_type();
    node["properties"]["number"] = schema::type<unsigned>();
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["wrap"]["enum"] =
        std::vector<std::string>{"terminate", "repeat", "loop"};
    node["properties"]["sampler"]["const"] = "regular";
    node["required"] = std::vector<std::string>({"sampler", "from", "to"});
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "regular";
};

template <> struct convert<GridSampler> {
  static Node encode(const GridSampler &rhs) {
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
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["from"] = schema::ref<Vector2>();
    node["properties"]["to"] = schema::ref<Vector2>();
    node["properties"]["step"] = schema::ref<Vector2>();
    node["properties"]["numbers"]["type"] = "array";
    node["properties"]["numbers"]["items"] = schema::type<unsigned>();
    node["properties"]["numbers"]["minItems"] = 2;
    node["properties"]["numbers"]["maxItems"] = 2;
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["wrap"]["enum"] =
        std::vector<std::string>{"terminate", "repeat", "loop"};
    node["properties"]["sampler"]["const"] = "grid";
    node["required"] =
        std::vector<std::string>({"sampler", "from", "to", "numbers"});
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "grid";
};

template <typename T> struct convert<UniformSampler<T>> {
  static Node encode(const UniformSampler<T> &rhs) {
    Node node;
    node["from"] = rhs.min;
    node["to"] = rhs.max;
    node["sampler"] = "uniform";
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
  static Node schema() {
    Node node = schema::generic();
    node["type"] = "object";
    node["properties"]["from"] = schema::generic_type();
    node["properties"]["to"] = schema::generic_type();
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["sampler"]["const"] = "uniform";
    node["required"] = std::vector<std::string>({"sampler", "from", "to"});
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "uniform";
};

template <typename T> struct convert<NormalSampler<T>> {
  static Node encode(const NormalSampler<T> &rhs) {
    Node node;
    if (rhs.min) {
      node["min"] = *rhs.min;
    }
    if (rhs.max) {
      node["max"] = *rhs.max;
    }
    node["mean"] = rhs.get_mean();
    node["std_dev"] = rhs.get_std_dev();
    node["sampler"] = "normal";
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    node["clamp"] = rhs.clamp;
    return node;
  }
  static Node schema() {
    Node node = schema::generic();
    node["type"] = "object";
    node["properties"]["min"] = schema::generic_type();
    node["properties"]["max"] = schema::generic_type();
    node["properties"]["mean"] = schema::type<ng_float_t>();
    node["properties"]["std_dev"] = schema::type<schema::positive_float>();
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["clamp"] = schema::type<bool>();
    node["properties"]["sampler"]["const"] = "normal";
    node["required"] = std::vector<std::string>({"sampler", "mean", "std_dev"});
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "normal";
};

template <> struct convert<NormalSampler2D> {
  static Node encode(const NormalSampler2D &rhs) {
    Node node;
    node["mean"] = rhs.get_mean();
    node["std_dev"] = rhs.get_std_dev();
    node["angle"] = rhs.get_angle();
    node["sampler"] = "normal";
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    Node std_dev;
    std_dev["type"] = "array";
    std_dev["items"] = schema::type<schema::positive_float>();
    std_dev["minItems"] = 2;
    std_dev["maxItems"] = 2;
    node["properties"]["mean"] = schema::ref<Vector2>();
    node["properties"]["std_dev"] = std_dev;
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["sampler"]["const"] = "normal";
    node["required"] = std::vector<std::string>({"sampler", "mean", "std_dev"});
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "normal2d";
};

inline std::unique_ptr<PropertySampler>
property_sampler(const Node &node, const Property::Field field) {
  return std::visit(
      [&node](auto &&arg) -> std::unique_ptr<PropertySampler> {
        using V = std::decay_t<decltype(arg)>;
        return std::make_unique<PropertySampler>(
            PropertySampler(read_sampler<V>(node)));
      },
      field);
}

inline std::unique_ptr<PropertySampler>
property_sampler(const Node &node, const Property &property) {
  return property_sampler(node, property.default_value);
}

inline std::unique_ptr<PropertySampler>
property_sampler(const Node &node, const std::string &type_name) {
  const auto field = Property::make_prototype(type_name);
  if (!field) {
    return nullptr;
  }
  return property_sampler(node, *field);
}

template <typename T> struct convert<Sampler<T> *> {
  static Node encode(const Sampler<T> *rhs) {
    if (!rhs)
      return Node();
    if constexpr (is_std_vector_v<T>) {
      using S = typename T::value_type;
      if (const UniformSizeSampler<S> *sampler =
              dynamic_cast<const UniformSizeSampler<S> *>(rhs)) {
        return Node(*sampler);
      }
      if (const PermutationSampler<S> *sampler =
              dynamic_cast<const PermutationSampler<S> *>(rhs)) {
        return Node(*sampler);
      }
    }
    if (const ConstantSampler<T> *sampler =
            dynamic_cast<const ConstantSampler<T> *>(rhs)) {
      return Node(*sampler);
    }
    if (const SequenceSampler<T> *sampler =
            dynamic_cast<const SequenceSampler<T> *>(rhs)) {
      return Node(*sampler);
    }
    if (const ChoiceSampler<T> *sampler =
            dynamic_cast<const ChoiceSampler<T> *>(rhs)) {
      return Node(*sampler);
    }
    if constexpr (std::is_same_v<T, Vector2>) {
      if (const GridSampler *sampler = dynamic_cast<const GridSampler *>(rhs)) {
        return Node(*sampler);
      }
      if (const NormalSampler2D *sampler =
              dynamic_cast<const NormalSampler2D *>(rhs)) {
        return Node(*sampler);
      }
    }
    if constexpr (std::is_convertible<bool, T>::value) {
      if (const BinarySampler<T> *sampler =
              dynamic_cast<const BinarySampler<T> *>(rhs)) {
        return Node(*sampler);
      }
    }
    if constexpr (is_algebra<T>) {
      if (const UniformSampler<T> *sampler =
              dynamic_cast<const UniformSampler<T> *>(rhs)) {
        return Node(*sampler);
      }
      if (const RegularSampler<T> *sampler =
              dynamic_cast<const RegularSampler<T> *>(rhs)) {
        return Node(*sampler);
      }
    }
    if constexpr (is_number<T>) {
      if (const NormalSampler<T> *sampler =
              dynamic_cast<const NormalSampler<T> *>(rhs)) {
        return Node(*sampler);
      }
    }
    return Node();
  }
};

template <typename T> struct convert<UniformSizeSampler<T>> {
  static Node encode(const UniformSizeSampler<T> &rhs) {
    Node node;
    node["sampler"] = "uniform_size";
    node["value"] = rhs.get_scalar_sampler();
    node["min_size"] = rhs.get_min_size();
    node["max_size"] = rhs.get_max_size();
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
  static Node schema() {
    // Too complex to be represented in JSON schema:
    // we won't be able to check that the scalar sampler is valid!
    Node node;
    node["type"] = "object";
    Node value;
    value["type"].push_back("number");
    value["type"].push_back("boolean");
    value["type"].push_back("string");
    value["type"].push_back("array");
    value["type"].push_back("object");
    node["properties"]["sampler"]["const"] = "uniform_size";
    node["properties"]["value"] = value;
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["max_size"] = schema::type<unsigned>();
    node["properties"]["min_size"] = schema::type<unsigned>();
    node["additionalProperties"] = false;
    node["required"] =
        std::vector<std::string>({"sampler", "max_size", "value"});
    return node;
  }
  static constexpr const char name[] = "uniform_size";
};

template <typename T> struct convert<PermutationSampler<T>> {
  static Node encode(const PermutationSampler<T> &rhs) {
    Node node;
    node["sampler"] = "permutation";
    node["value"] = rhs.get_values();
    node["random"] = rhs.get_random();
    node["forward"] = rhs.get_forward();
    if (rhs.once) {
      node["once"] = rhs.once;
    }
    return node;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    Node value;
    value["type"] = "array";
    // values["items"] = schema::generic_type();
    value["minItems"] = 1;
    node["properties"]["sampler"]["const"] = "permutation";
    node["properties"]["value"] = value;
    node["properties"]["once"] = schema::type<bool>();
    node["properties"]["random"] = schema::type<bool>();
    node["properties"]["forward"] = schema::type<bool>();
    node["additionalProperties"] = false;
    node["required"] = std::vector<std::string>({"sampler", "value"});
    return node;
  }
  static constexpr const char name[] = "permutation";
};

namespace schema {

#if 0
template <typename T> struct type_t<Sampler<T>> {
  static std::string name() { return type_t<T>::name() + "_sampler"; }
  static Node schema() {
    Node node;
    node["anyOf"] =
        std::vector<Node>{ref<ConstantSampler<T>>(), ref<SequenceSampler<T>>(),
                          ref<ChoiceSampler<T>>()};
    if constexpr (std::is_same_v<T, Vector2>) {
      node["anyOf"].push_back(ref<GridSampler>());
    }
    if constexpr (is_algebra<T>) {
      node["anyOf"].push_back(ref<RegularSampler<T>>());
    }
    if constexpr (is_number<T>) {
      node["anyOf"].push_back(ref<UniformSampler<T>>());
      node["anyOf"].push_back(ref<NormalSampler<T>>());
    }
    node["$defs"]["T"] = type<T>();
    node["$defs"]["T"]["$dynamicAnchor"] = "T";
    return node;
  }
};
#endif

struct StringSampler;

template <> struct type_t<StringSampler> {
  static std::string name() { return "string_sampler"; }
  static Node schema() {
    Node node;
    node["anyOf"] = std::vector<Node>{ref<ConstantSampler<void>>(),
                                      ref<SequenceSampler<void>>(),
                                      ref<ChoiceSampler<void>>()};
    return node;
  }
};

struct NumberSampler;

template <> struct type_t<NumberSampler> {
  static std::string name() { return "number_sampler"; }
  static Node schema() {
    Node node;
    node["anyOf"] = std::vector<Node>{
        ref<ConstantSampler<void>>(), ref<SequenceSampler<void>>(),
        ref<ChoiceSampler<void>>(),   ref<RegularSampler<void>>(),
        ref<UniformSampler<void>>(),  ref<NormalSampler<void>>(),
        ref<BinarySampler<void>>()};
    return node;
  }
};

struct BooleanSampler;

template <> struct type_t<BooleanSampler> {
  static std::string name() { return "boolean_sampler"; }
  static Node schema() {
    Node node;
    node["anyOf"] = std::vector<Node>{
        ref<ConstantSampler<void>>(), ref<SequenceSampler<void>>(),
        ref<ChoiceSampler<void>>(), ref<BinarySampler<void>>()};
    return node;
  }
};

struct Vector2Sampler;

template <> struct type_t<Vector2Sampler> {
  static std::string name() { return "vector2_sampler"; }
  static Node schema() {
    Node node;
    node["anyOf"] = std::vector<Node>{
        ref<ConstantSampler<void>>(), ref<SequenceSampler<void>>(),
        ref<ChoiceSampler<void>>(),   ref<GridSampler>(),
        ref<RegularSampler<void>>(),  ref<UniformSampler<void>>(),
        ref<NormalSampler2D>()};
    return node;
  }
};

struct ListSampler;

template <> struct type_t<ListSampler> {
  static std::string name() { return "list_sampler"; }
  static Node schema() {
    Node node;
    node["anyOf"] = std::vector<Node>{
        ref<ConstantSampler<void>>(), ref<SequenceSampler<void>>(),
        ref<ChoiceSampler<void>>(), ref<UniformSizeSampler<void>>(),
        ref<PermutationSampler<void>>()};
    return node;
  }
};

template <typename T> inline Node sampler_for_type() {
  if constexpr (std::is_same_v<T, Vector2>) {
    return ref<Vector2Sampler>();
  }
  if constexpr (is_number<T>) {
    return ref<NumberSampler>();
  }
  if constexpr (std::is_same_v<T, std::string>) {
    return ref<StringSampler>();
  }
  if constexpr (std::is_same_v<T, bool>) {
    return ref<BooleanSampler>();
  }
  if constexpr (is_std_vector_v<T>) {
    return ref<ListSampler>();
  }
  return Node();
}

template <typename T>
inline Node sampler(const Node &sample, const std::string &id) {
  Node node = sampler_for_type<T>();
  node["$id"] = id;
  node["$defs"]["of"] = sample;
  node["$defs"]["of"]["$dynamicAnchor"] = "T";
  return node;
}

} // namespace schema

template <typename T> struct convert<std::shared_ptr<Sampler<T>>> {
  static Node encode(const std::shared_ptr<Sampler<T>> &rhs) {
    return convert<Sampler<T> *>::encode(rhs.get());
  }
};

template <> struct convert<PropertySampler> {
  static Node encode(const PropertySampler &rhs) {
    return std::visit([](auto &&arg) { return Node(arg.get()); }, rhs.sampler);
  }
};

template <> struct convert<std::shared_ptr<PropertySampler>> {
  static Node encode(const std::shared_ptr<PropertySampler> &rhs) {
    return std::visit([](auto &&arg) { return Node(arg.get()); }, rhs->sampler);
  }
};

template <typename T>
bool decode_sr(const Node &node, SamplerFromRegister<T> *rhs) {
  if (!node.IsMap()) {
    return false;
  }
  rhs->type = node["type"].as<std::string>("");
  const auto &properties = T::type_properties();
  if (!properties.count(rhs->type)) {
    std::cerr << "No type " << rhs->type << " in register "
              << get_type_name<T>() << std::endl;
    for (const auto &s : T::types()) {
      std::cout << s << std::endl;
    }
    return false;
  }
  rhs->node = node;
  for (const auto &[name, property] : properties.at(rhs->type)) {
    if (node[name]) {
      rhs->properties[name] = property_sampler(node[name], property);
    } else {
      for (const auto &alt_name : property.deprecated_names) {
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

template <typename T> Node encode_sr(const SamplerFromRegister<T> &rhs) {
  Node node;
  if (!T::has_type(rhs.type)) {
    return node;
  }
  if (rhs.node && rhs.node.IsMap()) {
    for (const auto &c : rhs.node) {
      node[c.first] = c.second;
    }
  }
  // if (!rhs.type.empty()) {
  node["type"] = rhs.type;
  // }
  for (const auto &[name, sampler] : rhs.properties) {
    if (sampler) {
      node[name] = sampler;
    }
  }
  return node;
}

namespace schema {
// TODO(Jerome): add support for modifiers?

template <typename T>
inline void add_sampler(Node &node, const std::string &key) {
  Node sample;
  if constexpr (std::is_same_v<T, Vector2>) {
    sample = ref<T>();
  } else {
    sample = type<T>();
  }
  node["properties"][key] = sampler<T>(sample, key);
}

} // namespace schema

template <typename T, typename M> struct convert<BehaviorSampler<T, M>> {
  static Node encode(const BehaviorSampler<T, M> &rhs) {
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
  static bool decode(const Node &node, BehaviorSampler<T, M> &rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r)
      return false;
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
  // TODO(Jerome): If possible, add constraints at least to the generic
  // sampler, e.g., to define a sampler of positive integers. I could do it
  // brute force just for positive. I think it is possible but I need to
  // reverse samplers: now: field: {$ref: <type>_sampler} then: field: {$ref:
  // sampler, "T=<type>"}
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["type"] = schema::type<std::string>();
    schema::add_sampler<schema::positive_float>(node, "optimal_speed");
    schema::add_sampler<schema::positive_float>(node, "optimal_angular_speed");
    schema::add_sampler<schema::positive_float>(node, "rotation_tau");
    schema::add_sampler<schema::positive_float>(node, "safety_margin");
    schema::add_sampler<schema::positive_float>(node, "horizon");
    schema::add_sampler<schema::positive_float>(node, "path_look_ahead");
    schema::add_sampler<schema::positive_float>(node, "path_tau");
    schema::add_sampler<schema::positive_float>(node, "optimal_speed");
    schema::add_sampler<Behavior::Heading>(node, "heading");
    node["properties"]["modulations"]["type"] = "array";
    node["properties"]["modulations"]["items"] =
        schema::ref<BehaviorModulationSampler<M>>();
    return node;
  }
  static constexpr const char name[] = "behavior_sampler";
};

template <typename T> struct convert<BehaviorModulationSampler<T>> {
  static Node encode(const BehaviorModulationSampler<T> &rhs) {
    Node node = encode_sr<T>(rhs);
    if (rhs.enabled) {
      node["enabled"] = rhs.enabled;
    }
    return node;
  }
  static bool decode(const Node &node, BehaviorModulationSampler<T> &rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r)
      return false;
    if (node["enabled"]) {
      rhs.enabled = read_sampler<bool>(node["enabled"]);
    }
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["type"] = schema::type<std::string>();
    schema::add_sampler<bool>(node, "enabled");
    return node;
  }
  static constexpr const char name[] = "behavior_modulation_sampler";
};

template <typename T> struct convert<KinematicsSampler<T>> {
  static Node encode(const KinematicsSampler<T> &rhs) {
    Node node = encode_sr<T>(rhs);
    if (rhs.max_speed) {
      node["max_speed"] = rhs.max_speed;
    }
    if (rhs.max_angular_speed) {
      node["max_angular_speed"] = rhs.max_angular_speed;
    }
    return node;
  }
  static bool decode(const Node &node, KinematicsSampler<T> &rhs) {
    bool r = decode_sr<T>(node, &rhs);
    if (!r)
      return false;
    if (node["max_speed"]) {
      rhs.max_speed = read_sampler<ng_float_t>(node["max_speed"]);
    }
    if (node["max_angular_speed"]) {
      rhs.max_angular_speed =
          read_sampler<ng_float_t>(node["max_angular_speed"]);
    }
    return true;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["type"] = schema::type<std::string>();
    schema::add_sampler<ng_float_t>(node, "max_speed");
    schema::add_sampler<ng_float_t>(node, "max_angular_speed");
    return node;
  }
  static constexpr const char name[] = "kinematics_sampler";
};

// template <typename T> struct convert<SamplerFromRegister<T>> {
//   static Node encode(const SamplerFromRegister<T> &rhs) {
//     return encode_sr<T>(rhs);
//   }
//   static bool decode(const Node &node, SamplerFromRegister<T> &rhs) {
//     bool r = decode_sr<T>(node, &rhs);
//     return r;
//   }
//   static Node schema() {
//     Node node;
//     node["type"] = "object";
//     return node;
//   }
// };

template <typename T> struct convert<TaskSampler<T>> {
  static Node encode(const TaskSampler<T> &rhs) { return encode_sr<T>(rhs); }
  static bool decode(const Node &node, TaskSampler<T> &rhs) {
    bool r = decode_sr<T>(node, &rhs);
    return r;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    return node;
  }
  static constexpr const char name[] = "task_sampler";
};

template <typename T> struct convert<StateEstimationSampler<T>> {
  static Node encode(const StateEstimationSampler<T> &rhs) {
    return encode_sr<T>(rhs);
  }
  static bool decode(const Node &node, StateEstimationSampler<T> &rhs) {
    bool r = decode_sr<T>(node, &rhs);
    return r;
  }
  static Node schema() {
    Node node;
    node["type"] = "object";
    return node;
  }
  static constexpr const char name[] = "state_estimation_sampler";
};

template <typename W> struct convert<AgentSampler<W>> {
  using B = typename W::A::B;
  using M = typename W::A::M;
  using K = typename W::A::K;
  using T = typename W::A::T;
  using S = typename W::A::S;

  static Node encode(const AgentSampler<W> &rhs) {
    Node node;
    if (rhs.behavior.is_valid()) {
      node["behavior"] = rhs.behavior;
    }
    if (rhs.kinematics.is_valid()) {
      node["kinematics"] = rhs.kinematics;
    }
    if (rhs.task.is_valid()) {
      node["task"] = rhs.task;
    }
    const auto &ses = rhs.get_valid_state_estimations();
    if (ses.size() == 1) {
      node["state_estimation"] = ses[0];
    } else if (ses.size() > 1) {
      node["state_estimations"] = ses;
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
    if (rhs.speed_tolerance) {
      node["speed_tolerance"] = rhs.speed_tolerance;
    }
    if (rhs.angular_speed_tolerance) {
      node["angular_speed_tolerance"] = rhs.angular_speed_tolerance;
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
  static bool decode(const Node &node, AgentSampler<W> &rhs) {
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
      rhs.state_estimations = {
          node["state_estimation"].as<StateEstimationSampler<S>>()};
    }
    if (node["state_estimations"]) {
      rhs.state_estimations = node["state_estimations"]
                                  .as<std::vector<StateEstimationSampler<S>>>();
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
    if (node["speed_tolerance"]) {
      rhs.speed_tolerance = read_sampler<ng_float_t>(node["speed_tolerance"]);
    }
    if (node["angular_speed_tolerance"]) {
      rhs.angular_speed_tolerance =
          read_sampler<ng_float_t>(node["angular_speed_tolerance"]);
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
  static Node schema() {
    Node node;
    node["type"] = "object";
    node["properties"]["behavior"] = schema::ref<BehaviorSampler<B>>();
    node["properties"]["kinematics"] = schema::ref<KinematicsSampler<K>>();
    node["properties"]["state_estimation"] =
        schema::ref<StateEstimationSampler<S>>();
    node["properties"]["state_estimations"]["type"] = "array";
    node["properties"]["state_estimations"]["items"] =
        schema::ref<StateEstimationSampler<S>>();
    node["properties"]["task"] = schema::ref<TaskSampler<T>>();
    schema::add_sampler<Vector2>(node, "position");
    schema::add_sampler<ng_float_t>(node, "orientation");
    schema::add_sampler<ng_float_t>(node, "orientation");
    schema::add_sampler<schema::positive_float>(node, "radius");
    schema::add_sampler<schema::positive_float>(node, "control_period");
    schema::add_sampler<schema::positive_float>(node, "speed_tolerance");
    schema::add_sampler<schema::positive_float>(node,
                                                "angular_speed_tolerance");
    schema::add_sampler<unsigned>(node, "number");
    schema::add_sampler<std::string>(node, "type");
    schema::add_sampler<std::string>(node, "tags");
    schema::add_sampler<std::vector<std::string>>(node, "color");
    schema::add_sampler<unsigned>(node, "id");
    schema::add_sampler<std::string>(node, "name");
    node["additionalProperties"] = false;
    return node;
  }
  static constexpr const char name[] = "group";
};

} // namespace YAML

#endif // NAVGROUND_SIM_YAML_SAMPLER_H
