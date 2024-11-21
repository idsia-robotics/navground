/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_REGISTER_H
#define NAVGROUND_SIM_REGISTER_H

#include <iostream>
#include <memory>
#include <random>

#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"
#include "navground/core/types.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/world.h"

using navground::core::Behavior;
using navground::core::BehaviorModulation;
using navground::core::Kinematics;

namespace navground::sim {

template <typename T, typename C>
struct get {
  static T* ptr(const C&);
};

template <typename T>
struct get<T, std::shared_ptr<T>> {
  static T* ptr(const std::shared_ptr<T>& c) { return c.get(); }
};

/**
 * @brief      An inexhaustible sampler of objects from a class that has
 * register
 *
 * It creates objects of the sub-class identified by \ref type.
 *
 * The objects properties are sampled using the properties samplers stored in
 * \ref properties. Property that are not sampled, are assigned to their default
 * value.
 *
 * The created objects are wrapped in a container, which by default is a shared
 * pointer.
 *
 * @tparam     T     The type of the root class
 */
template <typename T>
struct SamplerFromRegister : public Sampler<typename T::C> {
  using C = typename T::C;
  using Type = T;

  bool is_valid() const {
    return T::has_type(type);
  }

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  type  The registered type name
   */
  explicit SamplerFromRegister(const std::string& type = "")
      : Sampler<C>(), type(type), properties() {}

  /**
   * @private
   */
  void reset(std::optional<unsigned> index = std::nullopt) override {
    Sampler<C>::reset(index);
    for (auto& [k, v] : properties) {
      if (v) v->reset(index);
    }
  }

  /**
   * The registered name of the sub-class to be sampled
   */
  std::string type;
  /**
   * A map of property samplers ``name -> sampler``
   * used configure the sampled object.
   */
  std::map<std::string, std::shared_ptr<PropertySampler>> properties;

  YAML::Node node;

 protected:
  typename T::C s(RandomGenerator& rg) override {
    C c = T::make_type(type);
    auto t = get<T, C>::ptr(c);
    if (!t) {
      // std::cerr << "Unknown type " << type << std::endl;
      return c;
    }
    for (const auto& [name, property] : properties) {
      if (property) {
        auto value = property->sample(rg);
        t->set(name, value);
      }
    }
    if (node) {
      t->decode(node);
    }
    return c;
  }
};

/**
 *  Samples \ref navground::core::BehaviorModulation
 *
 * @tparam     T     The type of the root class.
 * Used to generalize from C++ to Python.
 */
template <typename T = BehaviorModulation>
struct BehaviorModulationSampler : public SamplerFromRegister<T> {
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  type  The registered type name
   */
  explicit BehaviorModulationSampler(const std::string& type = "")
      : SamplerFromRegister<T>(type) {}

  /**
   * @private
   */
  using C = typename T::C;

  /**
   * @private
   */
  void reset(std::optional<unsigned> index = std::nullopt) override {
    SamplerFromRegister<T>::reset(index);
    if (enabled) enabled->reset(index);
  }

  std::shared_ptr<Sampler<bool>> enabled;

 protected:
  C s(RandomGenerator& rg) override {
    C c = SamplerFromRegister<T>::s(rg);
    auto modulation = get<T, C>::ptr(c);
    if (!modulation) return c;
    if (enabled) {
      modulation->set_enabled(enabled->sample(rg));
    }
    return c;
  }
};

template <typename CB, typename CM>
struct add_modulation {
  static void call(CB behavior, CM& modulation);
};

template <>
struct add_modulation<std::shared_ptr<Behavior>,
                      std::shared_ptr<BehaviorModulation>> {
  static void call(std::shared_ptr<Behavior> behavior,
                   std::shared_ptr<BehaviorModulation>& modulation) {
    // printf("Add modulation to behavior\n");
    behavior->add_modulation(modulation);
    // printf("-> %zu\n", behavior->get_modulations().size());
  }
};

/**
 * @brief      Samples \ref navground::core::Behavior
 *
 * @tparam     T     The type of the behavior root class.
 * Used to generalize from C++ to Python.
 *
 * Defines the same fields as \ref navground::core::Behavior
 * but as sampler of the respective type.
 */
template <typename T = Behavior, typename M = BehaviorModulation>
struct BehaviorSampler : public SamplerFromRegister<T> {
  /**
   * @private
   */
  using C = typename T::C;
  using Type = T;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  type  The registered type name
   */
  explicit BehaviorSampler(const std::string& type = "")
      : SamplerFromRegister<T>(type), modulations() {}

 protected:
  C s(RandomGenerator& rg) override {
    C c = SamplerFromRegister<T>::s(rg);
    auto behavior = get<T, C>::ptr(c);
    if (!behavior) return c;
    if (optimal_speed) {
      behavior->set_optimal_speed(optimal_speed->sample(rg));
    }
    if (optimal_angular_speed) {
      behavior->set_optimal_angular_speed(optimal_angular_speed->sample(rg));
    }
    if (horizon) {
      behavior->set_horizon(horizon->sample(rg));
    }
    if (path_look_ahead) {
      behavior->set_path_look_ahead(path_look_ahead->sample(rg));
    }
    if (path_tau) {
      behavior->set_path_tau(path_tau->sample(rg));
    }
    if (rotation_tau) {
      behavior->set_rotation_tau(rotation_tau->sample(rg));
    }
    if (safety_margin) {
      behavior->set_safety_margin(safety_margin->sample(rg));
    }
    if (heading) {
      behavior->set_heading_behavior(
          Behavior::heading_from_string(heading->sample(rg)));
    }
    for (auto& mod : modulations) {
      typename M::C value = mod.sample(rg);
      // printf("Sampled a modulation\n");
      add_modulation<typename T::C, typename M::C>::call(c, value);
      // behavior->add_modulation(mod.sample(rg));
    }
    return c;
  }

 public:
  /**
   * @private
   */
  void reset(std::optional<unsigned> index = std::nullopt) override {
    SamplerFromRegister<T>::reset(index);
    if (optimal_speed) optimal_speed->reset(index);
    if (optimal_angular_speed) optimal_angular_speed->reset(index);
    if (rotation_tau) rotation_tau->reset(index);
    if (safety_margin) safety_margin->reset(index);
    if (horizon) horizon->reset(index);
    if (heading) heading->reset(index);
    if (path_tau) path_tau->reset(index);
    if (path_look_ahead) path_look_ahead->reset(index);
  }

  std::shared_ptr<Sampler<ng_float_t>> optimal_speed;
  std::shared_ptr<Sampler<ng_float_t>> optimal_angular_speed;
  std::shared_ptr<Sampler<ng_float_t>> rotation_tau;
  std::shared_ptr<Sampler<ng_float_t>> safety_margin;
  std::shared_ptr<Sampler<ng_float_t>> horizon;
  std::shared_ptr<Sampler<ng_float_t>> path_tau;
  std::shared_ptr<Sampler<ng_float_t>> path_look_ahead;
  std::shared_ptr<Sampler<std::string>> heading;
  std::vector<BehaviorModulationSampler<M>> modulations;
};

/**
 * @brief      Samples \ref navground::core::Kinematics
 *
 * @tparam     T     The type of the behavior root class.
 * Used to generalize from C++ to Python.
 *
 * Defines the same fields as \ref navground::core::Kinematics
 * but as sampler of the respective type.
 */
template <typename T = Kinematics>
struct KinematicsSampler : public SamplerFromRegister<T> {

  using Type = T;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  type  The registered type name
   */
  explicit KinematicsSampler(const std::string& type = "")
      : SamplerFromRegister<T>(type) {}

  /**
   * @private
   */
  using C = typename T::C;

  /**
   * @private
   */
  void reset(std::optional<unsigned> index = std::nullopt) override {
    SamplerFromRegister<T>::reset(index);
    if (max_speed) max_speed->reset(index);
    if (max_angular_speed) max_angular_speed->reset(index);
  }

  std::shared_ptr<Sampler<ng_float_t>> max_speed;
  std::shared_ptr<Sampler<ng_float_t>> max_angular_speed;

 protected:
  C s(RandomGenerator& rg) override {
    C c = SamplerFromRegister<T>::s(rg);
    auto kinematics = get<T, C>::ptr(c);
    if (!kinematics) return c;
    if (max_speed) {
      kinematics->set_max_speed(max_speed->sample(rg));
    }
    if (max_angular_speed) {
      kinematics->set_max_angular_speed(max_angular_speed->sample(rg));
    }
    return c;
  }
};

/**
 *  Samples \ref Task
 *
 * @tparam     T     The type of the root class.
 * Used to generalize from C++ to Python.
 */
template <typename T = Task>
struct TaskSampler : public SamplerFromRegister<T> {
  using Type = T;
  using SamplerFromRegister<T>::SamplerFromRegister;
};

/**
 *  Samples \ref StateEstimation
 *
 * @tparam     T     The type of the root class.
 * Used to generalize from C++ to Python.
 */
template <typename T = StateEstimation>
struct StateEstimationSampler : public SamplerFromRegister<T> {
  using Type = T;
  using SamplerFromRegister<T>::SamplerFromRegister;
};

}  // namespace navground::sim

#endif  // NAVGROUND_SIM_REGISTER_H
