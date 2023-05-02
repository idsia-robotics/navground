/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_REGISTER_H
#define NAVGROUND_SIM_REGISTER_H

#include <iostream>
#include <memory>
#include <random>

#include "navground/core/behavior.h"
#include "navground/core/kinematics.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/world.h"
#include "navground/core/behaviors/HL.h"

using navground::core::Behavior;
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
  void reset() override {
    Sampler<C>::reset();
    for (auto& [k, v] : properties) {
      if (v) v->reset();
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

 protected:
  typename T::C s() override {
    C c = T::make_type(type);
    T* t = get<T, C>::ptr(c);
    if (!t) {
      // std::cerr << "Unknown type " << type << std::endl;
      return c;
    }
    for (const auto& [name, property] : properties) {
      if (property) {
        auto value = property->sample();
        t->set(name, value);
      }
    }
    return c;
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
template <typename T = Behavior>
struct BehaviorSampler : public SamplerFromRegister<T> {
  /**
   * @private
   */
  using C = typename T::C;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  type  The registered type name
   */
  explicit BehaviorSampler(const std::string& type = "")
      : SamplerFromRegister<T>(type) {}

 protected:
  C s() override {
    C c = SamplerFromRegister<T>::s();
    T* behavior = get<T, C>::ptr(c);
    if (!behavior) return c;
    if (optimal_speed) {
      behavior->set_optimal_speed(optimal_speed->sample());
    }
    if (optimal_angular_speed) {
      behavior->set_optimal_angular_speed(optimal_angular_speed->sample());
    }
    if (horizon) {
      behavior->set_horizon(horizon->sample());
    }
    if (rotation_tau) {
      behavior->set_rotation_tau(rotation_tau->sample());
    }
    if (safety_margin) {
      behavior->set_safety_margin(safety_margin->sample());
    }
    if (heading) {
      behavior->set_heading_behavior(
          Behavior::heading_from_string(heading->sample()));
    }
    return c;
  }

 public:
  /**
   * @private
   */
  void reset() override {
    SamplerFromRegister<T>::reset();
    if (optimal_speed) optimal_speed->reset();
    if (optimal_angular_speed) optimal_angular_speed->reset();
    if (rotation_tau) rotation_tau->reset();
    if (safety_margin) safety_margin->reset();
    if (horizon) horizon->reset();
    if (heading) heading->reset();
  }

  std::shared_ptr<Sampler<float>> optimal_speed;
  std::shared_ptr<Sampler<float>> optimal_angular_speed;
  std::shared_ptr<Sampler<float>> rotation_tau;
  std::shared_ptr<Sampler<float>> safety_margin;
  std::shared_ptr<Sampler<float>> horizon;
  std::shared_ptr<Sampler<std::string>> heading;
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
  void reset() override {
    SamplerFromRegister<T>::reset();
    if (max_speed) max_speed->reset();
    if (max_angular_speed) max_angular_speed->reset();
  }

  std::shared_ptr<Sampler<float>> max_speed;
  std::shared_ptr<Sampler<float>> max_angular_speed;

 protected:
  C s() override {
    C c = SamplerFromRegister<T>::s();
    T* kinematics = get<T, C>::ptr(c);
    if (!kinematics) return c;
    if (max_speed) {
      kinematics->set_max_speed(max_speed->sample());
    }
    if (max_angular_speed) {
      kinematics->set_max_angular_speed(max_angular_speed->sample());
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
using TaskSampler = SamplerFromRegister<T>;
/**
 *  Samples \ref StateEstimation
 *
 * @tparam     T     The type of the root class.
 * Used to generalize from C++ to Python.
 */
template <typename T = StateEstimation>
using StateEstimationSampler = SamplerFromRegister<T>;

}  // namespace navground::sim

#endif  // NAVGROUND_SIM_REGISTER_H
