/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_AGENT_H
#define NAVGROUND_SIM_AGENT_H

#include <iostream>
#include <memory>

#include "navground/sim/sampling/register.h"
#include "navground/sim/scenario.h"
#include "navground/sim/world.h"

namespace navground::sim {

/**
 * @brief      Implement an agent sample.
 *
 * Defines the same fields as \ref Agent but as sampler of the respective type.
 *
 * @tparam     W     The world type that the agents belong too. Used
 * to generalize from C++ to Python.
 */
template <typename W = World>
struct AgentSampler : virtual public Sampler<typename W::A::C>,
                      public Scenario::Group {
  /** @private */
  using A = typename W::A;
  /** @private */
  using C = typename A::C;
  /** @private */
  using B = typename A::B;
  /** @private */
  using K = typename A::K;
  /** @private */
  using T = typename A::T;
  /** @private */
  using S = typename A::S;
  /** @private */
  using Sampler<C>::sample;

  /**
   * @brief      Constructs a new instance.
   */
  explicit AgentSampler(const std::string& name = "")
      : Sampler<C>(), name(name), number{0} {}

  /**
   * @private
   */
  void add_to_world(World* world) override {
    if (W* w = dynamic_cast<W*>(world)) {
      for (unsigned i = 0; i < number; ++i) {
        w->add_agent(sample());
      }
    } else {
      std::cerr << "Trying to add agent sampler to wrong World type"
                << std::endl;
    }
  }

  /**
   * @private
   */
  void reset() override {
    Sampler<C>::reset();
    behavior.reset();
    kinematics.reset();
    task.reset();
    state_estimation.reset();
    if (position) position->reset();
    if (orientation) orientation->reset();
    if (radius) radius->reset();
    if (control_period) control_period->reset();
    if (id) id->reset();
    if (type) type->reset();
  }

  std::string name;
  BehaviorSampler<B> behavior;
  KinematicsSampler<K> kinematics;
  SamplerFromRegister<T> task;
  SamplerFromRegister<S> state_estimation;
  std::shared_ptr<Sampler<Vector2>> position;
  std::shared_ptr<Sampler<float>> orientation;
  std::shared_ptr<Sampler<float>> radius;
  std::shared_ptr<Sampler<float>> control_period;
  std::shared_ptr<Sampler<int>> id;
  std::shared_ptr<Sampler<std::string>> type;
  std::shared_ptr<Sampler<std::string>> color;
  unsigned number;

 protected:
  C s() override {
    C c = A::make(radius ? radius->sample() : 0.0f, behavior.sample(),
                  kinematics.sample(), task.sample(), state_estimation.sample(),
                  control_period ? control_period->sample() : 0.0f);
    A* agent = get<A, C>::ptr(c);
    if (position) {
      agent->pose.position = position->sample();
    }
    if (orientation) {
      agent->pose.orientation = orientation->sample();
    }
    if (type) {
      agent->type = type->sample();
    }
    if (color) {
      agent->color = color->sample();
    }
    if (id) {
      agent->id = id->sample();
    }
    if (!name.empty()) {
      agent->tags.insert(name);
    }
    return c;
  }
};

}  // namespace navground::sim

#endif  // NAVGROUND_SIM_SAMPLER_H
