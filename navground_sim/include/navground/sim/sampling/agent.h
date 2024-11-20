/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_AGENT_H
#define NAVGROUND_SIM_AGENT_H

#include <iostream>
#include <memory>

#include "navground/core/types.h"
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
struct AgentSampler : public Sampler<typename W::A::C>, public Scenario::Group {
  /** @private */
  using A = typename W::A;
  /** @private */
  using C = typename A::C;
  /** @private */
  using B = typename A::B;
  /** @private */
  using M = typename A::M;
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
  explicit AgentSampler(const std::string &name = "")
      : Sampler<C>(), name(name), number{0} {}

  /**
   * @private
   */
  void add_to_world(World *world,
                    std::optional<unsigned> index = std::nullopt) override {
    reset(index);
    RandomGenerator &rg = world->get_random_generator();
    unsigned num = number->sample(rg);
    if (W *w = dynamic_cast<W *>(world)) {
      for (unsigned i = 0; i < num; ++i) {
        w->add_agent(sample(rg));
      }
    } else {
      std::cerr << "Trying to add agent sampler to wrong World type"
                << std::endl;
    }
  }

  /**
   * @private
   */
  void reset(std::optional<unsigned> index = std::nullopt) override {
    Sampler<C>::reset(index);
    behavior.reset(index);
    kinematics.reset(index);
    task.reset(index);
    state_estimation.reset(index);
    if (position)
      position->reset(index);
    if (orientation)
      orientation->reset(index);
    if (radius)
      radius->reset(index);
    if (control_period)
      control_period->reset(index);
    if (speed_tolerance)
      speed_tolerance->reset(index);
    if (id)
      id->reset(index);
    if (type)
      type->reset(index);
    if (color)
      color->reset(index);
    if (number)
      number->reset(index);
    if (tags)
      tags->reset(index);
  }

  std::string name;
  BehaviorSampler<B, M> behavior;
  KinematicsSampler<K> kinematics;
  TaskSampler<T> task;
  StateEstimationSampler<S> state_estimation;
  // SamplerFromRegister<T> task;
  // SamplerFromRegister<S> state_estimation;
  std::shared_ptr<Sampler<Vector2>> position;
  std::shared_ptr<Sampler<ng_float_t>> orientation;
  std::shared_ptr<Sampler<ng_float_t>> radius;
  std::shared_ptr<Sampler<ng_float_t>> control_period;
  std::shared_ptr<Sampler<ng_float_t>> speed_tolerance;
  std::shared_ptr<Sampler<int>> id;
  std::shared_ptr<Sampler<std::string>> type;
  std::shared_ptr<Sampler<std::string>> color;
  std::shared_ptr<Sampler<unsigned>> number;
  std::shared_ptr<Sampler<std::vector<std::string>>> tags;

protected:
  C s(RandomGenerator &rg) override {
    C c = A::make(radius ? radius->sample(rg) : 0, behavior.sample(rg),
                  kinematics.sample(rg), task.sample(rg),
                  state_estimation.sample(rg),
                  control_period ? control_period->sample(rg) : 0);
    A *agent = get<A, C>::ptr(c);
    if (position) {
      agent->pose.position = position->sample(rg);
    }
    if (orientation) {
      agent->pose.orientation = orientation->sample(rg);
    }
    if (type) {
      agent->type = type->sample(rg);
    }
    if (color) {
      agent->color = color->sample(rg);
    }
    if (tags) {
      const auto &values = tags->sample(rg);
      agent->tags = std::set<std::string>(values.begin(), values.end());
    }
    if (id) {
      agent->id = id->sample(rg);
    }
    if (!name.empty()) {
      agent->tags.insert(name);
    }
    if (speed_tolerance) {
      agent->get_controller()->set_speed_tolerance(speed_tolerance->sample(rg));
    }
    return c;
  }
};

} // namespace navground::sim

#endif // NAVGROUND_SIM_SAMPLER_H
