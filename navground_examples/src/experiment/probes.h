/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *
 * An example of a custom probe
 */

#ifndef NAVGROUND_CORE_EXAMPLES_EXPERIMENT_CUSTOM_RECORD_H
#define NAVGROUND_CORE_EXAMPLES_EXPERIMENT_CUSTOM_RECORD_H

#include "navground/sim/probe.h"

// Let's record if agents are moving
class IsMovingProbe : public navground::sim::Probe {
 public:
  using navground::sim::Probe::Probe;
  using navground::sim::Probe::Shape;

  void update(const navground::sim::World &world) override {
    record_step();
    for (const auto &agent : world.get_agents()) {
      push(!agent->twist.is_almost_zero());
    }
  }

  Shape shape() const override {
    return {get_steps(), get_steps() ? size() / get_steps() : 0};
  }
};

// Let's record times when agents are not moving
class IsMovingSparseProbe : public navground::sim::MapProbe<unsigned> {
 public:
  using navground::sim::MapProbe<unsigned>::MapProbe;
  using navground::sim::BaseProbe::Shape;

  void update(const navground::sim::World &world) override {
    for (const auto &agent : world.get_agents()) {
      if (agent->twist.is_almost_zero()) {
        push(agent->uid, world.get_time());
      }
    }
  }

  Shape shape(const unsigned &key) const override { return {size(key)}; }
};

#endif  // NAVGROUND_CORE_EXAMPLES_EXPERIMENT_CUSTOM_RECORD_H