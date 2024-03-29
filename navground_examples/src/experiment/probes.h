/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *
 * An example of a custom probe
 */

#ifndef NAVGROUND_CORE_EXAMPLES_EXPERIMENT_CUSTOM_RECORD_H
#define NAVGROUND_CORE_EXAMPLES_EXPERIMENT_CUSTOM_RECORD_H

#include "navground/sim/experimental_run.h"
#include "navground/sim/probe.h"

// Let's just print a message if an agent is not moving
class CheckIfMoving : public navground::sim::Probe {
 public:
  using navground::sim::Probe::Probe;

  void update(navground::sim::ExperimentalRun *run) override {
    const auto world = run->get_world();
    for (const auto &agent : world->get_agents()) {
      if (agent->twist.is_almost_zero()) {
        std::cerr << "Agent " << agent->uid << " is not moving at time "
                  << world->get_time() << std::endl;
      }
    }
  }
};

// Let's record if agents are moving
class IsMovingProbe : public navground::sim::RecordProbe {
 public:
  using navground::sim::RecordProbe::RecordProbe;

  using Type = uint8_t;

  void update(navground::sim::ExperimentalRun *run) override {
    for (const auto &agent : run->get_world()->get_agents()) {
      data->push(!agent->twist.is_almost_zero());
    }
  }

  navground::sim::Dataset::Shape get_shape(const World &world) const override {
    return {world.get_agents().size()};
  }
};

// Let's record times when agents are not moving
class IsMovingSparseProbe : public navground::sim::GroupRecordProbe {
 public:
  using navground::sim::GroupRecordProbe::GroupRecordProbe;
  using navground::sim::GroupRecordProbe::ShapeMap;

  using Type = ng_float_t;

  void update(navground::sim::ExperimentalRun *run) override {
    const auto world = run->get_world();
    for (const auto &agent : world->get_agents()) {
      if (agent->twist.is_almost_zero()) {
        get_data(std::to_string(agent->uid))->push(world->get_time());
      }
    }
  }

  ShapeMap get_shapes(const World &world) const override {
    ShapeMap shapes;
    for (const auto &agent : world.get_agents()) {
      shapes[std::to_string(agent->uid)] = {};
    }
    return shapes;
  }
};

#endif  // NAVGROUND_CORE_EXAMPLES_EXPERIMENT_CUSTOM_RECORD_H