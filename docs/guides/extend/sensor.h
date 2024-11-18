#include "navground/sim/sensor.h"

namespace core = navground::core;
namespace sim = navground::sim;

struct MySensor : public sim::Sensor {
  // MUST override
  // return the description of the data written by the sensor
  sim::Sensor::Description get_description() const override {
    return {{"my_field", core::BufferDescription(...)}};
  };

  // CAN override
  // executed at the start of the simulation
  // void prepare(sim::Agent * agent, sim::World * world) override {
  //   // call the super class
  //   sim::Sensor::prepare(agent, world);
  // }

  // CAN override
  // executed during the the simulation
  // update the environment state according to the agent and world
  void update(sim::Agent *agent, sim::World *world,
              core::EnvironmentState state) override {
    if (auto se = dynamic_cast<core::SensingState>) {
      // compute the data
      std::valarray<int> my_data = ...;
      // update the sensing state
      auto buffer = get_or_init_buffer("my_field");
      buffer->set_data(my_data);
    }
  }
};