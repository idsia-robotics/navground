#include "navground/sim/state_estimation.h"

namespace core = navground::core;
namespace sim = navground::sim;

struct MyStateEstimation : public sim::StateEstimation {
  // CAN override
  // executed at the start of the simulation
  // void prepare(sim::Agent * agent, sim::World * world) override;

  // CAN override
  // executed during the the simulation
  // update the environment state according to the agent and world
  void update(sim::Agent *agent, sim::World *world,
              core::EnvironmentState state) ovveride {
    if (auto se = dynamic_cast<SupportedEnvironmentState>) {
      // update the state
    }
  }
};