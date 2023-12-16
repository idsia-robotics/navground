/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimation.h"

#include "navground/sim/agent.h"

namespace navground::sim {

void StateEstimation::update(Agent *agent, World *world) const {
  if (agent) {
    if (Behavior *behavior = agent->get_behavior()) {
      update(agent, world, behavior->get_environment_state());
    }
  }
};

}  // namespace navground::sim
