#include "navground/core/plugins.h"
#include "navground/sim/scenario.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/task.h"

namespace navground::sim {

static const bool _r = []() {
  core::add_register<StateEstimation>("state_estimations");
  core::add_register<Task>("tasks");
  core::add_register<Scenario>("scenarios");
  return true;
}();

} // namespace navground::sim
