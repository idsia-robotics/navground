/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <optional>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/info.h"
#include "navground/core/plugins.h"
#include "navground/core/property.h"
#include "navground/core/utilities.h"
#include "navground/sim/scenario.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/world.h"

int main(int argc, char* argv[]) {
  load_plugins();
  INFO info("info",
            {{"--behavior", "Behaviors"},
             {"--kinematics", "Kinematics"},
             {"--state_estimation", "State Estimations"},
             {"--task", "Tasks"},
             {"--scenario", "Scenarios"}},
            argc, argv);
  if (!info.valid) {
    std::exit(1);
  }
  info.print<navground::core::Behavior>("--behavior");
  info.print<navground::core::Kinematics>("--kinematics");
  info.print<navground::sim::StateEstimation>("--state_estimation");
  info.print<navground::sim::Task>("--task");
  info.print<navground::sim::Scenario>("--scenario");
  return 0;
}