/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/info.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"
#include "navground/sim/scenario.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/task.h"

int main(int argc, char *argv[]) {
  navground::core::InfoCommand cmd(
      "info",
      {{"Behaviors", navground::core::Behavior::type_properties},
       {"Kinematics", navground::core::Kinematics::type_properties},
       {"Modulations", navground::core::BehaviorModulation::type_properties},
       {"State Estimations", navground::sim::StateEstimation::type_properties},
       {"Tasks", navground::sim::Task::type_properties},
       {"Scenarios", navground::sim::Scenario::type_properties}});
  return cmd.run(argc, argv);
}
