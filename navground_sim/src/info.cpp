/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/info.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"
#include "navground/sim/build_info.h"
#include "navground/sim/scenario.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/task.h"
#include "navground/sim/version.h"

int main(int argc, char *argv[]) {
  navground::core::InfoCommand cmd(
      "info", navground::sim::build_info().get_version_string(),
      {{"Behaviors", navground::core::Behavior::type_properties},
       {"Kinematics", navground::core::Kinematics::type_properties},
       {"Modulations", navground::core::BehaviorModulation::type_properties},
       {"State Estimations", navground::sim::StateEstimation::type_properties},
       {"Tasks", navground::sim::Task::type_properties},
       {"Scenarios", navground::sim::Scenario::type_properties}},
      navground::sim::get_build_info(),
      navground::sim::get_build_dependencies());
  return cmd.run(argc, argv);
}
