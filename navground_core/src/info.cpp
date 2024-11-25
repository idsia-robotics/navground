/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/info.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/build_info.h"
#include "navground/core/kinematics.h"

int main(int argc, char *argv[]) {
  navground::core::InfoCommand cmd(
      "info",
      {{"Behaviors", navground::core::Behavior::type_properties},
       {"Kinematics", navground::core::Kinematics::type_properties},
       {"Modulations", navground::core::BehaviorModulation::type_properties}},
      navground::core::get_build_info(),
      navground::core::get_build_dependencies());
  return cmd.run(argc, argv);
}
