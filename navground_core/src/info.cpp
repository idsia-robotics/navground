/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/info.h"

#include <iostream>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/common.h"
#include "navground/core/kinematics.h"
#include "navground/core/plugins.h"
#include "navground/core/property.h"
#include "navground/core/utilities.h"

int main(int argc, char* argv[]) {
  navground::core::load_plugins();
  INFO info("info",
            {{"--behavior", "Behaviors"},
             {"--kinematics", "Kinematics"},
             {"--modulations", "Modulations"}},
            argc, argv);
  if (!info.valid) {
    std::exit(1);
  }
  info.print<navground::core::Behavior>("--behavior");
  info.print<navground::core::BehaviorModulation>("--modulations");
  info.print<navground::core::Kinematics>("--kinematics");
  return 0;
}
