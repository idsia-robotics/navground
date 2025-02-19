/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/list_plugins.h"
#include "navground/sim/version.h"

int main(int argc, char *argv[]) {
  navground::core::ListPluginsCommand cmd(
      "plugins", navground::sim::build_info().get_version_string(),
      {"behaviors", "kinematics", "modulations", "state_estimations", "tasks",
       "scenarios"});
  return cmd.run(argc, argv);
}
