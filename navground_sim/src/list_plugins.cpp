/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/list_plugins.h"

int main(int argc, char *argv[]) {
  navground::core::ListPluginsCommand cmd(
      "plugins", {"behaviors", "kinematics", "modulations", "state_estimations",
                  "tasks", "scenarios"});
  return cmd.run(argc, argv);
}
