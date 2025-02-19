/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/version.h"
#include "run_command.h"

int main(int argc, char *argv[]) {
  return navground::sim::RunCommand(
             "run", navground::sim::build_info().get_version_string())
      .run(argc, argv);
}
