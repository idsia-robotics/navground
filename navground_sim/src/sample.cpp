/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/version.h"
#include "sample_command.h"

int main(int argc, char *argv[]) {
  return navground::sim::SampleCommand(
             "sample", navground::sim::build_info().get_version_string())
      .run(argc, argv);
}
