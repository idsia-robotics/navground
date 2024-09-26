/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */


#include "run_command.h"

int main(int argc, char *argv[]) {
  return navground::sim::RunCommand("run").run(argc, argv);
}
