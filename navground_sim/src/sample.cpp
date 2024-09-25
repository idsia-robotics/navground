/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */


#include "sample_command.h"

int main(int argc, char *argv[]) {
  return navground::sim::SampleCommand("sample").run(argc, argv);
}
