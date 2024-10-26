#include "navground/core/list_plugins.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"

namespace core = navground::core;
using core::ListPluginsCommand;

int main(int argc, char *argv[]) {
  return core::ListPluginsCommand("plugins",
                                  {"behaviors", "modulations", "kinematics"})
      .run(argc, argv);
}