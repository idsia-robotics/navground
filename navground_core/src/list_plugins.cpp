#include "navground/core/list_plugins.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"
#include "navground/core/version.h"

namespace core = navground::core;
using core::ListPluginsCommand;

int main(int argc, char *argv[]) {
  return core::ListPluginsCommand(
             "plugins", navground::core::build_info().get_version_string(),
             {"behaviors", "modulations", "kinematics"})
      .run(argc, argv);
}