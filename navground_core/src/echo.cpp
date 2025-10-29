#include "navground/core/echo.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"
#include "navground/core/version.h"
#include "navground/core/yaml/core.h"

namespace core = navground::core;
using core::EchoCommand;

int main(int argc, char *argv[]) {
  return core::EchoCommand(
             "echo", navground::core::build_info().get_version_string(),
             {{"behavior", &core::echo<core::Behavior>},
              {"modulation", &core::echo<core::BehaviorModulation>},
              {"kinematics", &core::echo<core::Kinematics>},
              {"line", &core::echo<core::LineSegment>},
              {"disc", &core::echo<core::Disc>},
              {"neighbor", &core::echo<core::Neighbor>}})
      .run(argc, argv);
}