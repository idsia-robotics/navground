#include "navground/core/echo.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"
#include "navground/core/yaml/core.h"
#include "navground/sim/version.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/world.h"

namespace core = navground::core;
namespace sim = navground::sim;
using core::EchoCommand;

int main(int argc, char *argv[]) {
  return core::EchoCommand(
             "echo", navground::sim::build_info().get_version_string(),
             {
                 {"behavior", &core::echo<core::Behavior>},
                 {"modulation", &core::echo<core::BehaviorModulation>},
                 {"kinematics", &core::echo<core::Kinematics>},
                 {"state_estimation", &core::echo<sim::StateEstimation>},
                 {"task", &core::echo<sim::Task>},
                 {"scenario", &core::echo<sim::Scenario>},
                 {"world", &core::echo_s<sim::World>},
                 {"agent", &core::echo_s<sim::Agent>},
                 {"experiment", &core::echo_s<sim::Experiment>},
             })
      .run(argc, argv);
}