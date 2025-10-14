#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/echo.h"
#include "navground/core/kinematics.h"
#include "navground/core/yaml/core.h"
#include "navground/sim/version.h"
#include "navground/sim/yaml/experiment.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/world.h"

namespace core = navground::core;
namespace sim = navground::sim;

inline bool echo_sampler(const YAML::Node &node,
                         const argparse::ArgumentParser &parser) {
  const auto obj = property_sampler(node, parser.get<std::string>("type"));
  if (obj) {
    std::cout << YAML::dump<PropertySampler>(obj.get()) << std::endl;
    return true;
  }
  return false;
}

inline void sampler_setup(argparse::ArgumentParser &parser) {
  parser.add_argument("--type").help("The sampled type").default_value("");
}

inline core::EchoCommand::Echos echos() {
  return {{"behavior", &core::echo<core::Behavior>},
          {"modulation", &core::echo<core::BehaviorModulation>},
          {"kinematics", &core::echo<core::Kinematics>},
          {"state_estimation", &core::echo<sim::StateEstimation>},
          {"task", &core::echo<sim::Task>},
          {"scenario", &core::echo<sim::Scenario>},
          {"world", &core::echo_s<sim::World>},
          {"agent", &core::echo_s<sim::Agent>},
          {"experiment", &core::echo_s<sim::Experiment>},
          {"sampler", echo_sampler}};
}