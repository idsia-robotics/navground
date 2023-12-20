/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *
 * An example on how to add extra recording to a run
 */

#include <iomanip>
#include <iostream>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/probe.h"
#include "navground/sim/yaml/experiment.h"
#include "probes.h"
#include "yaml-cpp/yaml.h"

namespace sim = navground::sim;

static const char yaml[] = R"YAML(
steps: 300
time_step: 0.1
save_directory: '.'
record_time: true
runs: 2
name: custom_probes
scenario:
  type: Cross
  agent_margin: 0.1
  side: 4
  target_margin: 0.1
  tolerance: 0.5
  groups:
    -
      type: thymio
      number: 20
      radius: 0.08
      control_period: 0.1
      speed_tolerance: 0.02
      kinematics:
        type: 2WDiff
        wheel_axis: 0.094
        max_speed: 0.166
      behavior:
        type: HL
        optimal_speed: 0.12
        horizon: 5.0
        safety_margin: 0.02
      state_estimation:
        type: Bounded
        range: 5.0
)YAML";

int main() {
  YAML::Node node;
  try {
    node = YAML::Load(yaml);
  } catch (const YAML::ParserException &e) {
    std::cerr << "[Error] " << e.what() << std::endl;
    return 1;
  }
  sim::Experiment experiment;
  try {
    experiment = node.as<sim::Experiment>();
  } catch (const std::exception &e) {
    std::cerr << "[Error] Could not load the experiment " << e.what()
              << std::endl;
    std::exit(1);
  }
  experiment.make_probe<IsMovingProbe, uint8_t>("is_moving");
  experiment.make_map_probe<IsMovingSparseProbe, float>("still_times");
  experiment.run(false);
  std::cout << std::endl;
  return 0;
}
