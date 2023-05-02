/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "navground/sim/scenario.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/scenario.h"
#include "navground/sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

static const char scenario_yaml[] = R"YAML(
walls:
  - [[-1.0, -1.0], [-1.0, 1.0]]
obstacles:
  - 
    position: [2.0, 0.0]
    radius: 0.5
groups:
  -
    kinematics:
      type: Omni
      max_speed: 1.0
      wheel_axis: 0.12
    behavior:
      type: HL
      safety_margin: 0.5
      tau: 0.25
    state_estimation:
      type: Bounded
      range_of_view: 10.0
    task:
      type: Waypoints
      waypoints: [[1.0, 0.0], [-1.0, 0.0]]
      tolerance: 0.1
    radius: 0.1
    control_period: 0.1
    number: 2
    position:
      sampler: regular
      from: [0, 0]
      to: [10, 0]
      number: 2
    orientation: 0
)YAML";

namespace sim = navground::sim;

int main() {
  YAML::Node node;
  try {
    node = YAML::Load(scenario_yaml);
  } catch (const YAML::ParserException &e) {
    std::cerr << "[Error] " << e.what() << std::endl;
    return 1;
  }
  auto scenario = node.as<std::shared_ptr<sim::Scenario>>();
  if (!scenario) {
    std::cerr << "Could not load the scenario" << std::endl;
    return 1;
  }
  std::cout << "\nSCENARIO\n========\n" << std::endl;
  std::cout << YAML::dump<sim::Scenario>(scenario.get()) << std::endl;
  sim::World world;
  scenario->init_world(&world);
  std::cout << "\nWORLD\n=====\n" << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
