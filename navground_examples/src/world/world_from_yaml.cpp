/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "navground/core/yaml/yaml.h"
#include "navground/sim/yaml/world.h"
#include "yaml-cpp/yaml.h"

static const char world_yaml[] = R"YAML(
walls:
  - line: [[-1.0, -1.0], [-1.0, 1.0]]
obstacles:
  - position: [2.0, 0.0]
    radius: 0.5
agents:
  - kinematics:
      type: Omni
      max_speed: 1.0
    behavior:
      type: HL
    task:
      type: Waypoints
      waypoints: [[1.0, 0.0]]
      tolerance: 0.1
    radius: 0.1
    control_period: 0.1
)YAML";

namespace sim = navground::sim;

int main() {
  YAML::Node node;
  try {
    node = YAML::Load(world_yaml);
  } catch (const YAML::ParserException &e) {
    std::cerr << "[Error] " << e.what() << std::endl;
    return 1;
  }
  auto world = node.as<sim::World>();
  std::cout << "\nLoaded world\n====================" << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  world.run(100, (ng_float_t)0.1);
  std::cout
      << "\nAfter simulating for 10 s at 0.1 s steps\n===================="
      << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
