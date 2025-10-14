/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "../groups.h"
#include "navground/sim/sampling/agent.h"
#include "navground/sim/scenario.h"
#include "navground/sim/yaml/world.h"
#include "navground/sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

namespace sim = navground::sim;
using navground::core::Vector2;

int main() {
  sim::Scenario scenario;
  scenario.walls.emplace_back(Vector2{-1.0, -1.0}, Vector2{-1.0, 1.0});
  scenario.obstacles.emplace_back(Vector2{2.0, 0.0}, 0.5);
  scenario.add_group(std::make_unique<sim::AgentSampler<>>(robots()));
  std::cout << "\nSCENARIO\n========\n" << std::endl;
  std::cout << YAML::dump<sim::Scenario>(&scenario) << std::endl;
  std::cout << "\nWORLD\n=====\n" << std::endl;
  sim::World world;
  scenario.init_world(&world);
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
