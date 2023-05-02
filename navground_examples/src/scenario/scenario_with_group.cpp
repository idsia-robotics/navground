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
  scenario.walls.emplace_back(Vector2{-1.0f, -1.f}, Vector2{-1.f, 1.f});
  scenario.obstacles.emplace_back(Vector2{2.f, 0.f}, 0.5f);
  scenario.groups.push_back(std::make_unique<sim::AgentSampler<>>(robots()));
  std::cout << "\nSCENARIO\n========\n" << std::endl;
  std::cout << YAML::dump<sim::Scenario>(&scenario) << std::endl;
  std::cout << "\nWORLD\n=====\n" << std::endl;
  sim::World world;
  scenario.init_world(&world);
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
