/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>

#include "../groups.h"
#include "navground/sim/scenarios/cross.h"
#include "navground/sim/world.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/yaml/world.h"

const std::string behavior_name = "HL";

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  navground::sim::CrossScenario scenario(4.0);
  scenario.groups.push_back(std::make_unique<sim::AgentSampler<>>(group(behavior_name, 20)));
  navground::sim::World world;
  scenario.init_world(&world);
  // std::cout << YAML::dump<sim::World>(&world) << std::endl;
  std::cout << "Running benchmark" << std::endl;
  auto begin = std::chrono::high_resolution_clock::now();
  const unsigned steps = 10000;
  world.run(steps, 0.1);
  auto end = std::chrono::high_resolution_clock::now();
  unsigned ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
  printf("%s: total %.1f ms, per agent and step: %ld ns \n",
         behavior_name.c_str(), ns / 1e6, ns / (steps * world.get_agents().size()));
  return 0;
}