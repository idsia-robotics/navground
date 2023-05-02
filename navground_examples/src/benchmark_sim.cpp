/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <chrono>

#include "navground/sim/scenarios/cross.h"
#include "navground/sim/world.h"

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
  std::cout << "Benchmark" << std::endl;
  navground::sim::Cross sampler("HL", 4.0, 5, 1.0);
  World world = sampler.sample();
  std::cout << world.description(true);
  auto begin = std::chrono::high_resolution_clock::now();
  const unsigned steps = 10000;
  world.run(steps, 0.1);
  auto end = std::chrono::high_resolution_clock::now();
  unsigned ns =
      std::chrono::duration_cast<std::chrono::nanoseconds>(end - begin).count();
  printf("HL: total %.1f ms, per agent and step: %ld ns \n",
         ns / 1e6, ns / (steps * world.agents.size()));
  return 0;
}
