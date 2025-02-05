/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "../groups.h"
#include "navground/core/behaviors/dummy.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/world.h"

using navground::core::Vector2;

namespace sim = navground::sim;

int main() {
  sim::World world;
  world.add_wall(sim::Wall{Vector2{-1.0, -1.0}, Vector2{-1.0, 1.0}});
  world.add_obstacle(sim::Obstacle{Vector2{2.0, 0.0}, 0.5});
  robots().add_to_world(&world);
  std::cout << "\nLoaded world\n====================" << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  world.run(100, (ng_float_t)0.1);
  std::cout
      << "\nAfter simulating for 10 s at 0.1 s steps\n===================="
      << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
