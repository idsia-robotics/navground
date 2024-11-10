/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenario.h"

namespace navground::sim {

void Scenario::init_world(World *world, std::optional<int> seed) {
  if (seed) {
    world->set_seed(*seed);
  }
  for (auto &group : groups) {
    if (group) {
      group->add_to_world(world, seed);
    }
  }
  world->set_obstacles(obstacles);
  world->set_walls(walls);
  RandomGenerator &rg = world->get_random_generator();
  reset(seed);
  for (const auto &[name, property] : property_samplers) {
    if (property) {
      auto value = property->sample(rg);
      set(name, value);
    }
  }
  for (const auto &init : initializers) {
    init(world, seed);
  }
}

} // namespace navground::sim
