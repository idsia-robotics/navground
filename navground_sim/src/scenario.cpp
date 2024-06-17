/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenario.h"

namespace navground::sim {

void Scenario::init_world(World* world,
                          [[maybe_unused]] std::optional<int> seed) {
  if (seed) {
    world->set_seed(*seed);
  }
  for (auto& group : groups) {
    if (group) {
      group->reset();
      group->add_to_world(world);
    }
  }
  world->set_obstacles(obstacles);
  world->set_walls(walls);
  RandomGenerator & rg = world->get_random_generator();
  reset(world->get_seed());
  for (const auto& [name, property] : property_samplers) {
    if (property) {
      auto value = property->sample(rg);
      set(name, value);
    }
  }
  for (const auto& f : initializers) {
    f(world);
  }
}

}  // namespace navground::sim
