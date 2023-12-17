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
  for (const auto& f : initializers) {
    f(world);
  }
}

}  // namespace navground::sim
