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
  if (bounding_box) {
    world->set_bounding_box(*bounding_box);
  }
  RandomGenerator &rg = world->get_random_generator();
  reset(seed);
  for (const auto &[name, property] : property_samplers) {
    if (property) {
      auto value = property->sample(rg);
      set(name, value);
    }
  }
  // for (const auto &[_, init] : get_inits()) {
  //   init(world, seed);
  // }
}

std::shared_ptr<World> Scenario::make_world(std::optional<int> seed) {
  auto world = std::make_shared<World>();
  init_world(world.get(), seed);
  apply_inits(world.get());
  return world;
}

void Scenario::apply_inits(World *world) {
  const auto seed = world->get_seed();
  for (const auto &[_, init] : get_inits()) {
    init(world, seed);
  }
}


} // namespace navground::sim
