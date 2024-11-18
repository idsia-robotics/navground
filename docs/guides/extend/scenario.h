#include "navground/sim/scenario.h"

namespace sim = navground::sim;

struct MyScenario : public sim::Scenario {
  // SHOULD override
  // add here the logic to initialize the world
  void init_world(sim::World *world, seed = std::nullopt) override {
    // call the super class: when the scenario is configured thought YAML, 
    // it will add agents and obstacles as specified in the configuration.
    sim::Scenario::init_world(world, seed);
    // use the world random generator to sample random variable. 
    auto & rng = world->get_random_generator();
    // manipulate the world: create/add/modify/delete agents and obstacles
  }
};