/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/cross_torus.h"
#include "navground/core/yaml/schema.h"
#include <memory>
#include <utility>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/tasks/direction.h"

namespace navground::sim {

using namespace navground::core;
using navground::core::Property;

void CrossTorusScenario::init_world(World *world,
                                    [[maybe_unused]] std::optional<int> seed) {
  Scenario::init_world(world, seed);
  world->set_bounding_box(BoundingBox{0, side, 0, side});
  World::Lattice lattice = std::make_tuple<ng_float_t>(0.0, side);
  world->set_lattice(0, lattice);
  world->set_lattice(1, lattice);
  UniformSampler<ng_float_t> x(0.0, side);
  RandomGenerator &rg = world->get_random_generator();
  for (const auto &agent : world->get_agents()) {
    agent->pose.position = {x.sample(rg), x.sample(rg)};
  }
  world->space_agents_apart(agent_margin, add_safety_to_agent_margin);
  unsigned index = 0;
  // world->prepare();
  for (const auto &agent : world->get_agents()) {
    agent->pose.orientation = HALF_PI * (index % 4);
    // agent->get_controller()->follow_direction(unit(agent->pose.orientation));
    agent->set_task(
        std::make_shared<DirectionTask>(unit(agent->pose.orientation)));
    index++;
  }
}

const std::string CrossTorusScenario::type = register_type<CrossTorusScenario>(
    "CrossTorus",
    {{"side", Property::make(&CrossTorusScenario::get_side,
                             &CrossTorusScenario::set_side, default_side,
                             "Distance between targets",
                             &YAML::schema::strict_positive)},
     {"agent_margin",
      Property::make<ng_float_t>(
          &CrossTorusScenario::get_agent_margin,
          &CrossTorusScenario::set_agent_margin, static_cast<ng_float_t>(0.1),
          "initial minimal distance between agents", &YAML::schema::positive)},
     {"add_safety_to_agent_margin",
      Property::make(&CrossTorusScenario::get_add_safety_to_agent_margin,
                     &CrossTorusScenario::set_add_safety_to_agent_margin,
                     default_add_safety_to_agent_margin,
                     "Whether to add the safety margin to the agent margin")}});

} // namespace navground::sim
