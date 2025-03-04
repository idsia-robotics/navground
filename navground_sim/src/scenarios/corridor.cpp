/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/corridor.h"
#include "navground/core/yaml/schema.h"
#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/tasks/direction.h"

namespace navground::sim {

using namespace navground::core;
using navground::core::Property;

void CorridorScenario::init_world(World *world,
                                  [[maybe_unused]] std::optional<int> seed) {
  Scenario::init_world(world, seed);
  world->set_bounding_box(BoundingBox{0, length, 0, width});
  for (int side = 0; side < 2; ++side) {
    world->add_wall(Wall{{-length, side * width}, {2 * length, side * width}});
  }
  RandomGenerator &rg = world->get_random_generator();
  UniformSampler<ng_float_t> x(0.0, length);
  UniformSampler<ng_float_t> y(0.0, width);
  for (const auto &agent : world->get_agents()) {
    agent->pose.position = {x.sample(rg), y.sample(rg)};
    agent->set_task(nullptr);
  }
  world->set_lattice(0, std::make_tuple<ng_float_t>(0.0, length));
  world->space_agents_apart(agent_margin, add_safety_to_agent_margin);
  unsigned index = 0;
  // world->prepare();
  for (const auto &agent : world->get_agents()) {
    ng_float_t orientation = 0;
    Vector2 direction{1.0, 0.0};
    if (bidirectional && index % 2) {
      orientation = static_cast<ng_float_t>(PI);
      direction *= -1;
    }
    agent->pose.orientation = orientation;
    agent->set_task(std::make_shared<DirectionTask>(direction));
    // agent->get_controller()->follow_direction(direction);
    index++;
  }
}

const std::string CorridorScenario::type = register_type<CorridorScenario>(
    "Corridor",
    {{"width",
      Property::make(&CorridorScenario::get_width, &CorridorScenario::set_width,
                     default_width, "Corridor width",
                     &YAML::schema::strict_positive)},
     {"length",
      Property::make(&CorridorScenario::get_length,
                     &CorridorScenario::set_length, default_length,
                     "Corridor length", &YAML::schema::strict_positive)},
     {"agent_margin",
      Property::make(&CorridorScenario::get_agent_margin,
                     &CorridorScenario::set_agent_margin, ng_float_t(0.1),
                     "initial minimal distance between agents",
                     &YAML::schema::positive)},
     {"add_safety_to_agent_margin",
      Property::make(&CorridorScenario::get_add_safety_to_agent_margin,
                     &CorridorScenario::set_add_safety_to_agent_margin,
                     default_add_safety_to_agent_margin,
                     "Whether to add the safety margin to the agent margin")}});
} // namespace navground::sim
