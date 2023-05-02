/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/corridor.h"

#include <memory>
#include <utility>
#include <tuple>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/sim/sampling/sampler.h"

namespace navground::sim {

using namespace navground::core;

void CorridorScenario::init_world(World *world) {
  Scenario::init_world(world);
  for (int side = 0; side < 2; ++side) {
    world->add_wall(Wall{{-length, side * width}, {2 * length, side * width}});
  }
  UniformSampler<float> x(0.0f, length);
  UniformSampler<float> y(0.0f, width);
  for (const auto &agent : world->get_agents()) {
    agent->pose.position = {x.sample(), y.sample()};
    agent->set_task(nullptr);
  }
  world->set_lattice(0, std::make_tuple<float>(0.0f, length));
  world->space_agents_apart(agent_margin, add_safety_to_agent_margin);
  unsigned index = 0;
  world->prepare();
  for (const auto &agent : world->get_agents()) {
    float orientation = 0;
    Vector2 direction{1.0f, 0.0f};
    if (index % 2) {
      orientation = M_PI;
      direction *= -1;
    }
    agent->pose.orientation = orientation;
    agent->get_controller()->follow_direction(direction);
    index++;
  }
}

#if 0
const std::map<std::string, Property> CorridorScenario::properties = Properties{
    {"width", make_property<float, CorridorScenario>(
                  &CorridorScenario::get_width, &CorridorScenario::set_width,
                  default_width, "Corridor width")},
    {"length", make_property<float, CorridorScenario>(
                   &CorridorScenario::get_length, &CorridorScenario::set_length,
                   default_length, "Corridor length")},
    {"agent_margin", make_property<float, CorridorScenario>(
                         &CorridorScenario::get_agent_margin,
                         &CorridorScenario::set_agent_margin, 0.1f,
                         "initial minimal distance between agents")},
    {"add_safety_to_agent_margin",
     make_property<bool, CorridorScenario>(
         &CorridorScenario::get_add_safety_to_agent_margin,
         &CorridorScenario::set_add_safety_to_agent_margin,
         default_add_safety_to_agent_margin,
         "Whether to add the safety margin to the agent margin")}};

const std::string CorridorScenario::type =
    register_type<CorridorScenario>("Corridor");
#endif
}  // namespace navground::sim
