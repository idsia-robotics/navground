/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/cross.h"

#include <memory>
#include <utility>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/tasks/waypoints.h"

namespace navground::sim {

using namespace navground::core;

void CrossScenario::init_world(World *world,
                               [[maybe_unused]] std::optional<int> seed) {
  Scenario::init_world(world, seed);
  const ng_float_t t = 0.5 * side;
  const ng_float_t p = std::max<ng_float_t>(0, 0.5 * side - target_margin);
  UniformSampler<ng_float_t> x(-p, p);
  const Waypoints targets{{t, 0}, {-t, 0}, {0, t}, {0, -t}};
  RandomGenerator & rg = world->get_random_generator();
  for (const auto &agent : world->get_agents()) {
    agent->pose.position = {x.sample(rg), x.sample(rg)};
  }
  world->space_agents_apart(agent_margin, add_safety_to_agent_margin);
  unsigned index = 0;
  for (const auto &agent : world->get_agents()) {
    const auto target = targets[index % 4];
    agent->set_task(std::make_shared<WaypointsTask>(Waypoints{target, -target},
                                                    true, tolerance));
    agent->pose.orientation = orientation_of(target - agent->pose.position);
    index++;
  }
}

const std::map<std::string, Property> CrossScenario::properties = Properties{
    {"side", make_property<float, CrossScenario>(
                 &CrossScenario::get_side, &CrossScenario::set_side,
                 default_side, "Distance between targets")},
    {"tolerance",
     make_property<float, CrossScenario>(&CrossScenario::get_tolerance,
                                         &CrossScenario::set_tolerance,
                                         default_tolerance, "Goal tolerance")},
    {"agent_margin",
     make_property<float, CrossScenario>(
         &CrossScenario::get_agent_margin, &CrossScenario::set_agent_margin,
         0.1f, "initial minimal distance between agents")},
    {"add_safety_to_agent_margin",
     make_property<bool, CrossScenario>(
         &CrossScenario::get_add_safety_to_agent_margin,
         &CrossScenario::set_add_safety_to_agent_margin,
         default_add_safety_to_agent_margin,
         "Whether to add the safety margin to the agent margin")},
    {"target_margin",
     make_property<float, CrossScenario>(
         &CrossScenario::get_target_margin, &CrossScenario::set_target_margin,
         default_target_margin,
         "Initial minimal distance between agents and targets")}};

const std::string CrossScenario::type = register_type<CrossScenario>("Cross");

}  // namespace navground::sim
