/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/cross.h"
#include "navground/core/yaml/schema.h"
#include <memory>
#include <utility>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/tasks/waypoints.h"

namespace navground::sim {

using namespace navground::core;
using navground::core::Property;

void CrossScenario::init_world(World *world,
                               [[maybe_unused]] std::optional<int> seed) {
  Scenario::init_world(world, seed);
  world->set_bounding_box(
      BoundingBox{-side / 2, side / 2, -side / 2, side / 2});
  const ng_float_t t = 0.5 * side;
  const ng_float_t p = std::max<ng_float_t>(0, 0.5 * side - target_margin);
  UniformSampler<ng_float_t> x(-p, p);
  const Waypoints targets{{t, 0}, {-t, 0}, {0, t}, {0, -t}};
  RandomGenerator &rg = world->get_random_generator();
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

const std::string CrossScenario::type = register_type<CrossScenario>(
    "Cross",
    {{"side", Property::make(&CrossScenario::get_side, &CrossScenario::set_side,
                             default_side, "Distance between targets",
                             &YAML::schema::strict_positive)},
     {"tolerance",
      Property::make(&CrossScenario::get_tolerance,
                     &CrossScenario::set_tolerance, default_tolerance,
                     "Goal tolerance", &YAML::schema::strict_positive)},
     {"agent_margin",
      Property::make(&CrossScenario::get_agent_margin,
                     &CrossScenario::set_agent_margin, ng_float_t(0.1),
                     "initial minimal distance between agents",
                     &YAML::schema::positive)},
     {"add_safety_to_agent_margin",
      Property::make(&CrossScenario::get_add_safety_to_agent_margin,
                     &CrossScenario::set_add_safety_to_agent_margin,
                     default_add_safety_to_agent_margin,
                     "Whether to add the safety margin to the agent margin")},
     {"target_margin",
      Property::make(&CrossScenario::get_target_margin,
                     &CrossScenario::set_target_margin, default_target_margin,
                     "Initial minimal distance between agents and targets",
                     &YAML::schema::positive)}});

} // namespace navground::sim
