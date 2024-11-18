/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/scenarios/collisions.h"

#include <memory>
#include <utility>
#include <vector>

#include "navground/core/kinematics.h"
#include "navground/core/property.h"
#include "navground/sim/state_estimations/geometric_bounded.h"
#include "navground/sim/tasks/waypoints.h"

using navground::core::OmnidirectionalKinematics;
using navground::core::Property;

namespace navground::sim {

void CollisionsScenario::init_world(World *world,
                                    [[maybe_unused]] std::optional<int> seed) {
  Scenario::init_world(world, seed);
  const ng_float_t agent_radius = 0.1;
  Vector2 target{10, 10};
  auto task =
      std::make_shared<WaypointsTask>(Waypoints{target, -target}, true, 0.1);
  auto se = std::make_shared<BoundedStateEstimation>(nullptr, 10.0);
  auto kinematics = std::make_shared<OmnidirectionalKinematics>(1.0, 1.0);
  auto agent =
      std::make_shared<Agent>(agent_radius, Behavior::make_type(behavior_name),
                              kinematics, task, se, control_period);

  world->add_agent(agent);
  auto behavior = agent->get_behavior();
  behavior->set_horizon(10.0);
  behavior->set_safety_margin(0.1);
  // agent.controller.distance_tolerance = 1.0;
  // agent.controller.angle_tolerance = 4.0;
  agent->get_controller()->set_speed_tolerance(0.1);
  agent->pose = {{-8.0, -7.9}};
  world->add_wall(Wall{Vector2{-5.0, -5.0}, Vector2{-5.0, -8.0}});
  world->add_wall(Wall{Vector2{-3.0, -1.0}, Vector2{-0, -2.0}});
  // world.walls.emplace_back(Vector2{-3.0, -1.0}, Vector2{-3.0, -0});
  world->add_obstacle(Obstacle{Vector2{3.0, 2.0}, 3.0});
}

const std::string CollisionsScenario::type = register_type<CollisionsScenario>(
    "Collisions", {{"behavior_name",
                    Property::make_readwrite(&CollisionsScenario::behavior_name,
                                             "HL", "Behavior name")},
                   {"control_period", Property::make_readwrite(
                                          &CollisionsScenario::control_period,
                                          ng_float_t(0.1), "Control period")}});

} // namespace navground::sim
