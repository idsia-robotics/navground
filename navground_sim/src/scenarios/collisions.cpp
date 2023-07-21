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
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

void CollisionsScenario::init_world(World *world, [[maybe_unused]] int seed) {
  Scenario::init_world(world);
  const float agent_radius = 0.1f;
  Vector2 target{10.0f, 10.0f};
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
  agent->get_controller()->set_speed_tolerance(0.1f);
  agent->pose = {{-8.0f, -7.9f}};
  world->add_wall(Wall{Vector2{-5.0f, -5.0f}, Vector2{-5.0f, -8.0f}});
  world->add_wall(Wall{Vector2{-3.0f, -1.0f}, Vector2{-0.0f, -2.0f}});
  // world.walls.emplace_back(Vector2{-3.0f, -1.0f}, Vector2{-3.0f, -0.0f});
  world->add_obstacle(Obstacle{Vector2{3.0f, 2.0f}, 3.0f});
}

const std::map<std::string, Property> CollisionsScenario::properties =
    Properties{
        {"behavior_name",
         make_property<std::string, CollisionsScenario>(
             [](const CollisionsScenario *obj) -> std::string {
               return obj->behavior_name;
             },
             [](CollisionsScenario *obj, const std::string &value) {
               obj->behavior_name = value;
             },
             "HL", "Behavior name")},
        {"control_period", make_property<float, CollisionsScenario>(
                               [](const CollisionsScenario *obj) -> float {
                                 return obj->control_period;
                               },
                               [](CollisionsScenario *obj, const float &value) {
                                 obj->control_period = value;
                               },
                               0.1f, "Control period")}};

const std::string CollisionsScenario::type =
    register_type<CollisionsScenario>("Collisions");

}  // namespace navground::sim
