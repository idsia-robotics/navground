/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/waypoints.h"

#include "navground/core/states/geometric.h"
#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void WaypointsTask::update(Agent *agent, World *world, ng_float_t time) {
  auto c = agent->get_controller();
  if (c->idle()) {
    auto waypoint = next_waypoint(world);
    if (waypoint) {
      c->go_to_position(*waypoint, tolerance);
      running = true;
      for (const auto &cb : callbacks) {
        cb({time, 1.0, waypoint->x(), waypoint->y()});
      }
    } else if (running) {
      for (const auto &cb : callbacks) {
        cb({time, 0.0, 0.0, 0.0});
      }
      running = false;
    }
  }
}

std::optional<navground::core::Vector2> WaypointsTask::next_waypoint(
    World *world) {
  if (waypoints.size() == 0) return std::nullopt;
  if (random) {
    if (first) {
      std::uniform_int_distribution<int> d(0, waypoints.size() - 1);
      index = d(world->get_random_generator());
    } else {
      std::uniform_int_distribution<int> d(1, waypoints.size() - 1);
      index = (index + d(world->get_random_generator())) % waypoints.size();
    }
  } else {
    if (first) {
      index = 0;
    } else {
      index++;
      if (loop && index >= static_cast<int>(waypoints.size())) {
        index = 0;
      }
    }
  }
  first = false;
  if (index >= 0 && index < static_cast<int>(waypoints.size())) {
    return waypoints[index];
  }
  return std::nullopt;
}

bool WaypointsTask::done() const {
  // return waypoint == waypoints.end();
  return !running;
}

const std::map<std::string, Property> WaypointsTask::properties = Properties{
      {"waypoints",
       make_property<Waypoints, WaypointsTask>(&WaypointsTask::get_waypoints,
                                               &WaypointsTask::set_waypoints,
                                               Waypoints{}, "waypoints")},
      {"loop", make_property<bool, WaypointsTask>(&WaypointsTask::get_loop,
                                                  &WaypointsTask::set_loop,
                                                  default_loop, "loop")},
      {"tolerance",
       make_property<ng_float_t, WaypointsTask>(
           &WaypointsTask::get_tolerance, &WaypointsTask::set_tolerance,
           default_tolerance, "tolerance")},
      {"random",
       make_property<bool, WaypointsTask>(
           &WaypointsTask::get_random, &WaypointsTask::set_random,
           default_random, "Whether to pick the next waypoint randomly")},
};

const std::string WaypointsTask::type = register_type<WaypointsTask>("Waypoints");

}  // namespace navground::sim
