/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/waypoints.h"

#include "navground/core/states/geometric.h"
#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

using navground::core::Property;

void WaypointsTask::prepare(Agent *agent, World *world) { _running = true; }

void WaypointsTask::update(Agent *agent, World *world, ng_float_t time) {
  auto c = agent->get_controller();
  if (c->idle()) {
    auto waypoint = next_waypoint(world);
    if (waypoint) {
      c->go_to_position(*waypoint, _tolerance);
      _running = true;
      log_event({time, 1.0, waypoint->x(), waypoint->y()});
    } else if (_running) {
      log_event({time, 0.0, 0.0, 0.0});
      _running = false;
    }
  }
}

std::optional<navground::core::Vector2>
WaypointsTask::next_waypoint(World *world) {
  if (_waypoints.size() == 0)
    return std::nullopt;
  if (_random) {
    if (_first) {
      std::uniform_int_distribution<int> d(
          0, static_cast<int>(_waypoints.size() - 1));
      _index = d(world->get_random_generator());
    } else {
      std::uniform_int_distribution<int> d(
          1, static_cast<int>(_waypoints.size() - 1));
      _index = (_index + d(world->get_random_generator())) % _waypoints.size();
    }
  } else {
    if (_first) {
      _index = 0;
    } else {
      _index++;
      if (_loop && _index >= static_cast<int>(_waypoints.size())) {
        _index = 0;
      }
    }
  }
  _first = false;
  if (_index >= 0 && _index < static_cast<int>(_waypoints.size())) {
    return _waypoints[_index];
  }
  return std::nullopt;
}

bool WaypointsTask::done() const {
  // return waypoint == waypoints.end();
  return !_running;
}

const std::string WaypointsTask::type = register_type<WaypointsTask>(
    "Waypoints",
    {{"waypoints",
      Property::make(&WaypointsTask::get_waypoints,
                     &WaypointsTask::set_waypoints, Waypoints{}, "waypoints")},
     {"loop", Property::make(&WaypointsTask::get_loop, &WaypointsTask::set_loop,
                             default_loop, "loop")},
     {"tolerance", Property::make(&WaypointsTask::get_tolerance,
                                  &WaypointsTask::set_tolerance,
                                  default_tolerance, "tolerance")},
     {"random", Property::make(&WaypointsTask::get_random,
                               &WaypointsTask::set_random, default_random,
                               "Whether to pick the next waypoint randomly")}});

} // namespace navground::sim
