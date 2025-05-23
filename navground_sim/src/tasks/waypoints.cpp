/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/waypoints.h"

#include "navground/core/states/geometric.h"
#include "navground/core/yaml/schema.h"
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
      auto o = next_goal_orientation();
      auto tolerance = get_effective_tolerance(_index);
      auto angular_tolerance = get_effective_angular_tolerance(_index);
      if (o && angular_tolerance > 0) {
        c->go_to_pose(core::Pose2{*waypoint, *o}, tolerance, angular_tolerance);
      } else {
        c->go_to_position(*waypoint, tolerance);
      }
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

std::optional<ng_float_t> WaypointsTask::next_goal_orientation() const {
  int index = std::min<int>(_index, _orientations.size() - 1);
  if (index >= 0) {
    return _orientations[index];
  }
  return std::nullopt;
}

bool WaypointsTask::done() const {
  // return waypoint == waypoints.end();
  return !_running;
}

const std::string WaypointsTask::type = register_type<WaypointsTask>(
    "Waypoints",
    {{"waypoints", Property::make(&WaypointsTask::get_waypoints,
                                  &WaypointsTask::set_waypoints, Waypoints{},
                                  "waypoints", &YAML::schema::not_empty)},
     {"orientations",
      Property::make(&WaypointsTask::get_orientations,
                     &WaypointsTask::set_orientations,
                     std::vector<ng_float_t>{}, "orientations")},
     {"loop", Property::make(&WaypointsTask::get_loop, &WaypointsTask::set_loop,
                             default_loop, "loop")},
     {"tolerance",
      Property::make(&WaypointsTask::get_tolerance,
                     &WaypointsTask::set_tolerance, default_tolerance,
                     "Default spatial tolerance [m]", &YAML::schema::positive)},
     {"angular_tolerance", Property::make(&WaypointsTask::get_angular_tolerance,
                                          &WaypointsTask::set_angular_tolerance,
                                          default_angular_tolerance,
                                          "Default angular tolerance [rad]")},
     {"tolerances",
      Property::make(&WaypointsTask::get_tolerances,
                     &WaypointsTask::set_tolerances, std::vector<ng_float_t>{},
                     "Specific spatial tolerances [m]",
                     &YAML::schema::positive)},
     {"angular_tolerances",
      Property::make(&WaypointsTask::get_angular_tolerances,
                     &WaypointsTask::set_angular_tolerances,
                     std::vector<ng_float_t>{},
                     "Specific angular tolerances [rad]")},
     {"random", Property::make(&WaypointsTask::get_random,
                               &WaypointsTask::set_random, default_random,
                               "Whether to pick the next waypoint randomly")}});

} // namespace navground::sim
