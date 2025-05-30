/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/waypoints.h"

#include "navground/core/states/geometric.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/agent.h"
#include "navground/sim/world.h"
#include <cassert>

namespace navground::sim {

using navground::core::Property;

void WaypointsTask::prepare(Agent *agent, World *world) {
  _state = State::idle;
  _first = true;
  _index = 0;
}

void WaypointsTask::update(Agent *agent, World *world, ng_float_t time) {
  if (_state == State::done) {
    return;
  }
  if (_state == State::waiting) {
    if (time >= _deadline) {
      _state = State::waited;
    } else {
      return;
    }
  }
  auto c = agent->get_controller();
  if (_state == State::moving) {
    if (c->idle()) {
      _state = State::idle;
    }
  }
  if (_state == State::idle) {
    if (!next(world)) {
      log_event({time, 0, 0, 0, 0, 0, 0});
      _state = State::done;
      return;
    }
    const auto wait_time = get_effective_wait_time(_index);
    if (wait_time) {
      _state = State::waiting;
      _deadline = time + wait_time;
      log_event({time, 1, wait_time, 0, 0, 0, 0});
      return;
    }
  }
  if (_state == State::idle || _state == State::waited) {
    _state = State::moving;
    const auto waypoint = get_waypoint(_index);
    assert(waypoint);
    const auto o = get_orientation(_index);
    const auto tolerance = get_effective_tolerance(_index);
    const auto angular_tolerance = get_effective_angular_tolerance(_index);
    if (o && angular_tolerance > 0) {
      c->go_to_pose(core::Pose2{*waypoint, *o}, tolerance, angular_tolerance);
      log_event({time, 2, waypoint->x(), waypoint->y(), *o, tolerance,
                 angular_tolerance});
    } else {
      c->go_to_position(*waypoint, tolerance);
      log_event({time, 3, waypoint->x(), waypoint->y(), 0, tolerance, 0});
    }
  }
}

bool WaypointsTask::next(World *world) {
  if (_waypoints.size() == 0)
    return false;
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
    return true;
  }
  return false;
}

bool WaypointsTask::done() const {
  // return waypoint == waypoints.end();
  return _state == State::done;
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
                     "Specific spatial tolerances [m]")},
     {"angular_tolerances",
      Property::make(&WaypointsTask::get_angular_tolerances,
                     &WaypointsTask::set_angular_tolerances,
                     std::vector<ng_float_t>{},
                     "Specific angular tolerances [rad]")},
     {"wait_time",
      Property::make(&WaypointsTask::get_wait_time,
                     &WaypointsTask::set_wait_time, ng_float_t{0},
                     "Default wait time [s]", &YAML::schema::positive)},
     {"wait_times",
      Property::make(&WaypointsTask::get_wait_times,
                     &WaypointsTask::set_wait_times, std::vector<ng_float_t>{},
                     "Specific wait times [s]")},
     {"random", Property::make(&WaypointsTask::get_random,
                               &WaypointsTask::set_random, default_random,
                               "Whether to pick the next waypoint randomly")}});

} // namespace navground::sim
