/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_EXAMPLES_GROUP_H
#define NAVGROUND_CORE_EXAMPLES_GROUP_H

#include "navground/core/types.h"
#include "navground/sim/sampling/agent.h"
#include "navground/sim/sampling/sampler.h"
#include "navground/sim/tasks/waypoints.h"

namespace sim = navground::sim;

template <typename T>
std::unique_ptr<sim::Sampler<T>> make_const(const T& value) {
  return std::make_unique<sim::ConstantSampler<T>>(value);
}

template <typename T>
std::unique_ptr<sim::PropertySampler> make_const_property(const T& value) {
  return std::make_unique<sim::PropertySampler>(make_const<T>(value));
}


/*
 * Equivalent YAML:
 *
 *  number: 1
 *  kinematics:
 *    type: Omni
 *    max_speed: 1.0
 *  behavior:
 *    type: Dummy
 *  task:
 *    type: Waypoints
 *    waypoints: [[1.0, 0.0]]
 *    tolerance: 0.1
 *  radius: 0.1
 *  control_period: 0.1
 */

inline sim::AgentSampler<sim::World> agents() {
  sim::AgentSampler<sim::World> group;
  group.name = "agents";
  group.number = make_const<unsigned>(1);
  group.behavior = sim::BehaviorSampler<>("Dummy");
  group.kinematics = sim::KinematicsSampler<>("Omni");
  group.kinematics.max_speed = make_const<ng_float_t>(1);
  group.task = sim::TaskSampler<>("Waypoints");
  group.task.properties["waypoints"] =
      make_const_property<sim::Waypoints>({{1, 0}});
  group.radius = make_const<ng_float_t>(0.1);
  group.control_period = make_const<ng_float_t>(0.1);
  return group;
}

/*
 * Equivalent YAML:
 *
 *  number: 2
 *  kinematics:
 *    type: 2WDiff
 *    max_speed: 1.0
 *    wheel_axis: 0.12
 *  behavior:
 *    type: HL
 *  state_estimation:
 *    type: Bounded
 *    range: 10.0
 *  task:
 *    type: Waypoints
 *    waypoints: [[1.0, 0.0], [-1.0, 0.0]]
 *    tolerance: 0.1
 *    loop: true
 *  x:
 *    sampler: regular
 *    from: 0
 *    to: 10
 *    number: 2
 *  y: 0
 *  theta: 0
 *  control_period: 0.1
 */

inline sim::AgentSampler<sim::World> robots() {
  sim::AgentSampler<sim::World> group;
  group.name = "robots";
  group.id = make_const(1);
  const unsigned number = 2;
  group.number = make_const<unsigned>(number);
  group.behavior = sim::BehaviorSampler<>("HL");
  group.behavior.safety_margin = make_const<ng_float_t>(0.5);
  group.behavior.properties["tau"] = make_const_property<ng_float_t>(0.25);
  group.kinematics = sim::KinematicsSampler<>("2WDiff");
  group.kinematics.max_speed = make_const<ng_float_t>(1);
  group.kinematics.properties["wheel_axis"] = make_const_property<ng_float_t>(0.12);
  group.state_estimation =
      sim::StateEstimationSampler<>("Bounded");
  group.state_estimation.properties["range"] =
      make_const_property<ng_float_t>(10);
  group.task = sim::TaskSampler<>("Waypoints");
  group.task.properties["loop"] = make_const_property<bool>(true);
  group.task.properties["waypoints"] =
      make_const_property<sim::Waypoints>({{1, 0}, {-1, 0}});
  group.radius = make_const<ng_float_t>(0.1);
  group.position = std::make_unique<sim::RegularSampler<Vector2>>(
      sim::RegularSampler<Vector2>::make_with_interval(
          {0, 0}, {10, 0}, number));
  group.orientation = make_const<ng_float_t>(0);
  group.control_period = make_const<ng_float_t>(0.1);
  return group;
}

inline sim::AgentSampler<sim::World> group(const std::string & behavior, int number) {
  sim::AgentSampler<sim::World> group;
  group.id = make_const(1);
  group.number = make_const<unsigned>(number);;
  group.behavior = sim::BehaviorSampler<>(behavior);
  group.behavior.horizon = make_const<ng_float_t>(10);
  group.kinematics = sim::KinematicsSampler<>("Omni");
  group.kinematics.max_speed = make_const<ng_float_t>(1);
  group.state_estimation =
      sim::StateEstimationSampler<>("Bounded");
  group.state_estimation.properties["range"] =
      make_const_property<ng_float_t>(10);
  group.radius = make_const<ng_float_t>(0.1);
  group.control_period = make_const<ng_float_t>(0.1);
  return group;
}


#endif  // NAVGROUND_CORE_EXAMPLES_GROUP_H