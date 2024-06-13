/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior_modulations/relaxation.h"

#include "navground/core/behavior.h"

namespace navground::core {

ng_float_t relax(ng_float_t x0, ng_float_t x1, ng_float_t tau, ng_float_t dt) {
  if (tau == 0) return x1;
  return exp(-dt / tau) * (x0 - x1) + x1;
}

std::vector<ng_float_t> relax(const std::vector<ng_float_t>& v0,
                              const std::vector<ng_float_t>& v1, ng_float_t tau,
                              ng_float_t dt) {
  if (tau == 0) return v1;
  auto v2 = std::vector<ng_float_t>(v0.size());
  for (size_t i = 0; i < v0.size(); i++) {
    v2[i] = relax(v0[i], v1[i], tau, dt);
  }
  return v2;
}

Twist2 relax(const Twist2& v0, const Twist2& v1, ng_float_t tau,
             ng_float_t dt) {
  assert(v1.frame == v0.frame);
  if (tau == 0) {
    return v1;
  }
  return {{relax(v0.velocity[0], v1.velocity[0], tau, dt),
           relax(v0.velocity[1], v1.velocity[1], tau, dt)},
          relax(v0.angular_speed, v1.angular_speed, tau, dt),
          v1.frame};
}

Twist2 relax(Behavior& behavior, const Twist2& current_value,
             const Twist2& value, ng_float_t tau, ng_float_t dt) {
  if (behavior.get_kinematics()->is_wheeled()) {
    auto wheel_speeds = behavior.wheel_speeds_from_twist(value);
    auto current_wheel_speeds = behavior.wheel_speeds_from_twist(current_value);
    return behavior.twist_from_wheel_speeds(
        relax(current_wheel_speeds, wheel_speeds, tau, dt));
  } else {
    // TODO(Jerome old): same than before when I relaxed the absolute velocity,
    // not the relative but different than original paper CHANGED(J 2023): relax
    // in arbitrary frame
    return relax(current_value.frame == value.frame
                     ? current_value
                     : behavior.to_frame(current_value, value.frame),
                 value, tau, dt);
  }
}

void RelaxationModulation::pre(Behavior& behavior,
                               [[maybe_unused]] ng_float_t time_step) {
  _actuated_twist = behavior.get_actuated_twist();
}

Twist2 RelaxationModulation::post(Behavior& behavior, ng_float_t time_step,
                                  const Twist2& cmd_twist) {
  if (_tau <= 0) {
    return cmd_twist;
  }
  return behavior.to_frame(
      relax(behavior, _actuated_twist, cmd_twist, _tau, time_step),
      cmd_twist.frame);
}

const std::map<std::string, Property> RelaxationModulation::properties =
    Properties{
        {"tau", make_property<ng_float_t, RelaxationModulation>(
                    &RelaxationModulation::get_tau,
                    &RelaxationModulation::set_tau, default_tau, "Tau")}};

const std::string RelaxationModulation::type =
    register_type<RelaxationModulation>("Relaxation");

}  // namespace navground::core
