/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior_modulations/limit_twist.h"

#include "navground/core/behavior.h"

namespace navground::core {

Twist2 LimitTwistModulation::post(Behavior &behavior, [[maybe_unused]] ng_float_t time_step,
                                  const Twist2 &cmd_twist) {
  auto twist = behavior.to_relative(cmd_twist);
  twist.velocity[0] = std::clamp(twist.velocity[0], -get_max_backward_speed(),
                                 get_max_forward_speed());
  twist.velocity[1] = std::clamp(twist.velocity[1], -get_max_rightward_speed(),
                                 get_max_leftward_speed());
  twist.angular_speed = std::clamp(twist.angular_speed, -get_max_angular_speed(),
                             get_max_angular_speed());
  return twist;
}

const std::map<std::string, Property> LimitTwistModulation::properties =
    Properties{{"forward", make_property<ng_float_t, LimitTwistModulation>(
                               &LimitTwistModulation::get_max_forward_speed,
                               &LimitTwistModulation::set_max_forward_speed,
                               std::numeric_limits<ng_float_t>::infinity(),
                               "Maximal forward speed")},
               {"backward", make_property<ng_float_t, LimitTwistModulation>(
                                &LimitTwistModulation::get_max_backward_speed,
                                &LimitTwistModulation::set_max_backward_speed,
                                std::numeric_limits<ng_float_t>::infinity(),
                                "Maximal backward speed")},
               {"leftward", make_property<ng_float_t, LimitTwistModulation>(
                                &LimitTwistModulation::get_max_leftward_speed,
                                &LimitTwistModulation::set_max_leftward_speed,
                                std::numeric_limits<ng_float_t>::infinity(),
                                "Maximal leftward speed")},
               {"rightward", make_property<ng_float_t, LimitTwistModulation>(
                                 &LimitTwistModulation::get_max_rightward_speed,
                                 &LimitTwistModulation::set_max_rightward_speed,
                                 std::numeric_limits<ng_float_t>::infinity(),
                                 "Maximal rightward speed")},
               {"angular", make_property<ng_float_t, LimitTwistModulation>(
                               &LimitTwistModulation::get_max_angular_speed,
                               &LimitTwistModulation::set_max_angular_speed,
                               std::numeric_limits<ng_float_t>::infinity(),
                               "Maximal angular speed")}};

const std::string LimitTwistModulation::type =
    register_type<LimitTwistModulation>("LimitTwist");

} // namespace navground::core
