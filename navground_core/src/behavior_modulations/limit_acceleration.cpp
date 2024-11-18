/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior_modulations/limit_acceleration.h"

#include "navground/core/behavior.h"

namespace navground::core {

Twist2 LimitAccelerationModulation::post(Behavior &behavior,
                                         ng_float_t time_step,
                                         const Twist2 &cmd_twist) {
  const auto current = behavior.get_twist(cmd_twist.frame);
  return current.interpolate(cmd_twist, time_step, _max_acceleration,
                             _max_angular_acceleration);
}

const std::string LimitAccelerationModulation::type =
    register_type<LimitAccelerationModulation>(
        "LimitAcceleration",
        {{"max_acceleration",
          Property::make(&LimitAccelerationModulation::get_max_acceleration,
                         &LimitAccelerationModulation::set_max_acceleration,
                         std::numeric_limits<ng_float_t>::infinity(),
                         "Maximal acceleration")},
         {"max_angular_acceleration",
          Property::make(
              &LimitAccelerationModulation::get_max_angular_acceleration,
              &LimitAccelerationModulation::set_max_angular_acceleration,
              std::numeric_limits<ng_float_t>::infinity(),
              "Maximal angular acceleration")}});

} // namespace navground::core
