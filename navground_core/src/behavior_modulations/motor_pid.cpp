/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior_modulations/motor_pid.h"
#include "navground/core/behavior.h"

#include "navground/core/kinematics.h"

namespace navground::core {

Twist2 MotorPIDModulation::post(Behavior &behavior, ng_float_t time_step,
                                const Twist2 &cmd_twist) {
  const auto kinematics =
      dynamic_cast<DynamicTwoWheelsDifferentialDriveKinematics *>(
          behavior.get_kinematics().get());
  if (!kinematics) {
    return cmd_twist;
  }
  const auto current = behavior.get_twist(Frame::relative);
  const auto feasible_cmd =
      kinematics->feasible_from_current(cmd_twist, current, time_step);
  const auto target_torques =
      kinematics->wheel_torques(feasible_cmd, current, time_step);
  const auto max_torque = kinematics->get_max_wheel_torque();
  for (int i = 0; i < 2; ++i) {
    ng_float_t e = target_torques[i] - _torques[i];
    ng_float_t de = (time_step > 0) ? (e - _e[i]) / time_step : 0;
    _ie[i] += time_step * e;
    _torques[i] += _k_p * e + _k_d * de + _k_i * _ie[i];
    _e[i] = e;
    _torques[i] = std::clamp(_torques[i], -max_torque, max_torque);
  }
  const auto t_cmd =
      kinematics->twist_from_wheel_torques(_torques, current, time_step);
  return behavior.to_frame(t_cmd, cmd_twist.frame);
}

const std::string MotorPIDModulation::type = register_type<MotorPIDModulation>(
    "MotorPID",
    {{"k_p", Property::make<ng_float_t>(&MotorPIDModulation::get_k_p,
                                        &MotorPIDModulation::set_k_p, 1, "P")},
     {"k_i", Property::make<ng_float_t>(&MotorPIDModulation::get_k_i,
                                        &MotorPIDModulation::set_k_i, 0, "I")},
     {"k_d",
      Property::make<ng_float_t>(&MotorPIDModulation::get_k_d,
                                 &MotorPIDModulation::set_k_d, 0, "D")}});

} // namespace navground::core
