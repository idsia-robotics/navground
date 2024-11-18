#include "navground/core/kinematics.h"
#include <assert.h>

namespace navground::core {

Twist2 OmnidirectionalKinematics::feasible(const Twist2 &twist) const {
  return {clamp_norm(twist.velocity, get_max_speed()),
          std::clamp(twist.angular_speed, -get_max_angular_speed(),
                     get_max_angular_speed()),
          twist.frame};
}

const std::string OmnidirectionalKinematics::type =
    register_type<OmnidirectionalKinematics>("Omni");

Twist2 AheadKinematics::feasible(const Twist2 &twist) const {
  assert(twist.frame == Frame::relative);
  return {{std::clamp<ng_float_t>(twist.velocity[0], 0, get_max_speed()), 0},
          std::clamp<ng_float_t>(twist.angular_speed, -get_max_angular_speed(),
                                 get_max_angular_speed()),
          twist.frame};
}

const std::string AheadKinematics::type =
    register_type<AheadKinematics>("Ahead");

// a more efficient? alternative
Twist2
TwoWheelsDifferentialDriveKinematics::feasible(const Twist2 &value) const {
  assert(value.frame == Frame::relative);
  const ng_float_t w_max = get_max_angular_speed();
  const ng_float_t w = std::clamp(value.angular_speed, -w_max, w_max);
  const ng_float_t v_max = get_max_speed() - std::abs(w) * get_wheel_axis() / 2;
  const ng_float_t max_linear_speed = std::min(get_max_forward_speed(), v_max);
  const ng_float_t min_linear_speed =
      -std::min(get_max_backward_speed(), v_max);
  const ng_float_t v = std::clamp<ng_float_t>(
      value.velocity[0], min_linear_speed, max_linear_speed);
  // const ng_float_t v = std::clamp(value.velocity[0], -v_max, v_max);
  return Twist2{{v, 0}, w, Frame::relative};
}

Twist2
TwoWheelsDifferentialDriveKinematics::twist(const WheelSpeeds &speeds) const {
  if (speeds.size() == 2 && _axis > 0) {
    // {left, right}
    return {{(speeds[0] + speeds[1]) / 2, 0},
            (speeds[1] - speeds[0]) / _axis,
            Frame::relative};
  }
  return {};
}

WheelSpeeds
TwoWheelsDifferentialDriveKinematics::wheel_speeds(const Twist2 &twist) const {
  assert(twist.frame == Frame::relative);
  const ng_float_t rotation = twist.angular_speed * _axis / 2;
  const ng_float_t linear = twist.velocity[0];
  return {linear - rotation, linear + rotation};
}

const std::map<std::string, Property>
    TwoWheelsDifferentialDriveKinematics::properties = Properties{
        {"wheel_axis",
         make_property<ng_float_t, TwoWheelsDifferentialDriveKinematics>(
             &TwoWheelsDifferentialDriveKinematics::get_wheel_axis,
             &TwoWheelsDifferentialDriveKinematics::set_wheel_axis, 0,
             "Wheel Axis")},
        {"max_forward_speed",
         make_property<ng_float_t, TwoWheelsDifferentialDriveKinematics>(
             &TwoWheelsDifferentialDriveKinematics::get_max_forward_speed,
             &TwoWheelsDifferentialDriveKinematics::set_max_forward_speed, -1,
             "Maximal forward linear speed")},
        {"max_backward_speed",
         make_property<ng_float_t, TwoWheelsDifferentialDriveKinematics>(
             &TwoWheelsDifferentialDriveKinematics::get_max_backward_speed,
             &TwoWheelsDifferentialDriveKinematics::set_max_backward_speed, -1,
             "Maximal backward linear speed")},
    };

const std::string TwoWheelsDifferentialDriveKinematics::type =
    register_type<TwoWheelsDifferentialDriveKinematics>("2WDiff");

ng_float_t
DynamicTwoWheelsDifferentialDriveKinematics::get_max_angular_acceleration()
    const {
  const auto _axis = get_wheel_axis();
  if (moi > 0 && _axis > 0) {
    return 4 * max_acceleration / (moi * _axis);
  }
  return Kinematics::inf;
}

void DynamicTwoWheelsDifferentialDriveKinematics::set_max_angular_acceleration(
    ng_float_t value) {
  const auto _axis = get_wheel_axis();
  if (value > 0 && _axis > 0) {
    set_moi(4 * max_acceleration / (value * _axis));
  } else {
    set_moi(Kinematics::inf);
  }
}

Twist2 DynamicTwoWheelsDifferentialDriveKinematics::feasible_from_current(
    const Twist2 &target, const Twist2 &current, ng_float_t time_step) const {
  if (time_step <= 0) {
    return current;
  }
  assert(target.frame == current.frame && target.frame == Frame::relative);
  const Twist2 feasible_target = feasible(target);
  const ng_float_t dw_max = get_max_angular_acceleration() * time_step;
  const ng_float_t w0 = current.angular_speed;
  const ng_float_t w =
      std::clamp(feasible_target.angular_speed, w0 - dw_max, w0 + dw_max);
  const ng_float_t dv_max = get_max_acceleration() * time_step -
                            std::abs(w - w0) * get_wheel_axis() * get_moi() / 4;
  const ng_float_t v0 = current.velocity[0];
  const ng_float_t v =
      std::clamp(feasible_target.velocity[0], v0 - dv_max, v0 + dv_max);
  return Twist2{{v, 0}, w, Frame::relative};
}

// {left, right}
std::vector<ng_float_t>
DynamicTwoWheelsDifferentialDriveKinematics::wheel_torques(
    const Twist2 &value, const Twist2 &current, ng_float_t time_step) const {
  assert(value.frame == current.frame && value.frame == Frame::relative);
  if (time_step <= 0) {
    return {0, 0};
  }
  const ng_float_t k = moi * get_wheel_axis() / 4;
  const ng_float_t lin_acc =
      (value.velocity[0] - current.velocity[0]) / time_step;
  const ng_float_t ang_acc =
      k * (value.angular_speed - current.angular_speed) / time_step;
  return {lin_acc - ang_acc, lin_acc + ang_acc};
}

// {left, right}
Twist2 DynamicTwoWheelsDifferentialDriveKinematics::twist_from_wheel_torques(
    const std::vector<ng_float_t> &values, const Twist2 &current,
    ng_float_t time_step) const {
  const ng_float_t lin_acc = (values[1] + values[0]) / 2;
  const ng_float_t ang_acc =
      2 * (values[1] - values[0]) / (moi * get_wheel_axis());
  return {{current.velocity[0] + time_step * lin_acc, 0},
          current.angular_speed + time_step * ang_acc,
          Frame::relative};
}

const std::map<std::string, Property>
    DynamicTwoWheelsDifferentialDriveKinematics::properties =
        TwoWheelsDifferentialDriveKinematics::properties +
        Properties{
            {"max_acceleration",
             make_property<ng_float_t,
                           DynamicTwoWheelsDifferentialDriveKinematics>(
                 &DynamicTwoWheelsDifferentialDriveKinematics::
                     get_max_acceleration,
                 &DynamicTwoWheelsDifferentialDriveKinematics::
                     set_max_acceleration,
                 0, "Maximal acceleration")},
            {"moi", make_property<ng_float_t,
                                  DynamicTwoWheelsDifferentialDriveKinematics>(
                        &DynamicTwoWheelsDifferentialDriveKinematics::get_moi,
                        &DynamicTwoWheelsDifferentialDriveKinematics::set_moi,
                        1, "Scaled moment of inertia")},
        };

const std::string DynamicTwoWheelsDifferentialDriveKinematics::type =
    register_type<DynamicTwoWheelsDifferentialDriveKinematics>("2WDiffDyn");

Twist2 FourWheelsOmniDriveKinematics::twist(const WheelSpeeds &speeds) const {
  if (speeds.size() == 4 && _axis > 0) {
    // {front left, rear left, rear right, rear left}
    return {{(speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4,
             (-speeds[0] + speeds[1] - speeds[2] + speeds[3]) / 4},
            (-speeds[0] - speeds[1] + speeds[2] + speeds[3]) / 4 / _axis,
            Frame::relative};
  }
  return {};
}

// {front left, rear left, rear right, rear left}
WheelSpeeds
FourWheelsOmniDriveKinematics::wheel_speeds(const Twist2 &twist) const {
  assert(twist.frame == Frame::relative);
  const ng_float_t rotation = twist.angular_speed * _axis;
  const ng_float_t longitudinal = twist.velocity[0];
  const ng_float_t lateral = twist.velocity[1];
  return {longitudinal - lateral - rotation, longitudinal + lateral + rotation,
          longitudinal + lateral - rotation, longitudinal - lateral + rotation};
}

WheelSpeeds FourWheelsOmniDriveKinematics::feasible_wheel_speeds(
    const Twist2 &twist) const {
  assert(twist.frame == Frame::relative);
  // {front left, rear left, rear right, rear left}
  ng_float_t max_speed = get_max_speed();
  const ng_float_t rotation =
      std::clamp(twist.angular_speed * _axis, -max_speed, max_speed);
  const ng_float_t longitudinal =
      std::clamp(twist.velocity[0], -max_speed, max_speed);
  const ng_float_t lateral =
      std::clamp(twist.velocity[1], -max_speed, max_speed);
  ng_float_t front_left = longitudinal - lateral - rotation;
  ng_float_t front_right = longitudinal + lateral + rotation;
  ng_float_t rear_left = longitudinal + lateral - rotation;
  ng_float_t rear_right = longitudinal - lateral + rotation;
  if (std::abs(front_left) > max_speed) {
    front_left = std::clamp(front_left, -max_speed, max_speed);
    front_right = front_left + 2 * lateral + 2 * rotation;
    rear_left = front_left + 2 * lateral;
    rear_right = front_left + 2 * rotation;
  } else if (std::abs(front_right) > max_speed) {
    front_right = std::clamp(front_right, -max_speed, max_speed);
    front_left = front_right - 2 * lateral - 2 * rotation;
    rear_left = front_right - 2 * rotation;
    rear_right = front_right - 2 * lateral;
  } else if (std::abs(rear_left) > max_speed) {
    rear_left = std::clamp(rear_left, -max_speed, max_speed);
    front_left = rear_left - 2 * lateral;
    front_right = rear_left + 2 * rotation;
    rear_right = rear_left - 2 * rotation + 2 * rotation;
  } else if (std::abs(rear_right) > max_speed) {
    rear_right = std::clamp(rear_right, -max_speed, max_speed);
    front_left = rear_right - 2 * rotation;
    front_right = rear_right + 2 * lateral;
    rear_left = rear_right + 2 * lateral - 2 * rotation;
  }
  return {front_left, rear_left, rear_right, front_right};
}

Twist2 FourWheelsOmniDriveKinematics::feasible(const Twist2 &value) const {
  return twist(feasible_wheel_speeds(value));
}

const std::map<std::string, Property>
    FourWheelsOmniDriveKinematics::properties = Properties{
        {"wheel_axis",
         make_property<ng_float_t, FourWheelsOmniDriveKinematics>(
             &FourWheelsOmniDriveKinematics::get_wheel_axis,
             &FourWheelsOmniDriveKinematics::set_wheel_axis, 0, "Wheel Axis")}};

const std::string FourWheelsOmniDriveKinematics::type =
    register_type<FourWheelsOmniDriveKinematics>("4WOmni");

} // namespace navground::core
