#include "navground/core/kinematics.h"

namespace navground::core {

Twist2 OmnidirectionalKinematics::feasible(const Twist2& twist) const {
  return {clamp_norm(twist.velocity, get_max_speed()),
          std::clamp(twist.angular_speed, -get_max_angular_speed(),
                     get_max_angular_speed()),
          twist.frame};
}

Twist2 AheadKinematics::feasible(const Twist2& twist) const {
  assert(twist.frame == Frame::relative);
  return {{std::clamp<ng_float_t>(twist.velocity[0], 0, get_max_speed()), 0},
          std::clamp<ng_float_t>(twist.angular_speed, -get_max_angular_speed(),
                                 get_max_angular_speed()),
          twist.frame};
}

Twist2 WheeledKinematics::feasible(const Twist2& value) const {
  return twist(wheel_speeds(value));
}

Twist2 TwoWheelsDifferentialDriveKinematics::twist(
    const WheelSpeeds& speeds) const {
  if (speeds.size() == 2) {
    // {left, right}
    return {{(speeds[0] + speeds[1]) / 2, 0},
            (speeds[1] - speeds[0]) / axis,
            Frame::relative};
  }
  return {};
}

WheelSpeeds TwoWheelsDifferentialDriveKinematics::wheel_speeds(
    const Twist2& twist) const {
  assert(twist.frame == Frame::relative);
  // {left, right}
  ng_float_t max_speed = get_max_speed();
  const ng_float_t rotation = std::clamp<ng_float_t>(
      0.5 * twist.angular_speed * axis, -max_speed, max_speed);
  const ng_float_t linear =
      std::clamp<ng_float_t>(twist.velocity[0], 0, max_speed);
  ng_float_t left = linear - rotation;
  ng_float_t right = linear + rotation;
  if (abs(left) > max_speed) {
    left = std::clamp(left, -max_speed, max_speed);
    right = left + 2 * rotation;
  } else if (abs(right) > max_speed) {
    right = std::clamp(right, -max_speed, max_speed);
    left = right - 2 * rotation;
  }
  return {left, right};
}

Twist2 FourWheelsOmniDriveKinematics::twist(const WheelSpeeds& speeds) const {
  if (speeds.size() == 4) {
    // {front left, rear left, rear right, rear left}
    return {{(speeds[0] + speeds[1] + speeds[2] + speeds[3]) / 4,
             (-speeds[0] + speeds[1] - speeds[2] + speeds[3]) / 4},
            (-speeds[0] - speeds[1] + speeds[2] + speeds[3]) / 4 / axis,
            Frame::relative};
  }
  return {};
}

WheelSpeeds FourWheelsOmniDriveKinematics::wheel_speeds(
    const Twist2& twist) const {
  assert(twist.frame == Frame::relative);
  // {front left, rear left, rear right, rear left}
  ng_float_t max_speed = get_max_speed();
  const ng_float_t rotation =
      std::clamp(twist.angular_speed * axis, -max_speed, max_speed);
  const ng_float_t longitudinal =
      std::clamp(twist.velocity[0], -max_speed, max_speed);
  const ng_float_t lateral =
      std::clamp(twist.velocity[1], -max_speed, max_speed);
  ng_float_t front_left = longitudinal - lateral - rotation;
  ng_float_t front_right = longitudinal + lateral + rotation;
  ng_float_t rear_left = longitudinal + lateral - rotation;
  ng_float_t rear_right = longitudinal - lateral + rotation;
  if (abs(front_left) > max_speed) {
    front_left = std::clamp(front_left, -max_speed, max_speed);
    front_right = front_left + 2 * lateral + 2 * rotation;
    rear_left = front_left + 2 * lateral;
    rear_right = front_left + 2 * rotation;
  } else if (abs(front_right) > max_speed) {
    front_right = std::clamp(front_right, -max_speed, max_speed);
    front_left = front_right - 2 * lateral - 2 * rotation;
    rear_left = front_right - 2 * rotation;
    rear_right = front_right - 2 * lateral;
  } else if (abs(rear_left) > max_speed) {
    rear_left = std::clamp(rear_left, -max_speed, max_speed);
    front_left = rear_left - 2 * lateral;
    front_right = rear_left + 2 * rotation;
    rear_right = rear_left - 2 * rotation + 2 * rotation;
  } else if (abs(rear_right) > max_speed) {
    rear_right = std::clamp(rear_right, -max_speed, max_speed);
    front_left = rear_right - 2 * rotation;
    front_right = rear_right + 2 * lateral;
    rear_left = rear_right + 2 * lateral - 2 * rotation;
  }
  return {front_left, rear_left, rear_right, front_right};
}

}  // namespace navground::core
