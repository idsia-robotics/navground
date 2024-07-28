#include "navground/core/kinematics.h"

#include <limits>

namespace navground::core {

Twist2 OmnidirectionalKinematics::feasible(const Twist2& twist) const {
  return {clamp_norm(twist.velocity, get_max_speed()),
          std::clamp(twist.angular_speed, -get_max_angular_speed(),
                     get_max_angular_speed()),
          twist.frame};
}

const std::string OmnidirectionalKinematics::type =
    register_type<OmnidirectionalKinematics>("Omni");

Twist2 AheadKinematics::feasible(const Twist2& twist) const {
  assert(twist.frame == Frame::relative);
  return {{std::clamp<ng_float_t>(twist.velocity[0], 0, get_max_speed()), 0},
          std::clamp<ng_float_t>(twist.angular_speed, -get_max_angular_speed(),
                                 get_max_angular_speed()),
          twist.frame};
}

const std::string AheadKinematics::type =
    register_type<AheadKinematics>("Ahead");

Twist2 WheeledKinematics::feasible(const Twist2& value) const {
  return twist(feasible_wheel_speeds(value));
}

const std::map<std::string, Property> WheeledKinematics::properties =
    Properties{
        {"wheel_axis", make_property<ng_float_t, WheeledKinematics>(
                           &WheeledKinematics::get_axis,
                           &WheeledKinematics::set_axis, 0, "Wheel Axis")},
    };

// a more efficient? alternative
Twist2 TwoWheelsDifferentialDriveKinematics::feasible(
    const Twist2& value) const {
  assert(value.frame == Frame::relative);
  const ng_float_t w_max = get_max_angular_speed();
  const ng_float_t w = std::clamp(value.angular_speed, -w_max, w_max);
  const ng_float_t v_max = get_max_speed() - abs(w) * get_axis() / 2;
  const ng_float_t v = std::clamp<ng_float_t>(value.velocity[0], 0, v_max);
  // const ng_float_t v = std::clamp(value.velocity[0], -v_max, v_max);
  return Twist2{{v, 0}, w, Frame::relative};
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
  const ng_float_t rotation = 0.5 * twist.angular_speed * axis;
  const ng_float_t linear = twist.velocity[0];
  return {linear - rotation, linear + rotation};
}

WheelSpeeds TwoWheelsDifferentialDriveKinematics::feasible_wheel_speeds(
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

const std::string TwoWheelsDifferentialDriveKinematics::type =
    register_type<TwoWheelsDifferentialDriveKinematics>("2WDiff");

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

// {front left, rear left, rear right, rear left}
WheelSpeeds FourWheelsOmniDriveKinematics::wheel_speeds(
    const Twist2& twist) const {
  assert(twist.frame == Frame::relative);
  const ng_float_t rotation = twist.angular_speed * axis;
  const ng_float_t longitudinal = twist.velocity[0];
  const ng_float_t lateral = twist.velocity[1];
  return {longitudinal - lateral - rotation, longitudinal + lateral + rotation,
          longitudinal + lateral - rotation, longitudinal - lateral + rotation};
}

WheelSpeeds FourWheelsOmniDriveKinematics::feasible_wheel_speeds(
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

const std::string FourWheelsOmniDriveKinematics::type =
    register_type<FourWheelsOmniDriveKinematics>("4WOmni");

ng_float_t
DynamicTwoWheelsDifferentialDriveKinematics::get_max_angular_acceleration()
    const {
  if (moi > 0 && axis > 0) {
    return 4 * max_acceleration / (moi * axis);
  }
  return std::numeric_limits<ng_float_t>::infinity();
}

void DynamicTwoWheelsDifferentialDriveKinematics::set_max_angular_acceleration(
    ng_float_t value) {
  if (value > 0 && axis > 0) {
    set_moi(4 * max_acceleration / (value * axis));
  } else {
    set_moi(std::numeric_limits<ng_float_t>::infinity());
  }
}

Twist2 DynamicTwoWheelsDifferentialDriveKinematics::feasible(
    const Twist2& target, const Twist2& current, ng_float_t time_step) const {
  if (time_step <= 0) {
    return current;
  }
  assert(target.frame == current.frame == Frame::relative);
  const Twist2 feasible_target = feasible(target);
  const ng_float_t dw_max = get_max_angular_acceleration() * time_step;
  const ng_float_t w0 = current.angular_speed;
  const ng_float_t w =
      std::clamp(feasible_target.angular_speed, w0 - dw_max, w0 + dw_max);
  const ng_float_t dv_max = get_max_acceleration() * time_step -
                            abs(w - w0) * get_axis() * get_moi() / 4;
  const ng_float_t v0 = current.velocity[0];
  const ng_float_t v =
      std::clamp(feasible_target.velocity[0], v0 - dv_max, v0 + dv_max);
  return Twist2{{v, 0}, w, Frame::relative};
}

#if 0

//alternative via torques
Twist2 DynamicTwoWheelsDifferentialDriveKinematics::feasible(
    const Twist2& target, const Twist2& current, ng_float_t time_step) const {
  if (time_step <= 0) {
    return current;
  }
  const ng_float_t k = moi * axis / 4;
  const Twist2 ltarget = feasible(target);
  ng_float_t lin_acc =
      std::clamp((ltarget.velocity[0] - current.velocity[0]) / time_step,
                 -max_acceleration, max_acceleration);
  ng_float_t ang_acc = std::clamp(
      k * (ltarget.angular_speed - current.angular_speed) / time_step,
      -max_acceleration, max_acceleration);

  std::array<ng_float_t, 2> wheel_force_2{lin_acc - ang_acc, lin_acc + ang_acc};
  if (abs(wheel_force_2[0]) > max_acceleration ||
      abs(wheel_force_2[1]) > max_acceleration) {
    if (reduce_torques) {
      const ng_float_t f =
          std::max(abs(wheel_force_2[0]), abs(wheel_force_2[1])) /
          max_acceleration;
      wheel_force_2[0] /= f;
      wheel_force_2[1] /= f;
    } else {
      for (int i = 0; i < 2; ++i) {
        wheel_force_2[i] =
            std::clamp(wheel_force_2[i], -max_acceleration, max_acceleration);
      }
    } 
    lin_acc = 0.5 * (wheel_force_2[1] + wheel_force_2[0]);
    ang_acc = 0.5 * (wheel_force_2[1] - wheel_force_2[0]);
    // return Twist2{{current.velocity[0] + time_step * lin_acc, 0},
    //               current.angular_speed + time_step * ang_acc / k,
    //               Frame::relative};
  }
  return Twist2{{current.velocity[0] + time_step * lin_acc, 0},
                  current.angular_speed + time_step * ang_acc / k,
                  Frame::relative};
  // return ltarget;
}

//alternative via wheel speeds and torques
Twist2 DynamicTwoWheelsDifferentialDriveKinematics::feasible(
    const Twist2& target, const Twist2& current, ng_float_t time_step) const {
  if (time_step <= 0) {
    return current;
  }
  Twist2 ltarget = feasible(target);
  ltarget = current.interpolate(ltarget, time_step, max_acceleration,
                                get_max_angular_acceleration());
  auto ts = feasible_wheel_speeds(ltarget);
  const auto cs = feasible_wheel_speeds(current);
  std::array<ng_float_t, 2> wheel_acceleration;
  for (int i = 0; i < 2; ++i) {
    wheel_acceleration[i] = (ts[i] - cs[i]) / time_step;
  }
  std::array<ng_float_t, 2> wheel_force;
  for (int i = 0; i < 2; ++i) {
    wheel_force[i] = ((2 + moi) * wheel_acceleration[i] +
                      (2 - moi) * wheel_acceleration[1 - i]) /
                     8;
  }
  if (reduce_torques) {
    const ng_float_t f = 2 *
                         std::max(abs(wheel_force[0]), abs(wheel_force[1])) /
                         max_acceleration;
    if (f > 1) {
      wheel_force[0] /= f;
      wheel_force[1] /= f;
    }
  } else {
    for (int i = 0; i < 2; ++i) {
      wheel_force[i] = std::clamp(wheel_force[i], -max_acceleration / 2,
                                  max_acceleration / 2);
    }
  }
  for (int i = 0; i < 2; ++i) {
    const ng_float_t acc =
        ((2 + moi) * wheel_force[i] - (2 - moi) * wheel_force[1 - i]) / moi;
    ts[i] = cs[i] + time_step * acc;
  }
  return twist(ts);
}

#endif

// {left, right}
std::vector<ng_float_t>
DynamicTwoWheelsDifferentialDriveKinematics::wheel_torques(
    const Twist2& value, const Twist2& current, ng_float_t time_step) const {
  assert(value.frame == current.frame == Frame::relative);
  if (time_step <= 0) {
    return {0, 0};
  }
  const ng_float_t k = moi * axis / 4;
  const ng_float_t lin_acc =
      (value.velocity[0] - current.velocity[0]) / time_step;
  const ng_float_t ang_acc =
      k * (value.angular_speed - current.angular_speed) / time_step;
  return {lin_acc - ang_acc, lin_acc + ang_acc};
}

// {left, right}
Twist2 DynamicTwoWheelsDifferentialDriveKinematics::twist_from_wheel_torques(
    const std::vector<ng_float_t>& values, const Twist2& current,
    ng_float_t time_step) const {
  const ng_float_t lin_acc = (values[1] + values[0]) / 2;
  const ng_float_t ang_acc = 2 * (values[1] - values[0]) / (moi * axis);
  return {{current.velocity[0] + time_step * lin_acc, 0},
          current.angular_speed + time_step * ang_acc};
}

const std::map<std::string, Property>
    DynamicTwoWheelsDifferentialDriveKinematics::properties =
        WheeledKinematics::properties +
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
            // {"reduce_torques",
            //  make_property<bool,
            //  DynamicTwoWheelsDifferentialDriveKinematics>(
            //      &DynamicTwoWheelsDifferentialDriveKinematics::
            //          get_reduce_torques,
            //      &DynamicTwoWheelsDifferentialDriveKinematics::
            //          set_reduce_torques,
            //      false,
            //      "Whether to scale down torques instead of clipping "
            //      "independently")},
        };

const std::string DynamicTwoWheelsDifferentialDriveKinematics::type =
    register_type<DynamicTwoWheelsDifferentialDriveKinematics>("2WDiffDyn");

}  // namespace navground::core
