/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior.h"

#include <iostream>
#include <stdexcept>

namespace navground::core {

Twist2 Behavior::feasible_twist(const Twist2& value,
                                std::optional<Frame> frame) const {
  if (kinematics) {
    Twist2 f_value;
    if (kinematics->is_wheeled() && value.frame == Frame::absolute) {
      f_value = kinematics->feasible(to_relative(value));
    } else {
      f_value = kinematics->feasible(value);
    }
    return to_frame(f_value, frame.value_or(f_value.frame));
  }
  return Twist2{Vector2::Zero(), 0, frame.value_or(value.frame)};
}

Vector2 Behavior::desired_velocity_towards_point(
    [[maybe_unused]] const Vector2& point, [[maybe_unused]] ng_float_t speed,
    [[maybe_unused]] ng_float_t time_step) {
  return Vector2::Zero();
}

Vector2 Behavior::desired_velocity_towards_velocity(
    [[maybe_unused]] const Vector2& value,
    [[maybe_unused]] ng_float_t time_step) {
  return Vector2::Zero();
}

Twist2 Behavior::twist_towards_velocity(const Vector2& absolute_velocity,
                                        Frame frame) {
  ng_float_t delta_angle = 0;
  Twist2 twist;
  twist.frame = frame;
  if (frame == Frame::relative) {
    twist.velocity = to_relative(absolute_velocity);
  } else {
    twist.velocity = absolute_velocity;
  }
  switch (get_heading_behavior()) {
    case Heading::velocity:
      if (absolute_velocity.norm() == 0) {
        delta_angle = 0;
      } else {
        delta_angle = orientation_of(absolute_velocity) - pose.orientation;
      }
      break;
    case Heading::target_angle:
      if (target.orientation) {
        delta_angle = *(target.orientation) - pose.orientation;
      }
      break;
    case Heading::target_point:
      if (target.position) {
        delta_angle =
            orientation_of(*target.position - pose.position) - pose.orientation;
      }
      break;
    default:
      break;
  }
  const ng_float_t max_w = feasible_angular_speed(
      target.angular_speed.value_or(optimal_angular_speed));
  twist.angular_speed =
      std::clamp(normalize(delta_angle) / rotation_tau, -max_w, max_w);
  return twist;
}

Twist2 Behavior::cmd_twist_towards_point(const Vector2& point, ng_float_t speed,
                                         ng_float_t dt, Frame frame) {
  desired_velocity = desired_velocity_towards_point(point, speed, dt);
  const Twist2 desired_twist =
      twist_towards_velocity(desired_velocity, Frame::relative);
  return feasible_twist(desired_twist, frame);
}

Twist2 Behavior::cmd_twist_towards_velocity(const Vector2& velocity,
                                            ng_float_t dt, Frame frame) {
  desired_velocity = desired_velocity_towards_velocity(velocity, dt);
  const Twist2 desired_twist =
      twist_towards_velocity(desired_velocity, Frame::relative);
  return feasible_twist(desired_twist, frame);
}

Twist2 Behavior::cmd_twist_towards_orientation(Radians orientation,
                                               Radians angular_speed,
                                               ng_float_t dt, Frame frame) {
  const ng_float_t max_w = std::max<ng_float_t>(0, angular_speed);
  const ng_float_t w = normalize(orientation - pose.orientation) / rotation_tau;
  return cmd_twist_towards_angular_speed(std::clamp(w, -max_w, max_w), dt,
                                         frame);
}

Twist2 Behavior::cmd_twist_towards_angular_speed(ng_float_t angular_speed,
                                                 [[maybe_unused]] ng_float_t dt,
                                                 Frame frame) {
  // TODO(Jerome) Should not need validation ... but
  angular_speed = feasible_angular_speed(angular_speed);
  return {Vector2::Zero(), angular_speed, frame};
}

Twist2 Behavior::cmd_twist_towards_stopping([[maybe_unused]] ng_float_t dt,
                                            Frame frame) {
  return {Vector2::Zero(), 0, frame};
}

Twist2 Behavior::compute_cmd(ng_float_t dt, std::optional<Frame> _frame) {
  for (auto& modulation : modulations) {
    if (modulation->get_enabled()) {
      modulation->pre(*this, dt);
    }
  }
  if (!kinematics) {
    std::cerr << "Missing kinematics!" << std::endl;
    return {};
  }
  Frame frame = _frame.value_or(default_cmd_frame());
  Twist2 twist;
  if (target.position && !target.satisfied(pose.position)) {
    const ng_float_t speed =
        feasible_speed(target.speed.value_or(optimal_speed));
    twist = cmd_twist_towards_point(*(target.position), speed, dt, frame);
  } else if (target.orientation && !target.satisfied(pose.orientation)) {
    const ng_float_t angular_speed = feasible_angular_speed(
        target.angular_speed.value_or(optimal_angular_speed));
    twist = cmd_twist_towards_orientation(*(target.orientation), angular_speed,
                                          dt, frame);
  } else if (target.direction) {
    const ng_float_t n = target.direction->norm();
    Vector2 velocity = *(target.direction);
    if (n) {
      const ng_float_t speed = target.speed.value_or(optimal_speed);
      velocity *= speed / n;
    }
    twist = cmd_twist_towards_velocity(velocity, dt, frame);
  } else if (target.angular_speed) {
    const ng_float_t angular_speed = feasible_angular_speed(
        target.angular_speed.value_or(optimal_angular_speed));
    twist = cmd_twist_towards_angular_speed(angular_speed, dt, frame);
  }
  if (assume_cmd_is_actuated) {
    actuated_twist = twist;
  }
  for (auto it = std::rbegin(modulations); it != std::rend(modulations); it++) {
    if ((*it)->get_enabled()) {
      twist = (*it)->post(*this, dt, twist);
    }
  }
  return twist;
}

bool Behavior::check_if_target_satisfied() const {
  return target.satisfied(pose);
}

ng_float_t Behavior::estimate_time_until_target_satisfied() const {
  ng_float_t time = 0;
  if (!target.satisfied(pose.position)) {
    const ng_float_t speed =
        feasible_speed(target.speed.value_or(optimal_speed));
    if (speed) {
      time += (*target.position - pose.position).norm() / speed;
    } else {
      return std::numeric_limits<ng_float_t>::infinity();
    }
  }
  if (!target.satisfied(pose.orientation)) {
    const ng_float_t angular_speed = feasible_angular_speed(
        target.angular_speed.value_or(optimal_angular_speed));
    if (angular_speed) {
      time += normalize(*target.orientation - pose.orientation) / angular_speed;
    } else {
      return std::numeric_limits<ng_float_t>::infinity();
    }
  }
  return time;
}

bool Behavior::should_stop() const {
  if (!target.valid()) return true;
  const auto speed = target.speed.value_or(optimal_speed);
  if (target.position && !target.satisfied(pose.position) && speed)
    return false;
  const auto angular_speed =
      target.angular_speed.value_or(optimal_angular_speed);
  if (target.orientation && !target.satisfied(pose.orientation) &&
      angular_speed)
    return false;
  if (target.direction && speed) return false;
  if (target.angular_speed.value_or(0)) return false;
  return true;
}

bool Behavior::is_stopped(ng_float_t epsilon_speed,
                          ng_float_t epsilon_angular_speed) const {
  return twist.is_almost_zero(epsilon_speed, epsilon_angular_speed) &&
         actuated_twist.is_almost_zero(epsilon_speed, epsilon_angular_speed);
}

bool Behavior::is_stuck() const { return !should_stop() && is_stopped(); }

ng_float_t Behavior::get_efficacy() const {
  const auto v = target.get_ideal_velocity(pose.position, optimal_speed);
  if (!v.norm()) return 1;
  return v.dot(twist.velocity) / v.squaredNorm();
}

const std::string Behavior::type = register_type<Behavior>("");

}  // namespace navground::core
