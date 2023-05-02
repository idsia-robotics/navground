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
  return Twist2{Vector2::Zero(), 0.0f, frame.value_or(value.frame)};
}

Vector2 Behavior::desired_velocity_towards_point(
    [[maybe_unused]] const Vector2& point, [[maybe_unused]] float speed,
    [[maybe_unused]] float time_step) {
  return Vector2::Zero();
}

Vector2 Behavior::desired_velocity_towards_velocity(
    [[maybe_unused]] const Vector2& value, [[maybe_unused]] float time_step) {
  return Vector2::Zero();
}

Twist2 Behavior::twist_towards_velocity(const Vector2& absolute_velocity,
                                        Frame frame) {
  float delta_angle = 0.0f;
  Twist2 twist;
  twist.frame = frame;
  if (frame == Frame::relative) {
    twist.velocity = to_relative(absolute_velocity);
  } else {
    twist.velocity = absolute_velocity;
  }
  switch (get_heading_behavior()) {
    case Heading::velocity:
      delta_angle = orientation_of(absolute_velocity) - pose.orientation;
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
  const float max_w = feasible_angular_speed(
      target.angular_speed.value_or(optimal_angular_speed));
  twist.angular_speed =
      std::clamp(normalize(delta_angle) / rotation_tau, -max_w, max_w);
  return twist;
}

Twist2 Behavior::cmd_twist_towards_point(const Vector2& point, float speed,
                                         float dt, Frame frame) {
  desired_velocity = desired_velocity_towards_point(point, speed, dt);
  const Twist2 desired_twist =
      twist_towards_velocity(desired_velocity, Frame::relative);
  return feasible_twist(desired_twist, frame);
}

Twist2 Behavior::cmd_twist_towards_velocity(const Vector2& velocity, float dt,
                                            Frame frame) {
  desired_velocity = desired_velocity_towards_velocity(velocity, dt);
  const Twist2 desired_twist =
      twist_towards_velocity(desired_velocity, Frame::relative);
  return feasible_twist(desired_twist, frame);
}

Twist2 Behavior::cmd_twist_towards_orientation(Radians orientation,
                                               Radians angular_speed, float dt,
                                               Frame frame) {
  const float max_w = std::max(0.0f, angular_speed);
  const float w = normalize(orientation - pose.orientation) / rotation_tau;
  return cmd_twist_towards_angular_speed(std::clamp(w, -max_w, max_w), dt,
                                         frame);
}

Twist2 Behavior::cmd_twist_towards_angular_speed(float angular_speed,
                                                 [[maybe_unused]] float dt,
                                                 Frame frame) {
  // TODO(Jerome) Should not need validation ... but
  angular_speed = feasible_angular_speed(angular_speed);
  return {Vector2::Zero(), angular_speed, frame};
}

Twist2 Behavior::cmd_twist_towards_stopping([[maybe_unused]] float dt,
                                            Frame frame) {
  return {Vector2::Zero(), 0.0f, frame};
}

Twist2 Behavior::compute_cmd(float dt, std::optional<Frame> _frame) {
  if (!kinematics) {
    std::cerr << "Missing kinematics!" << std::endl;
    return {};
  }
  Frame frame = _frame.value_or(default_cmd_frame());
  Twist2 twist;
  if (target.position && !target.satisfied(pose.position)) {
    const float speed = feasible_speed(target.speed.value_or(optimal_speed));
    twist = cmd_twist_towards_point(*(target.position), speed, dt, frame);
  } else if (target.orientation && !target.satisfied(pose.orientation)) {
    const float angular_speed = feasible_angular_speed(
        target.angular_speed.value_or(optimal_angular_speed));
    twist = cmd_twist_towards_orientation(*(target.orientation), angular_speed,
                                          dt, frame);
  } else if (target.direction) {
    const float n = target.direction->norm();
    Vector2 velocity = *(target.direction);
    if (n) {
      const float speed = target.speed.value_or(optimal_speed);
      velocity *= speed / n;
    }
    twist = cmd_twist_towards_velocity(velocity, dt, frame);
  } else if (target.angular_speed) {
    const float angular_speed = feasible_angular_speed(
        target.angular_speed.value_or(optimal_angular_speed));
    twist = cmd_twist_towards_angular_speed(angular_speed, dt, frame);
  }
  if (assume_cmd_is_actuated) {
    actuated_twist = twist;
  }
  return twist;
}

bool Behavior::check_if_target_satisfied() const {
  return target.satisfied(pose);
}

float Behavior::estimate_time_until_target_satisfied() const {
  float time = 0.0f;
  if (!target.satisfied(pose.position)) {
    const float speed = feasible_speed(target.speed.value_or(optimal_speed));
    if (speed) {
      time += (*target.position - pose.position).norm() / speed;
    } else {
      return std::numeric_limits<float>::infinity();
    }
  }
  if (!target.satisfied(pose.orientation)) {
    const float angular_speed =
        feasible_angular_speed(target.angular_speed.value_or(optimal_speed));
    if (angular_speed) {
      time += normalize(*target.orientation - pose.orientation) / angular_speed;
    } else {
      return std::numeric_limits<float>::infinity();
    }
  }
  return time;
}

}  // namespace navground::core
