/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behavior.h"

#include <iostream>
#include <stdexcept>

namespace navground::core {

Twist2 Behavior::feasible_twist(const Twist2 &value) const {
  if (kinematics) {
    return kinematics->feasible(to_relative(value));
  }
  std::cerr << "Missing kinematics!" << std::endl;
  return Twist2{Vector2::Zero(), 0, Frame::relative};
}

Twist2 Behavior::feasible_twist_from_current(const Twist2 &value,
                                             ng_float_t time_step) const {
  if (kinematics) {
    return kinematics->feasible_from_current(
        to_relative(value), get_twist(Frame::relative), time_step);
  }
  std::cerr << "Missing kinematics!" << std::endl;
  return Twist2{Vector2::Zero(), 0, Frame::relative};
}

Vector2 Behavior::desired_velocity_towards_point(
    [[maybe_unused]] const Vector2 &point, [[maybe_unused]] ng_float_t speed,
    [[maybe_unused]] ng_float_t time_step) {
  return Vector2::Zero();
}

Vector2 Behavior::desired_velocity_towards_velocity(
    [[maybe_unused]] const Vector2 &value,
    [[maybe_unused]] ng_float_t time_step) {
  return Vector2::Zero();
}

Twist2 Behavior::twist_towards_velocity(const Vector2 &absolute_velocity) {
  ng_float_t delta_angle = 0;
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
  const ng_float_t max_w = get_target_angular_speed();
  const auto omega =
      std::clamp(normalize_angle(delta_angle) / rotation_tau, -max_w, max_w);
  return {absolute_velocity, omega, Frame::absolute};
}

Twist2 Behavior::cmd_twist_along_path(Path &path, ng_float_t speed,
                                      ng_float_t dt) {
  const auto &[point, orientation, _] =
      path.get_point(pose.position, path_look_ahead);
  const Vector2 delta = point - pose.position;
  const Vector2 v =
      path_tau ? unit(orientation) * speed + delta / path_tau : delta;
  change(TARGET);
  return cmd_twist_towards_velocity(v.normalized() * speed, dt);
}

Twist2 Behavior::cmd_twist_towards_point(const Vector2 &point, ng_float_t speed,
                                         ng_float_t dt) {
  desired_velocity = desired_velocity_towards_point(point, speed, dt);
  return feasible_twist(twist_towards_velocity(desired_velocity));
}

Twist2 Behavior::cmd_twist_towards_velocity(const Vector2 &velocity,
                                            ng_float_t dt) {
  desired_velocity = desired_velocity_towards_velocity(velocity, dt);
  return feasible_twist(twist_towards_velocity(desired_velocity));
}

Twist2 Behavior::cmd_twist_towards_pose(const Pose2 &pose, ng_float_t speed,
                                        [[maybe_unused]] Radians angular_speed,
                                        ng_float_t dt) {
  return cmd_twist_towards_point(pose.position, speed, dt);
}

Twist2 Behavior::cmd_twist_towards_orientation(Radians orientation,
                                               Radians angular_speed,
                                               ng_float_t dt) {
  const ng_float_t max_w = std::max<ng_float_t>(0, angular_speed);
  const ng_float_t w =
      normalize_angle(orientation - pose.orientation) / rotation_tau;
  return cmd_twist_towards_angular_speed(std::clamp(w, -max_w, max_w), dt);
}

Twist2
Behavior::cmd_twist_towards_angular_speed(ng_float_t angular_speed,
                                          [[maybe_unused]] ng_float_t dt) {
  // TODO(Jerome) Should not need validation ... but
  angular_speed = feasible_angular_speed(angular_speed);
  return {Vector2::Zero(), angular_speed, Frame::relative};
}

Twist2 Behavior::cmd_twist_towards_stopping([[maybe_unused]] ng_float_t dt) {
  return {Vector2::Zero(), 0, Frame::relative};
}

Twist2 Behavior::compute_cmd(ng_float_t dt, std::optional<Frame> frame,
                             bool enforce_feasibility) {
  for (auto &modulation : modulations) {
    if (modulation->get_enabled()) {
      modulation->pre(*this, dt);
    }
  }
  Twist2 cmd = compute_cmd_internal(dt);
  for (auto it = std::rbegin(modulations); it != std::rend(modulations); it++) {
    if ((*it)->get_enabled()) {
      cmd = (*it)->post(*this, dt, cmd);
    }
  }
  if (enforce_feasibility) {
    cmd = feasible_twist_from_current(cmd, dt);
  }
  if (frame) {
    cmd = to_frame(cmd, *frame);
  }
  if (assume_cmd_is_actuated) {
    actuated_twist = cmd;
  }
  return cmd;
}

Twist2 Behavior::compute_cmd_internal(ng_float_t dt) {
  const auto point = get_target_position(Frame::absolute);
  const auto orientation = get_target_orientation(Frame::absolute);
  if (point && target.path) {
    return cmd_twist_along_path(*(target.path), get_target_speed(), dt);
  }
  if (point && orientation) {
    return cmd_twist_towards_pose(Pose2(*point, *orientation),
                                  get_target_speed(),
                                  get_target_angular_speed(), dt);
  }
  if (point) {
    return cmd_twist_towards_point(*point, get_target_speed(), dt);
  }
  if (orientation) {
    return cmd_twist_towards_orientation(*orientation,
                                         get_target_angular_speed(), dt);
  }
  if (target.direction) {
    return cmd_twist_towards_velocity(get_target_velocity(Frame::absolute), dt);
  }
  if (target.angular_speed) {
    return cmd_twist_towards_angular_speed(get_target_angular_speed(), dt);
  }
  return cmd_twist_towards_stopping(dt);
}

bool Behavior::check_if_target_satisfied() const {
  return target.satisfied(pose);
}

ng_float_t Behavior::estimate_time_until_target_satisfied() const {
  ng_float_t time = 0;
  const auto dist = get_target_distance(false);
  if (dist) {
    const ng_float_t speed = get_target_speed();
    if (speed) {
      time += (*dist) / speed;
    } else {
      return std::numeric_limits<ng_float_t>::infinity();
    }
  }
  const auto ang_dist = get_target_orientation(Frame::relative);
  if (ang_dist) {
    const ng_float_t angular_speed = get_target_angular_speed();
    if (angular_speed) {
      time += (*ang_dist) / angular_speed;
    } else {
      return std::numeric_limits<ng_float_t>::infinity();
    }
  }
  return time;
}

bool Behavior::should_stop() const {
  if (!target.valid())
    return true;
  const auto speed = get_target_speed();
  if (target.position && !target.satisfied(pose.position) && speed)
    return false;
  const auto angular_speed = get_target_angular_speed();
  if (target.orientation && !target.satisfied(pose.orientation) &&
      angular_speed)
    return false;
  if (target.direction && speed)
    return false;
  if (target.angular_speed.value_or(0))
    return false;
  return true;
}

bool Behavior::is_stopped(ng_float_t epsilon_speed,
                          ng_float_t epsilon_angular_speed) const {
  return twist.is_almost_zero(epsilon_speed, epsilon_angular_speed) &&
         actuated_twist.is_almost_zero(epsilon_speed, epsilon_angular_speed);
}

bool Behavior::is_stuck() const { return !should_stop() && is_stopped(); }

ng_float_t Behavior::get_efficacy() const {
  const auto v = get_target_velocity(Frame::absolute);
  if (!v.norm())
    return 0;
  return v.dot(twist.velocity) / v.squaredNorm();
}

std::optional<Vector2> Behavior::get_target_position(Frame frame) const {
  if (target.position && !target.satisfied(pose.position)) {
    if (frame == Frame::relative) {
      return to_relative(*(target.position) - pose.position);
    }
    return *(target.position);
  }
  return std::nullopt;
}

std::optional<ng_float_t> Behavior::get_target_orientation(Frame frame) const {
  if (target.orientation && !target.satisfied(pose.orientation)) {
    if (frame == Frame::relative) {
      return normalize_angle(*(target.orientation) - pose.orientation);
    }
    return *(target.orientation);
  }
  return std::nullopt;
}

std::optional<Vector2> Behavior::get_target_direction(Frame frame) const {
  std::optional<Vector2> e = std::nullopt;
  if (target.position && !target.satisfied(pose.position)) {
    e = (*(target.position) - pose.position).normalized();
  } else if (target.direction) {
    e = target.direction->normalized();
  }
  if (e && frame == Frame::relative) {
    return to_relative(*e);
  }
  return e;
}

std::optional<ng_float_t>
Behavior::get_target_distance(bool ignore_tolerance) const {
  const auto delta = get_target_position(Frame::relative);
  if (delta) {
    ng_float_t distance = delta->norm();
    if (!ignore_tolerance) {
      distance -= target.position_tolerance;
    }
    if (target.path && target.path->coordinate >= 0) {
      distance =
          std::min(distance, target.path->length - target.path->coordinate -
                                 target.position_tolerance);
    }
    return std::max<ng_float_t>(0, distance);
  }
  return std::nullopt;
}

Vector2 Behavior::get_target_velocity(Frame frame) const {
  const auto e = get_target_direction(frame);
  if (e) {
    return (*e) * get_target_speed();
  }
  return Vector2::Zero();
}

ng_float_t Behavior::get_target_speed() const {
  return feasible_speed(target.speed.value_or(optimal_speed));
}

ng_float_t Behavior::get_target_angular_speed() const {
  return feasible_angular_speed(
      target.angular_speed.value_or(optimal_angular_speed));
}

void Behavior::set_state_from(const Behavior &other) {
  set_kinematics(other.get_kinematics());
  set_radius(other.get_radius());
  set_optimal_speed(other.get_optimal_speed());
  set_optimal_angular_speed(other.get_optimal_angular_speed());
  set_rotation_tau(other.get_rotation_tau());
  set_safety_margin(other.get_safety_margin());
  set_horizon(other.get_horizon());
  set_assume_cmd_is_actuated(other.get_assume_cmd_is_actuated());
  set_heading_behavior(other.get_heading_behavior());
  set_target(other.get_target());
  set_pose(other.get_pose());
  set_twist(other.get_twist());
  set_actuated_twist(other.get_actuated_twist());
}

void Behavior::actuate(const Twist2 &twist_cmd, ng_float_t time_step,
                       bool enforce_feasibility) {
  if (enforce_feasibility) {
    actuated_twist = feasible_twist_from_current(twist_cmd, time_step);
  } else {
    actuated_twist = twist_cmd;
  }
  twist = to_absolute(actuated_twist);
  pose = pose.integrate(twist, time_step);
  change(POSITION | ORIENTATION | VELOCITY | ANGULAR_SPEED);
}

const std::string Behavior::type = register_type<Behavior>("");

} // namespace navground::core
