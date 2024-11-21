/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/common.h"

namespace navground::core {

// Exact integration
Pose2 Pose2::integrate(const Twist2 &twist, ng_float_t dt) {
  const auto v = twist.frame == Frame::relative
                     ? transform_vector(twist.velocity)
                     : twist.velocity;
  if (twist.angular_speed) {
    const ng_float_t c = std::cos(twist.angular_speed * dt) - 1;
    const ng_float_t s = std::sin(twist.angular_speed * dt);
    const Eigen::Matrix<ng_float_t, 2, 2> m{{s, c}, {-c, s}};
    return {position + m * v / twist.angular_speed,
            orientation + dt * twist.angular_speed};
  }
  return {position + v * dt, orientation};
}

// Pose2 Pose2::integrate(const Twist2 &twist, ng_float_t dt) {
//   const auto new_orientation = orientation + dt * twist.angular_speed;
//   return {position + dt * (twist.frame == Frame::relative
//                                ? ::navground::core::rotate(twist.velocity,
//                                                            new_orientation)
//                                : twist.velocity),
//           new_orientation};
// }

Twist2 Twist2::relative(const Pose2 &reference) const {
  if (frame == Frame::absolute) {
    auto r_value = rotate(-reference.orientation);
    r_value.frame = Frame::relative;
    return r_value;
  }
  return *this;
}

Twist2 Twist2::absolute(const Pose2 &reference) const {
  if (frame == Frame::relative) {
    auto a_value = rotate(reference.orientation);
    a_value.frame = Frame::absolute;
    return a_value;
  }
  return *this;
}

Twist2 Twist2::to_frame(Frame frame, const Pose2 &reference) const {
  return frame == Frame::relative ? relative(reference) : absolute(reference);
}

Twist2 Twist2::interpolate(const Twist2 &target, ng_float_t time_step,
                           ng_float_t max_acceleration,
                           ng_float_t max_angular_acceleration) const {
  assert(target.frame == frame);
  if (time_step <= 0) {
    return Twist2(velocity, angular_speed);
  }
  Vector2 acc = (target.velocity - velocity) / time_step;
  auto ang_acc = (target.angular_speed - angular_speed) / time_step;
  if (acc.norm() > max_acceleration) {
    acc = acc.normalized() * max_acceleration;
  }
  if (std::abs(ang_acc) > max_angular_acceleration) {
    ang_acc = std::clamp(ang_acc, -max_angular_acceleration,
                         max_angular_acceleration);
  }
  return Twist2(velocity + acc * time_step, angular_speed + ang_acc * time_step,
                frame);
}

void Twist2::snap_to_zero(ng_float_t epsilon) {
  if (std::abs(velocity[0]) < epsilon) {
    velocity[0] = 0;
  }
  if (std::abs(velocity[1]) < epsilon) {
    velocity[1] = 0;
  }
  if (std::abs(angular_speed) < epsilon) {
    angular_speed = 0;
  }
}

} // namespace navground::core
