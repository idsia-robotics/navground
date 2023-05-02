/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_TARGET_H_
#define NAVGROUND_CORE_TARGET_H_

#include <optional>

#include "navground/core/common.h"

namespace navground::core {

// C++
// Target::Point(position, position_tolerance);
// python
// target = Target(position = position)

struct Target {
  /**
   * The position
   */
  std::optional<Vector2> position = std::nullopt;
  /**
   * The orientation
   */
  std::optional<Radians> orientation = std::nullopt;
  /**
   * The speed
   */
  std::optional<float> speed = std::nullopt;
  /**
   * The direction
   */
  std::optional<Vector2> direction = std::nullopt;
  /**
   * The angular speed
   */
  std::optional<float> angular_speed = std::nullopt;
  /**
   * The position tolerance
   */
  float position_tolerance = 0.0f;
  /**
   * The orientation tolerance
   */
  float orientation_tolerance = 0.0f;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  position               The position
   * @param[in]  orientation            The orientation
   * @param[in]  speed                  The speed
   * @param[in]  direction              The direction
   * @param[in]  angular_speed          The angular speed
   * @param[in]  position_tolerance     The position tolerance
   * @param[in]  orientation_tolerance  The orientation tolerance
   */
  Target(const std::optional<Vector2>& position = std::nullopt,
         const std::optional<Radians>& orientation = std::nullopt,
         const std::optional<float>& speed = std::nullopt,
         const std::optional<Vector2>& direction = std::nullopt,
         const std::optional<float>& angular_speed = std::nullopt,
         float position_tolerance = 0.0f, float orientation_tolerance = 0.0f)
      : position(position),
        orientation(orientation),
        speed(speed),
        direction(direction),
        angular_speed(angular_speed),
        position_tolerance(position_tolerance),
        orientation_tolerance(orientation_tolerance) {}

  bool valid() const {
    return position.has_value() || orientation.has_value() ||
           direction.has_value() || angular_speed.has_value();
  }

  bool satisfied(Radians value) const {
    return !orientation ||
           abs(normalize(*orientation - value)) < orientation_tolerance;
  }

  bool satisfied(const Vector2& value) const {
    return !position || (*position - value).norm() < position_tolerance;
  }

  bool satisfied(const Pose2& pose) const {
    return satisfied(pose.position) && satisfied(pose.orientation);
  }

  static Target Point(const Vector2& position, float tolerance = 0.0f) {
    Target t;
    t.position = position;
    t.position_tolerance = tolerance;
    return t;
  }

  static Target Pose(const Pose2& pose, float position_tolerance = 0.0f,
                     float orientation_tolerance = 0.0f) {
    Target t;
    t.position = pose.position;
    t.position_tolerance = position_tolerance;
    t.orientation = pose.orientation;
    t.orientation_tolerance = orientation_tolerance;
    return t;
  }

  static Target Orientation(Radians orientation, Radians tolerance = 0.0f) {
    Target t;
    t.orientation = orientation;
    t.orientation_tolerance = tolerance;
    return t;
  }

  static Target Velocity(const Vector2& velocity) {
    Target t;
    t.speed = velocity.norm();
    t.direction = velocity;
    return t;
  }

  static Target Direction(const Vector2& direction) {
    Target t;
    t.direction = direction;
    return t;
  }

  static Target Twist(const Twist2& twist) {
    Target t;
    t.speed = twist.velocity.norm();
    t.direction = twist.velocity;
    t.angular_speed = twist.angular_speed;
    return t;
  }

  static Target Stop() { return {}; }
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_H_