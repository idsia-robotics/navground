/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_TARGET_H_
#define NAVGROUND_CORE_TARGET_H_

#include <optional>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include <functional>
#include <tuple>

namespace navground::core {

/**
 * @brief      A generic, twice differentiable, curve (to follow),
 *             parametrized by length.
 */
struct Path {

  using Point = std::tuple<Vector2, float, float>;
  /**
   * A function that finds the nearest point on the curve in a given interval.
   *
   * (point, interval_start, interval_end) -> curve coordinate
   */
  using Projection =
      std::function<ng_float_t(const Vector2 &, ng_float_t, ng_float_t)>;
  /**
   * A function that return the point, orientation of the tangential vector,
   * and curvature at a given coordinate.
   *
   * (coordinate) -> (point, orientation, curvature)
   */
  using Curve = std::function<Point(ng_float_t)>;

  /**
   * The projection
   */
  Projection project;
  /**
   * The parametrized curve
   */
  Curve curve;
  /**
   * The current coordinate during tracking.
   * A negative value means undefined.
   */
  ng_float_t coordinate;
  /**
   * The curve length
   */
  ng_float_t length;
  /**
   * Whether the path is an infinite loop
   */
  bool loop;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  projection  The projection
   * @param[in]  curve_fn    The parametrized curve
   * @param[in]  length      The curve length
   * @param[in]  loop        Whether the path is an infinite loop
   */
  Path(const Projection &projection, const Curve &curve_fn, ng_float_t length,
       bool loop = false)
      : project(projection), curve(curve_fn), coordinate(-1), length(length),
        loop(loop) {}

  /**
   * @brief      Returns the coordinate ahead of a position
   *
   * It projects the point to the curve using \ref project,
   * updates \ref coordinate with this value.
   * It returns the value advanced by the look-ahead distance.
   *
   *
   * @param[in]  position    The position
   * @param[in]  look_ahead  The look ahead distance
   *
   * @return     The ng float.
   */
  ng_float_t track(const Vector2 &position, ng_float_t look_ahead = 0) {
    if (coordinate < 0) {
      coordinate = project(position, 0, length);
    } else {
      const ng_float_t end = coordinate + std::max<ng_float_t>(1, look_ahead);
      if (loop && end > length) {
        const ng_float_t s1 = project(position, coordinate, length);
        const ng_float_t s2 = project(position, 0, end - length);
        const ng_float_t d1 = (std::get<0>(curve(s1)) - position).norm();
        const ng_float_t d2 = (std::get<0>(curve(s2)) - position).norm();
        if (d1 < d2) {
          coordinate = s1;
        } else {
          coordinate = s2;
        }
      } else {
        coordinate = project(position, coordinate,
                             coordinate + std::max<ng_float_t>(1, look_ahead));
      }
    }
    if (loop) {
      return std::fmod(coordinate + look_ahead, length);
    }
    return std::min(length, coordinate + look_ahead);
  }

  /**
   * @brief      Returns the point the point corresponding the
   * \ref track coordinate.
   *
   * @param[in]  position    The position
   * @param[in]  look_ahead  The look ahead distance
   *
   * @return     The point.
   */
  Point get_point(const Vector2 &position, ng_float_t look_ahead = 0) {
    const auto s = track(position, look_ahead);
    return curve(s);
  }
};

// C++
// Target::Point(position, position_tolerance);
// python
// target = Target(position = position)

/**
 * @brief      Represents the union of all targets supported
 *             by a generic \ref Behavior:
 *             - poses
 *             - positions
 *             - orientations
 *             - directions
 *             - velocities
 *             - twists
 *             - paths to be followed to reach a position
 *
 */
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
  std::optional<ng_float_t> speed = std::nullopt;
  /**
   * The direction
   */
  std::optional<Vector2> direction = std::nullopt;
  /**
   * The angular speed
   */
  std::optional<ng_float_t> angular_speed = std::nullopt;
  /**
   * The path to follow.
   */
  std::optional<Path> path = std::nullopt;
  /**
   * The position tolerance
   */
  ng_float_t position_tolerance = 0;
  /**
   * The orientation tolerance
   */
  ng_float_t orientation_tolerance = 0;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  position               The position
   * @param[in]  orientation            The orientation
   * @param[in]  speed                  The speed
   * @param[in]  direction              The direction
   * @param[in]  angular_speed          The angular speed
   * @param[in]  path                   The path
   * @param[in]  position_tolerance     The position tolerance
   * @param[in]  orientation_tolerance  The orientation tolerance
   */
  Target(const std::optional<Vector2> &position = std::nullopt,
         const std::optional<Radians> &orientation = std::nullopt,
         const std::optional<ng_float_t> &speed = std::nullopt,
         const std::optional<Vector2> &direction = std::nullopt,
         const std::optional<ng_float_t> &angular_speed = std::nullopt,
         const std::optional<Path> &path = std::nullopt,
         ng_float_t position_tolerance = 0,
         ng_float_t orientation_tolerance = 0)
      : position(position), orientation(orientation), speed(speed),
        direction(direction), angular_speed(angular_speed), path(path),
        position_tolerance(position_tolerance),
        orientation_tolerance(orientation_tolerance) {}

  /**
   * @brief      Verify that a target is set.
   *
   * @return     True if the target is not empty.
   */
  bool valid() const {
    return position.has_value() || orientation.has_value() ||
           direction.has_value() || angular_speed.has_value();
  }

  /**
   * @brief      Returns whether a given orientation satisfies the target.
   *
   * The target is satisfied if it does not have an angular speed and
   * if the given value is near to the target's orientation.
   *
   * @param[in]  value  The value
   *
   * @return     True if the target is satisfied.
   */
  bool satisfied(Radians value) const {
    if (angular_speed && *angular_speed > 0) {
      return false;
    }
    return !orientation || std::abs(normalize_angle(*orientation - value)) <
                               orientation_tolerance;
  }

  /**
   * @brief      Returns whether a given position satisfies the target.
   *
   * The target is satisfied if it does not have an linear speed and
   * if the given value is near to the target's position.
   *
   * @param[in]  value  The value
   *
   * @return     True if the target is satisfied.
   */
  bool satisfied(const Vector2 &value) const {
    if (speed && *speed > 0) {
      return false;
    }
    return !position || (*position - value).norm() < position_tolerance;
  }

  /**
   * @brief      Returns whether a given pose satisfies the target.
   *
   * The target is satisfied if the position and orientation are both satisfied.
   *
   * @param[in]  value  The value
   *
   * @return     True if the target is satisfied.
   */
  bool satisfied(const Pose2 &pose) const {
    return satisfied(pose.position) && satisfied(pose.orientation);
  }

  // TODO(Jerome): extend to orientation / angular_speed
  // Vector2 get_ideal_velocity(const Vector2& position_,
  //                            ng_float_t speed_) const {
  //   if (position) {
  //     const auto delta = *position - position_;
  //     const auto n = delta.norm();
  //     if (n > 0 && n > position_tolerance) {
  //       return delta / n * speed.value_or(speed_);
  //     }
  //   }
  //   if (direction) {
  //     return *direction * speed.value_or(speed_);
  //   }
  //   return Vector2::Zero();
  // }

  /**
   * @brief      Constructs a target to reach a point.
   *
   * @param[in]  position    The target position
   * @param[in]  tolerance   The tolerance
   * @param[in]  along_path  The path to follow
   *
   * @return     The target
   */
  static Target Point(const Vector2 &position, ng_float_t tolerance = 0,
                      std::optional<Path> along_path = std::nullopt) {
    Target t;
    t.position = position;
    t.position_tolerance = tolerance;
    t.path = along_path;
    return t;
  }

  /**
   * @brief      Constructs a target to reach a pose.
   *
   * @param[in]  pose                   The pose
   * @param[in]  position_tolerance     The position tolerance
   * @param[in]  orientation_tolerance  The orientation tolerance
   * @param[in]  along_path             The path to follow
   *
   * @return     The target
   */
  static Target Pose(const Pose2 &pose, ng_float_t position_tolerance = 0,
                     ng_float_t orientation_tolerance = 0,
                     std::optional<Path> along_path = std::nullopt) {
    Target t;
    t.position = pose.position;
    t.position_tolerance = position_tolerance;
    t.orientation = pose.orientation;
    t.orientation_tolerance = orientation_tolerance;
    t.path = along_path;
    return t;
  }

  /**
   * @brief      Constructs a target to reach an orietation.
   *
   * @param[in]  orientation  The orientation
   * @param[in]  tolerance    The tolerance
   *
   * @return     The target
   */
  static Target Orientation(Radians orientation, Radians tolerance = 0) {
    Target t;
    t.orientation = orientation;
    t.orientation_tolerance = tolerance;
    return t;
  }

  /**
   * @brief      Constructs a target to follow a velocity.
   *
   * @param[in]  velocity  The velocity
   *
   * @return     The target
   */
  static Target Velocity(const Vector2 &velocity) {
    Target t;
    t.speed = velocity.norm();
    t.direction = velocity;
    return t;
  }

  /**
   * @brief      Constructs a target to follow a direction.
   *
   * @param[in]  direction  The direction
   *
   * @return     The target
   */
  static Target Direction(const Vector2 &direction) {
    Target t;
    t.direction = direction;
    return t;
  }

  /**
   * @brief      Constructs a target to follow a twist.
   *
   * @param[in]  twist  The twist
   *
   * @return     The target
   */
  static Target Twist(const Twist2 &twist) {
    Target t;
    t.speed = twist.velocity.norm();
    t.direction = twist.velocity;
    t.angular_speed = twist.angular_speed;
    return t;
  }

  /**
   * @brief      Constructs a target to stop.
   *
   * @return     The target
   */
  static Target Stop() { return {}; }
};

} // namespace navground::core

#endif // NAVGROUND_CORE_BEHAVIOR_H_