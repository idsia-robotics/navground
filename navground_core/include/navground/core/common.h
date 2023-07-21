/**
 * @author Jérôme Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_COMMON_H_
#define NAVGROUND_CORE_COMMON_H_

#include <math.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <ostream>

typedef unsigned int uint;

namespace navground::core {

/**
 * A two-dimensional vector, see <a
 * href="https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html">Eigen</a>
 */
using Vector2 = Eigen::Vector2f;

/**
 * Angle in radians.
 */
using Radians = float;

enum class Frame {
  /**
   * agent-fixed frame
   */
  relative,
  /**
   * world-fixed frame
   */
  absolute
};

/**
 * A vector that holds wheel speeds. The order depends on the kinematics.
 */
using WheelSpeeds = std::vector<float>;

/**
 * @brief      The orientation of a two dimensional vector
 *
 * @param[in]  vector The vector
 *
 * @return     The orientation of the vector
 */
inline Radians orientation_of(const Vector2& vector) {
  return std::atan2(vector.y(), vector.x());
}

/**
 * @brief      Normalize an angle to a value in \f$[-\pi, \pi]\f$.
 * Apply it when computing angular differences.
 *
 * @param[in]  value The original unbounded angle
 *
 * @return     The equivalent angle in \f$[-\pi, \pi]\f$
 */
inline Radians normalize(Radians value) {
  value = std::fmod(value, 2 * M_PI);
  if (value < -M_PI) {
    value += 2 * M_PI;
  } else if (value > M_PI) {
    value -= 2 * M_PI;
  }
  return value;
}

/**
 * @brief      Unit vector with a desired orientated.
 *
 * @param[in]  angle The desired orientation
 *
 * @return     Vector of norm one and desired orientation
 */
inline Vector2 unit(float angle) { return {cosf(angle), sinf(angle)}; }

/**
 * @brief      Rotate a two-dimensional vector.
 *
 * @param[in]  vector The original vector
 * @param[in]  angle  The rotation angle in radians
 *
 * @return     The rotated vector
 */
inline Vector2 rotate(const Vector2 vector, float angle) {
  Eigen::Rotation2D<float> rot(angle);
  return rot * vector;
}

/**
 * @brief      Clamp the norm of a vector
 *
 * @param[in]  vector The original vector
 * @param[in]  max_length  The maximum length
 *
 * @return     A vector with the original orientation but norm clamped to
 * ``max_length``.
 */
inline Vector2 clamp_norm(const Vector2& vector, float max_length) {
  float n = vector.norm();
  if (n > 0 && n > max_length) {
    return vector / n * max_length;
  }
  return vector;
}

/**
 * @brief      Two-dimensional twist composed of planar velocity and angular
 * speed.
 *
 * Twist coordinates are in the frame specified by \ref frame.
 */
struct Twist2 {
  /**
   * Velocity
   */
  Vector2 velocity;
  /**
   * Angular speed
   */
  Radians angular_speed;
  /**
   * Frame of reference.
   */
  Frame frame;

  Twist2(const Vector2& velocity, Radians angular_speed = 0.0,
         Frame frame = Frame::absolute)
      : velocity(velocity), angular_speed(angular_speed), frame(frame) {}
  Twist2() : Twist2({0.0f, 0.0f}) {}
  /**
   * @brief      Rotate the twist by an angle.
   *
   * @param[in]  angle  The rotation angle in radians.
   *
   * @return     The rotated twist.
   */
  Twist2 rotate(Radians angle) const {
    return {::navground::core::rotate(velocity, angle), angular_speed};
  }

  /**
   * @brief      Equality operator.
   *
   * @param[in]  other  The other twist
   *
   * @return     The result of the equality
   */
  bool operator==(const Twist2& other) const {
    return velocity == other.velocity && angular_speed == other.angular_speed &&
           frame == other.frame;
  }

  /**
   * @brief      Inequality operator.
   *
   * @param[in]  other  The other twist
   *
   * @return     The result of the inequality
   */
  bool operator!=(const Twist2& other) const { return !(operator==(other)); }
};

/**
 * @brief      Two-dimensional pose composed of planar position and
 * orientation. Poses are assumed to be the world-fixed frame.
 */
struct Pose2 {
  /**
   * Position in world frame
   */
  Vector2 position;
  /**
   * Orientation in world frame
   */
  Radians orientation;

  Pose2(const Vector2& position, Radians orientation = 0.0)
      : position(position), orientation(orientation) {}
  Pose2() : Pose2({0.0f, 0.0f}) {}
  /**
   * @brief      Rotate the pose by an angle.
   *
   * @param[in]  angle  The rotation angle in radians.
   *
   * @return     The rotated pose.
   */
  Pose2 rotate(Radians angle) const {
    return {::navground::core::rotate(position, angle), orientation + angle};
  }

  /**
   * @brief      Integrate a pose
   *
   * @param[in]  twist  The twist (in agent or world frame)
   * @param[in]  dt     The time step
   *
   * @return     The result of ``pose + dt * twist`` (in world frame)
   */
  Pose2 integrate(const Twist2& twist, float dt) {
    return {position + dt * (twist.frame == Frame::relative
                                 ? ::navground::core::rotate(twist.velocity,
                                                             orientation)
                                 : twist.velocity),
            orientation + dt * twist.angular_speed};
  }

  /**
   * @brief      Equality operator.
   *
   * @param[in]  other  The other pose
   *
   * @return     The result of the equality
   */
  bool operator==(const Pose2& other) const {
    return position == other.position && orientation == other.orientation;
  }

  /**
   * @brief      Inequality operator.
   *
   * @param[in]  other  The other pose
   *
   * @return     The result of the inequality
   */
  bool operator!=(const Pose2& other) const { return !(operator==(other)); }
};

/**
 * @brief  An helper class that track changes.
 *
 * Changes are tracked by a bit mask, where the bit index corresponds to
 * different fields that may change. The class is mainly used for tracking
 * changes in a \ref Behavior to enable caching when the relevant part of the
 * state has not changed.
 */
class TrackChanges {
 public:
  /**
   * @brief      Constructs a new instance.
   */
  TrackChanges() : changes{0xFFFFFFFF} {}
  /**
   * @brief      Query if there was a change.
   *
   * @param[in]  mask  The bit mask of the indices we are interested in.
   *
   * @return     True if there is a recorded change in one of the indices.
   */
  bool changed(unsigned mask = 0xFFFFFFFF) const { return changes & mask; }
  /**
   * @brief      Reset the bit mask.
   */
  void reset_changes() { changes = 0; }
  /**
   * @brief      Notify a change
   *
   * @param[in]  mask  A bit mask where indices that have changed are set to 1.
   *
   */
  void change(unsigned mask) { changes |= mask; }

 private:
  unsigned changes;
};

inline std::ostream& operator<<(std::ostream& os, const Frame& frame) {
  os << "Frame::"
     << (frame == navground::core::Frame::relative ? "relative" : "absolute");
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const Vector2& vector) {
  os << "Vector2(" << vector[0] << ", " << vector[1] << ")";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const Twist2& twist) {
  os << "Twist2(" << twist.velocity << ", " << twist.angular_speed << ", "
     << twist.frame << ")";
  return os;
}

inline std::ostream& operator<<(std::ostream& os, const Pose2& pose) {
  os << "Pose2(" << pose.position << ", " << pose.orientation << ")";
  return os;
}

}  // namespace navground::core

#endif  // NAVGROUND_CORE_COMMON_H_
