/**
 * @author Jérôme Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_COMMON_H_
#define NAVGROUND_CORE_COMMON_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <ostream>

#include "navground/core/types.h"
#include "navground_core_export.h"

typedef unsigned int uint;

namespace navground::core {

/**
 * A two-dimensional vector, see <a
 * href="https://eigen.tuxfamily.org/dox/group__matrixtypedefs.html">Eigen</a>
 */
using Vector2 = Eigen::Vector2<ng_float_t>;

/**
 * Angle in radians.
 */
using Radians = ng_float_t;

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
using WheelSpeeds = std::vector<ng_float_t>;

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
inline Vector2 unit(ng_float_t angle) {
  return {std::cos(angle), std::sin(angle)};
}

/**
 * @brief      Normal vector to a desired orientated.
 *
 * @param[in]  angle The desired orientation
 *
 * @return     Vector of norm one and orientation normal to the desired orientation
 */
inline Vector2 normal(ng_float_t angle) {
  return {-std::sin(angle), std::cos(angle)};
}

/**
 * @brief      Rotate a two-dimensional vector.
 *
 * @param[in]  vector The original vector
 * @param[in]  angle  The rotation angle in radians
 *
 * @return     The rotated vector
 */
inline Vector2 rotate(const Vector2 vector, ng_float_t angle) {
  Eigen::Rotation2D<ng_float_t> rot(angle);
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
inline Vector2 clamp_norm(const Vector2& vector, ng_float_t max_length) {
  ng_float_t n = vector.norm();
  if (n > 0 && n > max_length) {
    return vector / n * max_length;
  }
  return vector;
}

struct Pose2;

/**
 * @brief      Two-dimensional twist composed of planar velocity and angular
 * speed.
 *
 * Twist coordinates are in the frame specified by \ref frame.
 */
struct NAVGROUND_CORE_EXPORT Twist2 {
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

  Twist2(const Vector2& velocity, Radians angular_speed = 0,
         Frame frame = Frame::absolute)
      : velocity(velocity), angular_speed(angular_speed), frame(frame) {}
  Twist2() : Twist2({0, 0}) {}
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

  /**
   * @brief      Transform a twist to \ref Frame::absolute.
   *
   * @param[in]  value  The original twist
   *
   * @return     The same twist in \ref Frame::absolute
   *             (unchanged if already in \ref Frame::absolute)
   */
  Twist2 absolute(const Pose2& reference) const;

  /**
   * @brief      Transform a twist to \ref Frame::relative.
   *
   * @param[in]  frame  The reference pose
   *
   * @return     The same twist in \ref Frame::relative
   *             (unchanged if already in \ref Frame::relative)
   */
  Twist2 relative(const Pose2& reference) const;

  /**
   * @brief      Convert a twist to a reference frame.
   *
   * @param[in]  frame  The desired frame of reference
   * @param[in]  frame  The reference pose
   *
   * @return     The twist in the desired frame of reference.
   */
  Twist2 to_frame(Frame frame, const Pose2& reference) const;

  /**
   * @brief      Determines if almost zero.
   *
   * @param[in]  epsilon_speed          The epsilon speed
   * @param[in]  epsilon_angular_speed  The epsilon angular speed
   *
   * @return     True if almost zero, False otherwise.
   */
  bool is_almost_zero(ng_float_t epsilon_speed = 1e-6,
                      ng_float_t epsilon_angular_speed = 1e-6) const {
    return velocity.norm() < epsilon_speed &&
           abs(angular_speed) < epsilon_angular_speed;
  }

  /**
   * @brief      Snap components to zero if their module is smaller than epsilon.
   *
   * @param[in]  epsilon  The tolerance
   */
  void snap_to_zero(ng_float_t epsilon = 1e-6) {
    if (abs(velocity[0]) < epsilon) {
      velocity[0] = 0;
    }
    if (abs(velocity[1]) < epsilon) {
      velocity[1] = 0;
    }
    if (abs(angular_speed) < epsilon) {
      angular_speed = 0;
    }
  }
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

  Pose2(const Vector2& position, Radians orientation = 0)
      : position(position), orientation(orientation) {}
  Pose2() : Pose2({0, 0}) {}
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
  Pose2 integrate(const Twist2& twist, ng_float_t dt) {
    const auto new_orientation = orientation + dt * twist.angular_speed;
    return {position + dt * (twist.frame == Frame::relative
                                 ? ::navground::core::rotate(twist.velocity,
                                                             new_orientation)
                                 : twist.velocity),
            new_orientation};
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

  /**
   * @brief      Transform a relative pose to an absolute pose.
   *
   * @param[in]  reference  The reference pose
   *
   * @return     The absolute pose
   */
  Pose2 absolute(const Pose2& reference) const {
    return Pose2(::navground::core::rotate(position, reference.orientation) +
                     reference.position,
                 orientation + reference.orientation);
  }

  /**
   * @brief      Transform an absolute pose to a relative pose.
   *
   * @param[in]  pose  The reference pose
   *
   * @return     The relative pose
   */
  Pose2 relative(const Pose2& reference) const {
    return Pose2(::navground::core::rotate(position - reference.position,
                                           -reference.orientation),
                 orientation - reference.orientation);
  }
};

/**
 * @brief      Transform a relative to an absolute vector.
 *
 * @param[in]  value  The relative vector
 * @param[in]  reference  The reference pose
 *
 * @return     The relative vector
 */
inline Vector2 to_absolute(const Vector2& value, const Pose2& reference) {
  return rotate(value, reference.orientation);
}

/**
 * @brief      Transform an absolute to a relative vector.
 *
 * @param[in]  value  The absolute vector
 * @param[in]  reference  The reference pose
 *
 * @return     The absolute vector
 */
inline Vector2 to_relative(const Vector2& value, const Pose2& reference) {
  return rotate(value, -reference.orientation);
}

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
