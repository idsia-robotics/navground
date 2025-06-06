/**
 * @author Jérôme Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_COMMON_H_
#define NAVGROUND_CORE_COMMON_H_

#define _USE_MATH_DEFINES
#include <assert.h>
#include <math.h>
#include <time.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <cmath>
#include <ostream>

#include "navground/core/export.h"
#include "navground/core/types.h"

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
inline Radians orientation_of(const Vector2 &vector) {
  return std::atan2(vector.y(), vector.x());
}

constexpr Radians TWO_PI = static_cast<Radians>(2 * M_PI);
constexpr Radians PI = static_cast<Radians>(M_PI);
constexpr Radians HALF_PI = static_cast<Radians>(M_PI_2);

/**
 * @brief      Normalize an angle to a value in \f$[-\pi, \pi]\f$.
 * Apply it when computing angular differences.
 *
 * @param[in]  value The original unbounded angle
 *
 * @return     The equivalent angle in \f$[-\pi, \pi]\f$
 */

inline Radians normalize_angle(Radians value) {
  value = std::fmod(value, TWO_PI);
  if (value < -PI) {
    value += TWO_PI;
  } else if (value > PI) {
    value -= TWO_PI;
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
inline Vector2 unit(Radians angle) {
  return {std::cos(angle), std::sin(angle)};
}

/**
 * @brief      Normal vector to a desired orientated.
 *
 * @param[in]  angle The desired orientation
 *
 * @return     Vector of norm one and orientation normal to the desired
 * orientation
 */
inline Vector2 normal(Radians angle) {
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
inline Vector2 rotate(const Vector2 vector, Radians angle) {
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
inline Vector2 clamp_norm(const Vector2 &vector, ng_float_t max_length) {
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

  Twist2(const Vector2 &velocity, Radians angular_speed = 0,
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
  bool operator==(const Twist2 &other) const {
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
  bool operator!=(const Twist2 &other) const { return !(operator==(other)); }

  /**
   * @brief      Transform a twist to \ref Frame::absolute.
   *
   * @param[in]  reference  The pose of reference of this relative twist
   *
   * @return     The same twist in \ref Frame::absolute
   *             (unchanged if already in \ref Frame::absolute)
   */
  Twist2 absolute(const Pose2 &reference) const;

  /**
   * @brief      Transform a twist to \ref Frame::relative relative to a pose.
   *
   * @param[in]  reference  The desired pose of reference
   *
   * @return     The same twist in \ref Frame::relative
   *             (unchanged if already in \ref Frame::relative)
   */
  Twist2 relative(const Pose2 &reference) const;

  /**
   * @brief      Convert a twist to a reference frame.
   *
   * @param[in]  frame  The desired frame of reference
   * @param[in]  reference  The pose of reference
   *
   * @return     The twist in the desired frame of reference.
   */
  Twist2 to_frame(Frame frame, const Pose2 &reference) const;

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
           std::abs(angular_speed) < epsilon_angular_speed;
  }

  /**
   * @brief      Snap components to zero if their module is smaller than
   * epsilon.
   *
   * @param[in]  epsilon  The tolerance
   */
  void snap_to_zero(ng_float_t epsilon = 1e-6);

  /**
   * @brief      Interpolates a twist towards a target
   *             respecting acceleration limits
   *
   * @param[in]  target                     The target twist, must be in the
   * same frame
   * @param[in]  time_step                 The time step
   * @param[in]  max_acceleration          The maximum acceleration
   * @param[in]  max_angular_acceleration  The maximum angular acceleration
   *
   * @return     The interpolate twist
   */
  Twist2 interpolate(const Twist2 &target, ng_float_t time_step,
                     ng_float_t max_acceleration,
                     ng_float_t max_angular_acceleration) const;
};

/**
 * @brief      Two-dimensional pose composed of planar position and
 * orientation. When not specified, poses are assumed to be in the world-fixed
 * frame.
 *
 * Poses are also associated to rigid transformations in SE(2)
 * ``translation(pose.position) . rotation(pose.orientation)``.
 * We define the group operations as a multiplication and add methods like
 * \ref transform_pose for a pose to operate as a rigid transformation.
 */
struct NAVGROUND_CORE_EXPORT Pose2 {
  /**
   * Position in world frame
   */
  Vector2 position;
  /**
   * Orientation in world frame
   */
  Radians orientation;

  Pose2(const Vector2 &position, Radians orientation = 0)
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
  Pose2 integrate(const Twist2 &twist, ng_float_t dt);

  /**
   * @brief      Equality operator.
   *
   * @param[in]  other  The other pose
   *
   * @return     The result of the equality
   */
  bool operator==(const Pose2 &other) const {
    return position == other.position && orientation == other.orientation;
  }

  /**
   * @brief      Inequality operator.
   *
   * @param[in]  other  The other pose
   *
   * @return     The result of the inequality
   */
  bool operator!=(const Pose2 &other) const { return !(operator==(other)); }

  /**
   * @brief      Transform a relative pose to an absolute pose.
   *
   * Equivalent to ``reference.transform_pose(self) = reference * self``
   *
   * @param[in]  reference  The reference pose
   *
   * @return     The absolute pose
   */
  Pose2 absolute(const Pose2 &reference) const {
    return Pose2(::navground::core::rotate(position, reference.orientation) +
                     reference.position,
                 orientation + reference.orientation);
  }

  /**
   * @brief      Transform an absolute pose to a relative pose.
   *
   * Equivalent to ``reference.inverse().transform_pose(self) =
   * reference.inverse() * self``
   *
   * @param[in]  reference  The reference pose
   *
   * @return     The relative pose
   */
  Pose2 relative(const Pose2 &reference) const {
    return Pose2(::navground::core::rotate(position - reference.position,
                                           -reference.orientation),
                 orientation - reference.orientation);
  }

  /**
   * @brief      Concatenate two transformations, i.e. the SE(2) operation.
   *
   * @param[in]  other  The other pose
   *
   * @return     The composite transformation resulting from applying
   *             first the other transformation and then this transformation.
   */
  Pose2 operator*(const Pose2 &other) const {
    return Pose2(position +
                     ::navground::core::rotate(other.position, orientation),
                 orientation + other.orientation);
  }

  /**
   * @brief      SE(2) division of two transformations.
   *
   * @param[in]  other  The other pose
   *
   * @return     ``self * other.inverse()``
   */
  Pose2 operator/(const Pose2 &other) const {
    return (*this) * other.inverse();
  }

  /**
   * @brief      Invert a transformation, i.e., SE(2) inversion
   *
   * @return     The inverse transformation
   */
  Pose2 inverse() const {
    return Pose2(::navground::core::rotate(-position, -orientation),
                 -orientation);
  }

  /**
   * @brief      Applies this rigid transformation to another pose
   *
   * Equivalent to ``pose.absolute(self)``
   *
   * @param[in]  pose  The pose to transform
   *
   * @return     ``self * pose``
   */
  Pose2 transform_pose(const Pose2 &pose) const { return (*this) * pose; }

  /**
   * @brief      Applies this rigid transformation to a point
   *
   * Equivalent to ``to_absolute_point(point, self)``
   *
   * @param[in]  point  The point to transform
   *
   * @return     The roto-translated point
   */
  Vector2 transform_point(const Vector2 &point) const {
    return position + ::navground::core::rotate(point, orientation);
  }

  /**
   * @brief      Applies the rigid transformation to a vector
   *
   * Equivalent to ``to_absolute(vector, self)``
   *
   * @param[in]  vector  The vector to transform
   *
   * @return     The rotated vector
   */
  Vector2 transform_vector(const Vector2 &vector) const {
    return ::navground::core::rotate(vector, orientation);
  }

  /**
   * @brief      Returns this transformation in the desired frame of reference.
   *
   * The returned transformation is defined as
   * ``frame.inverse() * self * frame``, so that, for any frame ``f``,
   * the following expressions are equivalent:
   *
   * - ``pose.transform(other).relative(f)``
   *
   * - ``pose.get_transformation_in_frame(frame).transform(other.relative(f))``
   *
   * @param[in]  frame  A posed defining the desired frame of reference
   *                    for the transformation.
   *
   * @return     The transformation with respect to ``frame``
   */
  Pose2 get_transformation_in_frame(const Pose2 &frame) const {
    return frame.inverse() * (*this) * frame;
  }
};

/**
 * @brief      Transform a relative to an absolute vector.
 *
 * Equivalent to ``reference.transform_vector(value)``
 *
 * @param[in]  value  The relative vector
 * @param[in]  reference  The reference pose
 *
 *
 * @return     The relative vector
 */
inline Vector2 to_absolute(const Vector2 &value, const Pose2 &reference) {
  return rotate(value, reference.orientation);
}

/**
 * @brief      Transform an absolute to a relative vector.
 *
 * Equivalent to ``reference.inverse().transform_vector(value)``
 *
 * @param[in]  value  The absolute vector
 * @param[in]  reference  The reference pose
 *
 * @return     The absolute vector
 */
inline Vector2 to_relative(const Vector2 &value, const Pose2 &reference) {
  return rotate(value, -reference.orientation);
}

/**
 * @brief      Transform a relative to an absolute point.
 *
 * Equivalent to ``reference.transform_point(value)``
 *
 * @param[in]  value  The relative point
 * @param[in]  reference  The reference pose
 *
 * @return     The relative point
 */
inline Vector2 to_absolute_point(const Vector2 &value, const Pose2 &reference) {
  return reference.position + rotate(value, reference.orientation);
}

/**
 * @brief      Transform an absolute to a relative vector.
 *
 * Equivalent to ``reference.inverse().transform_point(value)``
 *
 * @param[in]  value  The absolute vector
 * @param[in]  reference  The reference pose
 *
 * @return     The absolute point
 */
inline Vector2 to_relative_point(const Vector2 &value, const Pose2 &reference) {
  return rotate(value - reference.position, -reference.orientation);
}

/**
 * @brief  An helper class that track changes.
 *
 * Changes are tracked by a bit mask, where the bit index corresponds to
 * different fields that may change. The class is mainly used for tracking
 * changes in a \ref Behavior to enable caching when the relevant part of the
 * state has not changed.
 */
class NAVGROUND_CORE_EXPORT TrackChanges {
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

} // namespace navground::core

inline std::ostream &operator<<(std::ostream &os, const navground::core::Frame &frame) {
  os << "Frame::"
     << (frame == navground::core::Frame::relative ? "relative" : "absolute");
  return os;
}

// inline std::ostream& operator<<(std::ostream& os, const Vector2& vector) {
//   os << "Vector2(" << vector[0] << ", " << vector[1] << ")";
//   return os;
// }

inline std::ostream &operator<<(std::ostream &os, const navground::core::Twist2 &twist) {
  os << "Twist2(" << twist.velocity << ", " << twist.angular_speed << ", "
     << twist.frame << ")";
  return os;
}

inline std::ostream &operator<<(std::ostream &os, const navground::core::Pose2 &pose) {
  os << "Pose2(" << pose.position << ", " << pose.orientation << ")";
  return os;
}

#endif // NAVGROUND_CORE_COMMON_H_
