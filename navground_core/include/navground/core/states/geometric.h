#ifndef NAVGROUND_CORE_BEHAVIOR_GEOMETRIC_H
#define NAVGROUND_CORE_BEHAVIOR_GEOMETRIC_H

#include <iostream>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/state.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief A circular shape. Used to represent obstacles.
 */
struct Disc {
  /**
   * The center of the disc in world frame
   */
  Vector2 position;
  /**
   * Velocity in world frame
   */
  // Vector2 velocity;
  /**
   * Radius
   */
  ng_float_t radius;
  /**
   * @brief      Constructs a new instance.
   *
   * @param  position       The position
   * @param  radius         The radius
   */
  Disc(const Vector2& position, ng_float_t radius)
      : position(position), radius(radius) {}

  Disc() : Disc(Vector2::Zero(), 0) {}

  /**
   * @brief      Equality operator.
   *
   * @param[in]  other  The other disc
   *
   * @return     True if positions and radii are equal
   */
  bool operator==(const Disc& other) const {
    return position == other.position && radius == other.radius;
  }

  /**
   * @brief      Addition operator.
   *
   * @param[in]  delta  A translation vector
   *
   * @return     A copy of the disc translated by delta
   */
  Disc operator+(const Vector2& delta) const {
    return Disc(position + delta, radius);
  }

  /**
   * @brief      Addition assignment operator.
   *
   * @param[in]  delta  A translation vector
   *
   * @return     The same disc translated by delta
   */
  Disc& operator+=(const Vector2& delta) {
    position += delta;
    return *this;
  }

  /**
   * @brief      Subtraction operator.
   *
   * @param[in]  delta  A translation vector
   *
   * @return     A copy of the disc translated by -delta
   */
  Disc operator-(const Vector2& delta) const {
    return Disc(position - delta, radius);
  }

  /**
   * @brief      Subtraction assignment operator.
   *
   * @param[in]  delta  A translation vector
   *
   * @return     The same disc translated by delta
   */
  Disc& operator-=(const Vector2& delta) {
    position -= delta;
    return *this;
  }

  /**
   * @brief      Inequality operator.
   *
   * @param[in]  other  The other disc
   *
   * @return     Negation of equality
   */
  bool operator!=(const Disc& other) const { return !(operator==(other)); }

  /**
   * @brief      Returns the signed distance to another disc.
   *             Negative if the discs overlaps.
   *
   * @param[in]  other  The other disc
   *
   * @return     The distance between the centers minus the radii.
   */
  ng_float_t distance(const Disc& other) const {
    return (position - other.position).norm() - radius - other.radius;
  }
};

inline std::ostream& operator<<(std::ostream& os, const Disc& disc) {
  os << "Disc(" << disc.position << ", " << disc.radius << ")";
  return os;
}

/**
 * @brief      A neighbor agent of circular shape.
 */
struct Neighbor : public Disc {
  /**
   * The velocity
   */
  Vector2 velocity;
  /**
   * An identifier for the id of agents or the individual agent.
   * The interpretation is up to the behavior.
   */
  unsigned id;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  position  The position
   * @param[in]  radius    The radius
   * @param[in]  velocity  The velocity
   * @param[in]  id        The identifier
   */
  Neighbor(const Vector2& position, ng_float_t radius, const Vector2 velocity,
           unsigned id = 0)
      : Disc(position, radius), velocity(velocity), id(id) {}
  /**
   * @brief      Constructs a new instance from a disc
   *
   * @param[in]  disc      The disc
   * @param[in]  velocity  The velocity
   * @param[in]  id      The id
   */
  explicit Neighbor(const Disc& disc, const Vector2 velocity = Vector2::Zero(),
                    unsigned id = 0)
      : Neighbor(disc.position, disc.radius, velocity, id) {}

  /**
   * @brief      Assignment operator from a disc.
   *
   * @param[in]  other  The other disc
   *
   * @return     The result of the assignment
   */
  Neighbor& operator=(const Disc& other) {
    position = other.position;
    radius = other.radius;
    id = 0;
    velocity = Vector2::Zero();
    return *this;
  }

  bool operator==(const Neighbor& other) const {
    return Disc::operator==(other) && velocity == other.velocity &&
           id == other.id;
  }

  bool operator!=(const Neighbor& other) const { return !(operator==(other)); }

  /**
   * @brief      Returns the same neighbor in a frame relative to a pose.
   *
   * @param[in]  reference  The reference pose
   *
   * @return     The neighbor with transformed position and velocity
   */
  Neighbor relative_to(const Pose2& reference) {
    const Pose2 pose{position, 0};
    return Neighbor(pose.relative(reference).position, radius,
                    to_relative(velocity, reference), id);
  }
};

inline std::ostream& operator<<(std::ostream& os, const Neighbor& disc) {
  os << "Neighbor(Disc(" << disc.position << ", " << disc.radius << "), "
     << disc.velocity << ", " << disc.id << ")";
  return os;
}

/**
 * @brief      A static obstacle of linear shape.
 */
struct LineSegment {
  /**
   * The position of the first vertex
   */
  Vector2 p1;
  /**
   * The position of the second vertex
   */
  Vector2 p2;
  /**
   * The unit vector along the segment
   */
  Vector2 e1;
  /**
   * The unit vector perpendicular to the segment. Oriented to the left with
   * respect to `e1`
   */
  Vector2 e2;
  /**
   * The segment length
   */
  ng_float_t length;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  p1    The first vertex
   * @param[in]  p2    The second vertex
   */
  LineSegment(const Vector2& p1, const Vector2& p2)
      : p1(p1),
        p2(p2),
        e1((p2 - p1).normalized()),
        e2(-e1[1], e1[0]),
        length((p2 - p1).norm()) {}

  LineSegment() : LineSegment({0, 0}, {1, 0}) {}

  void update() {
    e1 = (p2 - p1).normalized();
    e2 = {-e1[1], e1[0]};
    length = (p2 - p1).norm();
  }

  bool operator==(const LineSegment& other) const {
    return p1 == other.p1 && p2 == other.p2;
  }

  bool operator!=(const LineSegment& other) const {
    return !(operator==(other));
  }

  ng_float_t distance(const Vector2& point) const {
    const Vector2 delta = point - p1;
    const ng_float_t x = delta.dot(e1);
    if (x < 0) return delta.norm();
    if (x > length) return (point - p2).norm();
    return std::abs(delta.dot(e2));
  }

  // negative <=> penetration
  ng_float_t distance(const Disc& disc, bool penetration = false) const {
    const ng_float_t dist = distance(disc.position) - disc.radius;
    return (penetration || dist > 0) ? dist : 0;
  }

  /**
   * @brief      Computes the free distance along a ray before possibly
   * colliding
   *
   * @param[in]  point        The ray origin
   * @param[in]  direction    The ray direction
   * @param[in]  orientation  The ray orientation
   *
   * @return     The free distance along the ray before colliding
   *             with this line or -1 if no collision will happen.
   */
  ng_float_t distance_along(const Vector2& point, const Vector2& direction,
                            int orientation = 0) {
    const Vector2 delta = point - p1;
    const ng_float_t y = delta.dot(e2);
    if (orientation && orientation * y >= 0) {
      return -1;
    }
    const ng_float_t d = e2.dot(direction);
    if (y * d >= 0) {
      // moving away
      return -1;
    }
    const ng_float_t dist = -y / d;
    const ng_float_t x_delta = e1.dot(dist * direction + delta);
    if (x_delta < 0 || x_delta > length) {
      // no intersection
      return -1;
    }
    return dist;
  }

  /**
   * @brief      Constructs a copy.
   *
   * @param[in]  segment  The segment to copy.
   */
  // LineSegment(const LineSegment & segment) : LineSegment(segment.p1,
  // segment.p2) {}
};

inline std::ostream& operator<<(std::ostream& os, const LineSegment& line) {
  os << "LineSegment(" << line.p1 << ", " << line.p2 << ")";
  return os;
}

class GeometricState : public EnvironmentState, public TrackChanges {
 public:
  GeometricState()
      : EnvironmentState(),
        TrackChanges(),
        static_obstacles(),
        neighbors(),
        line_obstacles() {}

  // virtual ~GeometricState() = default;

  //----------- ENVIRONMENT STATE

  /**
   * @brief      Gets the current list of neighbors. Positions are in the world
   * fixed frame.
   *
   * @return     The neighbors.
   */
  const std::vector<Neighbor>& get_neighbors() const { return neighbors; }
  /**
   * @brief      Sets the neighbors. Positions are in the world fixed frame.
   *
   * @param[in]  value
   */
  virtual void set_neighbors(const std::vector<Neighbor>& value) {
    neighbors = value;
    change(NEIGHBORS);
  }
  /**
   * @brief      Gets the current list of static obstacles. Positions are in the
   * world fixed frame.
   *
   * @return     The static obstacles
   */
  const std::vector<Disc>& get_static_obstacles() const {
    return static_obstacles;
  }
  /**
   * @brief      Sets the static obstacles. Positions are in the world fixed
   * frame.
   *
   * @param[in]  value
   */
  virtual void set_static_obstacles(const std::vector<Disc>& value) {
    static_obstacles = value;
    change(STATIC_OBSTACLES);
  }
  /**
   * @brief      Gets the current list of line obstacles. Positions are in the
   * world fixed frame.
   *
   * @return     The line obstacles
   */
  const std::vector<LineSegment>& get_line_obstacles() const {
    return line_obstacles;
  }
  /**
   * @brief      Sets the line obstacles. Positions are in the world fixed
   * frame.
   *
   * @param[in]  value
   */
  virtual void set_line_obstacles(const std::vector<LineSegment>& value) {
    line_obstacles = value;
    change(LINE_OBSTACLES);
  }

  enum {
    NEIGHBORS = 1 << 0,
    STATIC_OBSTACLES = 1 << 1,
    LINE_OBSTACLES = 1 << 2
  };

 private:
  std::vector<Disc> static_obstacles;
  std::vector<Neighbor> neighbors;
  std::vector<LineSegment> line_obstacles;
};

#if 0
inline std::ostream& operator<<(std::ostream& os, const GeometricState& state) {
  os << "<GeometricState:\n";
  os << "\tline obstacles: {";
  for (const auto& line : state.get_line_obstacles()) {
    os << line;
  }
  os << "}\n";
  os << "\tstatic obstacles: {";
  for (const auto& obstacle : state.get_static_obstacles()) {
    os << obstacle;
  }
  os << "}\n";
  os << "\tneighbors: {";
  for (const auto& neighbor : state.get_neighbors()) {
    os << neighbor << ", ";
  }
  os << "}\n>";
  return os;
}

#endif

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_GEOMETRIC_H