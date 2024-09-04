/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/collision_computation.h"

// #define LINE_CAP_SQUARE

namespace navground::core {

DiscCache::DiscCache(Vector2 delta, ng_float_t margin, Vector2 velocity,
                     ng_float_t visible_angle)
    : delta(delta),
      velocity(velocity),
      distance(delta.norm() - margin),
      C(delta.squaredNorm() - margin * margin),
      gamma(orientation_of(delta)),
      visible_angle(visible_angle) {}

std::tuple<std::valarray<ng_float_t>, std::valarray<ng_float_t>>
CollisionComputation::get_contour_for_sector(Radians from, Radians length,
                                             size_t resolution,
                                             ng_float_t max_distance,
                                             bool dynamic, ng_float_t speed) {
  return {get_angles_for_sector(from, length, resolution),
          get_free_distance_for_sector(from, length, resolution, max_distance,
                                       dynamic, speed)};
}

void CollisionComputation::setup(Pose2 pose_, ng_float_t margin_,
                                 const std::vector<LineSegment> &line_segments,
                                 std::vector<DiscCache> static_discs,
                                 std::vector<DiscCache> dynamic_discs) {
  line_obstacles = line_segments;
  static_obstacles_cache = static_discs;
  neighbors_cache = dynamic_discs;
  pose = pose_;
  margin = margin_;
}

void CollisionComputation::setup(Pose2 pose_, ng_float_t margin_,
                                 const std::vector<LineSegment> &line_segments,
                                 const std::vector<Disc> &static_discs,
                                 const std::vector<Neighbor> &dynamic_discs) {
  line_obstacles = line_segments;
  pose = pose_;
  margin = margin_;
  neighbors_cache.clear();
  neighbors_cache.reserve(dynamic_discs.size());
  for (const auto &disc : dynamic_discs) {
    neighbors_cache.push_back(
        {disc.position - pose.position, margin_ + disc.radius, disc.velocity});
  }
  static_obstacles_cache.clear();
  static_obstacles_cache.reserve(static_discs.size());
  for (const auto &disc : static_discs) {
    static_obstacles_cache.push_back(
        {disc.position - pose.position, margin_ + disc.radius});
  }
}

std::valarray<ng_float_t> CollisionComputation::get_free_distance_for_sector(
    Radians from, Radians length, size_t resolution, ng_float_t max_distance,
    bool dynamic, ng_float_t speed) {
  std::valarray<ng_float_t> d(resolution + 1);
  Radians a = from;
  if (resolution == 0) {
    a += length / 2;
    d[0] = dynamic ? dynamic_free_distance(a, max_distance, speed)
                   : static_free_distance(a, max_distance, true);
    return d;
  }
  const Radians da = length / static_cast<ng_float_t>(resolution);
  for (size_t i = 0; i < resolution + 1; i++, a += da) {
    d[i] = dynamic ? dynamic_free_distance(a, max_distance, speed)
                   : static_free_distance(a, max_distance, true);
  }
  return d;
}

std::valarray<ng_float_t> CollisionComputation::get_angles_for_sector(
    Radians from, Radians length, size_t resolution) const {
  std::valarray<ng_float_t> d(resolution + 1);
  Radians a = from;
  if (resolution == 0) {
    a += length / 2;
    d[0] = a;
    return d;
  }
  const Radians da = length / static_cast<ng_float_t>(resolution);
  for (size_t i = 0; i < resolution + 1; i++, a += da) {
    d[i] = a;
  }
  return d;
}

// angle is absolute
// max_distance = horizon - radius
ng_float_t CollisionComputation::static_free_distance(Radians angle,
                                                      ng_float_t max_distance,
                                                      bool include_neighbors) {
  return static_free_distance(angle, unit(angle), max_distance,
                              include_neighbors);
}

ng_float_t CollisionComputation::static_free_distance(Radians angle,
                                                      const Vector2 &e,
                                                      ng_float_t max_distance,
                                                      bool include_neighbors) {
  max_distance = static_free_distance_to_collection(angle, e, max_distance,
                                                    line_obstacles);
  if (max_distance == 0) return 0;
  max_distance = static_free_distance_to_collection(angle, e, max_distance,
                                                    static_obstacles_cache);
  if (!include_neighbors || max_distance == 0) return max_distance;
  return static_free_distance_to_collection(angle, e, max_distance,
                                            neighbors_cache);
}

// angle is absolute
ng_float_t CollisionComputation::dynamic_free_distance(Radians angle,
                                                       ng_float_t max_distance,
                                                       ng_float_t speed) {
  const Vector2 e = unit(angle);
  max_distance = static_free_distance(angle, e, max_distance, false);
  if (max_distance == 0) return 0;
  return dynamic_free_distance_to_collection(e, max_distance, speed,
                                             neighbors_cache);
}

ng_float_t CollisionComputation::static_free_distance_to(
    const LineSegment &line, [[maybe_unused]] Radians alpha, const Vector2 &e) {
  const Vector2 delta = pose.position - line.p1;
  const ng_float_t y = delta.dot(line.e2);
  const ng_float_t x = delta.dot(line.e1);
  const ng_float_t d = line.e2.dot(e);
  if (y * d >= 0) {
    // moving away
    return no_collision;
  }
#if LINE_CAP_SQUARE
  if (std::abs(y) < margin && x > -margin && x < line.length + margin) {
    // already colliding
    return 0;
  }
#else
  // Does not consider as collision if the disc is colliding at the edges but
  // moving outwards.
  if (std::abs(y) < margin) {
    const ng_float_t ex = line.e1.dot(e);
    if (x < -margin) return no_collision;
    if (x < 0) return ex < 0 ? no_collision : 0;
    if (x < line.length) return 0.0;
    if (x < line.length + margin) return ex > 0 ? no_collision : 0;
    return no_collision;
  }
#endif  // LINE_CAP_SQUARE
  const ng_float_t distance = -y / d - margin;
  const ng_float_t x_delta = line.e1.dot(distance * e + delta);
  if (x_delta < -margin || x_delta > line.length + margin) {
    // will not collide
    return no_collision;
  }
  return distance;
}

ng_float_t CollisionComputation::static_free_distance_to(const DiscCache &disc,
                                                         Radians alpha,
                                                         const Vector2 &e) {
  if (disc.C < 0) {
    if (std::abs(normalize(alpha - disc.gamma)) < disc.visible_angle) return 0;
    return no_collision;
  }
  const ng_float_t B = disc.delta.dot(e);
  if (B < 0) return no_collision;
  const ng_float_t D = B * B - disc.C;
  if (D < 0) return no_collision;
  return B - std::sqrt(D);
}

// CHANGED: when colliding, instead of the same criteria as the static case,
// i.e. do not advance towards the obstacle, we want that for small dt,
// the relative position after dt, will be farther than now
// i.e. |p - dv |^2 > |p|^2 => p . dv = B < -dv^2 dt < 0
// we can impose B < 0. It is not continuous with respect to C!
ng_float_t CollisionComputation::dynamic_free_distance_to(const DiscCache &disc,
                                                          const Vector2 &v,
                                                          ng_float_t speed) {
  // if (disc.C < 0) {
  //   if (abs(normalize(alpha - disc.gamma)) < disc.visible_angle) return 0;
  //   return no_collision;
  // }
  Vector2 dv = v - disc.velocity;
  // TODO(J): use dot prod
  // TODO(J 2023): maybe pre-compute minimal distance
  // a = optimal_speed + agent_speed
  // A = a * a
  // B = -dist * a
  // C = dist * dist - r * r
  // D = dist * dist * A - dist * dist * A + r * r * A = r * r * a * a
  // min_dist = optimal_speed * (-B - sqrt(D)) / A = optimal_speed * (dist * a -
  // r * a) (a * a)
  //          = optimal_speed * (dist - r) / (optimal_speed + agent_speed)
  const ng_float_t B = disc.delta.dot(dv);

  if (disc.C < 0) {
    // colliding
    // return B < 0 ? no_collision : 0.0;
    // CHANGED(4/10/2023)
    // accept if delta angle > "visible_angle"
    // if dv * dx / |dv||dx| = cos(angle) < cos(visible_angle) (= 0 for
    // visible_angle = pi/2)
    //
    return B < dv.norm() * disc.delta.norm() * std::cos(disc.visible_angle)
               ? no_collision
               : 0;
  }

  if (B <= 0) return no_collision;
  const ng_float_t A = dv.squaredNorm();
  const ng_float_t D = B * B - A * disc.C;
  if (D < 0) return no_collision;
  return speed * (B - std::sqrt(D)) / A;
}

bool CollisionComputation::dynamic_may_collide(const DiscCache &c,
                                               ng_float_t max_distance,
                                               ng_float_t speed) {
  return true;
  return (speed * c.distance / (speed + c.velocity.norm())) < max_distance;
}

bool CollisionComputation::static_may_collide(const DiscCache &c,
                                              ng_float_t max_distance) {
  return c.distance < max_distance;
}

template <typename T>
ng_float_t CollisionComputation::static_free_distance_to_collection(
    Radians angle, const Vector2 &e, ng_float_t max_distance,
    const std::vector<T> &objects) {
  ng_float_t min_distance = max_distance;
  for (const auto &object : objects) {
    ng_float_t distance = static_free_distance_to(object, angle, e);
    if (distance < 0) continue;
    min_distance = std::min(min_distance, distance);
    if (min_distance == 0) return 0;
  }
  return min_distance;
}

template ng_float_t CollisionComputation::static_free_distance_to_collection<
    LineSegment>(Radians angle, const Vector2 &e, ng_float_t max_distance,
                 const std::vector<LineSegment> &objects);

template ng_float_t CollisionComputation::static_free_distance_to_collection<
    DiscCache>(Radians angle, const Vector2 &e, ng_float_t max_distance,
               const std::vector<DiscCache> &objects);

template <typename T>
ng_float_t CollisionComputation::dynamic_free_distance_to_collection(
    const Vector2 &e, ng_float_t max_distance, ng_float_t speed,
    const std::vector<T> &objects) {
  ng_float_t min_distance = max_distance;
  const Vector2 v = speed * e;
  for (const auto &object : objects) {
    ng_float_t distance = dynamic_free_distance_to(object, v, speed);
    if (distance < 0) continue;
    min_distance = std::min(min_distance, distance);
    if (min_distance == 0) return 0;
  }
  return min_distance;
}

template ng_float_t CollisionComputation::dynamic_free_distance_to_collection<
    DiscCache>(const Vector2 &e, ng_float_t max_distance, ng_float_t speed,
               const std::vector<DiscCache> &objects);

}  // namespace navground::core
