/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/cached_collision_computation.h"

namespace navground::core {

void CachedCollisionComputation::set_resolution(size_t value) {
  if (value > 0 && value != resolution) {
    resolution = value;
    dynamic_cache.resize(value);
    for (auto &s : static_cache) {
      s.resize(value);
    }
    reset();
  }
}

void CachedCollisionComputation::set_min_angle(Radians value) {
  value = normalize(value);
  if (value != from_relative_angle) {
    from_relative_angle = value;
    reset();
  }
}

void CachedCollisionComputation::set_length(ng_float_t value) {
  if (value > 0) {
    value = std::min(value, TWO_PI);
    if (length != value) {
      length = value;
      reset();
    }
  }
}

void CachedCollisionComputation::set_max_distance(ng_float_t value) {
  if (value > 0 && value != max_distance) {
    max_distance = value;
    reset();
  }
}

void CachedCollisionComputation::set_speed(ng_float_t value) {
  if (value > 0 && value != speed) {
    speed = value;
  }
  std::fill(std::begin(dynamic_cache), std::end(dynamic_cache), uncomputed);
}

void CachedCollisionComputation::setup(
    Pose2 pose_, ng_float_t margin_,
    const std::vector<LineSegment> &line_segments,
    const std::vector<DiscCache> &static_discs,
    const std::vector<DiscCache> &dynamic_discs) {
  CollisionComputation::setup(pose_, margin_, line_segments, static_discs,
                              dynamic_discs);
  reset();
}

void CachedCollisionComputation::setup(
    Pose2 pose_, ng_float_t margin_,
    const std::vector<LineSegment> &line_segments,
    const std::vector<Disc> &static_discs,
    const std::vector<Neighbor> &dynamic_discs) {
  CollisionComputation::setup(pose_, margin_, line_segments, static_discs,
                              dynamic_discs);
  reset();
}

void CachedCollisionComputation::reset() {
  std::fill(std::begin(static_cache[false]), std::end(static_cache[false]),
            uncomputed);
  std::fill(std::begin(static_cache[true]), std::end(static_cache[true]),
            uncomputed);
  std::fill(std::begin(dynamic_cache), std::end(dynamic_cache), uncomputed);
}

const std::valarray<ng_float_t> &CachedCollisionComputation::get_cache(
    bool assuming_static, bool include_neighbors) {
  if (assuming_static) return static_cache[include_neighbors];
  return dynamic_cache;
}

int CachedCollisionComputation::index_of(Radians delta_relative_angle) {
  if (resolution < 2) return 0;
  return static_cast<int>((delta_relative_angle - from_relative_angle) /
                          length * static_cast<ng_float_t>(resolution - 1));
}

ng_float_t CachedCollisionComputation::static_free_distance(
    Radians angle, bool include_neighbors) {
  int k = index_of(normalize(angle - pose.orientation));
  const bool can_be_cached = (k >= 0 && k < static_cast<int>(resolution));
  ng_float_t value =
      can_be_cached ? static_cache[include_neighbors][k] : uncomputed;
  if (value == uncomputed) {
    if (include_neighbors) {
      value = static_free_distance(angle, false);
      if (value != 0) {
        value = static_free_distance_to_collection(angle, unit(angle), value,
                                                   neighbors_cache);
      }
    } else {
      value = CollisionComputation::static_free_distance(angle, max_distance,
                                                         false);
    }
    if (can_be_cached) {
      static_cache[include_neighbors][k] = value;
    }
  }
  return value;
}

ng_float_t CachedCollisionComputation::dynamic_free_distance(Radians angle) {
  int k = index_of(normalize(angle - pose.orientation));
  const bool can_be_cached = (k >= 0 && k < static_cast<int>(resolution));
  ng_float_t value = can_be_cached ? dynamic_cache[k] : uncomputed;
  if (value == uncomputed) {
    value =
        CollisionComputation::dynamic_free_distance(angle, max_distance, speed);
    if (can_be_cached) {
      dynamic_cache[k] = value;
    }
  }
  return value;
}

std::valarray<ng_float_t> CachedCollisionComputation::get_free_distance(
    bool dynamic) {
  // TODO(Jerome): check the size [res vs res + 1]
  std::valarray<ng_float_t> d(resolution);
  if (resolution < 1) return d;
  Radians a = from_relative_angle;
  Radians da = length / static_cast<ng_float_t>(resolution - 1);
  for (size_t i = 0; i < resolution; i++, a += da) {
    d[i] = dynamic ? (dynamic_cache[i] != uncomputed) ? dynamic_cache[i]
                                                      : dynamic_free_distance(a)
           : (static_cache[true][i] != uncomputed)
               ? static_cache[true][i]
               : static_free_distance(a, true);
  }
  return d;
}

}  // namespace navground::core
