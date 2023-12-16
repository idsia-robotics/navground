/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/cached_collision_computation.h"

namespace navground::core {

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
        value =
            static_free_distance_to_collection(angle, value, neighbors_cache);
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
