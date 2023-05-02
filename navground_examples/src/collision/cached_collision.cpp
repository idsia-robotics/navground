/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <iterator>
#include <vector>

#include "navground/core/cached_collision_computation.h"

using navground::core::CachedCollisionComputation;
using navground::core::Disc;
using navground::core::LineSegment;

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
  CachedCollisionComputation f;
  f.set_resolution(11);
  f.set_min_angle(-1.0);
  f.set_length(2.0);
  // f.set_speed(1.0);
  f.set_max_distance(3.0);
  f.setup({}, 0.0, {}, {Disc{{2.0, 0.0}, 1.0}}, {});
  for (const auto& [a, d] :
       f.get_free_distance_for_sector(-1.0, 2.0, 3, 3.0, false)) {
    printf("%.3f: %.3f\n", a, d);
  }
  printf("Cached\n");
  for (const auto& [a, d] : f.get_free_distance(false)) {
    printf("%.3f: %.3f\n", a, d);
  }
  return 0;
}
