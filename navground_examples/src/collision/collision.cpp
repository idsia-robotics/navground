/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <iterator>
#include <vector>

#include "navground/core/collision_computation.h"

using navground::core::CollisionComputation;
using navground::core::Disc;
using navground::core::LineSegment;

int main([[maybe_unused]] int argc, [[maybe_unused]] char* argv[]) {
  CollisionComputation f;
  f.setup({}, 0.0, {}, {Disc{{2.0, 0.0}, 1.0}}, {});
  auto angles = f.get_angles_for_sector(-1.0, 2.0, 3);
  auto distances = f.get_free_distance_for_sector(-1.0, 2.0, 3, 3.0, false);
  for (size_t i = 0; i < angles.size(); ++i) {
    printf("%.3f: %.3f\n", angles[i], distances[i]);
  }
  return 0;
}
