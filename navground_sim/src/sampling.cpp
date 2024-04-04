/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/sampling/geometry.h"

namespace navground::sim {

std::vector<core::Disc> sample_discs(RandomGenerator &rng, unsigned number,
                                     const BoundingBox &bounding_box,
                                     ng_float_t min_radius,
                                     ng_float_t max_radius, ng_float_t margin,
                                     ng_float_t disc_margin,
                                     const std::vector<core::Disc> &discs,
                                     unsigned max_tries,
                                     std::vector<core::Vector2> lattice_grid) {
  if (lattice_grid.size() == 0) {
    lattice_grid = {core::Vector2()};
  }
  std::vector<core::Disc> rs;
  unsigned tries = 0;

  std::uniform_real_distribution<ng_float_t> x{bounding_box.getMinX(),
                                               bounding_box.getMaxX()};
  std::uniform_real_distribution<ng_float_t> y{bounding_box.getMinY(),
                                               bounding_box.getMaxY()};
  std::uniform_real_distribution<ng_float_t> r{min_radius, max_radius};

  while (rs.size() < number && tries <= max_tries) {
    core::Disc disc({x(rng), y(rng)}, r(rng));
    bool valid = true;
    for (const auto &delta : lattice_grid) {
      for (const core::Disc &other : discs) {
        if (disc.distance(other + delta) < disc_margin) {
          valid = false;
          break;
        }
      }
      for (const core::Disc &other : rs) {
        if (disc.distance(other + delta) < margin) {
          valid = false;
          break;
        }
      }
      if (!valid) {
        break;
      }
    }
    if (!valid) {
      tries++;
      continue;
    }
    rs.push_back(disc);
  }
  return rs;
}

}  // namespace navground::sim
