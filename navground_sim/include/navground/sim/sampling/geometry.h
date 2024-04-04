/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_GEOMETRY_H
#define NAVGROUND_SIM_GEOMETRY_H

#include <geos/geom/Envelope.h>

#include <random>

#include "navground/core/states/geometric.h"
#include "navground_sim_export.h"

namespace navground::sim {

using BoundingBox = geos::geom::Envelope;

std::vector<core::Disc> NAVGROUND_SIM_EXPORT sample_discs(
    RandomGenerator &rng, unsigned number, const BoundingBox &bounding_box,
    ng_float_t min_radius, ng_float_t max_radius, ng_float_t margin = 0.0,
    ng_float_t disc_margin = 0.0,
    const std::vector<core::Disc> &discs = {}, unsigned max_tries = 1000,
    std::vector<core::Vector2> lattice_grid = {});

}  // namespace navground::sim

#endif  // NAVGROUND_SIM_GEOMETRY_H
