/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_GEOMETRY_H
#define NAVGROUND_SIM_GEOMETRY_H

#include <geos/geom/Envelope.h>

#include <random>

#include "navground/core/states/geometric.h"
#include "navground/sim/export.h"

namespace navground::sim {

using BoundingBox = geos::geom::Envelope;

/**
 * @brief      Sample discs
 *
 * Centers are sampled uniformly inside a bounding box and rejected if too near
 * to any other disc, taking into account the lattice. The maximal number of
 * rejection is bound by ``max_tries``. The function exit when too many
 * rejection are reached, even if not enough discs have been sampled.
 *
 * @param      rng           The random number generator
 * @param[in]  number        The number of discs
 * @param[in]  bounding_box  The bounding box inside of which 
 *                           to sample disc centers
 * @param[in]  min_radius    The minimum value for disc radius
 * @param[in]  max_radius    The maximum value for disc radius
 * @param[in]  margin        The minimal distance between newly sampled discs
 * @param[in]  disc_margin   The minimal distance between newly 
 *                           sampled discs and discs in ``discs``
 * @param[in]  discs         A list of discs already present.
 * @param[in]  max_tries     The maximum sampling failures 
 *                           before exiting the function.
 * @param[in]  lattice_grid  The lattice grid
 *
 * @return     A list of valid disc
 */
std::vector<core::Disc> NAVGROUND_SIM_EXPORT sample_discs(
    RandomGenerator &rng, unsigned number, const BoundingBox &bounding_box,
    ng_float_t min_radius, ng_float_t max_radius, ng_float_t margin = 0.0,
    ng_float_t disc_margin = 0.0, const std::vector<core::Disc> &discs = {},
    unsigned max_tries = 1000, std::vector<core::Vector2> lattice_grid = {});

}  // namespace navground::sim

#endif  // NAVGROUND_SIM_GEOMETRY_H
