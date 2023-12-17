/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <random>

#ifndef NAVGROUND_TYPES_H_
#define NAVGROUND_TYPES_H_

#ifdef NAVGROUND_USES_DOUBLE
using ng_float_t = double;
#else
using ng_float_t = float;
#endif  // NAVGROUND_USES_DOUBLE

using RandomGenerator = std::mt19937;

#endif  // NAVGROUND_TYPES_H_
