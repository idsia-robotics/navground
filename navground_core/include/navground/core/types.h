/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <random>
#include <type_traits>
#include <vector>

#ifndef NAVGROUND_TYPES_H_
#define NAVGROUND_TYPES_H_

#ifdef NAVGROUND_USES_DOUBLE
using ng_float_t = double;
#else
using ng_float_t = float;
#endif // NAVGROUND_USES_DOUBLE

using RandomGenerator = std::mt19937;

template <typename> struct is_std_vector : std::false_type {};

template <typename T, typename A>
struct is_std_vector<std::vector<T, A>> : std::true_type {};

template <class T> constexpr bool is_std_vector_v = is_std_vector<T>::value;

#endif // NAVGROUND_TYPES_H_
