/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_CACHED_COLLISION_COMPUTATION_H_
#define NAVGROUND_CORE_CACHED_COLLISION_COMPUTATION_H_

#include <algorithm>
#include <array>
#include <valarray>
#include <vector>

#include "navground/core/behavior.h"
#include "navground/core/collision_computation.h"
#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/core/export.h"

namespace navground::core {

/**
 * @brief      This class extend \ref CollisionComputation to cache the results.
 *
 * It assumes that the agents is only interested in possible collisions when
 * moving in directions comprised in an interval \f$[\alpha, \alpha +
 * \Delta]\f$, represented by in \f$N\f$ points at regular steps, only up to a
 * maximal distance \f$D\f$, and that the agent moves at at given speed.
 */
class NAVGROUND_CORE_EXPORT CachedCollisionComputation
    : public CollisionComputation {
 public:
  /**
   * Construct an instance
   */
  CachedCollisionComputation()
      : CollisionComputation(),
        from_relative_angle(0),
        length(0),
        resolution(0),
        speed(0),
        max_distance(0),
        dynamic_cache(),
        static_cache() {}

  /**
   * @brief      Sets the resolution: the number of discrete angles \f$N\f$.
   *
   * @param[in]  value  A positive value
   */
  void set_resolution(size_t value);

  /**
   * @brief      Gets the resolution: the number of discrete angles \f$N\f$.
   *
   * @return     The size of the cache.
   */
  size_t get_resolution() const { return resolution; }

  /**
   * @brief      Sets the cache interval lower bound \f$\alpha\f$.
   *
   * @param[in]  value  The lower bound
   */
  void set_min_angle(Radians value);
  /**
   * @brief      Gets the cache interval lower bound \f$\alpha\f$.
   *
   * @return     The lower bound.
   */
  ng_float_t get_min_angle() const { return from_relative_angle; }

  /**
   * @brief      Sets the cache interval length \f$\Delta\f$.
   *
   * @param[in]  value  The interval length
   */
  void set_length(ng_float_t value);

  /**
   * @brief      Gets the cache interval length \f$\Delta\f$.
   *
   * @return     The interval length
   */
  ng_float_t get_length() const { return length; }

  /**
   * @brief      Sets the maximal distance \f$D\f$ to consider
   *
   * @param[in]  value  The maximal distance
   */
  void set_max_distance(ng_float_t value);
  /**
   * @brief      Gets the maximal distance \f$D\f$ to consider
   *
   * @return     The maximal distance
   */
  ng_float_t get_max_distance() const { return max_distance; }

  /**
   * @brief      Sets the agent speed.
   *
   * @param[in]  value  The speed
   */
  void set_speed(ng_float_t value);

  /**
   * @brief      Gets the agent speed.
   *
   * @return     The agent speed.
   */
  ng_float_t get_speed() const { return speed; }

  /**
   * @brief      Returns the free distance if the agent will be static
   *
   * @param[in]  angle              The angle (absolute)
   * @param[in]  include_neighbors  Indicates if the neighbors should be
   * included in the computation
   *
   * @return     The distance in direction `angle` before possibly colliding
   */
  ng_float_t static_free_distance(Radians angle, bool include_neighbors = true);

  /**
   * @brief      Returns the free distance if the agent will be move
   *
   * @param[in]  angle         The angle (absolute)
   *
   * @return     The distance in direction `angle` before possibly colliding
   */
  ng_float_t dynamic_free_distance(Radians angle);

  void setup(Pose2 pose_, ng_float_t margin_,
             const std::vector<LineSegment> &line_segments,
             const std::vector<DiscCache> &static_discs,
             const std::vector<DiscCache> &dynamic_discs);

  void setup(Pose2 pose_, ng_float_t margin_,
             const std::vector<LineSegment> &line_segments,
             const std::vector<Disc> &static_discs,
             const std::vector<Neighbor> &dynamic_discs);
  /**
   * @brief      Resets the cache.
   */
  void reset();

  /**
   * @brief      Gets a pointer where collision distances are cached. Negative
   * entries mean distance not computed (-2) or no collision (-1).

   * @param[in]  assuming_static  The assuming static
   * @param[in]  include_neighbors  Indicates if the neighbors should be
   * included in the computation
   *
   * @return     The collision distance cache.
   */
  const std::valarray<ng_float_t> &get_cache(bool assuming_static = false,
                                             bool include_neighbors = true);
  /**
   * @brief      Returns the free distance to collision for the cached
   * interval of headings.
   *
   * @param[in]  dynamic       If the agent is moving
   *
   * @return     The distance before possibly colliding for each direction
   * in the interval \f$[\alpha, \alpha + \Delta]\f$.
   */
  std::valarray<ng_float_t> get_free_distance(bool dynamic = false);

 private:
  /**
   * Marks yet-not-computed entries in \ref get_cache
   */
  static constexpr ng_float_t uncomputed = -2;

  ng_float_t from_relative_angle;
  ng_float_t length;
  size_t resolution;
  ng_float_t speed;
  ng_float_t max_distance;
  std::valarray<ng_float_t> dynamic_cache;
  std::array<std::valarray<ng_float_t>, 2> static_cache;

  // can be outsize of 0 ... resolution range
  int index_of(Radians delta_relative_angle);
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_CACHED_COLLISION_COMPUTATION_H_
