/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_COLLISION_COMPUTATION_H_
#define NAVGROUND_CORE_COLLISION_COMPUTATION_H_

#include <algorithm>
#include <valarray>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/states/geometric.h"
#include "navground/core/types.h"
#include "navground_core_export.h"

namespace navground::core {

/**
 * @brief      An struct that holds pre-computed values about a disc-shaped
 * obstacle to speed up collision checking. For collision checking, the agent is
 * considered point-like, i.e, the radius of the agent should be added to the
 * disc-cache margin.
 */
struct NAVGROUND_CORE_EXPORT DiscCache {
  /**
   *  the difference of positions (obstacle - agent) in absolute frame
   */
  Vector2 delta;
  /**
   * The disc velocity in absolute frame
   */
  Vector2 velocity;
  /**
   * The distance between disc and agent (computed)
   */
  ng_float_t distance;
  ng_float_t C;
  Radians gamma;
  Radians visible_angle;

  /**
   * @brief      Construct a disc cache
   *
   * @param[in]  delta     The difference of positions (``obstacle - agent``) in
   * absolute frame
   * @param[in]  margin    The margin (sum of the radii and safety margin)
   * @param[in]  velocity  The disc velocity
   */
  DiscCache(Vector2 delta, ng_float_t margin,
            Vector2 velocity = Vector2::Zero(),
            ng_float_t visible_angle = M_PI_2);
};

/**
 * @brief      This class compute collisions of moving points with lists of \ref
 * DiscCache and \ref LineSegment.
 */
class NAVGROUND_CORE_EXPORT CollisionComputation {
 public:
  static inline std::vector<LineSegment> empty = {};

  /**
   * Construct an instance.
   */
  CollisionComputation()
      : line_obstacles(empty),
        neighbors_cache(),
        static_obstacles_cache(),
        pose(),
        margin(0.0) {}

  /**
   * @brief      Return the free distance to collision for an interval of
   * headings.
   *
   * @param[in]  from          The interval lower bound
   * @param[in]  length        The length of the interval
   * @param[in]  resolution    The number of values in the interval
   * @param[in]  max_distance  The maximum distance to consider
   *                           (collision behind this distance are effectively
   * ignored)
   * @param[in]  dynamic       If the agent is moving
   * @param[in]  speed         The speed at which the agent is moving
   *
   * @return     The free distance for each angle in the interval [from, from +
   * length].
   */
  std::valarray<ng_float_t> get_free_distance_for_sector(
      Radians from, Radians length, size_t resolution, ng_float_t max_distance,
      bool dynamic = false, ng_float_t speed = 0);

  /**
   * @brief      Return regularly sampled angles.
   *
   * @param[in]  from          The interval lower bound
   * @param[in]  length        The length of the interval
   * @param[in]  resolution    The number of values in the interval
   *
   * @return     Angles regularly sampled in the interval [from, from +
   * length].
   */
  std::valarray<ng_float_t> get_angles_for_sector(Radians from, Radians length,
                                                  size_t resolution) const;

  /**
   * @brief      Return the polar contour for an interval of
   * headings.
   *
   * @param[in]  from          The interval lower bound
   * @param[in]  length        The length of the interval
   * @param[in]  resolution    The number of values in the interval
   * @param[in]  max_distance  The maximum distance to consider
   *                           (collision behind this distance are effectively
   * ignored)
   * @param[in]  dynamic       If the agent is moving
   * @param[in]  speed         The speed at which the agent is moving
   *
   * @return     An arrays of angles sampled regularly in the interval [from,
   * from + length] and one array with the free distance in their direction.
   */
  std::tuple<std::valarray<ng_float_t>, std::valarray<ng_float_t>>
  get_contour_for_sector(Radians from, Radians length, size_t resolution,
                         ng_float_t max_distance, bool dynamic = false,
                         ng_float_t speed = 0) {
    return {get_angles_for_sector(from, length, resolution),
            get_free_distance_for_sector(from, length, resolution, max_distance,
                                         dynamic, speed)};
  }

  /**
   * @brief      Set the state from collections of \ref LineSegment and \ref
   * DiscCache.
   *
   * @param[in]  pose_          The pose of the agent
   * @param[in]  margin_        The margin
   * @param[in]  line_segments  The line segments
   * @param[in]  static_discs   The static discs
   * @param[in]  dynamic_discs  The dynamic discs
   */
  void setup(Pose2 pose_, ng_float_t margin_,
             const std::vector<LineSegment> &line_segments,
             std::vector<DiscCache> static_discs,
             std::vector<DiscCache> dynamic_discs) {
    line_obstacles = line_segments;
    static_obstacles_cache = static_discs;
    neighbors_cache = dynamic_discs;
    pose = pose_;
    margin = margin_;
  }

  /**
   * @brief      Set the state from collections of \ref LineSegment, \ref Disc,
   * and \ref Neighbor.
   *
   * @param[in]  pose_          The pose
   * @param[in]  margin_        The margin
   * @param[in]  line_segments  The line segments
   * @param[in]  static_discs   The static discs
   * @param[in]  dynamic_discs  The dynamic discs
   */
  void setup(Pose2 pose_, ng_float_t margin_,
             const std::vector<LineSegment> &line_segments,
             const std::vector<Disc> &static_discs,
             const std::vector<Neighbor> &dynamic_discs) {
    line_obstacles = line_segments;
    pose = pose_;
    margin = margin_;
    neighbors_cache.clear();
    neighbors_cache.reserve(dynamic_discs.size());
    for (const auto &disc : dynamic_discs) {
      neighbors_cache.push_back({disc.position - pose.position,
                                 margin_ + disc.radius, disc.velocity});
    }
    static_obstacles_cache.clear();
    static_obstacles_cache.reserve(static_discs.size());
    for (const auto &disc : static_discs) {
      static_obstacles_cache.push_back(
          {disc.position - pose.position, margin_ + disc.radius});
    }
  }

  /**
   * @brief      Returns the free distance if the agent will be static
   *
   * @param[in]  angle              The angle (absolute)
   * @param[in]  max_distance       The maximal distance to consider
   * @param[in]  include_neighbors  Indicates if the neighbors should be
   * included in the computation
   *
   * @return     The distance in direction `angle` before possibly colliding
   */
  ng_float_t static_free_distance(Radians angle, ng_float_t max_distance,
                                  bool include_neighbors = true);
  /**
   * @brief      Returns the free distance if the agent will be move
   *
   * @param[in]  angle         The angle (absolute)
   * @param[in]  max_distance  The maximal distance to consider
   * @param[in]  speed         The speed of the agent
   *
   * @return     The distance in direction `angle` before possibly colliding
   */
  ng_float_t dynamic_free_distance(Radians angle, ng_float_t max_distance,
                                   ng_float_t speed);

  /**
   * @brief      Tentatively checks whenever a moving disc-cache may collide
   * with the agent within an horizon
   *
   * @param[in]  obstacle      The obstacle
   * @param[in]  max_distance  The maximal distance to consider
   * @param[in]  speed         The speed of the agent
   *
   * @return     False if it is impossible that the agent collides with the
   * obstacle within `max_distance`.
   */
  bool dynamic_may_collide(const DiscCache &obstacle, ng_float_t max_distance,
                           ng_float_t speed);
  /**
   * @brief      Tentatively checks whenever a static disc-cache may collide
   * with the agent within an horizon
   *
   * @param[in]  obstacle      The obstacle
   * @param[in]  max_distance  The maximal distance to consider
   *
   * @return     False if it is impossible that the agent collides with the
   * obstacle within `max_distance`.
   */
  bool static_may_collide(const DiscCache &obstacle, ng_float_t max_distance);

 protected:
  // Should be a ref to avoid copies
  std::vector<LineSegment> &line_obstacles;
  std::vector<DiscCache> neighbors_cache;
  std::vector<DiscCache> static_obstacles_cache;
  Pose2 pose;
  ng_float_t margin;

  /**
   * Marks absence of collisions
   */
  static constexpr int no_collision = -1;
  ng_float_t static_free_distance_to(const LineSegment &line, Radians alpha);

  ng_float_t static_free_distance_to(const DiscCache &disc, Radians alpha);

  ng_float_t dynamic_free_distance_to(const DiscCache &disc, Radians alpha,
                                      ng_float_t speed);

  template <typename T>
  ng_float_t static_free_distance_to_collection(Radians angle,
                                                ng_float_t max_distance,
                                                const std::vector<T> &objects) {
    ng_float_t min_distance = max_distance;
    for (const auto &object : objects) {
      ng_float_t distance = static_free_distance_to(object, angle);
      if (distance < 0) continue;
      min_distance = fmin(min_distance, distance);
      if (min_distance == 0) return 0;
    }
    return min_distance;
  }

  template <typename T>
  ng_float_t dynamic_free_distance_to_collection(
      Radians angle, ng_float_t max_distance, ng_float_t speed,
      const std::vector<T> &objects) {
    ng_float_t min_distance = max_distance;
    for (const auto &object : objects) {
      ng_float_t distance = dynamic_free_distance_to(object, angle, speed);
      if (distance < 0) continue;
      min_distance = fmin(min_distance, distance);
      if (min_distance == 0) return 0;
    }
    return min_distance;
  }
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_COLLISION_COMPUTATION_H_
