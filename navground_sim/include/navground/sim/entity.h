/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_ENTITY_H_
#define NAVGROUND_SIM_ENTITY_H_

#include "navground/core/types.h"
#include "navground/sim/export.h"

namespace navground::sim {

/**
 * @brief      Super-class that adds a unique ID to world entities.
 *
 * This unique ID should not be fed to navigation behaviors,
 * but only used internally by the simulation,
 * for instance, to identify entities in a UI.
 */
struct NAVGROUND_SIM_EXPORT Entity {
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  id    The identifier
   * @param[in]  ignore_collisions    Whether to ignore collisions
   */
  explicit Entity(unsigned id, bool ignore_collisions)
      : uid(id), last_collision_time(-1),
        _ignore_collisions(ignore_collisions) {}

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  ignore_collisions    Whether to ignore collisions
   */
  explicit Entity(bool ignore_collisions = false)
      : Entity(_uid++, ignore_collisions) {}

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  id    The identifier
   * @param[in]  ignore_collisions    Whether to ignore collisions
   */
  explicit Entity(const Entity &other)
      : Entity(other.get_ignore_collisions()) {}

  /**
   * @brief      Reset the UID counter to zero.
   */
  static void reset_uid() { _uid = 0; }

  virtual ~Entity() = default;

  /**
   * Unique identifier
   */
  unsigned uid;

  /**
   * @brief      Determines if the entity has been in collision at least once
   * since the given time.
   *
   * @param[in]  time  The time
   *
   * @return     True if been in collision since, False otherwise.
   */
  bool has_been_in_collision_since(ng_float_t time) const {
    return last_collision_time >= 0 && last_collision_time >= time;
  }

  /**
   * @private
   * @brief      Sets as colliding at a given time.
   *
   * @param[in]  time  The time
   */
  void set_as_colliding_at(ng_float_t time) { last_collision_time = time; }

  /**
   * @brief      Returns the last collision time
   *
   * @return     The time
   */
  ng_float_t get_last_collision_time() const { return last_collision_time; }

  /**
   * @brief      Resets the entity.
   */
  void reset() { last_collision_time = -1; }

  /**
   * @brief      Gets whether collisions are ignored for this entity.
   *
   * @return     True if collisions are ignored
   */
  bool get_ignore_collisions() const { return _ignore_collisions; }

  /**
   * @brief      Sets whether collisions are ignored for this entity.
   *
   * @param[in]  value   True to ignore collisions.
   */
  void set_ignore_collisions(bool value) {
    _ignore_collisions = value;
    if (!value) {
      last_collision_time = -1;
    }
  }

private:
  static inline unsigned _uid = 0;
  ng_float_t last_collision_time;
  bool _ignore_collisions;
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_ENTITY_H_ */
