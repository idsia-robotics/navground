/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_ENTITY_H_
#define NAVGROUND_SIM_ENTITY_H_

#include "navground_sim_export.h"

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
   */
  Entity() : uid(_uid++), last_collision_time(-1) {}

  virtual ~Entity() = default;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  id    The identifier
   */
  explicit Entity(unsigned id) : uid(id), last_collision_time(-1) {}

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
  bool has_been_in_collision_since(float time) const {
    return last_collision_time >= 0 && last_collision_time >= time;
  }

  /**
   * @private
   * @brief      Sets as colliding at a given time.
   *
   * @param[in]  time  The time
   */
  void set_as_colliding_at(float time) { last_collision_time = time; }

 private:
  static inline unsigned _uid = 0;
  float last_collision_time;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_ENTITY_H_ */
