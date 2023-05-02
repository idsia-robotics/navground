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
  Entity() : uid(_uid++) {}

  virtual ~Entity() = default;

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  id    The identifier
   */
  explicit Entity(unsigned id) : uid(id) {}

  /**
   * Unique identifier
   */
  unsigned uid;

 private:
  static inline unsigned _uid = 0;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_ENTITY_H_ */
