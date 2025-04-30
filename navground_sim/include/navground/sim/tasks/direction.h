/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_TASKS_DIRECTION_H_
#define NAVGROUND_SIM_TASKS_DIRECTION_H_

#include <vector>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/task.h"

namespace navground::sim {

/**
 * @brief      This class implement a task that makes the agent follow a fixed
 * direction
 *
 * *Registered properties*:
 *
 *   - `direction` (\ref navground::core::Vector2, \ref get_direction)
 */
struct NAVGROUND_SIM_EXPORT DirectionTask : Task {
  static const std::string type;
  
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  direction   The direction
   */
  explicit DirectionTask(core::Vector2 direction = core::Vector2(1, 0))
      : Task(), _direction(direction), _stop(direction.norm() == 0) {}

  virtual ~DirectionTask() = default;

  /**
   * @private
   */
  bool done() const override;

  /**
   * @brief      Sets the direction.
   *
   * @param[in]  value  The desired direction
   */
  void set_direction(const core::Vector2 &value) {
    _direction = value;
    _stop = _direction.norm() == 0;
  }

  /**
   * @brief      Gets the direction.
   *
   * @return     The direction.
   */
  core::Vector2 get_direction() const { return _direction; }

protected:
  /**
   * @private
   */
  void prepare(Agent *agent, World *world) override;

private:
  core::Vector2 _direction;
  bool _stop;
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASKS_DIRECTION_H_ */
