/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_TASKS_DIRECTION_H_
#define NAVGROUND_SIM_TASKS_DIRECTION_H_

#include <vector>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/task.h"
#include "navground/sim/export.h"

using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;
using navground::core::Vector2;

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
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  direction   The direction
   */
  explicit DirectionTask(Vector2 direction = Vector2(1, 0))
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
  void set_direction(const Vector2 &value) {
    _direction = value;
    _stop = _direction.norm();
  }

  /**
   * @brief      Gets the direction.
   *
   * @return     The direction.
   */
  Vector2 get_direction() const { return _direction; }

  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static const std::map<std::string, Property> properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 protected:
  /**
   * @private
   */
  void prepare(Agent *agent, World *world) const override;

 private:
  Vector2 _direction;
  bool _stop;
  const static std::string type;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_TASKS_DIRECTION_H_ */
