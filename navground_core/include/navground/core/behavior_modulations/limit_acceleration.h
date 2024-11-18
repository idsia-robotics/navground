/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_MODULATIONS_LIMIT_ACCELERATION_H_
#define NAVGROUND_CORE_BEHAVIOR_MODULATIONS_LIMIT_ACCELERATION_H_

#include <limits>

#include "navground/core/behavior_modulation.h"
#include "navground/core/export.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief      A modulation that limits accelerations.
 *
 * *Registered properties*:
 *
 *   - max_acceleration (see \ref get_max_acceleration)
 *
 *   - max_angular_acceleration (see \ref get_max_angular_acceleration)
 */
class NAVGROUND_CORE_EXPORT LimitAccelerationModulation
    : public BehaviorModulation {
public:
  static const std::string type;
  
  /**
   * @brief      Construct a new instance
   *
   * @param[in]  max_acceleration          The maximal acceleration
   * @param[in]  max_angular_acceleration  The maximal angular acceleration
   */
  explicit LimitAccelerationModulation(
      ng_float_t max_acceleration = std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t max_angular_acceleration =
          std::numeric_limits<ng_float_t>::infinity())
      : BehaviorModulation(), _max_acceleration(max_acceleration),
        _max_angular_acceleration(max_angular_acceleration) {}

  /**
   * @private
   */
  Twist2 post(Behavior &behavior, ng_float_t time_step,
              const Twist2 &cmd) override;

  /**
   * @brief      Gets the maximal acceleration.
   *
   * @return     A positive number
   */
  ng_float_t get_max_acceleration() const { return _max_acceleration; }
  /**
   * @brief      Sets the maximal acceleration.
   *
   * @param[in]  value  A positive number. If lower than zero or infinite,
   * clipping is disabled.
   */
  void set_max_acceleration(ng_float_t value) {
    if (value >= 0) {
      _max_acceleration = value;
    } else {
      _max_acceleration = std::numeric_limits<ng_float_t>::infinity();
    }
  }

  /**
   * @brief      Gets the maximal angular acceleration.
   *
   * @return     A positive number
   */
  ng_float_t get_max_angular_acceleration() const {
    return _max_angular_acceleration;
  }
  /**
   * @brief      Sets the maximal angular acceleration.
   *
   * @param[in]  value  A positive number. If lower than zero or infinite,
   * clipping is disabled.
   */
  void set_max_angular_acceleration(ng_float_t value) {
    if (value >= 0) {
      _max_angular_acceleration = value;
    } else {
      _max_angular_acceleration = std::numeric_limits<ng_float_t>::infinity();
    }
  }

private:
  ng_float_t _max_acceleration;
  ng_float_t _max_angular_acceleration;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_BEHAVIOR_MODULATIONS_LIMIT_ACCELERATION_H_
