/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_MODULATIONS_LIMIT_TWIST_H_
#define NAVGROUND_CORE_BEHAVIOR_MODULATIONS_LIMIT_TWIST_H_

#include <limits>

#include "navground/core/behavior_modulation.h"
#include "navground/core/export.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief      A simple modulation that clip the twist command.
 *
 * *Registered properties*:
 *
 *   - forward (see \ref get_max_forward_speed)
 *
 *   - backward (see \ref get_max_backward_speed)
 *
 *   - leftward (see \ref get_max_leftward_speed)
 *
 *   - rightward (see \ref get_max_rightward_speed)
 *
 *   - angular (see \ref get_max_angular_speed)
 */
class NAVGROUND_CORE_EXPORT LimitTwistModulation : public BehaviorModulation {
public:
  /**
   * @brief      Construct a new instance
   *
   * @param[in]  tau      The relaxation time
   */
  explicit LimitTwistModulation(
      ng_float_t forward = std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t backward = std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t leftward = std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t rightward = std::numeric_limits<ng_float_t>::infinity(),
      ng_float_t angular = std::numeric_limits<ng_float_t>::infinity())
      : BehaviorModulation(), _max_forward_speed(forward),
        _max_backward_speed(backward), _max_leftward_speed(leftward),
        _max_rightward_speed(rightward), _max_angular_speed(angular) {}

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  Twist2 post(Behavior &behavior, ng_float_t time_step,
              const Twist2 &cmd) override;

  /**
   * @brief      Gets the maximal forward speed.
   *
   * @return     A positive number
   */
  ng_float_t get_max_forward_speed() const { return _max_forward_speed; }
  /**
   * @brief      Sets the maximal forward speed.
   *
   * @param[in]  value  If lower than zero or infinite,
   * clipping is effectively disabled.
   */
  void set_max_forward_speed(ng_float_t value) {
    if (value >= 0) {
      _max_forward_speed = value;
    } else {
      _max_forward_speed = std::numeric_limits<ng_float_t>::infinity();
    }
  }
  /**
   * @brief      Gets the maximal backward speed.
   *
   * @return     A positive number
   */
  ng_float_t get_max_backward_speed() const { return _max_backward_speed; }
  /**
   * @brief      Sets the maximal backward speed.
   *
   * @param[in]  value  If lower than zero or infinite,
   * clipping is effectively disabled.
   */
  void set_max_backward_speed(ng_float_t value) {
    if (value >= 0) {
      _max_backward_speed = value;
    } else {
      _max_backward_speed = std::numeric_limits<ng_float_t>::infinity();
    }
  }
  /**
   * @brief      Gets the maximal leftward speed.
   *
   * @return     A positive number
   */
  ng_float_t get_max_leftward_speed() const { return _max_leftward_speed; }
  /**
   * @brief      Sets the maximal leftward speed.
   *
   * @param[in]  value  If lower than zero or infinite,
   * clipping is effectively disabled.
   */
  void set_max_leftward_speed(ng_float_t value) {
    if (value >= 0) {
      _max_leftward_speed = value;
    } else {
      _max_leftward_speed = std::numeric_limits<ng_float_t>::infinity();
    }
  }
  /**
   * @brief      Gets the maximal rightward speed.
   *
   * @return     A positive number
   */
  ng_float_t get_max_rightward_speed() const { return _max_rightward_speed; }
  /**
   * @brief      Sets the maximal rightward speed.
   *
   * @param[in]  value  If lower than zero or infinite,
   * clipping is effectively disabled.
   */
  void set_max_rightward_speed(ng_float_t value) {
    if (value >= 0) {
      _max_rightward_speed = value;
    } else {
      _max_rightward_speed = std::numeric_limits<ng_float_t>::infinity();
    }
  }
  /**
   * @brief      Gets the maximal angular speed.
   *
   * @return     A positive number
   */
  ng_float_t get_max_angular_speed() const { return _max_angular_speed; }
  /**
   * @brief      Sets the maximal angular speed.
   *
   * @param[in]  value  If lower than zero or infinite,
   * clipping is effectively disabled.
   */
  void set_max_angular_speed(ng_float_t value) {
    if (value >= 0) {
      _max_angular_speed = value;
    } else {
      _max_angular_speed = std::numeric_limits<ng_float_t>::infinity();
    }
  }

  /** @private
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * Properties: forward, backward, leftward, rightward, angular
   * @private
   */
  static const std::map<std::string, Property> properties;

private:
  static const std::string type;
  ng_float_t _max_forward_speed;
  ng_float_t _max_backward_speed;
  ng_float_t _max_leftward_speed;
  ng_float_t _max_rightward_speed;
  ng_float_t _max_angular_speed;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_BEHAVIOR_MODULATIONS_LIMIT_TWIST_H_
