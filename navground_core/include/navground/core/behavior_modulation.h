/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_MODULATION_H_
#define NAVGROUND_CORE_BEHAVIOR_MODULATION_H_

#include "navground/core/common.h"
#include "navground/core/property.h"
#include "navground/core/register.h"
#include "navground/core/types.h"
#include "navground_core_export.h"

namespace navground::core {

class Behavior;

/**
 * @brief      This class describes a generic behavior modulation that happens
 *             right before (\ref pre) and right after (\ref post) computing a
 *             command.
 *
 */
class NAVGROUND_CORE_EXPORT BehaviorModulation
    : virtual public HasProperties,
      virtual public HasRegister<BehaviorModulation> {
 public:
  using HasRegister<BehaviorModulation>::C;

  BehaviorModulation() : _enabled(true) {}

  virtual ~BehaviorModulation() = default;

  /**
   * @brief      Called right before the evaluation of behavior in
   *             \ref Behavior::compute_cmd.
   *
   * @param      behavior   The behavior being evaluated
   * @param[in]  time_step  The time step
   */
  virtual void pre([[maybe_unused]] Behavior& behavior,
                   [[maybe_unused]] ng_float_t time_step) {}
  /**
   * @brief      Called right after the evaluation of behavior in
   *             \ref Behavior::compute_cmd.
   *
   * @param      behavior   The behavior being evaluated
   * @param[in]  time_step  The time step
   * @param[in]  cmd        The command just computed by the behavior
   */
  virtual Twist2 post([[maybe_unused]] Behavior& behavior,
                      [[maybe_unused]] ng_float_t time_step,
                      const Twist2& cmd) {
    return cmd;
  }

  /**
   * @brief      Returns whether the modulation is enabled
   *
   * @return     True if enabled.
   */
  bool get_enabled() const { return _enabled; }

  /**
   * @brief      Sets whether the modulation is enabled.
   *
   * @param[in]  value  The desired value
   */
  void set_enabled(bool value) { _enabled = value; }

 private:
  bool _enabled;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_MODULATION_H_
