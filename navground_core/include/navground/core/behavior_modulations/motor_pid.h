/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_MODULATIONS_MOTOR_PID_H_
#define NAVGROUND_CORE_BEHAVIOR_MODULATIONS_MOTOR_PID_H_

#include "navground/core/behavior_modulation.h"
#include "navground/core/export.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief      A modulation that control motor torques using a PID.
 *
 *             Requires that the behavior has a
 *             \ref DynamicTwoWheelsDifferentialDriveKinematics kinematics.
 *
 * *Registered properties*:
 *
 *   - k_p (see \ref get_k_p, default 1)
 *
 *   - k_d (see \ref get_k_d, default 0)
 *
 *   - k_i (see \ref get_k_i, default 0)
 */
class NAVGROUND_CORE_EXPORT MotorPIDModulation : public BehaviorModulation {
public:
  static const std::string type;

  /**
   * @brief      Construct a new instance
   *
   * @param[in]  k_p      The P factor
   * @param[in]  k_i      The I factor
   * @param[in]  k_d      The D factor
   */
  explicit MotorPIDModulation(ng_float_t k_p = 1, ng_float_t k_i = 0,
                              ng_float_t k_d = 0)
      : BehaviorModulation(), _k_p(k_p), _k_i(k_i), _k_d(k_d), _e({0, 0}),
        _ie({0, 0}), _torques({0, 0}) {}

  /**
   * @private
   */
  Twist2 post(Behavior &behavior, ng_float_t time_step,
              const Twist2 &cmd) override;

  /**
   * @brief      Gets the P factor.
   *
   * @return     A number
   */
  ng_float_t get_k_p() const { return _k_p; }
  /**
   * @brief      Sets the P factor.
   *
   * @param[in]  value  A number.
   */
  void set_k_p(ng_float_t value) { _k_p = value; }

  /**
   * @brief      Gets the P factor.
   *
   * @return     A number
   */
  ng_float_t get_k_i() const { return _k_i; }
  /**
   * @brief      Sets the I factor.
   *
   * @param[in]  value  A number.
   */
  void set_k_i(ng_float_t value) { _k_i = value; }

  /**
   * @brief      Gets the D factor.
   *
   * @return     A number
   */
  ng_float_t get_k_d() const { return _k_d; }
  /**
   * @brief      Sets the D factor.
   *
   * @param[in]  value  A number.
   */
  void set_k_d(ng_float_t value) { _k_d = value; }

private:
  ng_float_t _k_p, _k_i, _k_d;
  std::vector<ng_float_t> _e;
  std::vector<ng_float_t> _ie;
  std::vector<ng_float_t> _torques;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_BEHAVIOR_MODULATIONS_MOTOR_PID_H_
