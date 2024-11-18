/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_ODOMETRY_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_ODOMETRY_H_

#include <random>
#include <vector>

#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"

namespace navground::sim {

/**
 * @brief      A sensor that add normal error to the true twist and integrates
 * it.
 *
 * *Registered properties*:
 *
 *   - `longitudinal_speed_std_dev` (float, \ref get_longitudinal_speed_std_dev)
 *
 *   - `longitudinal_speed_bias` (float, \ref get_longitudinal_speed_bias)
 *
 *   - `transversal_speed_std_dev` (float, \ref get_transversal_speed_std_dev)
 *
 *   - `transversal_speed_bias` (float, \ref get_transversal_speed_bias)
 *
 *   - `angular_speed_std_dev` (float, \ref get_angular_speed_std_dev)
 *
 *   - `angular_speed_bias` (float, \ref get_angular_speed_bias)
 *
 *   - `update_ego_state` (bool, \ref get_update_ego_state)
 *
 *   - `update_sensig_state` (bool, \ref get_update_sensing_state)
 *
 */
struct NAVGROUND_SIM_EXPORT OdometryStateEstimation : public Sensor {
  DECLARE_TYPE_AND_PROPERTIES

  using Error = std::normal_distribution<ng_float_t>;
  /**
   * The default longitudinal speed standard deviation.
   */
  inline static const ng_float_t default_longitudinal_speed_std_dev = 0;
  /**
   * The default transversal speed standard deviation.
   */
  inline static const ng_float_t default_transversal_speed_std_dev = 0;
  /**
   * The default angular speed standard deviation.
   */
  inline static const ng_float_t default_angular_speed_std_dev = 0;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  longitudinal_speed_bias  The longitudinal speed bias
   * @param[in]  longitudinal_speed_std_dev  The longitudinal speed standard
   * deviation
   * @param[in]  transversal_speed_bias  The longitudinal speed bias
   * @param[in]  transversal_speed_std_dev  The transversal speed standard
   * deviation
   * @param[in]  angular_speed_bias  The longitudinal speed bias
   * @param[in]  angular_speed_std_dev  The angular speed standard deviation
   * @param[in]  update_sensing_state  Whether to update the behavior
   * environment state
   * @param[in]  update_ego_state  Whether to update the behavior ego state
   * @param[in]  name  The name to use as a prefix
   */
  explicit OdometryStateEstimation(
      ng_float_t longitudinal_speed_bias = 0,
      ng_float_t longitudinal_speed_std_dev =
          default_longitudinal_speed_std_dev,
      ng_float_t transversal_speed_bias = 0,
      ng_float_t transversal_speed_std_dev = default_transversal_speed_std_dev,
      ng_float_t angular_speed_bias = 0,
      ng_float_t angular_speed_std_dev = default_angular_speed_std_dev,
      bool update_sensing_state = true, bool update_ego_state = false,
      const std::string &name = "")
      : Sensor(name), _pose(), _twist(), _time(0),
        _update_sensing_state(update_sensing_state),
        _update_ego_state(update_ego_state),
        _longitudinal_speed_error{longitudinal_speed_bias,
                                  longitudinal_speed_std_dev},
        _transversal_speed_error{transversal_speed_bias,
                                 transversal_speed_std_dev},
        _angular_speed_error{angular_speed_bias, angular_speed_std_dev} {}

  virtual ~OdometryStateEstimation() = default;

  /**
   * @brief      Sets the longitudinal speed relative bias.
   *
   * @param[in]  value     The desired value
   */
  void set_longitudinal_speed_bias(ng_float_t value) {
    _longitudinal_speed_error.param(
        Error::param_type{value, get_longitudinal_speed_std_dev()});
  }
  /**
   * @brief      Sets the longitudinal speed relative standard deviation.
   *
   * @param[in]  value     The desired (positive) value
   */
  void set_longitudinal_speed_std_dev(ng_float_t value) {
    _longitudinal_speed_error.param(Error::param_type{
        get_longitudinal_speed_bias(), std::max<ng_float_t>(0, value)});
  }
  /**
   * @brief      Gets the longitudinal speed relative bias.
   * rangings.
   *
   * @return     The bias.
   */
  ng_float_t get_longitudinal_speed_bias() const {
    return _longitudinal_speed_error.mean();
  }
  /**
   * @brief      Gets the longitudinal speed relative standard deviation.
   * rangings.
   *
   * @return     The standard deviation.
   */
  ng_float_t get_longitudinal_speed_std_dev() const {
    return _longitudinal_speed_error.stddev();
  }
  /**
   * @brief      Sets the transversal speed relative bias.
   *
   * @param[in]  value     The desired value
   */
  void set_transversal_speed_bias(ng_float_t value) {
    _transversal_speed_error.param(
        Error::param_type{value, get_transversal_speed_std_dev()});
  }
  /**
   * @brief      Sets the transversal speed relative standard deviation.
   *
   * @param[in]  value     The desired (positive) value
   */
  void set_transversal_speed_std_dev(ng_float_t value) {
    _transversal_speed_error.param(Error::param_type{
        get_transversal_speed_bias(), std::max<ng_float_t>(0, value)});
  }
  /**
   * @brief      Gets the transversal speed relative bias.
   * rangings.
   *
   * @return     The bias.
   */
  ng_float_t get_transversal_speed_bias() const {
    return _transversal_speed_error.mean();
  }
  /**
   * @brief      Gets the transversal speed relative standard deviation.
   * rangings.
   *
   * @return     The standard deviation.
   */
  ng_float_t get_transversal_speed_std_dev() const {
    return _transversal_speed_error.stddev();
  }
  /**
   * @brief      Sets the angular speed relative bias.
   *
   * @param[in]  value     The desired value
   */
  void set_angular_speed_bias(ng_float_t value) {
    _angular_speed_error.param(
        Error::param_type{value, get_angular_speed_std_dev()});
  }
  /**
   * @brief      Sets the angular speed relative standard deviation.
   *
   * @param[in]  value     The desired (positive) value
   */
  void set_angular_speed_std_dev(ng_float_t value) {
    _angular_speed_error.param(Error::param_type{
        get_angular_speed_bias(), std::max<ng_float_t>(0, value)});
  }
  /**
   * @brief      Gets the angular speed relative bias.
   * rangings.
   *
   * @return     The bias.
   */
  ng_float_t get_angular_speed_bias() const {
    return _angular_speed_error.mean();
  }
  /**
   * @brief      Gets the angular speed relative standard deviation.
   * rangings.
   *
   * @return     The standard deviation.
   */
  ng_float_t get_angular_speed_std_dev() const {
    return _angular_speed_error.stddev();
  }

  /**
   * @brief      Gets whether to update the behavior environment state.
   *
   * @return     True if it updates the behavior environment state.
   */
  bool get_update_sensing_state() const { return _update_sensing_state; }
  /**
   * @brief      Sets whether to update the behavior environment state
   *
   * @param[in]  value     The desired value
   */
  void set_update_sensing_state(bool value) { _update_sensing_state = value; }

  /**
   * @brief      Gets whether to update the behavior ego state.
   *
   * @return     True if it updates the behavior ego state.
   */
  bool get_update_ego_state() const { return _update_ego_state; }
  /**
   * @brief      Sets whether to update the behavior ego state.
   *
   * @param[in]  value     The desired value
   */
  void set_update_ego_state(bool value) { _update_ego_state = value; }

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) override;

  /**
   * @brief      Gets the current pose.
   *
   * @return     The pose.
   */
  Pose2 get_pose() const { return _pose; }

  /**
   * @brief      Gets the current twist.
   *
   * @return     The twist.
   */
  Twist2 get_twist() const { return _twist; }

  /**
   * @private
   */
  Description get_description() const override {
    if (!get_update_sensing_state()) {
      return {};
    }
    return {{get_field_name("pose"),
             core::BufferDescription::make<ng_float_t>(
                 {3}, std::numeric_limits<ng_float_t>::lowest(),
                 std::numeric_limits<ng_float_t>::max())},
            {get_field_name("twist"),
             core::BufferDescription::make<ng_float_t>(
                 {3}, std::numeric_limits<ng_float_t>::lowest(),
                 std::numeric_limits<ng_float_t>::max())}};
  }

private:
  core::Pose2 _pose;
  core::Twist2 _twist;
  ng_float_t _time;
  bool _update_sensing_state;
  bool _update_ego_state;
  Error _longitudinal_speed_error;
  Error _transversal_speed_error;
  Error _angular_speed_error;
};

} // namespace navground::sim

#endif /* end of include guard:                                                \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_ODOMETRY_H_ */
