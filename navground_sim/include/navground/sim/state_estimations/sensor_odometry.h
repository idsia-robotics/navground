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
 *   - `longitudinal_speed_error` (float, \ref get_longitudinal_speed_error)
 *
 *   - `transversal_speed_error` (float, \ref get_transversal_speed_error)
 *
 *   - `angular_speed_error` (float, \ref get_angular_speed_error)
 *
 */
struct NAVGROUND_SIM_EXPORT OdometryStateEstimation : public Sensor {

  using Error = std::normal_distribution<ng_float_t>;
  /**
   * The default longitudinal speed standard deviation.
   */
  inline static const ng_float_t default_longitudinal_speed_error = 0;
  /**
   * The default transversal speed standard deviation.
   */
  inline static const ng_float_t default_transversal_speed_error = 0;
  /**
   * The default angular speed standard deviation.
   */
  inline static const ng_float_t default_angular_speed_error = 0;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  longitudinal_speed_error  The longitudinal speed standard
   * deviation
   * @param[in]  transversal_speed_error  The transversal speed standard
   * deviation
   * @param[in]  angular_speed_error  The angular speed standard deviation
   * @param[in]  name  The name to use as a prefix
   */
  explicit OdometryStateEstimation(
      ng_float_t longitudinal_speed_error = default_longitudinal_speed_error,
      ng_float_t transversal_speed_error = default_transversal_speed_error,
      ng_float_t angular_speed_error = default_angular_speed_error,
      const std::string &name = "")
      : Sensor(name), _pose(), _time(0),
        _longitudinal_speed_error{0, longitudinal_speed_error},
        _transversal_speed_error{0, transversal_speed_error},
        _angular_speed_error{0, angular_speed_error} {}

  virtual ~OdometryStateEstimation() = default;

  /**
   * @brief      Sets the longitudinal speed standard deviation.
   *
   * @param[in]  value     The desired value
   */
  void set_longitudinal_speed_error(ng_float_t value) {
    _longitudinal_speed_error.param(
        Error::param_type{0, std::max<ng_float_t>(0, value)});
  }
  /**
   * @brief      Gets the longitudinal speed standard deviation.
   * rangings.
   *
   * @return     The standard deviation.
   */
  ng_float_t get_longitudinal_speed_error() const {
    return _longitudinal_speed_error.stddev();
  }
  /**
   * @brief      Gets the transversal speed standard deviation.
   * rangings.
   *
   * @return     The standard deviation.
   */
  ng_float_t get_transversal_speed_error() const {
    return _transversal_speed_error.stddev();
  }
  /**
   * @brief      Sets the transversal speed standard deviation.
   *
   * @param[in]  value     The desired value
   */
  void set_transversal_speed_error(ng_float_t value) {
    _transversal_speed_error.param(
        Error::param_type{0, std::max<ng_float_t>(0, value)});
  }
  /**
   * @brief      Gets the longitudinal speed standard deviation.
   * rangings.
   *
   * @return     The standard deviation.
   */
  ng_float_t get_angular_speed_error() const {
    return _angular_speed_error.stddev();
  }
  /**
   * @brief      Sets the angular speed standard deviation.
   *
   * @param[in]  value     The desired value
   */
  void set_angular_speed_error(ng_float_t value) {
    _angular_speed_error.param(
        Error::param_type{0, std::max<ng_float_t>(0, value)});
  }
  /**
   * @private
   */
  virtual const core::Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static const std::map<std::string, core::Property> properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) override;

  /**
   * @private
   */
  Description get_description() const override {
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
  ng_float_t _time;
  Error _longitudinal_speed_error;
  Error _transversal_speed_error;
  Error _angular_speed_error;
  const static std::string type;
};

} // namespace navground::sim

#endif /* end of include guard:                                                \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_ODOMETRY_H_ */
