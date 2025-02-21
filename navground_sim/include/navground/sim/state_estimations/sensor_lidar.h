/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_LIDAR_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_LIDAR_H_

#include <random>
#include <vector>

#include "navground/core/collision_computation.h"
#include "navground/core/common.h"
#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"

namespace navground::sim {

/**
 * @brief      A distance scanner.
 *
 * *Registered properties*:
 *
 *   - `range` (float, \ref get_range)
 *
 *   - `start_angle` (float, \ref get_start_angle)
 *
 *   - `field_of_view` (float, \ref get_field_of_view)
 *
 *   - `resolution` (float, \ref get_resolution)
 *
 *   - `position` (Vector2, \ref get_position)
 *
 *   - `error_bias` (float, \ref get_error_bias)
 *
 *   - `error_std_dev` (float, \ref get_error_std_dev)
 *
 */
struct NAVGROUND_SIM_EXPORT LidarStateEstimation : public Sensor {
  static const std::string type;

  using Error = std::normal_distribution<ng_float_t>;
  /**
   * The default range
   */
  inline static const ng_float_t default_range = 1;
  /**
   * The default start angle [radians]
   */
  inline static const ng_float_t default_start_angle =
      -static_cast<ng_float_t>(core::PI);
  /**
   * The default field of view [radians]
   */
  inline static const ng_float_t default_field_of_view = core::TWO_PI;
  /**
   * The default resolution
   */
  inline static const int default_resolution = 100;
  /**
   * The default error bias
   */
  inline static const ng_float_t default_error_bias = 0;
  /**
   * The default error standard deviation
   */
  inline static const ng_float_t default_error_std_dev = 0;
  /**
   * The name of the buffer set by the sensor
   */
  inline static const std::string field_name = "range";

  struct Data {
    const std::valarray<ng_float_t> &ranges;
    ng_float_t start_angle;
    ng_float_t fov;
    ng_float_t max_range;
    ng_float_t get_angular_increment() const {
      return compute_angular_increment(fov, ranges.size());
    }
  };
  // TODO(Jerome): are we copying ranges?
  static std::optional<Data> read_measure(core::SensingState &state,
                                          const std::string &name = "") {
    std::string prefix = name.empty() ? "" : name + "/";
    auto buffer = state.get_buffer(prefix + "range");
    if (!buffer)
      return std::nullopt;
    const auto ranges = buffer->get_data<ng_float_t>();
    if (!ranges)
      return std::nullopt;
    // data.ranges = *ranges;
    buffer = state.get_buffer(prefix + "start_angle");
    if (!buffer)
      return std::nullopt;
    const auto start_angle = buffer->get_data<ng_float_t>();
    if (!start_angle)
      return std::nullopt;
    // data.start_angle = *start_angle;
    buffer = state.get_buffer(prefix + "fov");
    if (!buffer)
      return std::nullopt;
    const auto fov = buffer->get_data<ng_float_t>();
    if (!fov)
      return std::nullopt;
    // data.fov = *fov;
    buffer = state.get_buffer(prefix + "max_range");
    if (!buffer)
      return std::nullopt;
    const auto max_range = buffer->get_data<ng_float_t>();
    if (!max_range)
      return std::nullopt;
    // data.max_range = *max_range;
    return Data{*ranges, (*start_angle)[0], (*fov)[0], (*max_range)[0]};
  }

  static ng_float_t compute_angular_increment(ng_float_t field_of_view,
                                              ng_float_t resolution) {
    if (resolution > 1) {
      return field_of_view / (resolution - 1);
    }
    return 0;
  }

  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range  The maximal range of the sensor
   * @param[in]  start_angle  The starting angle
   * @param[in]  field_of_view  The field of view
   * @param[in]  resolution  The number of ranging measurements per scan
   * @param[in]  position  The position of the sensor with respect
   *                       to the agent origin.
   * @param[in]  error_bias  The systematic error for all rangings.
   * @param[in]  error_std_dev  The rangings error standard deviation.
   * @param[in]  name  The name to use as a prefix
   */
  explicit LidarStateEstimation(
      ng_float_t range = default_range,
      ng_float_t start_angle = default_start_angle,
      ng_float_t field_of_view = default_field_of_view,
      unsigned resolution = default_resolution,
      const Vector2 &position = Vector2::Zero(),
      ng_float_t error_bias = default_error_bias,
      ng_float_t error_std_dev = default_error_std_dev,
      const std::string &name = "")
      : Sensor(name), _range(range), _start_angle(start_angle),
        _field_of_view(field_of_view), _resolution(resolution),
        _position(position), _error{error_bias, error_std_dev}, _cc() {}

  virtual ~LidarStateEstimation() = default;

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_range(ng_float_t value) { _range = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  ng_float_t get_range() const { return _range; }

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_start_angle(ng_float_t value) { _start_angle = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  ng_float_t get_start_angle() const { return _start_angle; }

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_field_of_view(ng_float_t value) { _field_of_view = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  ng_float_t get_field_of_view() const { return _field_of_view; }

  /**
   * @brief      Sets the error bias, i.e., the systematic error for all
   * rangings.
   *
   * @param[in]  value     The new value
   */
  void set_error_bias(ng_float_t value) {
    _error.param(Error::param_type{value, _error.stddev()});
  }
  /**
   * @brief      Gets the error bias, i.e., the systematic error for all
   * rangings.
   *
   * @return     The error.
   */
  ng_float_t get_error_bias() const { return _error.mean(); }
  /**
   * @brief      Sets the rangings error standard deviation.
   *
   * @param[in]  value     The positive value
   */
  void set_error_std_dev(ng_float_t value) {
    _error.param(
        Error::param_type{_error.mean(), std::max<ng_float_t>(0, value)});
  }
  /**
   * @brief      Gets the rangings error standard deviation.
   *
   * @return     The error.
   */
  ng_float_t get_error_std_dev() const { return _error.stddev(); }

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_resolution(int value) { _resolution = value; }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  int get_resolution() const { return _resolution; }

  /**
   * @brief      Sets the position of the sensor with respect
   *             to the agent origin.
   *
   * @param[in]  value     The new value
   */
  void set_position(const core::Vector2 &value) { _position = value; }

  /**
   * @brief      Gets the position of the sensor with respect
   *             to the agent origin.
   *
   * @return     The position.
   */
  core::Vector2 get_position() const { return _position; }

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) override;

  /**
   * @private
   */
  Description get_description() const override {
    return {
        {get_field_name(field_name),
         core::BufferDescription::make<ng_float_t>({_resolution}, 0.0, _range)},
        {get_field_name("start_angle"),
         core::BufferDescription::make<ng_float_t>({1}, -core::TWO_PI,
                                                   core::TWO_PI)},
        {get_field_name("fov"),
         core::BufferDescription::make<ng_float_t>({1}, 0.0, core::TWO_PI)},
        {get_field_name("max_range"),
         core::BufferDescription::make<ng_float_t>({1}, 0.0, 10.0)}};
  }

  /**
   * @brief      Gets the angular increment between rays.
   *
   * @return     The angular increment.
   */
  ng_float_t get_angular_increment() const;

  /**
   * @brief      Gets the ray angles.
   *
   * @return     The angles.
   */
  std::valarray<ng_float_t> get_angles() const;

  /**
   * @brief      Compute ranges assuming the sensor is attached to an agent
   *
   * @param      agent  The agent
   * @param      world  The world
   *
   * @return     The array of ranges
   */
  std::valarray<ng_float_t> measure_ranges(Agent *agent, World *world);

  /**
   * @brief      Reads the ranges stored in a sensing state
   *
   * @param      state  The state
   *
   * @return     The array of ranges
   */
  const std::valarray<ng_float_t> &read_ranges(core::SensingState &state) const;

  bool has_error() const {
    return get_error_bias() != 0 || get_error_std_dev() != 0;
  }

private:
  // std::valarray<ng_float_t> sample_error(World * world);

  ng_float_t _range;
  ng_float_t _start_angle;
  ng_float_t _field_of_view;
  int _resolution;
  core::Vector2 _position;
  Error _error;
  core::CollisionComputation _cc;
};

} // namespace navground::sim

#endif /* end of include guard:                                                \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_LIDAR_H_ */
