/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_

#include <algorithm>
#include <vector>

#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"

namespace navground::sim {

/**
 * @brief      Perceive a fixed number of nearest neighbors and obstacles
 *
 * Empty places are filled with zeros
 *
 * *Registered properties*:
 *
 *   - `range` (float, \ref get_range)
 *
 *   - `number` (int, \ref get_number)
 *
 *   - `max_radius` (int, \ref get_max_radius)
 *
 *   - `max_speed` (int, \ref get_max_speed)
 *
 *   - `include_valid` (bool, \ref get_include_valid)
 *
 *   - `use_nearest_point` (bool, \ref get_use_nearest_point)
 *
 *   - `max_id` (int, \ref get_max_id)
 *
 *   - `include_x` (int, \ref get_include_x)
 *
 *   - `include_y` (int, \ref get_include_y)
 *
 */
struct NAVGROUND_SIM_EXPORT DiscsStateEstimation : public Sensor {
  static const std::string type;

  /**
   * The default range
   */
  inline static const ng_float_t default_range = 1;
  /**
   * The default start angle
   */
  inline static const ng_float_t default_number = 1;
  /**
   * The default maximal neighbor radius (zero means that it will include radii)
   */
  inline static const ng_float_t default_max_radius = 0;
  /**
   * The default maximal neighbor speed (zero means that it will not include
   * velocities)
   */
  inline static const ng_float_t default_max_speed = 0;
  /**
   * The default for whether to include validity
   */
  inline static const bool default_include_valid = true;
  /**
   * The default for whether to use nearest point as position
   */
  inline static const bool default_use_nearest_point = true;
  /**
   * The default maximal id (zero means that it will not include ids)
   */
  inline static const unsigned default_max_id = 0;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range   The range of view
   * @param[in]  number  Number of discs
   * @param[in]  max_radius  Maximal neighbor radius
   * @param[in]  max_speed  Maximal neighbor speed
   * @param[in]  include_valid  Whether to include validity
   * @param[in]  use_nearest_point  Whether to use nearest point as position
   * @param[in]  max_id  Maximal neighbor id
   * @param[in]  include_x  Whether to include the x-coordinate
   * @param[in]  include_y  Whether to include the y-coordinate
   * @param[in]  name     The name to use as a prefix
   */
  explicit DiscsStateEstimation(
      ng_float_t range = default_range, unsigned number = default_number,
      ng_float_t max_radius = default_max_radius,
      ng_float_t max_speed = default_max_speed,
      bool include_valid = default_include_valid,
      bool use_nearest_point = default_use_nearest_point,
      unsigned max_id = default_max_id, bool include_x = true,
      bool include_y = true, const std::string &name = "")
      : Sensor(name), _range(range), _number(number), _max_radius(max_radius),
        _max_speed(max_speed), _include_valid(include_valid),
        _use_nearest_point(use_nearest_point), _max_id(max_id),
        _include_x(include_x), _include_y(include_y) {}

  virtual ~DiscsStateEstimation() = default;

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_range(ng_float_t value) { _range = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  ng_float_t get_range() const { return _range; }

  /**
   * @brief      Sets the number of discs.
   *
   * @param[in]  value     The new value
   */
  void set_number(int value) { _number = std::max(0, value); }

  /**
   * @brief      Gets the number of discs.
   *
   * @return     The number of discs.
   */
  int get_number() const { return _number; }

  /**
   * @brief      Sets the maximal neighbor radius.
   *
   * @param[in]  value     The new value
   */
  void set_max_radius(ng_float_t value) {
    _max_radius = std::max<ng_float_t>(0, value);
  }

  /**
   * @brief      Gets the maximal neighbor radius.
   *
   * @return     The maximal radius.
   */
  ng_float_t get_max_radius() const { return _max_radius; }

  /**
   * @brief      Sets the maximal neighbor speed.
   *
   * @param[in]  value     The new value
   */
  void set_max_speed(ng_float_t value) {
    _max_speed = std::max<ng_float_t>(0, value);
  }

  /**
   * @brief      Gets the maximal neighbor speed.
   *
   * @return     The maximal speed.
   */
  ng_float_t get_max_speed() const { return _max_speed; }

  /**
   * @brief      Sets whether to include validity.
   *
   * @param[in]  value     The new value
   */
  void set_include_valid(bool value) { _include_valid = value; }

  /**
   * @brief      Gets whether to include validity.
   *
   * @return     Whether to include validity.
   */
  bool get_include_valid() const { return _include_valid; }
  /**
   * @brief      Sets whether to use the nearest or the center as position
   *
   * @param[in]  value     The new value
   */
  void set_use_nearest_point(bool value) { _use_nearest_point = value; }

  /**
   * @brief      Gets whether to use the nearest or the center as position
   *
   * @return     Whether to use nearest point as position.
   */
  bool get_use_nearest_point() const { return _use_nearest_point; }

  /**
   * @brief      Sets the maximal possible id.
   *
   *             Set to zero to not include ids.
   *
   * @param[in]  value  The new value
   */
  void set_max_id(int value) { _max_id = std::max(0, value); }

  /**
   * @brief      Gets the maximal possible id.
   *
   * @return     The maximal id.
   */
  int get_max_id() const { return _max_id; }

  /**
   * @brief      Sets whether to include the x-coordinate.
   *
   * @param[in]  value  True to include it.
   */
  void set_include_x(bool value) { _include_x = value; }
  /**
   * @brief      Gets whether to include the x-coordinate.
   *
   * @return     True when including it.
   */
  bool get_include_x() const { return _include_x; }
  /**
   * @brief      Sets whether to include the y-coordinate.
   *
   * @param[in]  value  True to include it.
   */
  void set_include_y(bool value) { _include_y = value; }
  /**
   * @brief      Gets whether to include the y-coordinate.
   *
   * @return     True when including it.
   */
  bool get_include_y() const { return _include_y; }

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) override;

  // {(relative x, relative y, v_x, v_y, radius)}
  // DONE(Jerome): max_radius / max_velocity

  size_t get_dimensions() const {
    return size_t(_include_x) + size_t(_include_y);
  }

  /**
   * @private
   */
  Description get_description() const override {
    Description desc;
    const long dim = get_dimensions();
    if (!_number)
      return desc;
    if (include_radius()) {
      desc.emplace(
          get_field_name("radius"),
          core::BufferDescription::make<ng_float_t>({_number}, 0, _max_radius));
    }
    if (include_velocity()) {
      desc.emplace(get_field_name("velocity"),
                   core::BufferDescription::make<ng_float_t>(
                       {_number, dim}, -_max_speed, _max_speed));
    }
    if (include_position()) {
      desc.emplace(get_field_name("position"),
                   core::BufferDescription::make<ng_float_t>({_number, dim},
                                                             -_range, _range));
    }
    if (get_include_valid()) {
      desc.emplace(get_field_name("valid"),
                   core::BufferDescription::make<uint8_t>({_number}, 0, 1));
    }
    if (include_id()) {
      desc.emplace(
          get_field_name("id"),
          core::BufferDescription::make<unsigned>({_number}, 0, _max_id, true));
    }
    return desc;
  }

private:
  ng_float_t _range;
  unsigned _number;
  ng_float_t _max_radius;
  ng_float_t _max_speed;
  bool _include_valid;
  bool _use_nearest_point;
  unsigned _max_id;
  bool _include_x;
  bool _include_y;

  bool include_velocity() const {
    return _max_speed > 0 && (_include_x || _include_y);
  }
  bool include_radius() const { return _max_radius > 0; }
  bool include_position() const {
    return _range > 0 && (_include_x || _include_y);
  }
  bool include_id() const { return _max_id > 0; }
};

} // namespace navground::sim

#endif /* end of include guard:                                                \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_ */
