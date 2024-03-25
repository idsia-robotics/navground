/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_

#include <vector>
#include <algorithm>

#include "navground/core/types.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"
#include "navground_sim_export.h"

using navground::core::BufferDescription;
using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

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
 *   - `max_radius` (int, \ref get_number)
 *
 *   - `max_speed` (int, \ref get_number)
 *
 */
struct NAVGROUND_SIM_EXPORT DiscsStateEstimation : public Sensor {
  /**
   * The default range
   */
  inline static const ng_float_t default_range = 1;
  /**
   * The default start angle
   */
  inline static const ng_float_t default_number = 1;
  /**
   * The default maximal neighbor radius
   */
  inline static const ng_float_t default_max_radius = 1;
  /**
   * The default maximal neighbor speed
   */
  inline static const ng_float_t default_max_speed = 1;
  /**
   * The default maximal neighbor speed
   */
  inline static const bool default_include_valid = true;
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range_   The range of view
   * @param[in]  number_  Number of discs
   */
  explicit DiscsStateEstimation(ng_float_t range_ = default_range,
                                unsigned number_ = default_number,
                                ng_float_t max_radius_ = default_max_radius,
                                ng_float_t max_speed_ = default_max_speed,
                                bool include_valid_ = default_include_valid)
      : Sensor(),
        range(range_),
        number(number_),
        max_radius(max_radius_),
        max_speed(max_speed_),
        include_valid(include_valid_) {}

  virtual ~DiscsStateEstimation() = default;

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_range(ng_float_t value) { range = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the range of view.
   *
   * @return     The range of view.
   */
  ng_float_t get_range() const { return range; }

  /**
   * @brief      Sets the number of discs.
   *
   * @param[in]  value     The new value
   */
  void set_number(int value) { number = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the number of discs.
   *
   * @return     The number of discs.
   */
  int get_number() const { return number; }

  /**
   * @brief      Sets the maximal neighbor radius.
   *
   * @param[in]  value     The new value
   */
  void set_max_radius(ng_float_t value) { max_radius = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the maximal neighbor radius.
   *
   * @return     The maximal radius.
   */
  ng_float_t get_max_radius() const { return max_radius; }

  /**
   * @brief      Sets the maximal neighbor speed.
   *
   * @param[in]  value     The new value
   */
  void set_max_speed(ng_float_t value) { max_speed = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the maximal neighbor speed.
   *
   * @return     The maximal speed.
   */
  ng_float_t get_max_speed() const { return max_speed; }

  /**
   * @brief      Sets whether to include validity.
   *
   * @param[in]  value     The new value
   */
  void set_include_valid(bool value) { include_valid = value; }

  /**
   * @brief      Gets whether to include validity.
   *
   * @return     Whether to include validity.
   */
  bool get_include_valid() const { return include_valid; }
  /**
   * @private
   */
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

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) const override;

  // {(relative x, relative y, v_x, v_y, radius)}
  // DONE(Jerome): max_radius / max_velocity
  Description get_description() const override {
    Description desc;
    if (!number) return desc;
    if (include_radius()) {
      desc.emplace("radius", BufferDescription::make<ng_float_t>({number}, 0.0,
                                                                 max_radius));
    }
    if (include_velocity()) {
      desc.emplace("velocity", BufferDescription::make<ng_float_t>(
                                   {number, 2}, -max_speed, max_speed));
    }
    if (include_position()) {
      desc.emplace("position", BufferDescription::make<ng_float_t>(
                                   {number, 2}, -range, range));
    }
    if (get_include_valid()) {
      desc.emplace("valid", BufferDescription::make<uint8_t>({number}, 0, 1));
    }
    return desc;
  }

 private:
  ng_float_t range;
  unsigned number;
  ng_float_t max_radius;
  ng_float_t max_speed;
  bool include_valid;
  const static std::string type;

  bool include_velocity() const { return max_speed > 0; }
  bool include_radius() const { return max_radius > 0; }
  bool include_position() const { return range > 0; }
};

}  // namespace navground::sim

#endif /* end of include guard: \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_ */
