/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_BOUNDARY_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_BOUNDARY_H_

#include <algorithm>
#include <cmath>
#include <limits>

#include "navground/core/types.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"
#include "navground/sim/export.h"

using navground::core::BufferDescription;
using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

/**
 * @brief      Returns the distance to a rectangular boundary
 *
 * *Registered properties*:
 *
 *   - `range` (float, \ref get_range)
 *   - `min_x` (float, \ref get_min_x)
 *   - `max_x` (float, \ref get_max_x)
 *   - `min_y` (float, \ref get_min_y)
 *   - `max_y` (float, \ref get_max_y)
 *
 */
struct NAVGROUND_SIM_EXPORT BoundarySensor : public Sensor {
  /**
   * The default range
   */
  inline static const ng_float_t default_range = 1;
  inline static const ng_float_t low = -std::numeric_limits<ng_float_t>::infinity();
  inline static const ng_float_t high = std::numeric_limits<ng_float_t>::infinity();
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range_   The range of view
   * @param[in]  min_x
   * @param[in]  max_x
   * @param[in]  min_y
   * @param[in]  max_y
   */
  explicit BoundarySensor(ng_float_t range = default_range,
                          ng_float_t min_x = low, ng_float_t max_x = high,
                          ng_float_t min_y = low, ng_float_t max_y = high)
      : Sensor(),
        _range(range),
        _min_x(min_x),
        _max_x(max_x),
        _min_y(min_y),
        _max_y(max_y) {}

  virtual ~BoundarySensor() = default;

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

  void set_min_x(ng_float_t value) { _min_x = value; }
  ng_float_t get_min_x() const { return _min_x; }
  void set_max_x(ng_float_t value) { _max_x = value; }
  ng_float_t get_max_x() const { return _max_x; }
  void set_min_y(ng_float_t value) { _min_y = value; }
  ng_float_t get_min_y() const { return _min_y; }
  void set_max_y(ng_float_t value) { _max_y = value; }
  ng_float_t get_max_y() const { return _max_y; }
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

  Description get_description() const override {
    Description desc;
    unsigned n = 0;
    if (std::isfinite(_min_x)) n++;
    if (std::isfinite(_max_x)) n++;
    if (std::isfinite(_min_y)) n++;
    if (std::isfinite(_max_y)) n++;
    desc.emplace("boundary_distance",
                 BufferDescription::make<ng_float_t>({n}, 0.0, _range));
    return desc;
  }

 private:
  ng_float_t _range;
  ng_float_t _min_x, _max_x, _min_y, _max_y;
  const static std::string type;
};

}  // namespace navground::sim

#endif /* end of include guard: \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_BOUNDARY_H_ */
