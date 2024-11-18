/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_BOUNDARY_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_BOUNDARY_H_

#include <algorithm>
#include <cmath>
#include <limits>

#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/state_estimations/sensor.h"
#include "navground/sim/world.h"

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
  static const std::string type;

  /**
   * The default range
   */
  inline static const ng_float_t default_range = 1;
  /**
   * Default lower bound
   */
  inline static const ng_float_t low =
      -std::numeric_limits<ng_float_t>::infinity();
  /**
   * Default upper bound
   */
  inline static const ng_float_t high =
      std::numeric_limits<ng_float_t>::infinity();
  /**
   * @brief      Constructs a new instance.
   *
   * @param[in]  range    The range of view
   * @param[in]  min_x    The minimal x coordinate
   * @param[in]  max_x    The maximal x coordinate
   * @param[in]  min_y    The minimal y coordinate
   * @param[in]  max_y    The maximal y coordinate
   * @param[in]  name     The name to use as a prefix
   */
  explicit BoundarySensor(ng_float_t range = default_range,
                          ng_float_t min_x = low, ng_float_t max_x = high,
                          ng_float_t min_y = low, ng_float_t max_y = high,
                          const std::string &name = "")
      : Sensor(name), _range(range), _min_x(min_x), _max_x(max_x),
        _min_y(min_y), _max_y(max_y) {}

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

  /**
   * @brief      Sets the minimum x.
   *
   * @param[in]  value  The value
   */
  void set_min_x(ng_float_t value) { _min_x = value; }
  /**
   * @brief      Gets the minimum x.
   *
   * @return     The minimum x.
   */
  ng_float_t get_min_x() const { return _min_x; }
  /**
   * @brief      Sets the maximum x.
   *
   * @param[in]  value  The value
   */
  void set_max_x(ng_float_t value) { _max_x = value; }
  /**
   * @brief      Gets the maximum x.
   *
   * @return     The maximum x.
   */
  ng_float_t get_max_x() const { return _max_x; }
  /**
   * @brief      Sets the minimum y.
   *
   * @param[in]  value  The value
   */
  void set_min_y(ng_float_t value) { _min_y = value; }
  /**
   * @brief      Gets the minimum y.
   *
   * @return     The minimum y.
   */
  ng_float_t get_min_y() const { return _min_y; }
  /**
   * @brief      Sets the maximum y.
   *
   * @param[in]  value  The value
   */
  void set_max_y(ng_float_t value) { _max_y = value; }
  /**
   * @brief      Gets the maximum y.
   *
   * @return     The maximum y.
   */
  ng_float_t get_max_y() const { return _max_y; }
  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) override;

  /**
   * @private
   */
  Description get_description() const override {
    Description desc;
    unsigned n = 0;
    if (std::isfinite(_min_x))
      n++;
    if (std::isfinite(_max_x))
      n++;
    if (std::isfinite(_min_y))
      n++;
    if (std::isfinite(_max_y))
      n++;
    desc.emplace(get_field_name("boundary_distance"),
                 core::BufferDescription::make<ng_float_t>({n}, 0.0, _range));
    return desc;
  }

private:
  ng_float_t _range;
  ng_float_t _min_x, _max_x, _min_y, _max_y;
};

} // namespace navground::sim

#endif /* end of include guard:                                                \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_BOUNDARY_H_ */
