/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_
#define NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_

#include <vector>

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
   * @brief      Constructs a new instance.
   *
   * @param[in]  range_   The range of view
   * @param[in]  number_  Number of discs
   */
  explicit DiscsStateEstimation(ng_float_t range_ = default_range,
                                unsigned number_ = default_number)
      : Sensor(), range(range_), number(number_) {}

  virtual ~DiscsStateEstimation() = default;

  /**
   * @brief      Sets the range of view.
   *
   * @param[in]  value     The new value
   */
  void set_range(ng_float_t value) { range = value; }

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
  void set_number(int value) { number = value; }

  /**
   * @brief      Gets the number of discs.
   *
   * @return     The number of discs.
   */
  int get_number() const { return number; }

  /**
   * @private
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline std::map<std::string, Property> properties =
      Properties{
          {"range", make_property<ng_float_t, DiscsStateEstimation>(
                        &DiscsStateEstimation::get_range,
                        &DiscsStateEstimation::set_range, default_range,
                        "Maximal range")},
          {"number",
           make_property<int, DiscsStateEstimation>(
               &DiscsStateEstimation::get_number,
               &DiscsStateEstimation::set_number, default_number, "Number")},
      } +
      StateEstimation::properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  /**
   * @private
   */
  virtual void update(Agent *agent, World *world,
                      EnvironmentState *state) const override;

  // {(x, y, v_x, v_y, radius)}
  // TODO(Jerome): max_radius / max_velocity
  Description get_description() const override {
    return {{"radius", BufferDescription::make<ng_float_t>({number}, 0.0, 1.0)},
            {"position",
             BufferDescription::make<ng_float_t>({number, 2}, 0.0, range)},
            {"velocity",
             BufferDescription::make<ng_float_t>({number, 2}, 0.0, 1.0)}};
  }

 private:
  ng_float_t range;
  unsigned number;
  inline const static std::string type =
      register_type<DiscsStateEstimation>("Discs");
};

}  // namespace navground::sim

#endif /* end of include guard: \
          NAVGROUND_SIM_STATE_ESTIMATIONS_SENSOR_DISCS_H_ */
