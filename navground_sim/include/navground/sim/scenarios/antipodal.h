/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_ANTIPODAL_H
#define NAVGROUND_SIM_SCENARIOS_ANTIPODAL_H

#include <memory>

#include "navground/core/types.h"
#include "navground/sim/scenario.h"
#include "navground/sim/tasks/waypoints.h"
#include "navground/sim/world.h"
#include "navground_sim_export.h"

using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

/**
 * @brief      A scenario that place the agents around a circle at regular
 * intervals and task them to reach the opposite ("antipode") side.
 *
 * *Registered properties*:
 *
 *   - `radius` (float, \ref get_radius)
 *
 *   - `tolerance` (float, \ref get_tolerance)
 *
 *   - `position_noise` (float, \ref get_position_noise)
 *
 *   - `orientation_noise` (float, \ref get_orientation_noise)
 *
 *   - `shuffle` (bool, \ref get_shuffle)
 */
struct NAVGROUND_SIM_EXPORT AntipodalScenario : public Scenario {
  /**
   * The default circle radius
   */
  static constexpr ng_float_t default_radius = 1;
  /**
   * The default goal tolerance
   */
  static constexpr ng_float_t default_tolerance = static_cast<ng_float_t>(0.1);
  /**
   * The default position noise
   */
  static constexpr ng_float_t default_position_noise = 0;
  /**
   * The default orientation noise
   */
  static constexpr ng_float_t default_orientation_noise = 0;
  /**
   * The default shuffle
   */
  static constexpr ng_float_t default_shuffle = false;
  /**
   * @brief      Constructs a new instance.
   */
  AntipodalScenario(ng_float_t radius = default_radius,
                    ng_float_t tolerance = default_tolerance,
                    ng_float_t position_noise = default_position_noise,
                    ng_float_t orientation_noise = default_orientation_noise,
                    bool shuffle = default_shuffle)
      : Scenario(),
        radius(radius),
        tolerance(tolerance),
        position_noise(default_position_noise),
        orientation_noise(default_orientation_noise),
        shuffle(default_shuffle){};

  /**
   * @brief      Gets the circle radius.
   *
   * @return     The radius.
   */
  ng_float_t get_radius() const { return radius; }
  /**
   * @brief      Sets the circle radius.
   *
   * @param[in]  value  The desired value
   */
  void set_radius(ng_float_t value) { radius = std::max<ng_float_t>(value, 0); }
  /**
   * @brief      Gets the goal tolerance.
   *
   * @return     The tolerance.
   */
  ng_float_t get_tolerance() const { return tolerance; }
  /**
   * @brief      Sets the goal tolerance.
   *
   * @param[in]  value  The desired value
   */
  void set_tolerance(ng_float_t value) {
    tolerance = std::max<ng_float_t>(value, 0);
  }

  /**
   * @brief      Gets the position_noise.
   *
   * @return     The position_noise.
   */
  ng_float_t get_position_noise() const { return position_noise; }
  /**
   * @brief      Sets the position noise.
   *
   * @param[in]  value  The desired value
   */
  void set_position_noise(ng_float_t value) {
    position_noise = std::max<ng_float_t>(value, 0);
  }

  /**
   * @brief      Gets the orientation.
   *
   * @return     The orientation noise.
   */
  ng_float_t get_orientation_noise() const { return orientation_noise; }
  /**
   * @brief      Sets the position_noise.
   *
   * @param[in]  value  The desired value
   */
  void set_orientation_noise(ng_float_t value) {
    orientation_noise = std::max<ng_float_t>(value, 0);
  }

  /**
   * @brief      Gets whether it should shuffle the agents before initializing
   * them.
   *
   * @return     True if it shuffles the agents.
   */
  bool get_shuffle() const { return shuffle; }
  /**
   * @brief      Sets whether it should shuffle the agents.
   *
   * @param[in]  value  The desired value
   */
  void set_shuffle(bool value) { shuffle = value; }

  /**
   * @private
   */
  void init_world(World *world,
                  std::optional<int> seed = std::nullopt) override;

  /**
   * @private
   */
  const Properties &get_properties() const override { return properties; };

  /**
   * @private
   */
  const static std::map<std::string, Property> properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  ng_float_t radius;
  ng_float_t tolerance;
  ng_float_t position_noise;
  ng_float_t orientation_noise;
  bool shuffle;

 private:
  const static std::string type;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_ANTIPODAL_H */
