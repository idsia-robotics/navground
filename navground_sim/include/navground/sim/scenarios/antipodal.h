/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_ANTIPODAL_H
#define NAVGROUND_SIM_SCENARIOS_ANTIPODAL_H

#include <memory>

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
 */
struct NAVGROUND_SIM_EXPORT AntipodalScenario : public Scenario {
  /**
   * The default circle radius
   */
  static constexpr float default_radius = 1.0f;
  /**
   * The default goal tolerance
   */
  static constexpr float default_tolerance = 0.1f;
  /**
   * The default position noise
   */
  static constexpr float default_position_noise = 0.0f;
  /**
   * The default orientation noise
   */
  static constexpr float default_orientation_noise = 0.0f;
  /**
   * The default shuffle
   */
  static constexpr float default_shuffle = false;
  /**
   * @brief      Constructs a new instance.
   */
  AntipodalScenario(float radius = default_radius,
                    float tolerance = default_tolerance,
                    float position_noise = default_position_noise,
                    float orientation_noise = default_orientation_noise,
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
  float get_radius() const { return radius; }
  /**
   * @brief      Sets the circle radius.
   *
   * @param[in]  value  The desired value
   */
  void set_radius(float value) { radius = std::max(value, 0.0f); }
  /**
   * @brief      Gets the goal tolerance.
   *
   * @return     The tolerance.
   */
  float get_tolerance() const { return tolerance; }
  /**
   * @brief      Sets the goal tolerance.
   *
   * @param[in]  value  The desired value
   */
  void set_tolerance(float value) { tolerance = std::max(value, 0.0f); }

  /**
   * @brief      Gets the position_noise.
   *
   * @return     The position_noise.
   */
  float get_position_noise() const { return position_noise; }
  /**
   * @brief      Sets the position noise.
   *
   * @param[in]  value  The desired value
   */
  void set_position_noise(float value) {
    position_noise = std::max(value, 0.0f);
  }

  /**
   * @brief      Gets the orientation.
   *
   * @return     The orientation noise.
   */
  float get_orientation_noise() const { return orientation_noise; }
  /**
   * @brief      Sets the position_noise.
   *
   * @param[in]  value  The desired value
   */
  void set_orientation_noise(float value) {
    orientation_noise = std::max(value, 0.0f);
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
  void init_world(World *world, std::optional<int> seed = std::nullopt) override;

  /**
   * @private
   */
  const Properties &get_properties() const override { return properties; };

  /**
   * @private
   */
  inline const static std::map<std::string, Property> properties = Properties{
      {"radius",
       make_property<float, AntipodalScenario>(
           &AntipodalScenario::get_radius, &AntipodalScenario::set_radius,
           default_radius, "Radius of the circle")},
      {"tolerance",
       make_property<float, AntipodalScenario>(
           &AntipodalScenario::get_tolerance, &AntipodalScenario::set_tolerance,
           default_tolerance, "Goal tolerance")},
      {"position_noise",
       make_property<float, AntipodalScenario>(
           &AntipodalScenario::get_position_noise,
           &AntipodalScenario::set_position_noise, default_position_noise,
           "Noise added to the initial position")},
      {"orientation_noise",
       make_property<float, AntipodalScenario>(
           &AntipodalScenario::get_orientation_noise,
           &AntipodalScenario::set_orientation_noise, default_orientation_noise,
           "Noise added to the initial orientation")},
      {"shuffle",
       make_property<bool, AntipodalScenario>(
           &AntipodalScenario::get_shuffle, &AntipodalScenario::set_shuffle,
           default_shuffle,
           "Whether to shuffle the agents before initializing them")}};

  /**
   * @private
   */
  std::string get_type() const override { return type; }

  float radius;
  float tolerance;
  float position_noise;
  float orientation_noise;
  bool shuffle;

 private:
  inline const static std::string type =
      register_type<AntipodalScenario>("Antipodal");
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_ANTIPODAL_H */
