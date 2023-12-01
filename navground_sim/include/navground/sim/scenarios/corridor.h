/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_CORRIDOR_H
#define NAVGROUND_SIM_SCENARIOS_CORRIDOR_H

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "navground/sim/scenario.h"
#include "navground_sim_export.h"

using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

/**
 * @brief      A scenario where agents travel along an infinite 
 * corridor in opposite directions. Agents are initialize at 
 * non-overlapping random poses.
 * 
 * *Registered properties*: 
 * 
 *   - `width` (float, \ref get_width)
 *   
 *   - `length` (float, \ref get_length)
 *   
 *   - `agent_margin` (float, \ref get_agent_margin)
 *   
 *   - `add_safety_to_agent_margin` (bool, \ref get_add_safety_to_agent_margin)
 */
struct NAVGROUND_SIM_EXPORT CorridorScenario : public Scenario {
 public:
  // corridor width
  float width;
  inline static float default_width = 1.0f;
  // corridor length
  float length;
  inline static float default_length = 10.0f;
  // initial minimal distance between agents
  float agent_margin;
  inline static float default_agent_margin = 0.1f;
  // whether to add the safety margin to the agent margin;
  bool add_safety_to_agent_margin;
  inline static bool default_add_safety_to_agent_margin = true;

  CorridorScenario(
      float width = default_width, float length = default_length,
      float agent_margin = default_agent_margin,
      bool add_safety_to_agent_margin = default_add_safety_to_agent_margin)
      : Scenario(),
        width(width),
        length(length),
        agent_margin(agent_margin),
        add_safety_to_agent_margin(add_safety_to_agent_margin) {}

  /**
   * @brief      Gets the width of the corridor.
   *
   * @return     The width.
   */
  float get_width() const { return width; }

  /**
   * @brief      Sets the width of the corridor.
   *
   * @param[in]  value  The desired width (positive)
   */
  void set_width(float value) { width = std::max(0.0f, value); }

  /**
   * @brief      Gets the length of the simulated portion of corridor. 
   * Agents experience an infinite corridor, as it wraps around.
   *
   * @return     The length.
   */
  float get_length() const { return length; }

  /**
   * @brief      Sets the length of the simulated portion of corridor.
   * Agents experience an infinite corridor, as it wraps around.
   *
   * @param[in]  value  The desired length (positive).
   */
  void set_length(float value) { length = std::max(0.0f, value); }

  /**
   * @brief      Gets the initial minimal distance between agents.
   *
   * @return     The initial minimal distance between agents .
   */
  float get_agent_margin() const { return agent_margin; }

  /**
   * @brief      Sets the initial minimal distance between agents.
   *
   * @param[in]  value  The desired margin (positive)
   */
  void set_agent_margin(float value) { agent_margin = std::max(0.0f, value); }

  /**
   * @brief      Whenever the agent's safety margin should be considered in addition 
   * to \ref get_agent_margin when initializing the agents' poses.
   *
   * @return     The add safety to agent margin.
   */
  bool get_add_safety_to_agent_margin() const {
    return add_safety_to_agent_margin;
  }

  /**
   * @brief      Sets whenever the agent's safety margin should be considered in addition 
   * to \ref get_agent_margin when initializing the agents' poses.
   *
   * @param[in]  value  Whenever to consider the safety margin or not.
   */
  void set_add_safety_to_agent_margin(bool value) {
    add_safety_to_agent_margin = value;
  }

  /**
   * @private
   */
  void init_world(World *world, std::optional<int> seed = std::nullopt) override;

  /**
   * @private
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  inline static const std::map<std::string, Property> properties = {
    {"width", make_property<float, CorridorScenario>(
                  &CorridorScenario::get_width, &CorridorScenario::set_width,
                  default_width, "Corridor width")},
    {"length", make_property<float, CorridorScenario>(
                   &CorridorScenario::get_length, &CorridorScenario::set_length,
                   default_length, "Corridor length")},
    {"agent_margin", make_property<float, CorridorScenario>(
                         &CorridorScenario::get_agent_margin,
                         &CorridorScenario::set_agent_margin, 0.1f,
                         "initial minimal distance between agents")},
    {"add_safety_to_agent_margin",
     make_property<bool, CorridorScenario>(
         &CorridorScenario::get_add_safety_to_agent_margin,
         &CorridorScenario::set_add_safety_to_agent_margin,
         default_add_safety_to_agent_margin,
         "Whether to add the safety margin to the agent margin")}};

  /**
   * @private
   */
  std::string get_type() const override { return type; }
  // const static std::string type;

 private:
  inline const static std::string type =
      register_type<CorridorScenario>("Corridor");
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_CORRIDOR_H */
