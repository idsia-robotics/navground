/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_CORRIDOR_H
#define NAVGROUND_SIM_SCENARIOS_CORRIDOR_H

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "navground/core/types.h"
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
  ng_float_t width;
  inline static ng_float_t default_width = 1.0;
  // corridor length
  ng_float_t length;
  inline static ng_float_t default_length = 10.0;
  // initial minimal distance between agents
  ng_float_t agent_margin;
  inline static ng_float_t default_agent_margin = 0.1;
  // whether to add the safety margin to the agent margin;
  bool add_safety_to_agent_margin;
  inline static bool default_add_safety_to_agent_margin = true;

  CorridorScenario(
      ng_float_t width = default_width, ng_float_t length = default_length,
      ng_float_t agent_margin = default_agent_margin,
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
  ng_float_t get_width() const { return width; }

  /**
   * @brief      Sets the width of the corridor.
   *
   * @param[in]  value  The desired width (positive)
   */
  void set_width(ng_float_t value) { width = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the length of the simulated portion of corridor.
   * Agents experience an infinite corridor, as it wraps around.
   *
   * @return     The length.
   */
  ng_float_t get_length() const { return length; }

  /**
   * @brief      Sets the length of the simulated portion of corridor.
   * Agents experience an infinite corridor, as it wraps around.
   *
   * @param[in]  value  The desired length (positive).
   */
  void set_length(ng_float_t value) { length = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the initial minimal distance between agents.
   *
   * @return     The initial minimal distance between agents .
   */
  ng_float_t get_agent_margin() const { return agent_margin; }

  /**
   * @brief      Sets the initial minimal distance between agents.
   *
   * @param[in]  value  The desired margin (positive)
   */
  void set_agent_margin(ng_float_t value) {
    agent_margin = std::max<ng_float_t>(0, value);
  }

  /**
   * @brief      Whenever the agent's safety margin should be considered in
   * addition to \ref get_agent_margin when initializing the agents' poses.
   *
   * @return     The add safety to agent margin.
   */
  bool get_add_safety_to_agent_margin() const {
    return add_safety_to_agent_margin;
  }

  /**
   * @brief      Sets whenever the agent's safety margin should be considered in
   * addition to \ref get_agent_margin when initializing the agents' poses.
   *
   * @param[in]  value  Whenever to consider the safety margin or not.
   */
  void set_add_safety_to_agent_margin(bool value) {
    add_safety_to_agent_margin = value;
  }

  /**
   * @private
   */
  void init_world(World *world,
                  std::optional<int> seed = std::nullopt) override;

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
      {"width", make_property<ng_float_t, CorridorScenario>(
                    &CorridorScenario::get_width, &CorridorScenario::set_width,
                    default_width, "Corridor width")},
      {"length",
       make_property<ng_float_t, CorridorScenario>(
           &CorridorScenario::get_length, &CorridorScenario::set_length,
           default_length, "Corridor length")},
      {"agent_margin", make_property<ng_float_t, CorridorScenario>(
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
