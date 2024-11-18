/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_CROSS_H
#define NAVGROUND_SIM_SCENARIOS_CROSS_H

#include <memory>
#include <tuple>
#include <utility>
#include <vector>

#include "navground/core/types.h"
#include "navground/sim/export.h"
#include "navground/sim/scenario.h"

using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

/**
 * @brief      A scenario where agents move between two waypoints,
 * one half of the agents vertically and the other horizontally.
 * Agents are initialize at non-overlapping random poses in a box.
 *
 * *Registered properties*:
 *
 *   - `side` (float, \ref get_side)
 *
 *   - `tolerance` (float, \ref get_tolerance)
 *
 *   - `agent_margin` (float, \ref get_agent_margin)
 *
 *   - `add_safety_to_agent_margin` (bool, \ref get_add_safety_to_agent_margin)
 *
 *   - `target_margin` (float,  \ref get_target_margin)
 */
struct NAVGROUND_SIM_EXPORT CrossScenario : public Scenario {
  DECLARE_TYPE_AND_PROPERTIES
  
  // distance between targets
  ng_float_t side;
  inline static ng_float_t default_side = 2;
  // goal tolerance
  ng_float_t tolerance;
  inline static ng_float_t default_tolerance = static_cast<ng_float_t>(0.25);
  // initial minimal distance between agents
  ng_float_t agent_margin;
  inline static ng_float_t default_agent_margin = static_cast<ng_float_t>(0.1);
  // whether to add the safety margin to the agent margin;
  bool add_safety_to_agent_margin;
  inline static bool default_add_safety_to_agent_margin = true;
  // initial minimal distance between agents and targets
  ng_float_t target_margin;
  inline static ng_float_t default_target_margin = static_cast<ng_float_t>(0.5);

  CrossScenario(
      ng_float_t side = default_side, ng_float_t tolerance = default_tolerance,
      ng_float_t agent_margin = default_agent_margin,
      bool add_safety_to_agent_margin = default_add_safety_to_agent_margin,
      ng_float_t target_margin = default_target_margin)
      : Scenario(), side(side), tolerance(tolerance),
        agent_margin(agent_margin),
        add_safety_to_agent_margin(add_safety_to_agent_margin),
        target_margin(target_margin) {}

  /**
   * @brief      Gets the half-length of the squared arena.
   * Waypoints are placed at (+/-side, 0) and (0, +/-side)
   *
   * @return     The half-side of the squared arena.
   */
  ng_float_t get_side() const { return side; }

  /**
   * @brief      Sets the half-length of the squared arena.
   * Waypoints are placed at (+/-side, 0) and (0, +/-side)
   *
   * @param[in]  value  The desired value (positive)
   */
  void set_side(ng_float_t value) { side = std::max<ng_float_t>(0, value); }

  /**
   * @brief      Gets the task goal tolerance (i.e., agents will
   * change target when they arrive closer than this to their current target).
   *
   * @return     The tolerance.
   */
  ng_float_t get_tolerance() const { return tolerance; }

  /**
   * @brief      Sets the task goal tolerance (i.e., agents will
   * change target when they arrive closer than this to their current target).
   *
   * @param[in]  value  The desired value (positive)
   */
  void set_tolerance(ng_float_t value) {
    tolerance = std::max<ng_float_t>(0, value);
  }

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
   * @brief      Gets the minimal distance between agents and targets at
   * initialization.
   *
   * @return     The margin.
   */
  ng_float_t get_target_margin() const { return target_margin; }

  /**
   * @brief      Sets the minimal distance between agents and targets at
   * initialization.
   *
   * @param[in]  value  The desired value (positive)
   */
  void set_target_margin(ng_float_t value) { target_margin = value; }

  /**
   * @private
   */
  void init_world(World *world,
                  std::optional<int> seed = std::nullopt) override;
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_CROSS_H */
