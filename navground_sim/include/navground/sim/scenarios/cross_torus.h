/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_CROSS_TORUS_H
#define NAVGROUND_SIM_SCENARIOS_CROSS_TORUS_H

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
 * @brief      A scenario where agents move crosses in a middle zone,
 * one half of the agents moving vertically
 * and the other half horizontally. This world is period in both directions.
 *
 * *Registered properties*:
 *
 *   - `side` (float, \ref get_side)
 *
 *   - `agent_margin` (float, \ref get_agent_margin)
 *
 *   - `add_safety_to_agent_margin` (bool, \ref get_add_safety_to_agent_margin)
 */
struct NAVGROUND_SIM_EXPORT CrossTorusScenario : public Scenario {
 public:
  // distance between targets
  ng_float_t side;
  inline static ng_float_t default_side = 2;
  // initial minimal distance between agents
  ng_float_t agent_margin;
  inline static ng_float_t default_agent_margin = static_cast<ng_float_t>(0.1);
  // whether to add the safety margin to the agent margin;
  bool add_safety_to_agent_margin;
  inline static bool default_add_safety_to_agent_margin = true;
  // initial minimal distance between agents and targets

  CrossTorusScenario(
      ng_float_t side = default_side,
      ng_float_t agent_margin = default_agent_margin,
      bool add_safety_to_agent_margin = default_add_safety_to_agent_margin)
      : Scenario(),
        side(side),
        agent_margin(agent_margin),
        add_safety_to_agent_margin(add_safety_to_agent_margin) {}

  /**
   * @brief      Gets the side of simulate cell of the infinite lattice.
   *
   * @return     The half-side of the squared arena.
   */
  ng_float_t get_side() const { return side; }

  /**
   * @brief      Sets the side of simulate cell of the infinite lattice.
   *
   * @param[in]  value  The desired value (positive)
   */
  void set_side(ng_float_t value) { side = std::max<ng_float_t>(0, value); }

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
  static const std::map<std::string, Property> properties;

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  static const std::string type;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_CROSS_TORUS_H */
