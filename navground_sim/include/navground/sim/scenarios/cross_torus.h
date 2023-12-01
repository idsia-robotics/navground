/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_CROSS_TORUS_H
#define NAVGROUND_SIM_SCENARIOS_CROSS_TORUS_H

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

struct NAVGROUND_SIM_EXPORT CrossTorusScenario : public Scenario {
 public:
  // distance between targets
  float side;
  inline static float default_side = 2.0f;
  // initial minimal distance between agents
  float agent_margin;
  inline static float default_agent_margin = 0.1f;
  // whether to add the safety margin to the agent margin;
  bool add_safety_to_agent_margin;
  inline static bool default_add_safety_to_agent_margin = true;
  // initial minimal distance between agents and targets

  CrossTorusScenario(
      float side = default_side, float agent_margin = default_agent_margin,
      bool add_safety_to_agent_margin = default_add_safety_to_agent_margin)
      : Scenario(),
        side(side),
        agent_margin(agent_margin),
        add_safety_to_agent_margin(add_safety_to_agent_margin) {}

  float get_side() const { return side; }

  void set_side(float value) { side = std::max(0.0f, value); }

  float get_agent_margin() const { return agent_margin; }

  void set_agent_margin(float value) { agent_margin = std::max(0.0f, value); }

  bool get_add_safety_to_agent_margin() const {
    return add_safety_to_agent_margin;
  }

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
  inline static const std::map<std::string, Property> properties{
      {"side", make_property<float, CrossTorusScenario>(
                   &CrossTorusScenario::get_side, &CrossTorusScenario::set_side,
                   default_side, "Distance between targets")},
      {"agent_margin", make_property<float, CrossTorusScenario>(
                           &CrossTorusScenario::get_agent_margin,
                           &CrossTorusScenario::set_agent_margin, 0.1f,
                           "initial minimal distance between agents")},
      {"add_safety_to_agent_margin",
       make_property<bool, CrossTorusScenario>(
           &CrossTorusScenario::get_add_safety_to_agent_margin,
           &CrossTorusScenario::set_add_safety_to_agent_margin,
           default_add_safety_to_agent_margin,
           "Whether to add the safety margin to the agent margin")}};

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  inline static const std::string type =
      register_type<CrossTorusScenario>("CrossTorus");
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_CROSS_TORUS_H */
