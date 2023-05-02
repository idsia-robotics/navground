/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_CROSS_H
#define NAVGROUND_SIM_SCENARIOS_CROSS_H

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

struct NAVGROUND_SIM_EXPORT CrossScenario : public Scenario {
 public:
  // distance between targets
  float side;
  inline static float default_side = 2.0f;
  // goal tolerance
  float tolerance;
  inline static float default_tolerance = 0.25f;
  // initial minimal distance between agents
  float agent_margin;
  inline static float default_agent_margin = 0.1f;
  // whether to add the safety margin to the agent margin;
  bool add_safety_to_agent_margin;
  inline static bool default_add_safety_to_agent_margin = true;
  // initial minimal distance between agents and targets
  float target_margin;
  inline static float default_target_margin = 0.5f;

  CrossScenario(
      float side = default_side, float tolerance = default_tolerance,
      float agent_margin = default_agent_margin,
      bool add_safety_to_agent_margin = default_add_safety_to_agent_margin,
      float target_margin = default_target_margin)
      : Scenario(),
        side(side),
        tolerance(tolerance),
        agent_margin(agent_margin),
        add_safety_to_agent_margin(add_safety_to_agent_margin),
        target_margin(target_margin) {}

  float get_side() const { return side; }

  void set_side(float value) { side = std::max(0.0f, value); }

  float get_tolerance() const { return tolerance; }

  void set_tolerance(float value) { tolerance = std::max(0.0f, value); }

  float get_agent_margin() const { return agent_margin; }

  void set_agent_margin(float value) { agent_margin = std::max(0.0f, value); }

  bool get_add_safety_to_agent_margin() const {
    return add_safety_to_agent_margin;
  }

  void set_add_safety_to_agent_margin(bool value) {
    add_safety_to_agent_margin = value;
  }

  float get_target_margin() const { return target_margin; }

  void set_target_margin(float value) { target_margin = value; }

  /**
   * @private
   */
  void init_world(World *world) override;

  /**
   * @private
   */
  virtual const Properties &get_properties() const override {
    return properties;
  };

  /**
   * @private
   */
  static inline const std::map<std::string, Property> properties{
      {"side", make_property<float, CrossScenario>(
                   &CrossScenario::get_side, &CrossScenario::set_side,
                   default_side, "Distance between targets")},
      {"tolerance",
       make_property<float, CrossScenario>(
           &CrossScenario::get_tolerance, &CrossScenario::set_tolerance,
           default_tolerance, "Goal tolerance")},
      {"agent_margin",
       make_property<float, CrossScenario>(
           &CrossScenario::get_agent_margin, &CrossScenario::set_agent_margin,
           0.1f, "initial minimal distance between agents")},
      {"add_safety_to_agent_margin",
       make_property<bool, CrossScenario>(
           &CrossScenario::get_add_safety_to_agent_margin,
           &CrossScenario::set_add_safety_to_agent_margin,
           default_add_safety_to_agent_margin,
           "Whether to add the safety margin to the agent margin")},
      {"target_margin",
       make_property<float, CrossScenario>(
           &CrossScenario::get_target_margin, &CrossScenario::set_target_margin,
           default_target_margin,
           "Initial minimal distance between agents and targets")}};

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 private:
  static inline const std::string type = register_type<CrossScenario>("Cross");
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_CROSS_H */
