/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_COLLISIONS_H
#define NAVGROUND_SIM_SCENARIOS_COLLISIONS_H

#include <memory>
#include <utility>
#include <vector>

#include "navground/sim/scenario.h"

using navground::core::Properties;
using navground::core::Property;
using navground::core::make_property;


namespace navground::sim {

struct CollisionsScenario : public Scenario {
  CollisionsScenario(const char *behavior_name = "HL",
                     float control_period = 0.1f)
      : Scenario(),
        behavior_name{behavior_name},
        control_period{control_period} {}

  void init_world(World *world) override;

  virtual const Properties &get_properties() const override {
    return properties;
  };

  static const std::map<std::string, Property> properties;
  std::string get_type() const override { return type; }
  const static std::string type;

  std::string behavior_name;
  float control_period;
};

}  // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_COLLISIONS_H */
