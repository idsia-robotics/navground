/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SCENARIOS_COLLISIONS_H
#define NAVGROUND_SIM_SCENARIOS_COLLISIONS_H

#include <memory>
#include <utility>
#include <vector>

#include "navground/core/types.h"
#include "navground/sim/scenario.h"

using navground::core::make_property;
using navground::core::Properties;
using navground::core::Property;

namespace navground::sim {

struct CollisionsScenario : public Scenario {
  DECLARE_TYPE_AND_PROPERTIES
  
  CollisionsScenario(const char *behavior_name = "HL",
                     ng_float_t control_period = 0.1)
      : Scenario(), behavior_name{behavior_name},
        control_period{control_period} {}

  void init_world(World *world,
                  std::optional<int> seed = std::nullopt) override;

  std::string behavior_name;
  ng_float_t control_period;
};

} // namespace navground::sim

#endif /* end of include guard: NAVGROUND_SIM_SCENARIOS_COLLISIONS_H */
