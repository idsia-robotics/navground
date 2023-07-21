/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_EXAMPLES_MY_SCENARIO_H_
#define NAVGROUND_CORE_EXAMPLES_MY_SCENARIO_H_

#include "navground/sim/scenario.h"

namespace navground::sim {

/**
 * @brief      Idle behavior that always stay still.
 *
 * It showcases how to define and use a new behavior from an external shared
 * library.
 */
struct EmptyScenario : Scenario {
 public:
  using Scenario::Scenario;

  std::string get_type() const override { return type; }

  void init_world(World *world, [[maybe_unused]] int seed = 0) override {
    Scenario::init_world(world);
    // ...
  }

  static inline const std::string type = register_type<EmptyScenario>("Empty");
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_EXAMPLES_NEW_BEHAVIOR_H_
