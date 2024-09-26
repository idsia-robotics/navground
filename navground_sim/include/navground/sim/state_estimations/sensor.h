/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SENSOR_H_
#define NAVGROUND_SIM_SENSOR_H_

#include <vector>

#include "navground/core/buffer.h"
#include "navground/core/states/sensing.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/world.h"
#include "navground/sim/export.h"

namespace navground::sim {

/**
 * @brief      Base class for agents using a \ref SensingState.
 *
 */
struct NAVGROUND_SIM_EXPORT Sensor : public StateEstimation {
  using Description = std::map<std::string, core::BufferDescription>;

  /**
   * @brief      Constructs a new instance.
   */
  Sensor() : StateEstimation() {}

  /**
   * @private
   */
  virtual void prepare(Agent *agent, World *world) const {
    auto state = get_state(agent);
    if (!state) return;
    prepare(*state);
  }

  /**
   * @brief      Gets the environment state of an agent.
   *
   * @private
   *
   * @return     The geometric state or ``nullptr`` if the agent behavior does
   * not have a environment state that is a subclass of \ref
   * navground::core::SensingState
   */
  core::SensingState *get_state(Agent *agent) const {
    if (agent) {
      if (Behavior *behavior = agent->get_behavior()) {
        return dynamic_cast<core::SensingState *>(
            behavior->get_environment_state());
      }
    }
    return nullptr;
  }

  /**
   * @brief      Prepare a sensing state to have the correct buffers.
   *
   * @param      state  The state
   */
  void prepare(core::SensingState &state) const {
    for (const auto &[k, v] : get_description()) {
      state.init_buffer(k, v);
    }
  }

  /**
   * @brief      Gets the description of the buffers set by the sensors.
   *
   * @return     The description.
   */
  virtual Description get_description() const = 0;
};

}  // namespace navground::sim

#endif /* end of include guard: \
          NAVGROUND_SIM_SENSOR_H_ */
