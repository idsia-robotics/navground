/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_SIM_SENSOR_H_
#define NAVGROUND_SIM_SENSOR_H_

#include <vector>

#include "navground/core/buffer.h"
#include "navground/core/states/sensing.h"
#include "navground/sim/export.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/world.h"

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
  Sensor(const std::string &name = "") : StateEstimation(), _name(name) {}

  /**
   * @private
   */
  virtual void prepare(Agent *agent, World *world) {
    auto state = get_state(agent);
    if (!state)
      return;
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

  /**
   * @brief      Sets the sensor name.
   *
   * @param[in]  value     The new value
   */
  void set_name(const std::string &value) { _name = value; }

  /**
   * @brief      Gets the sensor name.
   *
   * @return     The name.
   */
  const std::string &get_name() const { return _name; }

  /**
   * @brief      Returns the field prefixed by the sensor name if set.
   *
   * @param[in]  field  The field
   *
   * @return     The name.
   */
  std::string get_field_name(const std::string &field) const {
    if (_name.empty()) {
      return field;
    }
    return _name + "/" + field;
  }

  core::Buffer * get_or_init_buffer(core::SensingState &state,
                                                   const std::string &field) const {
    const std::string name = get_field_name(field);
    auto buffer = state.get_buffer(name);
    if (!buffer) {
      buffer = state.init_buffer(name, get_description().at(name));
    }
    return buffer;
  }

  /**
   * @private
   */
  static const std::map<std::string, core::Property> properties;

private:
  std::string _name;
};

} // namespace navground::sim

#endif /* end of include guard:                                                \
          NAVGROUND_SIM_SENSOR_H_ */
