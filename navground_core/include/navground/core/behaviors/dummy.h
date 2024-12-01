/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_DUMMY_H_
#define NAVGROUND_CORE_BEHAVIOR_DUMMY_H_

#include "navground/core/behavior.h"
#include "navground/core/export.h"
#include "navground/core/types.h"

namespace navground::core {

/**
 * @brief      Dummy behavior that ignores obstacles.
 *
 * Mainly useful to test the interaction with other components.
 *
 * Can be assigned an arbitrary environment state,
 * using \ref set_environment_state, and/or the property "environment"
 *
 * *Registered properties*: environment (string, \ref
 * get_environment_state_type),
 *
 * *State*: any
 */
class NAVGROUND_CORE_EXPORT DummyBehavior : public Behavior {
public:
  static const std::string type;
  /**
   * @brief      Construct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  explicit DummyBehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
                         ng_float_t radius = 0)
      : Behavior(kinematics, radius), _state(nullptr) {}

  /** @private */
  EnvironmentState *get_environment_state() override { return _state.get(); }

  /**
   * @brief      Sets the environment state.
   *
   * @param[in]  state  The state
   */
  void set_environment_state(const std::shared_ptr<EnvironmentState> &state) {
    _state = state;
  }
  /**
   * @brief      Gets the environment state type.
   *
   * - "Geometric" for \ref GeometriState,
   *
   * - "Sensing" for \ref SensingState,
   *
   * - "" for an null state.
   *
   * @return     The environment state type.
   */
  std::string get_environment_state_type() const;
  /**
   * @brief      Sets the environment state type.
   *
   * - "Geometric" for \ref GeometriState,
   *
   * - "Sensing" for \ref SensingState,
   *
   * - anything else for an null state.
   *
   * @param[in]  value  The value
   */
  void set_environment_state_type(const std::string &value);

protected:
  Vector2 desired_velocity_towards_point(const Vector2 &point, ng_float_t speed,
                                         ng_float_t time_step) override;
  Vector2 desired_velocity_towards_velocity(const Vector2 &velocity,
                                            ng_float_t time_step) override;

private:
  std::shared_ptr<EnvironmentState> _state;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_BEHAVIOR_DUMMY_H_
