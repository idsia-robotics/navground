/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_DUMMY_H_
#define NAVGROUND_CORE_BEHAVIOR_DUMMY_H_

#include "navground/core/behavior.h"
#include "navground/core/types.h"
#include "navground_core_export.h"

namespace navground::core {

/**
 * @brief      Dummy behavior that ignores obstacles.
 *
 * Mainly useful to test the interaction with other components
 *
 * *Registered properties*: none
 *
 * *State*: empty
 */
class NAVGROUND_CORE_EXPORT DummyBehavior : public Behavior {
 public:
  /**
   * @brief      Contruct a new instance
   *
   * @param[in]  kinematics  The kinematics
   * @param[in]  radius      The radius
   */
  DummyBehavior(std::shared_ptr<Kinematics> kinematics = nullptr,
                ng_float_t radius = 0)
      : Behavior(kinematics, radius) {}

  /**
   * @private
   */
  std::string get_type() const override { return type; }

 protected:
  Vector2 desired_velocity_towards_point(const Vector2& point, ng_float_t speed,
                                         ng_float_t time_step) override;
  Vector2 desired_velocity_towards_velocity(const Vector2& velocity,
                                            ng_float_t time_step) override;

 private:
     static const std::string type;
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_DUMMY_H_
