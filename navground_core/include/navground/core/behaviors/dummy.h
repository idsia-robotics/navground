/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_BEHAVIOR_DUMMY_H_
#define NAVGROUND_CORE_BEHAVIOR_DUMMY_H_

#include "navground/core/behavior.h"
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
                float radius = 0.0f)
      : Behavior(kinematics, radius) {}

  /** 
   * @private
  */
  std::string get_type() const override { return type; }

 protected:
  Vector2 desired_velocity_towards_point(const Vector2 & point, float speed, float time_step) override;
  Vector2 desired_velocity_towards_velocity(const Vector2 & velocity, float time_step) override;

 private:
  static inline const std::string type = register_type<DummyBehavior>("Dummy");
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_BEHAVIOR_DUMMY_H_
