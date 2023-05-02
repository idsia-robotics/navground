/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_EXAMPLES_MY_BEHAVIOR_H_
#define NAVGROUND_CORE_EXAMPLES_MY_BEHAVIOR_H_

#include "navground/core/behavior.h"

namespace navground::core {

/**
 * @brief      Idle behavior that always stay still.
 *
 * It showcases how to define and use a new behavior from an external shared
 * library.
 */
class IdleBehavior : public Behavior {
 public:
  using Behavior::Behavior;

  std::string get_type() const override { return type; }

 protected:
  Vector2 desired_velocity_towards_point([[maybe_unused]] const Vector2 & point, [[maybe_unused]] float speed, [[maybe_unused]] float time_step) override {
    return Vector2::Zero();
  }

  Vector2 desired_velocity_towards_velocity(const Vector2 & velocity, [[maybe_unused]] float time_step) override {
    return Vector2::Zero();
  }

 private:
  static inline const std::string type = register_type<IdleBehavior>("Idle");
};

}  // namespace navground::core

#endif  // NAVGROUND_CORE_EXAMPLES_MY_BEHAVIOR_H_
