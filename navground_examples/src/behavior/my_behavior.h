/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#ifndef NAVGROUND_CORE_EXAMPLES_MY_BEHAVIOR_H_
#define NAVGROUND_CORE_EXAMPLES_MY_BEHAVIOR_H_

#include "my_behavior_export.h"
#include "navground/core/behavior.h"

namespace navground::core {

/**
 * @brief      Idle behavior that always stay still.
 *
 * It showcases how to define and use a new behavior from an external shared
 * library.
 */
class MY_BEHAVIOR_EXPORT IdleBehavior : public Behavior {
public:
  using Behavior::Behavior;
  DECLARE_TYPE_AND_PROPERTIES
protected:
  Vector2 desired_velocity_towards_point(
      [[maybe_unused]] const Vector2 &point, [[maybe_unused]] ng_float_t speed,
      [[maybe_unused]] ng_float_t time_step) override {
    if (get_ignore_obstacles()) {
      return clamp_norm(point - get_position(), speed);
    }    
    return Vector2::Zero();
  }

  Vector2 desired_velocity_towards_velocity(
      [[maybe_unused]] const Vector2 &velocity,
      [[maybe_unused]] ng_float_t time_step) override {
    if (get_ignore_obstacles()) {
      return velocity;
    }
    return Vector2::Zero();
  }

  bool get_ignore_obstacles() const { return _ignore_obstacles; }
  void set_ignore_obstacles(bool value) { _ignore_obstacles = value; }

private:
  bool _ignore_obstacles;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_EXAMPLES_MY_BEHAVIOR_H_
