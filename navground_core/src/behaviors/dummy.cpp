/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behaviors/dummy.h"

namespace navground::core {

Vector2 DummyBehavior::desired_velocity_towards_velocity(
    const Vector2& value, [[maybe_unused]] ng_float_t dt) {
  return value;
}

Vector2 DummyBehavior::desired_velocity_towards_point(
    const Vector2& point, ng_float_t speed, [[maybe_unused]] ng_float_t dt) {
  auto delta = point - pose.position;
  const auto n = delta.norm();
  if (n) {
    return speed * delta / n;
  }
  return Vector2::Zero();
}

// std::string DummyBehavior::type = register_type<DummyBehavior>("Dummy");

// const char* DummyBehavior::name = register_type<DummyBehavior>("Dummy");

}  // namespace navground::core
