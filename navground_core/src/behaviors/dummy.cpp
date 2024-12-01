/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/behaviors/dummy.h"
#include "navground/core/states/geometric.h"
#include "navground/core/states/sensing.h"

namespace navground::core {

std::string DummyBehavior::get_environment_state_type() const {
  const EnvironmentState *state =
      const_cast<DummyBehavior *>(this)->get_environment_state();
  if (dynamic_cast<const SensingState *>(state)) {
    return "Sensing";
  }
  if (dynamic_cast<const GeometricState *>(state)) {
    return "Geometric";
  }
  return "";
}

void DummyBehavior::set_environment_state_type(const std::string &value) {
  const std::string current = get_environment_state_type();
  if (value == "Sensing") {
    if (current != value) {
      set_environment_state(std::make_shared<SensingState>());
    }
    return;
  }
  if (value == "Geometric") {
    if (current != value) {
      set_environment_state(std::make_shared<GeometricState>());
    }
    return;
  }
  set_environment_state(nullptr);
}

Vector2 DummyBehavior::desired_velocity_towards_velocity(
    const Vector2 &value, [[maybe_unused]] ng_float_t dt) {
  return value;
}

Vector2 DummyBehavior::desired_velocity_towards_point(
    const Vector2 &point, ng_float_t speed, [[maybe_unused]] ng_float_t dt) {
  auto delta = point - pose.position;
  const auto n = delta.norm();
  if (n) {
    return speed * delta / n;
  }
  return Vector2::Zero();
}

const std::string DummyBehavior::type = register_type<DummyBehavior>(
    "Dummy", {{"environment",
               Property::make(
                   &DummyBehavior::get_environment_state_type,
                   &DummyBehavior::set_environment_state_type, std::string(""),
                   "The type on environment state: \"Geometric\" for "
                   "GeometriState, \"Sensing\" for SensingState. Other "
                   "values correspond to a null state.")}});

} // namespace navground::core
