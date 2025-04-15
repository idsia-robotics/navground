/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_marker.h"
#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

using navground::core::Properties;
using navground::core::Property;

void MarkerStateEstimation::update_marker(Agent *agent, World *world) {

  switch (_reference_orientation) {
  case MarkerStateEstimation::ReferenceOrientation::agent:
    _measured_marker_position =
        core::to_relative_point(_marker_position, agent->get_pose());
    return;
  case MarkerStateEstimation::ReferenceOrientation::world:
    _measured_marker_position = _marker_position - agent->get_pose().position;
    return;
  case MarkerStateEstimation::ReferenceOrientation::target_direction:
    Behavior *b = agent->get_behavior();
    if (b) {
      auto target = b->get_target_ref();
      if (target.direction) {
        const core::Vector2 e = *(target.direction);
        const core::Vector2 n{-e[1], e[0]};
        const core::Vector2 delta =
            _marker_position - agent->get_pose().position;
        // we assume that e is of norm 1
        _measured_marker_position = {e.dot(delta), n.dot(delta)};
      }
    }
  }
}

void MarkerStateEstimation::update(Agent *agent, World *world,
                                   EnvironmentState *state) {
  update_marker(agent, world);
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    const std::array<std::string, 2> cs{"x", "y"};
    for (size_t i = 0; i < cs.size(); ++i) {
      auto buffer = get_or_init_buffer(*_state, cs[i]);
      if (buffer) {
        buffer->set_data(
            std::valarray<ng_float_t>{_measured_marker_position[i]});
      }
    }
  }
}

std::optional<core::Vector2>
MarkerStateEstimation::read_marker_position_with_name(core::SensingState &state,
                                                      const std::string &name) {
  core::Vector2 value;
  const std::array<std::string, 2> cs{"x", "y"};
  for (size_t i = 0; i < cs.size(); ++i) {
    auto buffer = state.get_buffer(Sensor::get_field_name(cs[i], name));
    if (!buffer) {
      return std::nullopt;
    }
    const auto data = buffer->get_data<ng_float_t>();
    if (!data || data->size() != 1) {
      return std::nullopt;
    } else {
      value[i] = (*data)[0];
    }
  }
  return value;
}

const std::string MarkerStateEstimation::type = register_type<
    MarkerStateEstimation>(
    "Marker",
    Properties{
        {"marker_position",
         Property::make<core::Vector2>(
             &MarkerStateEstimation::get_marker_position,
             &MarkerStateEstimation::set_marker_position, core::Vector2::Zero(),
             "Marker position in world frame")},
        {"min_x", Property::make(&MarkerStateEstimation::get_min_x,
                                 &MarkerStateEstimation::set_min_x,
                                 -std::numeric_limits<ng_float_t>::infinity(),
                                 "Measured position minimal x coordinate")},
        {"min_y", Property::make(&MarkerStateEstimation::get_min_y,
                                 &MarkerStateEstimation::set_min_y,
                                 -std::numeric_limits<ng_float_t>::infinity(),
                                 "Measured position minimal y coordinate")},
        {"max_x", Property::make(&MarkerStateEstimation::get_max_x,
                                 &MarkerStateEstimation::set_max_x,
                                 std::numeric_limits<ng_float_t>::infinity(),
                                 "Measured position maximal x coordinate")},
        {"max_y", Property::make(&MarkerStateEstimation::get_max_y,
                                 &MarkerStateEstimation::set_max_y,
                                 std::numeric_limits<ng_float_t>::infinity(),
                                 "Measured position maximal y coordinate")},
        {"reference_orientation",
         Property::make(&MarkerStateEstimation::get_reference_orientation_name,
                        &MarkerStateEstimation::set_reference_orientation_name,
                        std::string("agent"),
                        "The reference frame used for orientation")},
    } + Sensor::properties);

} // namespace navground::sim
