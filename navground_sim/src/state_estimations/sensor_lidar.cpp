/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_lidar.h"
#include "navground/core/yaml/schema.h"
#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

using navground::core::Properties;
using navground::core::Property;

std::valarray<ng_float_t> LidarStateEstimation::measure_ranges(Agent *agent,
                                                               World *world) {
  auto &c = const_cast<core::CollisionComputation &>(_cc);
  const auto neighbors = world->get_neighbors(agent, _range);
  const auto pose = core::Pose2(_position, 0).absolute(agent->pose);
  c.setup(pose, 0.0, world->get_line_obstacles(), world->get_discs(),
          neighbors);
  auto ranges = c.get_free_distance_for_sector(
      agent->pose.orientation + _start_angle, _field_of_view, _resolution - 1,
      _range, false);
  if (has_error()) {
    auto &rg = world->get_random_generator();
    for (size_t i = 0; i < ranges.size(); ++i) {
      ranges[i] = std::clamp<ng_float_t>(ranges[i] + _error(rg), 0, _range);
    }
  }
  return ranges;
}

void LidarStateEstimation::update(Agent *agent, World *world,
                                  EnvironmentState *state) {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    auto buffer = get_or_init_buffer(*_state, field_name);
    if (buffer) {
      buffer->set_data(measure_ranges(agent, world));
    }
    buffer = get_or_init_buffer(*_state, "start_angle");
    if (buffer) {
      buffer->set_data(std::valarray<ng_float_t>{_start_angle});
    }

    buffer = get_or_init_buffer(*_state, "fov");
    if (buffer) {
      buffer->set_data(std::valarray<ng_float_t>{_field_of_view});
    }

    buffer = get_or_init_buffer(*_state, "max_range");
    if (buffer) {
      buffer->set_data(std::valarray<ng_float_t>{_range});
    }
  }
}

// std::valarray<ng_float_t> LidarStateEstimation::sample_error(World *world) {
//   auto rg = world->get_random_generator();
//   std::valarray<ng_float_t> vs(_resolution);
//   for (size_t i = 0; i < _resolution; ++i) {
//     vs[i] = _error(rg);
//   }
//   return vs;
// }

const std::valarray<ng_float_t> &
LidarStateEstimation::read_ranges_with_name(core::SensingState &state, const std::string & name) {
  const auto buffer = state.get_buffer(Sensor::get_field_name(field_name, name));
  return *(buffer->get_data<ng_float_t>());
}

ng_float_t LidarStateEstimation::get_angular_increment() const {
  return compute_angular_increment(_field_of_view, _resolution);
}

std::valarray<ng_float_t>
LidarStateEstimation::compute_angles(ng_float_t start, ng_float_t field_of_view,
                                     unsigned resolution) {
  std::valarray<ng_float_t> vs(resolution);
  ng_float_t angle = start;
  const ng_float_t delta = compute_angular_increment(field_of_view, resolution);
  for (size_t i = 0; i < vs.size() - 1; ++i) {
    vs[i] = angle;
    angle += delta;
  }
  vs[vs.size() - 1] = start + field_of_view;
  return vs;
}

std::valarray<ng_float_t> LidarStateEstimation::get_angles() const {
  return compute_angles(_start_angle, _field_of_view, _resolution);
}

const std::string LidarStateEstimation::type =
    register_type<LidarStateEstimation>(
        "Lidar",
        Properties{
            {"range",
             Property::make(&LidarStateEstimation::get_range,
                            &LidarStateEstimation::set_range, default_range,
                            "Maximal range", &YAML::schema::positive)},
            {"start_angle",
             Property::make(&LidarStateEstimation::get_start_angle,
                            &LidarStateEstimation::set_start_angle,
                            default_start_angle, "Start angle")},
            {"field_of_view",
             Property::make(&LidarStateEstimation::get_field_of_view,
                            &LidarStateEstimation::set_field_of_view,
                            default_field_of_view, "Total angle",
                            &YAML::schema::positive)},
            {"resolution",
             Property::make<int>(&LidarStateEstimation::get_resolution,
                                 &LidarStateEstimation::set_resolution,
                                 default_resolution, "Resolution",
                                 &YAML::schema::strict_positive)},
            {"position", Property::make<core::Vector2>(
                             &LidarStateEstimation::get_position,
                             &LidarStateEstimation::set_position,
                             core::Vector2::Zero(), "Relative position")},
            {"error_bias", Property::make(&LidarStateEstimation::get_error_bias,
                                          &LidarStateEstimation::set_error_bias,
                                          default_error_bias, "Error bias",
                                          &YAML::schema::positive)},
            {"error_std_dev",
             Property::make(&LidarStateEstimation::get_error_std_dev,
                            &LidarStateEstimation::set_error_std_dev,
                            default_error_std_dev, "Error standard deviation",
                            &YAML::schema::positive)},
        } + Sensor::properties);

} // namespace navground::sim
