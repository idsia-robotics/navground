/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_lidar.h"

#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void LidarStateEstimation::update(Agent *agent, World *world,
                                  EnvironmentState *state) {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    auto &c = const_cast<core::CollisionComputation &>(_cc);
    const auto neighbors = world->get_neighbors(agent, _range);
    const auto pose = core::Pose2(_position, 0).absolute(agent->pose);
    c.setup(pose, 0.0, world->get_line_obstacles(), world->get_discs(),
            neighbors);
    auto ranges = c.get_free_distance_for_sector(
        agent->pose.orientation + _start_angle, _field_of_view, _resolution - 1,
        _range, false);
    auto buffer = get_or_init_buffer(*_state, field_name);
    if (buffer) {
      if (has_error()) {
        auto &rg = world->get_random_generator();
        for (size_t i = 0; i < ranges.size(); ++i) {
          ranges[i] = std::clamp<ng_float_t>(ranges[i] + _error(rg), 0, _range);
        }
      }
      buffer->set_data(ranges);
    }

    buffer = get_or_init_buffer(*_state, "start_angle");
    if (buffer) {
      buffer->set_data(std::valarray<ng_float_t>{_start_angle});
    }

    buffer = get_or_init_buffer(*_state, "fov");
    if (buffer) {
      buffer->set_data(std::valarray<ng_float_t>{_field_of_view});
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
LidarStateEstimation::read_ranges(core::SensingState &state) const {
  const auto buffer = get_or_init_buffer(state, field_name);
  return *(buffer->get_data<ng_float_t>());
}

ng_float_t LidarStateEstimation::get_angular_increment() const {
  if (_resolution > 1) {
    return _field_of_view / (_resolution - 1);
  }
  return 0;
}

std::valarray<ng_float_t> LidarStateEstimation::get_angles() const {
  std::valarray<ng_float_t> vs(_resolution);
  ng_float_t angle = _start_angle;
  const ng_float_t delta = get_angular_increment();
  for (size_t i = 0; i < vs.size() - 1; ++i) {
    vs[i] = angle;
    angle += delta;
  }
  vs[vs.size() - 1] = _start_angle + _field_of_view;
  return vs;
}

const std::map<std::string, Property> LidarStateEstimation::properties =
    Properties{
        {"range",
         make_property<ng_float_t, LidarStateEstimation>(
             &LidarStateEstimation::get_range, &LidarStateEstimation::set_range,
             default_range, "Maximal range")},
        {"start_angle", make_property<ng_float_t, LidarStateEstimation>(
                            &LidarStateEstimation::get_start_angle,
                            &LidarStateEstimation::set_start_angle,
                            default_start_angle, "Start angle")},
        {"field_of_view", make_property<ng_float_t, LidarStateEstimation>(
                              &LidarStateEstimation::get_field_of_view,
                              &LidarStateEstimation::set_field_of_view,
                              default_field_of_view, "Total angle")},
        {"resolution", make_property<int, LidarStateEstimation>(
                           &LidarStateEstimation::get_resolution,
                           &LidarStateEstimation::set_resolution,
                           default_resolution, "Resolution")},
        {"position", make_property<core::Vector2, LidarStateEstimation>(
                         &LidarStateEstimation::get_position,
                         &LidarStateEstimation::set_position,
                         core::Vector2::Zero(), "Relative position")},
        {"error_bias", make_property<ng_float_t, LidarStateEstimation>(
                           &LidarStateEstimation::get_error_bias,
                           &LidarStateEstimation::set_error_bias,
                           default_error_bias, "Error bias")},
        {"error_std_dev",
         make_property<ng_float_t, LidarStateEstimation>(
             &LidarStateEstimation::get_error_std_dev,
             &LidarStateEstimation::set_error_std_dev, default_error_std_dev,
             "Error standard deviation")},
    } +
    Sensor::properties;

const std::string LidarStateEstimation::type =
    register_type<LidarStateEstimation>("Lidar");

} // namespace navground::sim
