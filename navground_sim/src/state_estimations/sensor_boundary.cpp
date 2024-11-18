/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_boundary.h"

#include <valarray>

namespace navground::sim {

using navground::core::Properties;
using navground::core::Property;

void BoundarySensor::update(Agent *agent, World *world,
                            EnvironmentState *state) {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    std::valarray<ng_float_t> distances(_range, 4);
    const auto p = agent->pose.position;
    size_t i = 0;
    if (std::isfinite(_min_x)) {
      distances[i++] = std::clamp<ng_float_t>(p[0] - _min_x, 0, _range);
    }
    if (std::isfinite(_max_x)) {
      distances[i++] = std::clamp<ng_float_t>(_max_x - p[0], 0, _range);
    }
    if (std::isfinite(_min_y)) {
      distances[i++] = std::clamp<ng_float_t>(p[1] - _min_y, 0, _range);
    }
    if (std::isfinite(_max_y)) {
      distances[i++] = std::clamp<ng_float_t>(_max_y - p[1], 0, _range);
    }
    auto buffer = get_or_init_buffer(*_state, "boundary_distance");
    if (buffer)
      buffer->set_data(distances[std::slice(0, i, 1)]);
  }
}

const std::string BoundarySensor::type = register_type<BoundarySensor>(
    "Boundary", Properties{
                    {"range", Property::make(&BoundarySensor::get_range,
                                             &BoundarySensor::set_range,
                                             default_range, "Maximal range")},
                    {"min_x", Property::make(&BoundarySensor::get_min_x,
                                             &BoundarySensor::set_min_x, low,
                                             "Boundary min x")},
                    {"max_x", Property::make(&BoundarySensor::get_max_x,
                                             &BoundarySensor::set_max_x, high,
                                             "Boundary max x")},
                    {"min_y", Property::make(&BoundarySensor::get_min_y,
                                             &BoundarySensor::set_min_y, low,
                                             "Boundary min y")},
                    {"max_y", Property::make(&BoundarySensor::get_max_y,
                                             &BoundarySensor::set_max_y, high,
                                             "Boundary max y")},
                } + Sensor::properties);

} // namespace navground::sim
