/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_discs.h"
#include "navground/core/yaml/schema.h"
#include <algorithm>

namespace navground::sim {

using navground::core::Properties;
using navground::core::Property;

void DiscsStateEstimation::update(Agent *agent, World *world,
                                  EnvironmentState *state) {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    if (!_number)
      return;
    const auto &p = agent->pose;
    const auto &xy = p.position;
    const auto r = agent->radius;
    const auto neighbors = world->get_neighbors(agent, _range);
    const auto obstacles = world->get_discs_in_region(envelop(xy, _range));

    std::vector<std::tuple<ng_float_t, size_t>> distance(neighbors.size() +
                                                         obstacles.size());
    size_t i = 0;

    for (; i < neighbors.size(); ++i) {
      distance[i] = {
          (neighbors[i].position - xy).norm() - neighbors[i].radius - r, i};
    }
    for (size_t j = 0; j < obstacles.size(); ++i, ++j) {
      distance[i] = {
          (obstacles[j].position - xy).norm() - obstacles[j].radius - r, i};
    }

    std::sort(distance.begin(), distance.end());

    std::valarray<unsigned> id(static_cast<unsigned>(0), _number);
    std::valarray<ng_float_t> radius(0.0, _number);
    std::valarray<ng_float_t> position(0.0, 2 * _number);
    std::valarray<ng_float_t> velocity(0.0, 2 * _number);
    std::valarray<uint8_t> valid((uint8_t)0, _number);

    for (size_t i = 0; i < std::min<size_t>(_number, distance.size()); ++i) {
      valid[i] = 1;
      const auto index = std::get<1>(distance[i]);
      Vector2 pn;
      const auto num_neighbors = neighbors.size();
      if (index < num_neighbors) {
        const auto &n = neighbors[index];
        // radius[i] = n.radius;
        radius[i] = std::min(n.radius, _max_radius);
        id[i] = std::min(n.id, _max_id);
        pn = to_relative(n.position - p.position, p);
        if (_use_nearest_point) {
          pn -= pn.normalized() * n.radius;
        }
        const auto vn = to_relative(n.velocity, p);
        velocity[2 * i] = std::min(vn[0], _max_speed);
        velocity[2 * i + 1] = std::min(vn[1], _max_speed);
      } else {
        const auto &n = obstacles[index - num_neighbors];
        radius[i] = std::min(n.radius, _max_radius);
        pn = to_relative(n.position - p.position, p);
        if (_use_nearest_point) {
          pn -= pn.normalized() * n.radius;
        }
      }
      position[2 * i] = pn[0];
      position[2 * i + 1] = pn[1];
    }
    if (include_radius()) {
      auto buffer = get_or_init_buffer(*_state, "radius");
      if (buffer)
        buffer->set_data(radius);
    }
    if (include_position()) {
      auto buffer = get_or_init_buffer(*_state, "position");
      if (buffer)
        buffer->set_data(position);
    }
    if (include_velocity()) {
      auto buffer = get_or_init_buffer(*_state, "velocity");
      if (buffer)
        buffer->set_data(velocity);
    }
    if (get_include_valid()) {
      auto buffer = get_or_init_buffer(*_state, "valid");
      if (buffer)
        buffer->set_data(valid);
    }
    if (include_id()) {
      auto buffer = get_or_init_buffer(*_state, "id");
      if (buffer)
        buffer->set_data(id);
    }
  }
}

const std::string DiscsStateEstimation::type =
    register_type<DiscsStateEstimation>(
        "Discs",
        Properties{
            {"range",
             Property::make(&DiscsStateEstimation::get_range,
                            &DiscsStateEstimation::set_range, default_range,
                            "Maximal range", &YAML::schema::positive)},
            {"number", Property::make<int>(&DiscsStateEstimation::get_number,
                                           &DiscsStateEstimation::set_number,
                                           default_number, "Number",
                                           &YAML::schema::positive)},
            {"max_radius", Property::make(&DiscsStateEstimation::get_max_radius,
                                          &DiscsStateEstimation::set_max_radius,
                                          default_max_radius, "Maximal radius",
                                          &YAML::schema::positive)},
            {"max_speed", Property::make(&DiscsStateEstimation::get_max_speed,
                                         &DiscsStateEstimation::set_max_speed,
                                         default_max_speed, "Maximal speed",
                                         &YAML::schema::positive)},
            {"include_valid",
             Property::make(&DiscsStateEstimation::get_include_valid,
                            &DiscsStateEstimation::set_include_valid,
                            default_include_valid, "Include validity field")},
            {"use_nearest_point",
             Property::make(&DiscsStateEstimation::get_use_nearest_point,
                            &DiscsStateEstimation::set_use_nearest_point,
                            default_use_nearest_point,
                            "Whether to use the nearest point as position")},
            {"max_id", Property::make<int>(
                           &DiscsStateEstimation::get_max_id,
                           &DiscsStateEstimation::set_max_id, default_max_id,
                           "The maximal possible id", &YAML::schema::positive)},
        } + Sensor::properties);

} // namespace navground::sim
