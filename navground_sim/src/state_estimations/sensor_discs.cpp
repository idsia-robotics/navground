/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_discs.h"

#include <algorithm>

namespace navground::sim {

void DiscsStateEstimation::update(Agent *agent, World *world,
                                  EnvironmentState *state) const {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    if (!number) return;
    const auto &p = agent->pose;
    const auto &xy = p.position;
    const auto r = agent->radius;
    const auto neighbors = world->get_neighbors(agent, range);
    const auto obstacles =
        world->get_static_obstacles_in_region(envelop(xy, range));

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

    std::valarray<ng_float_t> radius(0.0, number);
    std::valarray<ng_float_t> position(0.0, 2 * number);
    std::valarray<ng_float_t> velocity(0.0, 2 * number);
    std::valarray<uint8_t> valid((uint8_t)0, number);

    for (size_t i = 0; i < std::min<size_t>(number, distance.size()); ++i) {
      valid[i] = 1;
      const auto index = std::get<1>(distance[i]);
      Vector2 pn;
      const auto num_neighbors = neighbors.size();
      if (index < num_neighbors) {
        const auto &n = neighbors[index];
        // radius[i] = n.radius;
        radius[i] = std::min(n.radius, max_radius);
        pn = to_relative(n.position - p.position, p);
        const auto vn = to_relative(n.velocity, p);
        velocity[2 * i] = std::min(vn[0], max_speed);
        velocity[2 * i + 1] = std::min(vn[1], max_speed);
      } else {
        const auto &n = obstacles[index - num_neighbors];
        radius[i] = std::min(n.radius, max_radius);
        pn = to_relative(n.position - p.position, p);
      }
      position[2 * i] = pn[0];
      position[2 * i + 1] = pn[1];
    }
    if (include_radius()) {
      auto buffer = _state->get_buffer("radius");
      if (!buffer) {
        buffer = _state->init_buffer("radius", get_description().at("radius"));
      }
      if (buffer) buffer->set_data(radius);
    }
    if (include_position()) {
      auto buffer = _state->get_buffer("position");
      if (!buffer) {
        buffer =
            _state->init_buffer("position", get_description().at("position"));
      }
      if (buffer) buffer->set_data(position);
    }
    if (include_velocity()) {
      auto buffer = _state->get_buffer("velocity");
      if (!buffer) {
        buffer =
            _state->init_buffer("velocity", get_description().at("velocity"));
      }
      if (buffer) buffer->set_data(velocity);
    }
    if (get_include_valid()) {
      auto buffer = _state->get_buffer("valid");
      if (!buffer) {
        buffer =
            _state->init_buffer("valid", get_description().at("valid"));
      }
      if (buffer) buffer->set_data(valid);
    }
  }
}

const std::map<std::string, Property> DiscsStateEstimation::properties =
Properties{
    {"range", make_property<ng_float_t, DiscsStateEstimation>(
                  &DiscsStateEstimation::get_range,
                  &DiscsStateEstimation::set_range, default_range,
                  "Maximal range")},
    {"number",
     make_property<int, DiscsStateEstimation>(
         &DiscsStateEstimation::get_number,
         &DiscsStateEstimation::set_number, default_number, "Number")},
    {"max_radius", make_property<ng_float_t, DiscsStateEstimation>(
                       &DiscsStateEstimation::get_max_radius,
                       &DiscsStateEstimation::set_max_radius,
                       default_max_radius, "Maximal radius")},
    {"max_speed", make_property<ng_float_t, DiscsStateEstimation>(
                      &DiscsStateEstimation::get_max_speed,
                      &DiscsStateEstimation::set_max_speed,
                      default_max_speed, "Maximal speed")},
    {"include_valid", make_property<bool, DiscsStateEstimation>(
                      &DiscsStateEstimation::get_include_valid,
                      &DiscsStateEstimation::set_include_valid,
                      default_include_valid, "Include validity field")},
} +
StateEstimation::properties;

const std::string DiscsStateEstimation::type = register_type<DiscsStateEstimation>("Discs");

}  // namespace navground::sim
