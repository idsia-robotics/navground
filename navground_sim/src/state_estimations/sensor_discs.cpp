/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_discs.h"

#include <algorithm>

namespace navground::sim {

void DiscsStateEstimation::update(Agent *agent, World *world,
                                  EnvironmentState *state) const {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    const auto &p = agent->pose;
    const auto &xy = p.position;
    const auto r = agent->radius;
    const auto neighbors = world->get_neighbors(agent, range);
    const auto obstacles =
        world->get_static_obstacles_in_region(envelop(xy, range));

    std::vector<std::tuple<float, size_t>> distance(neighbors.size() +
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

    std::valarray<float> radius(0.0f, number);
    std::valarray<float> position(0.0f, 2 * number);
    std::valarray<float> velocity(0.0f, 2 * number);

    for (size_t i = 0; i < std::min<size_t>(number, distance.size()); ++i) {
      const auto index = std::get<1>(distance[i]);
      Vector2 pn;
      if (index < neighbors.size()) {
        const auto &n = neighbors[index];
        radius[i] = n.radius;
        pn = to_relative(n.position, p);
        const auto vn = to_relative(n.velocity, p);
        velocity[2 * i] = vn[0];
        velocity[2 * i + 1] = vn[1];
      } else {
        const auto &n = obstacles[index];
        radius[i] = n.radius;
        pn = to_relative(n.position, p);
      }
      position[2 * i] = pn[0];
      position[2 * i + 1] = pn[1];
    }

    auto buffer = _state->get_buffer("radius");
    if (!buffer) {
      buffer = _state->init_buffer("radius", get_description().at("radius"));
    }
    if (buffer) buffer->set_data(radius);
    buffer = _state->get_buffer("position");
    if (!buffer) {
      buffer =
          _state->init_buffer("position", get_description().at("position"));
    }
    if (buffer) buffer->set_data(position);
    buffer = _state->get_buffer("velocity");
    if (!buffer) {
      buffer =
          _state->init_buffer("velocity", get_description().at("velocity"));
    }
    if (buffer) buffer->set_data(velocity);
  }
}

}  // namespace navground::sim
