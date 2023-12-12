/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_lidar.h"

#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void LidarStateEstimation::update(Agent *agent, World *world,
                                  EnvironmentState *state) const {
  if (core::SensingState *_state = dynamic_cast<core::SensingState *>(state)) {
    auto &c = const_cast<core::CollisionComputation &>(cc);
    const auto neighbors = world->get_neighbors(agent, range);
    c.setup(agent->pose, 0.0, world->get_line_obstacles(), world->get_discs(),
            neighbors);
    const auto ranges = c.get_free_distance_for_sector(
        agent->pose.orientation + start_angle, field_of_view, resolution - 1,
        range, false);
    // _state->set<float>(field_name, ranges, true);
    auto buffer = _state->get_buffer(field_name);
    if (!buffer) {
      buffer =
          _state->init_buffer(field_name, get_description().at(field_name));
    }
    if (!buffer) return;
    buffer->set_data(ranges);
  }
}

}  // namespace navground::sim
