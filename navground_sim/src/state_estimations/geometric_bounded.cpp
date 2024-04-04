/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/geometric_bounded.h"

#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void BoundedStateEstimation::update(Agent *agent, World *world,
                                    EnvironmentState *state) const {
  if (GeometricState *geo_state = dynamic_cast<GeometricState *>(state)) {
    geo_state->set_neighbors(neighbors_of_agent(agent, world));
    if (update_static_obstacles) {
      geo_state->set_static_obstacles(
          world->get_discs_in_region(envelop(agent->pose.position, range)));
    }
  }
}

void BoundedStateEstimation::prepare(Agent *agent, World *world) const {
  if (GeometricState *state = get_geometric_state(agent)) {
    if (!update_static_obstacles) {
      state->set_static_obstacles(world->get_discs());
    }
    state->set_line_obstacles(world->get_line_obstacles());
  } else {
    std::cerr << "Agent does not have a geometric environmental state despite "
                 "that it is using a geometric state estimation"
              << std::endl;
  }
}

std::vector<Neighbor> BoundedStateEstimation::neighbors_of_agent(
    const Agent *agent, World *world) const {
  return world->get_neighbors(agent, range);
}

const std::map<std::string, Property> BoundedStateEstimation::properties =
    Properties{
        // {"field_of_view", make_property<float, BoundedStateEstimation>(
        //                       &BoundedStateEstimation::get_field_of_view,
        //                       &BoundedStateEstimation::set_field_of_view,
        //                       0.0f, "Field of view (< 0 infinite)")},
        {"range", make_property<ng_float_t, BoundedStateEstimation>(
                      &BoundedStateEstimation::get_range,
                      &BoundedStateEstimation::set_range, default_range,
                      "Maximal range (< 0 =infinite)", {"range_of_view"})},
        {"update_static_obstacles",
         make_property<bool, BoundedStateEstimation>(
             &BoundedStateEstimation::get_update_static_obstacles,
             &BoundedStateEstimation::set_update_static_obstacles,
             default_update_static_obstacles,
             "Whether to update static obstacles")},
    } +
    StateEstimation::properties;

const std::string BoundedStateEstimation::type =
    register_type<BoundedStateEstimation>("Bounded");

#if 0
std::vector<Neighbor> BoundedStateEstimation::neighbors_of_agent(
    const Agent *agent) const {
  std::vector<Neighbor> ns;

  auto const cs = world->get_agents_in_region(bounding_box(agent));
  for (const Agent *neighbor : cs) {
    if (neighbor != agent && visible(agent, neighbor)) {
      ns.push_back(perceive_neighbor(agent, neighbor));
    }
  }
  return ns;
}

Neighbor BoundedStateEstimation::perceive_neighbor(
    [[maybe_unused]] const Agent *agent, const Agent *neighbor) const {
  return Neighbor(neighbor->pose.position, neighbor->radius,
                  neighbor->twist.velocity, neighbor->id);
}

bool BoundedStateEstimation::visible(
    [[maybe_unused]] const Agent *agent,
    [[maybe_unused]] const Agent *neighbor) const {
  return true;
}

BoundingBox BoundedStateEstimation::bounding_box(const Agent *agent) const {
  return {agent->pose.position[0] - range,
          agent->pose.position[0] + range,
          agent->pose.position[1] - range,
          agent->pose.position[1] + range};
}

#endif

}  // namespace navground::sim
