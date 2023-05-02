/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/geometric_bounded.h"

#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void BoundedStateEstimation::update(Agent *agent, World *world) const {
  if (GeometricState *state = get_geometric_state(agent)) {
    state->set_neighbors(neighbors_of_agent(agent, world));
  }
}

void BoundedStateEstimation::prepare(Agent *agent, World *world) const {
  if (GeometricState *state = get_geometric_state(agent)) {
    state->set_static_obstacles(world->get_discs());
    state->set_line_obstacles(world->get_line_obstacles());
  } else {
    std::cerr << "Agent does not have a geometric environmental state despite "
                 "that it is using a geometric state estimation"
              << std::endl;
  }
}

std::vector<Neighbor> BoundedStateEstimation::neighbors_of_agent(
    const Agent *agent, const World *world) const {
  return world->get_neighbors(agent, range_of_view);
}

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
  return {agent->pose.position[0] - range_of_view,
          agent->pose.position[0] + range_of_view,
          agent->pose.position[1] - range_of_view,
          agent->pose.position[1] + range_of_view};
}

#endif

}  // namespace navground::sim
