/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/waypoints.h"

#include "navground/core/states/geometric.h"
#include "navground/sim/agent.h"

namespace navground::sim {

void WaypointsTask::update(Agent *agent, [[maybe_unused]] World * world, float time) {
  auto c = agent->get_controller();
  if (c->idle()) {
    if (waypoint != waypoints.end()) {
      c->go_to_position(*waypoint, tolerance);
      running = true;
      for (const auto &cb : callbacks) {
        cb({time, 1.0f, waypoint->x(), waypoint->y()});
      }
      ++waypoint;
      if (loop && waypoint == waypoints.end()) {
        waypoint = waypoints.begin();
      }
    } else if (running) {
      for (const auto &cb : callbacks) {
        cb({time, 0.0f, 0.0f, 0.0f});
      }
      running = false;
    }
  }
}

bool WaypointsTask::done() const { 
  // return waypoint == waypoints.end(); 
  return !running;
}

}  // namespace navground::sim
