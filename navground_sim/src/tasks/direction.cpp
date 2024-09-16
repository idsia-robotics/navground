/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/direction.h"

#include "navground/core/states/geometric.h"
#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

void DirectionTask::prepare(Agent *agent, World *world) const {
  if (_direction.norm()) {
    agent->get_controller()->follow_direction(_direction);
  }
};

bool DirectionTask::done() const { return _stop; }

const std::map<std::string, Property> DirectionTask::properties =
    Properties{{"direction", make_property<Vector2, DirectionTask>(
                                 &DirectionTask::get_direction,
                                 &DirectionTask::set_direction, Vector2{1, 0},
                                 "direction")}};

const std::string DirectionTask::type =
    register_type<DirectionTask>("Direction");

}  // namespace navground::sim
