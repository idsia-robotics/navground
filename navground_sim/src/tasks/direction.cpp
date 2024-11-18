/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/direction.h"

#include "navground/core/states/geometric.h"
#include "navground/sim/agent.h"
#include "navground/sim/world.h"

namespace navground::sim {

using navground::core::Property;

void DirectionTask::prepare(Agent *agent, World *world) {
  if (_direction.norm()) {
    agent->get_controller()->follow_direction(_direction);
  }
};

bool DirectionTask::done() const { return _stop; }

const std::string DirectionTask::type = register_type<DirectionTask>(
    "Direction", {{"direction", Property::make(&DirectionTask::get_direction,
                                               &DirectionTask::set_direction,
                                               Vector2{1, 0}, "direction")}});

} // namespace navground::sim
