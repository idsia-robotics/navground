/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/register.h"
#include "navground/sim/scenario.h"
#include "navground/sim/state_estimation.h"
#include "navground/sim/task.h"

namespace navground::core {

DEFINE_REGISTERS(navground::sim::Scenario)
DEFINE_REGISTERS(navground::sim::StateEstimation)
DEFINE_REGISTERS(navground::sim::Task)

} // namespace navground::core