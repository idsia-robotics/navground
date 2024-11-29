/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/core/register.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/kinematics.h"

namespace navground::core {

DEFINE_REGISTERS(Behavior)
DEFINE_REGISTERS(BehaviorModulation)
DEFINE_REGISTERS(Kinematics)

} // namespace navground::core