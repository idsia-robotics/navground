/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "./my_behavior_group.h"

namespace navground::core {

const std::string IdleBehaviorGroupMember::type =
    register_type<IdleBehaviorGroupMember>("IdleGroup", {});

} // namespace navground::core
