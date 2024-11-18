/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "./my_behavior.h"

namespace navground::core {

const core::Properties IdleBehavior::properties =
    Properties{{"ignore_obstacles",
                make_property<bool, IdleBehavior>(
                    &IdleBehavior::get_ignore_obstacles,
                    &IdleBehavior::set_ignore_obstacles, false,
                    "whether to move towards the target, ignoring obstacles")}};
const std::string IdleBehavior::type = register_type<IdleBehavior>("Idle");

} // namespace navground::core
