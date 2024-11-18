/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "./my_behavior.h"

namespace navground::core {

const std::string IdleBehavior::type = register_type<IdleBehavior>(
    "Idle", {{"ignore_obstacles",
              Property::make(
                  &IdleBehavior::get_ignore_obstacles,
                  &IdleBehavior::set_ignore_obstacles, false,
                  "whether to move towards the target, ignoring obstacles")}});

} // namespace navground::core
