/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/tasks/go_to_pose.h"
#include "navground/core/yaml/schema.h"

namespace navground::sim {

using navground::core::Property;

const std::string GoToPoseTask::type = register_type<GoToPoseTask>(
    "GoToPose",
    {{"point",
      Property::make(&GoToPoseTask::get_point, &GoToPoseTask::set_point,
                     default_point, "Goal point [m]")},
     {"orientation", Property::make(&GoToPoseTask::get_orientation,
                                    &GoToPoseTask::set_orientation,
                                    (ng_float_t)0, "Goal orientation [rad]")},
     {"tolerance",
      Property::make(&GoToPoseTask::get_tolerance, &GoToPoseTask::set_tolerance,
                     default_tolerance, "Spatial tolerance [m]",
                     &YAML::schema::positive)},
     {"angular_tolerance",
      Property::make(&GoToPoseTask::get_angular_tolerance,
                     &GoToPoseTask::set_angular_tolerance,
                     default_angular_tolerance, "Angular tolerance [rad]")}});

} // namespace navground::sim
