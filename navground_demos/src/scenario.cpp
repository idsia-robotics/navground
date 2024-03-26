/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "scenario.h"

const std::map<std::string, Property> ThymioDemo::properties =
    Properties{{"behavior", make_property<std::string, ThymioDemo>(
                                &ThymioDemo::get_behavior_type,
                                &ThymioDemo::set_behavior_type, "HL",
                                "The navigation behavior")}};

const std::string ThymioDemo::type = register_type<ThymioDemo>("ThymioDemo");