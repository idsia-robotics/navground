/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "scenario.h"

const std::string ThymioDemo::type = register_type<ThymioDemo>(
    "ThymioDemo", {{"behavior", core::Property::make<std::string>(
                                    &ThymioDemo::get_behavior_type,
                                    &ThymioDemo::set_behavior_type, "HL",
                                    "The navigation behavior")}});