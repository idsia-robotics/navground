/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor.h"

#include <algorithm>

namespace navground::sim {

const std::map<std::string, core::Property> Sensor::properties =
    core::Properties{
        {"name", core::make_property<std::string, Sensor>(
                     &Sensor::get_name, &Sensor::set_name, "", "Name")},
    } +
    StateEstimation::properties;
} // namespace navground::sim
