/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor.h"

#include <algorithm>

namespace navground::sim {

const std::map<std::string, core::Property> Sensor::properties = {
    {"name", core::Property::make<std::string>(&Sensor::get_name,
                                               &Sensor::set_name, "", "Name")},
};

} // namespace navground::sim
