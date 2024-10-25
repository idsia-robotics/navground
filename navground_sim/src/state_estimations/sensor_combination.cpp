/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include "navground/sim/state_estimations/sensor_combination.h"

#include "navground/sim/agent.h"
#include "navground/sim/world.h"
#include "navground/sim/yaml/world.h"

namespace navground::sim {

Sensor::Description SensorCombination::get_description() const {
  Description desc;
  for (const auto &sensor : _sensors) {
    desc.merge(sensor->get_description());
  }
  return desc;
}

void SensorCombination::update(Agent *agent, World *world,
                               EnvironmentState *state) {
  for (const auto &sensor : _sensors) {
    sensor->update(agent, world, state);
  }
}

void SensorCombination::encode(YAML::Node &node) const {
  for (const auto &sensor : _sensors) {
    node["sensors"].push_back(
        *std::static_pointer_cast<StateEstimation>(sensor));
  }
}

void SensorCombination::decode(const YAML::Node &node) {
  _sensors.clear();
  if (node["sensors"]) {
    auto candidates =
        node["sensors"].as<std::vector<std::shared_ptr<StateEstimation>>>();
    for (const auto &se : candidates) {
      if (auto sensor = std::dynamic_pointer_cast<Sensor>(se)) {
        _sensors.push_back(sensor);
      }
    }
  }
}

const std::string SensorCombination::type =
    register_type<SensorCombination>("Combination");

}  // namespace navground::sim
