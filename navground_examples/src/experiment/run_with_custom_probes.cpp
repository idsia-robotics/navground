/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 *
 * An example on how to add extra recording to a run
 */

#include <iomanip>
#include <iostream>

#include "navground/core/types.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experimental_run.h"
#include "navground/sim/probe.h"
#include "navground/sim/yaml/world.h"
#include "probes.h"
#include "yaml-cpp/yaml.h"

namespace sim = navground::sim;

static const char world_yaml[] = R"YAML(
walls:
  - line: [[-1.0, -1.0], [-1.0, 1.0]]
obstacles:
  - position: [2.0, 0.0]
    radius: 0.5
agents:
  - kinematics:
      type: Omni
      max_speed: 1.0
    behavior:
      type: HL
    task:
      type: Waypoints
      waypoints: [[1.0, 0.0]]
      tolerance: 0.1
    radius: 0.1
    control_period: 0.1
)YAML";

int main() {
  YAML::Node node;
  try {
    node = YAML::Load(world_yaml);
  } catch (const YAML::ParserException &e) {
    std::cerr << "[Error] " << e.what() << std::endl;
    return 1;
  }
  auto world = std::make_shared<sim::World>(node.as<sim::World>());
  sim::ExperimentalRun run(world, sim::RunConfig{0.1, 20, true},
                           sim::RecordConfig::all(false));
  run.add_probe(std::make_shared<CheckIfMoving>());
  run.add_record_probe<IsMovingProbe>("is_moving");
  run.add_group_record_probe<IsMovingSparseProbe>("still_times");
  run.run();
  auto record = run.get_record("is_moving");
  std::cout << "Recorded movements:" << std::endl;
  auto shape = record->get_shape();
  const uint8_t *ptr = record->get_typed_data<uint8_t>()->data();
  std::cout << std::boolalpha;
  for (size_t i = 0; i < shape[0]; ++i) {
    std::cout << "- ";
    for (size_t j = 0; j < shape[1]; ++j) {
      std::cout << bool(*ptr++) << " ";
    }
    std::cout << std::endl;
  }
  std::cout << std::endl;
  auto mrecord = run.get_records("still_times");
  std::cout << "Recorded still times:" << std::endl;
  for (const auto &[k, ds] : mrecord) {
    std::cout << k << ": ";
    const auto vs = ds->get_typed_data<ng_float_t>();
    for (const auto &v : *vs) {
      std::cout << v << " ";
    }
    std::cout << std::endl;
  }
  return 0;
}
