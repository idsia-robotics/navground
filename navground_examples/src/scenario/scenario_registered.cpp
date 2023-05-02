/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "navground/sim/scenario.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/scenarios/cross.h"
#include "navground/sim/scenarios/collisions.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/yaml/world.h"
#include "navground/sim/yaml/scenario.h"
#include "yaml-cpp/yaml.h"

static std::string s_type = "Simple";
namespace sim = navground::sim;

int main() {
  std::cout << "Selecting " << s_type << " from the registered scenarios ";
  for (const auto & type : sim::Scenario::types()) {
    std::cout << " "<< type;
  }
  std::cout << std::endl;
  auto scenario = navground::sim::Scenario::make_type(s_type);
  if (!scenario) {
    std::cerr << "Could not load scenario" << std::endl;
    return 1;
  }
  std::cout << "\nSCENARIO\n========\n" << std::endl;
  std::cout << YAML::dump<sim::Scenario>(scenario.get()) << std::endl;
  sim::World world;
  scenario->init_world(&world);
  std::cout << "\nWORLD\n=====\n" << std::endl;
  std::cout << YAML::dump<sim::World>(&world) << std::endl;
  return 0;
}
