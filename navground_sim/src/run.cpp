/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>

#include "navground/core/plugins.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/yaml/experiment.h"
#include "yaml-cpp/yaml.h"

static void show_usage(std::string name) {
  std::cout << "Usage: " << name << " <option(s)>" << std::endl
            << "Options:" << std::endl
            << "  --input <FILE>\t\t\tPath to the yaml file" << std::endl
            << "  <YAML>\t\t\tInline YAML string" << std::endl;
}

using namespace navground::core;
using namespace navground::sim;

int main(int argc, char *argv[]) {
  load_plugins();
  YAML::Node node;
  bool loaded_file = false;
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--help") {
      show_usage(argv[0]);
      return 0;
    }
    if (std::string(argv[i]) == "--input" && i < argc - 1) {
      try {
        node = YAML::LoadFile(argv[i + 1]);
      } catch (const YAML::ParserException &e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        return 1;
      }
      loaded_file = true;
      break;
    }
  }
  if (!loaded_file) {
    if (argc <= 1) return 1;
    try {
      node = YAML::Load(argv[1]);
    } catch (const YAML::ParserException &e) {
      std::cerr << "[Error] " << e.what() << std::endl;
      return 1;
    }
  }
  Experiment experiment = node.as<Experiment>();
  std::cout << YAML::dump<Experiment>(&experiment) << std::endl;
  experiment.run();
}
