/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <iostream>
#include <memory>
#include <iomanip>

#include "navground/core/behavior.h"
#include "navground/core/yaml/yaml.h"
#include "navground/core/yaml/core.h"

using navground::core::Behavior;

static void show_usage(const std::string &name) {
  std::cout << "Usage: " << name << " yaml_string" << std::endl;
}

int main(int argc, char *argv[]) {
  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "--help") == 0) {
      show_usage(argv[0]);
      return 0;
    }
  }
  if (argc <= 1) {
    std::cerr << "No YAML provided" << std::endl;
    return 1;
  }
  std::string yaml = std::string(argv[1]);
  auto behavior = YAML::load_string<Behavior>(yaml);
  if (!behavior) {
    std::cerr << "Failed loading behavior from " << std::quoted(yaml) << std::endl;
    return 1;
  }  
  std::cerr << "Loaded behavior:" << std::endl << std::endl;
  YAML::Emitter out;
  out << YAML::Node(behavior);
  std::cout << out.c_str() << std::endl << std::endl;
  return 0;
}
