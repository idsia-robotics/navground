/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <argparse/argparse.hpp>
#include <filesystem>
#include <iostream>

#include "navground/core/plugins.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/scenarios/antipodal.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/yaml/experiment.h"
#include "yaml-cpp/yaml.h"

using namespace navground::core;
using namespace navground::sim;

int main(int argc, char *argv[]) {
  load_plugins();
  argparse::ArgumentParser parser("run");
  parser.add_description("Runs an experiment.");
  parser.add_argument("YAML").help(
      "YAML string, or path to a YAML file, describing an experiment");

  try {
    parser.parse_args(argc, argv);
  } catch (const std::runtime_error &err) {
    std::cerr << err.what() << std::endl;
    std::cerr << parser;
    std::exit(1);
  }

  YAML::Node node;
  const std::string yaml = parser.get<std::string>("YAML");
  if (std::filesystem::exists(yaml)) {
    try {
      node = YAML::LoadFile(yaml);
    } catch (const YAML::ParserException &e) {
      std::cerr << "[Error] " << e.what() << std::endl;
      std::exit(1);
    }
  } else {
    try {
      node = YAML::Load(yaml);
    } catch (const YAML::ParserException &e) {
      std::cerr << "[Error] " << e.what() << std::endl;
      std::exit(1);
    }
  }
  Experiment experiment;
  try {
    experiment = node.as<Experiment>();
  } catch (const std::exception &e) {
    std::cerr << "[Error] Could not load the experiment " << e.what() << std::endl;
    std::exit(1);
  }
  // if (!experiment) {
  //   std::cerr << "[Error] Could not load the experiment" << std::endl;
  //   std::exit(1);
  // }
  // std::cout << YAML::dump<Experiment>(&experiment) << std::endl;
  experiment.run();
  std::cout << "Experiment done" << std::endl;
  std::cout << "Duration: " << experiment.get_duration_ns().count() / 1e9
            << " s" << std::endl;
  if (experiment.has_file()) {
    std::cout << "Saved to: " << *(experiment.get_path()) << std::endl;
  }
}
