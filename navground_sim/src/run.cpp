/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <argparse/argparse.hpp>
#include <filesystem>
#include <iostream>

#include "navground/core/plugins.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/scenarios/simple.h"
#include "navground/sim/yaml/experiment.h"
#include "tqdm.h"
#include "yaml-cpp/yaml.h"

using namespace navground::core;
using namespace navground::sim;

int main(int argc, char *argv[]) {
  load_plugins();
  argparse::ArgumentParser parser("run");
  parser.add_description("Runs an experiment.");
  parser.add_argument("YAML").help(
      "YAML string, or path to a YAML file, describing an experiment");
  parser.add_argument("--tqdm")
      .help("Display tqdm bar")
      .default_value(false)
      .implicit_value(true);
  parser.add_argument("--run_index")
      .help("Will overwrite the experiment own run_index if positive.")
      .default_value(-1)
      .scan<'i', int>();
  parser.add_argument("--runs")
      .help("Will overwrite the experiment own runs if positive.")
      .default_value(-1)
      .scan<'i', int>();
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
    std::cerr << "[Error] Could not load the experiment " << e.what()
              << std::endl;
    std::exit(1);
  }
  const int run_index = parser.get<int>("run_index");
  const int runs = parser.get<int>("runs");
  if (run_index >= 0) {
    experiment.run_index = run_index;
  }
  if (runs >= 0) {
    experiment.runs = runs;
  }
  // if (!experiment) {
  //   std::cerr << "[Error] Could not load the experiment" << std::endl;
  //   std::exit(1);
  // }
  // std::cout << YAML::dump<Experiment>(&experiment) << std::endl;
  const bool should_display_tqdm = parser.get<bool>("tqdm");
  std::cout << "Performing experiment ..." << std::endl;
  if (should_display_tqdm) {
    tqdm bar;
    unsigned i = 0;
    experiment.add_run_callback(
        [&bar, &experiment, &i]() { bar.progress(++i, experiment.runs); });
    experiment.run();
    bar.finish();
  } else {
    experiment.run();
  }

  // experiment.store_yaml(YAML::dump(&experiment));

  std::cout << "Experiment done" << std::endl;
  std::cout << "Duration: " << experiment.get_duration_ns().count() / 1e9
            << " s" << std::endl;
  if (experiment.has_file()) {
    std::cout << "Saved to: " << *(experiment.get_path()) << std::endl;
  }
}
