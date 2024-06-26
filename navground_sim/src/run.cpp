/**
 * @author Jerome Guzzi - <jerome@idsia.ch>
 */

#include <argparse/argparse.hpp>
#include <filesystem>
#include <iostream>

#include "navground/core/plugins.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/yaml/experiment.h"
#if !defined(_MSC_VER)
#include "tqdm.h"
#define TQDM
#endif // _MSV

#include "yaml-cpp/yaml.h"

using namespace navground::core;
using namespace navground::sim;

int main(int argc, char *argv[]) {
  navground::core::load_plugins();
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
  parser.add_argument("--threads")
      .help("Number of threads")
      .default_value(1)
      .scan<'i', int>();
  parser.add_argument("--processes")
      .help("Number of processes [only supported by run_py]")
      .default_value(1)
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
    experiment.number_of_runs = runs;
  }
  const bool should_display_tqdm = parser.get<bool>("tqdm");
  const int p = parser.get<int>("processes");
  const int j = parser.get<int>("threads");
  if (p != 1 && j == 1) {
    std::cout << "Use run_py to parallelize over multiple process or specify "
                 "the number of threads instead."
              << std::endl;
  }
  std::cout << "Performing experiment ..." << std::endl;
#ifdef TQDM
  if (should_display_tqdm) {
    tqdm bar;
    unsigned i = 0;
    experiment.add_run_callback([&bar, &experiment, &i](ExperimentalRun *) {
      bar.progress(++i, experiment.number_of_runs);
    });
    experiment.run(false, j);
    bar.finish();
  } else 
#endif
  {
    experiment.run(false, j);
  }

  std::cout << "Experiment done" << std::endl;
  std::cout << "Duration: " << experiment.get_duration_ns().count() / 1e9
            << " s" << std::endl;
  if (experiment.has_file()) {
    std::cout << "Saved to: " << *(experiment.get_path()) << std::endl;
  }
}
