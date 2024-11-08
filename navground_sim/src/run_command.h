#include <filesystem>

#include "navground/core/command.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/yaml/experiment.h"
#if !defined(_MSC_VER)
#include "tqdm.h"
#define TQDM
#endif // _MSV

#include "yaml-cpp/yaml.h"

namespace navground::sim {

struct RunCommand : Command<RunCommand> {

  using Command<RunCommand>::Command;

  int execute(const argparse::ArgumentParser &parser) {
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
    const int p = parser.get<int>("processes");
    const int j = parser.get<int>("threads");
    if (p != 1 && j == 1) {
      std::cout << "Use Python to parallelize over multiple process or specify "
                   "the number of threads instead."
                << std::endl;
    }
    std::cout << "Performing experiment ..." << std::endl;
#ifdef TQDM
    const bool should_display_tqdm = parser.get<bool>("tqdm");
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
    std::cout << "Duration: " << experiment.get_duration().count() / 1e9
              << " s" << std::endl;
    if (experiment.has_file()) {
      std::cout << "Saved to: " << *(experiment.get_path()) << std::endl;
    }
    return 0;
  }

  void setup(argparse::ArgumentParser &parser) {
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
        .help("Number of processes [only supported by Python]")
        .default_value(1)
        .scan<'i', int>();
  }
};

} // namespace navground::sim
