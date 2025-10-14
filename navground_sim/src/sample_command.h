#include <filesystem>

#include "navground/core/command.h"
#include "navground/core/cwd.h"
#include "navground/core/yaml/yaml.h"
#include "navground/sim/experiment.h"
#include "navground/sim/yaml/experiment.h"

#include "yaml-cpp/yaml.h"

namespace navground::sim {

struct SampleCommand : Command<SampleCommand> {

public:
  using Command<SampleCommand>::Command;

  void execute_scenario(const YAML::Node &node, int seed) {
    std::shared_ptr<navground::sim::Scenario> scenario;
    try {
      scenario = YAML::load_node<navground::sim::Scenario>(node);
    } catch (const std::exception &e) {
      std::cerr << "[Error] Could not load the scenario " << std::endl
                << e.what() << std::endl;
      std::exit(1);
    }
    // std::cout << "Scenario" << std::endl;
    // std::cout << "========" << std::endl;
    // std::cout << YAML::dump<navground::sim::Scenario>(scenario.get());
    // std::cout << std::endl << std::endl;
    // std::cout << "Sampled world" << std::endl;
    // std::cout << std::endl << std::string(30, '-') << std::endl;
    World world;
    world.set_seed(seed);
    // is equivalent to:
    // navground::sim::set_random_seed(seed);
    scenario->init_world(&world);
    std::string title = "Sampled world";
    std::cout << title << std::endl
              << std::string(title.size(), '=') << std::endl
              << std::endl
              << YAML::dump<navground::sim::World>(&world) << std::endl;
  }

  void execute_sampler(const YAML::Node &node, int seed,
                       const std::string &type_name, int number) {
    std::unique_ptr<PropertySampler> p;
    try {
      p = property_sampler(node, type_name);
    } catch (const std::exception &e) {
      std::cerr << "[Error] Could not load the sampler " << std::endl
                << e.what() << std::endl;
      exit(1);
    }
    if (!(p && p->valid())) {
      std::cerr << "[Error] Could not load the sampler" << std::endl;
      exit(1);
    }
    RandomGenerator g;
    g.seed(seed);
    std::string title = "Sampled " + type_name;
    std::cout << title << std::endl
              << std::string(title.size(), '=') << std::endl
              << std::endl;
    for (int i = 0; i < number; ++i) {
      std::cout << i << ": " << p->sample(g) << std::endl;
    }
  }

  int execute(const argparse::ArgumentParser &parser) {
    YAML::Node node;
    const std::string yaml = parser.get<std::string>("YAML");
    const int seed = parser.get<int>("seed");
    std::optional<std::filesystem::path> wd = std::nullopt;
    if (std::filesystem::exists(yaml)) {
      try {
        node = YAML::LoadFile(yaml);
      } catch (const YAML::ParserException &e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        std::exit(1);
      }
      if (parser.get<bool>("chdir")) {
        wd = std::filesystem::path(yaml).parent_path();
      }
    } else {
      try {
        node = YAML::Load(yaml);
      } catch (const YAML::ParserException &e) {
        std::cerr << "[Error] " << e.what() << std::endl;
        std::exit(1);
      }
    }
    core::CurrentWorkingDirectory cwd(wd);
    const int number = parser.get<int>("number");
    const std::string type_name = parser.get<std::string>("type");
    if (type_name.empty()) {
      execute_scenario(node, seed);
    } else {
      execute_sampler(node, seed, type_name, number);
    }
    return 0;
  }

  void setup(argparse::ArgumentParser &parser) {
    parser.add_description(
        "Samples a world from a scenario or from a sampler.");
    parser.add_argument("YAML").help("YAML string, or path to a YAML file, "
                                     "describing a scenario or a sampler");
    parser.add_argument("--seed")
        .help("Seed")
        .default_value(0)
        .scan<'i', int>();
    parser.add_argument("--type").help("The sampled type").default_value("");
    parser.add_argument("--number")
        .help("The number of samples")
        .default_value(1)
        .scan<'i', int>();
    parser.add_argument("--chdir")
        .help("Whether to change working directory to the directory containing "
              "the file. Useful when the config contains relative paths.")
        .default_value(false)
        .implicit_value(true);
  }
};

} // namespace navground::sim
