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
    std::shared_ptr<navground::sim::Scenario> scenario;
    try {
      scenario = YAML::load_node<navground::sim::Scenario>(node);
    } catch (const std::exception &e) {
      std::cerr << "[Error] Could not load the scenario " << e.what()
                << std::endl;
      std::exit(1);
    }
    // std::cout << "Scenario" << std::endl;
    // std::cout << "========" << std::endl;
    std::cout << YAML::dump<navground::sim::Scenario>(scenario.get());
    // std::cout << std::endl << std::endl;
    // std::cout << "Sampled world" << std::endl;
    std::cout << std::endl;
    for (int i = 0; i < 30; ++i) {
      std::cout << "-";
    }
    std::cout << std::endl;
    World world;
    world.set_seed(seed);
    // is equivalent to:
    // navground::sim::set_random_seed(seed);
    scenario->init_world(&world);
    std::cout << YAML::dump<navground::sim::World>(&world);
    std::cout << std::endl;
    return 0;
  }

  void setup(argparse::ArgumentParser &parser) {
    parser.add_description("Samples a world from a scenario.");
    parser.add_argument("YAML").help(
        "YAML string, or path to a YAML file, describing a scenario");
    parser.add_argument("--seed")
        .help("Seed")
        .default_value(0)
        .scan<'i', int>();
    parser.add_argument("--chdir")
        .help("Whether to change working directory to the directory containing "
              "the file. Useful when the config contains relative paths.")
        .default_value(false)
        .implicit_value(true);
  }
};

} // namespace navground::sim
