#ifndef NAVGROUND_CORE_ECHO_H
#define NAVGROUND_CORE_ECHO_H

#include "navground/core/command.h"
#include "navground/core/yaml/yaml.h"

#include <iostream>
#include <string>

namespace navground::core {

// Example: echo behavior "{}"

template <typename T> bool echo(const YAML::Node &node) {
  const auto obj = YAML::load_node<T>(node);
  if (!obj) {
    return false;
  }
  std::cout << YAML::dump<T>(obj.get()) << std::endl;
  return true;
}

template <typename T> bool echo_s(const YAML::Node &node) {
  const auto obj = node.as<T>();
  std::cout << YAML::dump<T>(&obj) << std::endl;
  return true;
}

struct EchoCommand : Command<EchoCommand> {

  using Echos = std::map<std::string, std::function<bool(const YAML::Node &)>>;

  explicit EchoCommand(const std::string &name, const Echos &echos)
      : Command<EchoCommand>(name), echos(echos) {}

  void setup(argparse::ArgumentParser &parser) {
    parser.add_description(
        "Load an object from YAML and print its YAML representation");
    auto kind = parser.add_argument("kind").help("The kind of object to load");
    for (const auto &[k, _] : echos) {
      kind.add_choice(k);
    }
    parser.add_argument("YAML").help(
        "YAML string, or path to a YAML file, describing an experiment");
  }

  int execute(const argparse::ArgumentParser &parser) {

    YAML::Node node;
    const std::string kind = parser.get<std::string>("kind");
    if (echos.count(kind) == 0) {
      std::cerr << "Unknown kind of object to load: " << kind << std::endl;
      std::exit(1);
    }
    const std::string yaml = parser.get<std::string>("YAML");
    if (std::filesystem::exists(yaml)) {
      try {
        node = YAML::LoadFile(yaml);
      } catch (const YAML::ParserException &e) {
        std::cerr << "Could not load YAML file: " << e.what() << std::endl;
        std::exit(1);
      }
    } else {
      try {
        node = YAML::Load(yaml);
      } catch (const YAML::ParserException &e) {
        std::cerr << "Could not load YAML string: " << e.what() << std::endl;
        std::exit(1);
      }
    }
    try {
      if (!echos.at(kind)(node)) {
        std::cerr << "Failed to load " << kind << std::endl;
        std::exit(1);
      }
    } catch (const std::exception &e) {
      std::cerr << "Failed to load " << kind << ": " << e.what() << std::endl;
      std::exit(1);
    }
    return 0;
  }
  Echos echos;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_INFO_H
