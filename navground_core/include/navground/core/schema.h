#ifndef NAVGROUND_CORE_SCHEMA_H
#define NAVGROUND_CORE_SCHEMA_H

#include "navground/core/command.h"
#include "navground/core/yaml/schema_core.h"
#include "navground/core/yaml/yaml.h"

#include <iostream>
#include <string>

namespace navground::core {

struct SchemaCommand : Command<SchemaCommand> {

  template <typename T>
  static YAML::Node component(const argparse::ArgumentParser &parser) {
    if (parser.get<bool>("register")) {
      return YAML::schema::register_schema<T>();
    }
    return YAML::schema::schema<T>(true, parser.present<std::string>("type"));
  }

  template <typename T>
  static YAML::Node schema(const argparse::ArgumentParser &) {
    return YAML::schema::schema<T>();
  }

  using Schemas =
      std::map<const std::string, std::function<YAML::Node(
                                      const argparse::ArgumentParser &parser)>>;

  inline const static Schemas core_schemas{
      {"core",
       [](const argparse::ArgumentParser &) { return bundle_schema(); }},
      {"behavior", &component<core::Behavior>},
      {"behavior_modulation", &component<core::BehaviorModulation>},
      {"kinematics", &component<core::Kinematics>}};

  explicit SchemaCommand(const std::string &name,
                         const std::string &default_schema,
                         const Schemas &extra_schemas = {})
      : Command<SchemaCommand>(name), schemas(core_schemas),
        default_schema(default_schema) {
    schemas.insert(extra_schemas.begin(), extra_schemas.end());
  }

  void setup(argparse::ArgumentParser &parser) {
    parser.add_description("Prints the YAML schema");
    auto kind = parser.add_argument("kind")
                    .help("The target of the schema")
                    .default_value(default_schema);
    for (const auto &[k, _] : schemas) {
      kind.add_choice(k);
    }
    parser.add_argument("--register")
        .help(
            "Whether to generate the register schema instead of the base class "
            "schema")
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--type").help("If provided, generates the schema "
                                       "for the sub-class registered under "
                                       "this name");
  }

  int execute(const argparse::ArgumentParser &parser) {

    const std::string kind = parser.get<std::string>("kind");
    if (schemas.count(kind) == 0) {
      std::cerr << "Unknown kind " << kind << std::endl;
      std::exit(1);
    }
    const YAML::Node node = schemas.at(kind)(parser);
    if (node.IsNull()) {
      std::cerr << "Empty schema" << std::endl;
      std::exit(1);
    }
    YAML::Emitter out;
    out << node;
    std::cout << out.c_str();
    std::cout << std::endl;
    return 0;
  }
  Schemas schemas;
  std::string default_schema;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_SCHEMA_H
