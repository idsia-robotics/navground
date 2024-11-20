#ifndef NAVGROUND_CORE_SCHEMA_H
#define NAVGROUND_CORE_SCHEMA_H

#include "navground/core/command.h"
#include "navground/core/yaml/yaml.h"

#include <iostream>
#include <string>

namespace navground::core {

struct SchemaCommand : Command<SchemaCommand> {

  using Schemas = std::map<std::string, std::function<YAML::Node()>>;

  explicit SchemaCommand(const std::string &name,
                         const std::string &default_schema,
                         const Schemas &schemas)
      : Command<SchemaCommand>(name), schemas(schemas),
        default_schema(default_schema) {}

  void setup(argparse::ArgumentParser &parser) {
    std::string kinds = "";
    bool first = true;
    for (const auto &[k, _] : schemas) {
      if (!first) {
        kinds += ", ";
      }
      first = false;
      kinds += k;
    }
    parser.add_description("Prints the YAML schema");
    parser.add_argument("kind")
        .help("The target type of the schema: " + kinds)
        .default_value(default_schema);
  }

  int execute(const argparse::ArgumentParser &parser) {

    const std::string kind = parser.get<std::string>("kind");
    if (schemas.count(kind) == 0) {
      std::cerr << "Unknown kind of object: " << kind << std::endl;
      std::exit(1);
    }
    const YAML::Node node = schemas.at(kind)();
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
