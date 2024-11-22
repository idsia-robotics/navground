#ifndef NAVGROUND_CORE_SCHEMA_H
#define NAVGROUND_CORE_SCHEMA_H

#include "navground/core/command.h"
#include "navground/core/yaml/schema_core.h"
#include "navground/core/yaml/yaml.h"

#include <iostream>
#include <string>

namespace navground::core {

struct SchemaCommand : Command<SchemaCommand> {

  template <typename T> static YAML::Node component(const std::string &type) {
    if (type.empty()) {
      return YAML::schema::base_with_ref<T>();
    }
    return YAML::schema::schema_of_type<T>(type);
  }

  template <typename T> static YAML::Node components(const std::string &) {
    return YAML::schema::registered<T>();
  }

  template <typename T> static YAML::Node schema(const std::string &) {
    return YAML::schema::schema<T>();
  }

  using Schemas = std::map<const std::string, std::function<YAML::Node(std::string)>>;

  inline const static Schemas default_schemas{
      {"core", [](const std::string &) { return YAML::schema::core(); }},
      {"behavior", &component<core::Behavior>},
      {"behavior_modulation", &component<core::BehaviorModulation>},
      {"kinematics", &component<core::Kinematics>},
      {"behavior_register", &components<core::Behavior>},
      {"behavior_modulation_register", &components<core::BehaviorModulation>},
      {"kinematics_register", &components<core::Kinematics>}};

  explicit SchemaCommand(const std::string &name,
                         const std::string &default_schema,
                         const Schemas &extra_schemas = {})
      : Command<SchemaCommand>(name), schemas(default_schemas),
        default_schema(default_schema) {
    schemas.insert(extra_schemas.begin(), extra_schemas.end());
  }

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
        .help("The target of the schema: " + kinds)
        .default_value(default_schema);
    parser.add_argument("--type")
        .help("Registered component name")
        .default_value(std::string(""));
  }

  int execute(const argparse::ArgumentParser &parser) {

    const std::string kind = parser.get<std::string>("kind");
    if (schemas.count(kind) == 0) {
      std::cerr << "Unknown kind of object: " << kind << std::endl;
      std::exit(1);
    }
    const std::string type = parser.get<std::string>("type");
    const YAML::Node node = schemas.at(kind)(type);
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
