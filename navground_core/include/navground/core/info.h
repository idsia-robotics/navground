#ifndef NAVGROUND_CORE_INFO_H
#define NAVGROUND_CORE_INFO_H

#include "navground/core/build_info.h"
#include "navground/core/command.h"
#include "navground/core/register.h"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

namespace navground::core {

// inline void print_build_info(const BuildInfo &bi) {
//   std::cout << "version:             " << bi.get_version_string() << std::endl;
//   std::cout << "git commit:          " << bi.git_commit << std::endl;
//   std::cout << "build date:          " << bi.date << std::endl;
//   std::cout << "floating-point type: " << bi.floating_point_type << std::endl;
// }

// inline void print_build_dependencies(const BuildDependencies &bd) {
//   for (const auto & [name, vs] : bd) {
//     std::cout << "name: " << dependencies_difference_to_string(vs) << std::endl; 
//   }
// }

struct InfoCommand : Command<InfoCommand> {

  using TitledRegisters =
      std::map<std::string, std::function<PropertyRegister()>>;

  explicit InfoCommand(const std::string &name,
                       const TitledRegisters titled_registers,
                       const BuildInfo &bi, const BuildDependencies &bd)
      : Command<InfoCommand>(name), titled_registers(titled_registers), bi(bi),
        bd(bd) {}

  void setup(argparse::ArgumentParser &parser) {
    parser.add_description("Lists registered components.");
    parser.add_argument("--properties")
        .help("Include properties")
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--description")
        .help("Include property descriptions")
        .default_value(false)
        .implicit_value(true);
    parser.add_argument("--build")
        .help("Include build infos")
        .default_value(false)
        .implicit_value(true);
    for (const auto &[title, reg] : titled_registers) {
      std::string name = title;
      std::string metavar = title;
      std::transform(name.begin(), name.end(), name.begin(), ::tolower);
      std::transform(name.begin(), name.end(), metavar.begin(), ::toupper);
      parser.add_argument(get_arg(title))
          .help("selects " + name)
          .default_value("")
          .metavar(title)
          .nargs(argparse::nargs_pattern::optional);
    }
  }

  void print_register(const std::string &title,
                      const PropertyRegister &registered_properties,
                      const std::string &name = "",
                      bool with_properties = false,
                      bool with_description = false) {
    std::cout << title << std::endl;
    for (unsigned i = 0; i < title.size(); ++i) {
      std::cout << '-';
    }
    std::cout << std::endl;
    if (with_properties) {
      for (const auto &[_name, properties] : registered_properties) {
        if ((!name.empty() && name != _name) || _name.empty())
          continue;
        std::cout << _name << std::endl;
        for (const auto &[k, p] : properties) {
          std::cout << "    " << k << ": " << p.default_value << " ("
                    << p.type_name << ")";
          if (p.readonly) {
            std::cout << " readonly";
          }
          if (!p.deprecated_names.empty()) {
            std::cout << ", deprecated synonyms: ";
            for (const auto &alt_name : p.deprecated_names) {
              std::cout << alt_name << " ";
            }
          }
          if (with_description && !p.description.empty()) {
            std::cout << std::endl;
            std::cout << "      " << p.description;
          }
          std::cout << std::endl;
        }
      }
    } else {
      bool first = true;
      for (const auto &[_name, _] : registered_properties) {
        if ((!name.empty() && name != _name) || _name.empty())
          continue;
        if (!first) {
          std::cout << ", ";
        }
        first = false;
        std::cout << _name;
      }
      std::cout << std::endl;
    }
  }

  std::string get_arg(const std::string &title) {
    std::string arg = title;
    std::replace(arg.begin(), arg.end(), ' ', '_');
    std::transform(arg.begin(), arg.end(), arg.begin(), ::tolower);
    arg = "--" + arg;
    return arg;
  }

  int execute(const argparse::ArgumentParser &parser) {
    if (parser.get<bool>("build")) {
      std::cout << "Build: " << bi.to_string() << std::endl;
      if (bd.size()) {
        std::cout << "Dependencies:" << std::endl;
        for (const auto &[k, vs] : bd) {
          std::cout << "- " << k << ": " << build_infos_to_string(vs) << std::endl;
        }
      }
      std::cout << std::endl;
    }
    const bool with_properties = parser.get<bool>("properties");
    const bool with_description = parser.get<bool>("description");
    std::cout << "Installed components" << std::endl;
    std::cout << "====================" << std::endl;
    for (const auto &[title, reg] : titled_registers) {
      std::string arg = get_arg(title);
      if (parser.is_used(arg)) {
        const auto name = parser.get<std::string>(arg);
        print_register(title, reg(), name, with_properties, with_description);
        return 0;
      }
    }
    for (const auto &[title, reg] : titled_registers) {
      print_register(title, reg(), "", with_properties, with_description);
      std::cout << std::endl;
    }
    return 0;
  }
  TitledRegisters titled_registers;
  BuildInfo bi;
  BuildDependencies bd;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_INFO_H
