#ifndef NAVGROUND_CORE_INFO_H
#define NAVGROUND_CORE_INFO_H

#include "navground/core/command.h"
#include "navground/core/register.h"
#include <algorithm>
#include <cctype>
#include <iostream>
#include <string>

namespace navground::core {

struct InfoCommand : Command<InfoCommand> {

  using TitledRegisters =
      std::map<std::string, std::function<PropertyRegister()>>;

  explicit InfoCommand(const std::string &name,
                       const TitledRegisters titled_registers)
      : Command<InfoCommand>(name), titled_registers(titled_registers) {}

  void setup(argparse::ArgumentParser &parser) {
    parser.add_description("Lists registered components.");
    parser.add_argument("--properties")
        .help("Include properties")
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
                      bool with_properties = false) {
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
          std::cout << "     " << k << ": " << p.default_value << " ("
                    << p.type_name << ")";
          if (!p.deprecated_names.empty()) {
            std::cout << ", deprecated synonyms: ";
            for (const auto &alt_name : p.deprecated_names) {
              std::cout << alt_name << " ";
            }
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
    const bool with_properties = parser.get<bool>("properties");
    for (const auto &[title, reg] : titled_registers) {
      std::string arg = get_arg(title);
      if (parser.is_used(arg)) {
        const auto name = parser.get<std::string>(arg);
        print_register(title, reg(), name, with_properties);
        return 0;
      }
    }
    for (const auto &[title, reg] : titled_registers) {
      print_register(title, reg(), "", with_properties);
      std::cout << std::endl;
    }
    return 0;
  }
  TitledRegisters titled_registers;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_INFO_H
