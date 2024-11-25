#ifndef NAVGROUND_CORE_LIST_PLUGINS_H
#define NAVGROUND_CORE_LIST_PLUGINS_H

#include "navground/core/command.h"
#include "navground/core/plugins.h"

#include <cctype>
#include <iostream>
#include <string>

namespace navground::core {

inline std::string title(const std::string &name) {
  std::string s = name;
  std::replace(s.begin(), s.end(), '_', ' ');
  s[0] = std::toupper(s[0]);
  const auto i = s.find(' ');
  if (i != std::string::npos && i < s.size() - 1) {
    s[i + 1] = std::toupper(s[i + 1]);
  }
  return s;
}

struct ListPluginsCommand : Command<ListPluginsCommand> {

  explicit ListPluginsCommand(const std::string &name,
                              const std::vector<std::string> &kinds)
      : Command<ListPluginsCommand>(name), kinds(kinds) {}

  void setup(argparse::ArgumentParser &parser) {
    parser.add_description("Load and list plugins");
    parser.add_argument("--dependencies")
        .help("Display dependencies of C++ plugins")
        .default_value(false)
        .implicit_value(true);
  }

  int execute([[maybe_unused]] const argparse::ArgumentParser &parser) {
    const PkgDependencies deps = parser.get<bool>("dependencies")
                                     ? get_plugins_dependencies()
                                     : PkgDependencies{};
    for (const auto &[pkg, plugins] : get_loaded_plugins()) {
      std::string s = "";
      for (const auto &kind : kinds) {
        if (plugins.count(kind)) {
          if (plugins.at(kind).size()) {
            s += title(kind) + ": ";
            bool first = true;
            for (const auto &p : plugins.at(kind)) {
              if (!first) {
                s += ", ";
              }
              first = false;
              s += p;
            }
            s += "\n";
          }
        } else {
          std::cerr << "Unknown plugin type " << kind << std::endl;
        }
      }
      if (!s.empty()) {
        std::cout << pkg << std::endl;
        for (unsigned i = 0; i < pkg.size(); ++i) {
          std::cout << '-';
        }
        std::cout << std::endl << s;
        if (deps.count(pkg) && deps.at(pkg).size()) {
          std::cout << "Dependencies:" << std::endl;
          for (const auto &[path, ds] : deps.at(pkg)) {
            std::cout << "- " << path.filename().string() << ":" << std::endl;
            for (const auto &[name, vs] : ds) {
              std::cout << "  - " << name << ": " << build_infos_to_string(vs)
                        << std::endl;
            }
          }
        }
        std::cout << std::endl;
      }
    }
    return 0;
  }

  std::vector<std::string> kinds;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_LIST_PLUGINS_H
