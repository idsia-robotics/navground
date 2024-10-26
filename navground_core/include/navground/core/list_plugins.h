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
  }

  int execute([[maybe_unused]] const argparse::ArgumentParser &parser) {
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
        std::cout << std::endl;
        std::cout << s << std::endl;
      }
    }
    return 0;
  }

  std::vector<std::string> kinds;
};

} // namespace navground::core

#endif // NAVGROUND_CORE_LIST_PLUGINS_H
