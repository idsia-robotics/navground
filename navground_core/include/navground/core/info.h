#ifndef NAVGROUND_CORE_INFO_H
#define NAVGROUND_CORE_INFO_H

#include <algorithm>
#include <argparse/argparse.hpp>
#include <cctype>
#include <iostream>
#include <string>

#include "./register.h"

template <typename T>
void print_register(const std::string& title, const std::string& name = "") {
  if (!title.empty()) {
    std::cout << title << std::endl;
    for (unsigned i = 0; i < title.size(); ++i) {
      std::cout << '-';
    }
    std::cout << std::endl;
  }
  for (const auto& [_name, properties] : T::type_properties()) {
    if (!name.empty() && name != _name) continue;
    std::cout << _name << std::endl;
    for (const auto& [k, p] : properties) {
      std::cout << "     " << k << ": " << p.default_value << " ["
                << p.type_name << "]" << std::endl;
    }
  }
  std::cout << std::endl;
}

struct INFO {
  INFO(const std::string& cmd, const std::map<std::string, std::string> args,
       int argc, char* argv[])
      : valid(true), unique(false), selection(), title(args) {
    argparse::ArgumentParser parser(cmd);
    parser.add_description("Lists registered components.");
    for (const auto& [arg, title] : args) {
      std::string name = arg;
      while (name[0] == '-') {
        name.erase(name.begin());
      }
      std::string select_name = title;
      select_name[0] = std::tolower(select_name[0]);
      std::string metavar = name;
      std::transform(name.begin(), name.end(), metavar.begin(), ::toupper);
      parser.add_argument(arg)
          .help("selects " + select_name)
          .default_value("")
          .metavar(metavar)
          .nargs(argparse::nargs_pattern::optional);
    }

    try {
      parser.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
      std::cerr << err.what() << std::endl;
      std::cerr << parser;
      valid = false;
      return;
    }

    for (const auto& [arg, _] : args) {
      if (parser.is_used(arg)) {
        selection[arg] = parser.get<std::string>(arg);
      }
    }

    unique = selection.size() == 1;
    if (unique) {
      for (const auto& [arg, _] : selection) {
        title[arg] = "";
      }
    }
    if (selection.size() == 0) {
      for (const auto& [arg, _] : args) {
        selection[arg] = "";
      }
    }
  }

  template <typename T>
  void print(const std::string& arg) {
    if (selection.count(arg)) {
      print_register<T>(title[arg], selection[arg]);
    }
  }

  bool valid;
  bool unique;
  std::map<std::string, std::string> selection;
  std::map<std::string, std::string> title;
};

#endif  // NAVGROUND_CORE_INFO_H
