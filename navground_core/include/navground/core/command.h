#ifndef NAVGROUND_CORE_COMMAND_H
#define NAVGROUND_CORE_COMMAND_H

#include "navground/core/plugins.h"
#include "navground/core/version.h"
#include <argparse/argparse.hpp>
#include <iostream>
#include <string>

template <typename T> struct Command {

public:
  explicit Command(const std::string &name = "",
                   const std::string &version = "")
      : name(name), version(version) {}

  int run(int argc, char *argv[]) {
    argparse::ArgumentParser parser(name, version);
    parser.add_argument("--no-plugins")
        .help("Do not load plugins")
        .default_value(false)
        .implicit_value(true);
    static_cast<T *>(this)->setup(parser);
    try {
      parser.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
      std::cerr << err.what() << std::endl;
      std::cerr << parser;
      std::exit(1);
    }
    if (!parser.get<bool>("--no-plugins")) {
      navground::core::load_plugins();
    }
    return static_cast<T *>(this)->execute(parser);
  }
  std::string name;
  std::string version;
};

#endif // NAVGROUND_CORE_COMMAND_H
