#ifndef NAVGROUND_CORE_COMMAND_H
#define NAVGROUND_CORE_COMMAND_H

#include <argparse/argparse.hpp>
#include <navground/core/plugins.h>
#include <iostream>
#include <string>

template <typename T> struct Command {

public:
  explicit Command(const std::string &name = "") : name(name) {}

  int run(int argc, char *argv[]) {
    navground::core::load_plugins();
    argparse::ArgumentParser parser(name);
    static_cast<T *>(this)->setup(parser);
    try {
      parser.parse_args(argc, argv);
    } catch (const std::runtime_error &err) {
      std::cerr << err.what() << std::endl;
      std::cerr << parser;
      std::exit(1);
    }
    
    return static_cast<T *>(this)->execute(parser);
  }
  std::string name;
};

#endif // NAVGROUND_CORE_COMMAND_H
