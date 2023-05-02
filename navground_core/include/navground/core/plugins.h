#ifndef NAVGROUND_CORE_PLUGINS_H
#define NAVGROUND_CORE_PLUGINS_H

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <vector>

#if defined(WIN32) || defined(_WIN32) || \
    defined(__WIN32) && !defined(__CYGWIN__)
#include <libloaderapi.h>
#else
#include <dlfcn.h>
#endif

void load_library(const std::string& path) {
#if defined(WIN32) || defined(_WIN32) || \
    defined(__WIN32) && !defined(__CYGWIN__)
  auto lib = LoadLibraryA(path.c_str());
#else
  void* lib = dlopen(path.c_str(), RTLD_LAZY);
#endif
  if (lib) {
    std::cout << "Loaded " << path << std::endl;
  }
}

static inline std::string default_plugins_env_name = "navground_PLUGINS";

inline std::vector<std::string> get_plugins(
    const std::string& name = default_plugins_env_name) {
  char* env = getenv(name.c_str());
  if (!env) return {};
  std::vector<std::string> paths;
  std::istringstream f(env);
  std::string s;
  while (std::getline(f, s, ':')) {
    paths.push_back(s);
  }
  return paths;
}

inline void load_plugins(const std::string& name = default_plugins_env_name) {
  for (const auto& path : get_plugins(name)) {
    load_library(path);
  }
}

#endif  // NAVGROUND_CORE_PLUGINS_H
