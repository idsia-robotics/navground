#ifndef NAVGROUND_CORE_PLUGINS_H
#define NAVGROUND_CORE_PLUGINS_H

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;

#if defined(WIN32) || defined(_WIN32) || \
    defined(__WIN32) && !defined(__CYGWIN__)
#define NOMINMAX
#include <Windows.h>
#include <libloaderapi.h>
#else
#include <dlfcn.h>
#endif

#define STRINGIZE(x) #x

namespace navground::core {

void load_library(const std::string& path) {
#if defined(WIN32) || defined(_WIN32) || \
    defined(__WIN32) && !defined(__CYGWIN__)
  const auto lib = LoadLibraryA(path.c_str());
#else
  const void* lib = dlopen(path.c_str(), RTLD_LAZY);
#endif
  if (lib) {
    // std::cerr << "Loaded plugin " << path << std::endl;
  }
}

const static inline std::string default_plugins_env_name = "NAVGROUND_PLUGINS";

#ifdef NAVGROUND_PLUGINS_PATH
const static inline std::optional<fs::path> default_plugins_directory =
    fs::path(NAVGROUND_PLUGINS_PATH);
#else
const static inline std::optional<fs::path> default_plugins_directory =
    std::nullopt;
#endif

inline std::set<std::string> get_plugins(
    const std::string& plugins = "",
    const std::string& env = default_plugins_env_name,
    const std::optional<fs::path>& directory = default_plugins_directory) {
  std::stringstream all_plugins(plugins);
  all_plugins << ";";
  char* env_value = getenv(env.c_str());
  if (env_value) {
    all_plugins << std::string(env_value) << ";";
  }
  if (directory) {
    for (const auto& entry : fs::directory_iterator(*directory)) {
      std::ifstream file(entry.path());
      if (file) {
        all_plugins << file.rdbuf() << ";";
      }
    }
  }
  std::set<std::string> paths;
  std::string s;
  while (std::getline(all_plugins, s, ';')) {
    paths.insert(s);
  }
  return paths;
}

/**
 * @brief      Loads plugins
 *
 * Plugins are shared libraries that extend one or more registered classes.
 *
 * @param[in]  plugins    A list of paths separated by ";".
 * @param[in]  env        An environment variable with additional paths
 *                        separated by ";". If empty, it defaults to
 *                        ``"NAVGROUND_PLUGINS"``
 * @param[in]  directory  A directory with files containing additional paths
 *                        separated by ";". If null, it defaults to
 *                        the value of the macro ``NAVGROUND_PLUGINS_PATH``
 */
inline void load_plugins(const std::string& plugins = "", std::string env = "",
                         std::optional<fs::path> directory = std::nullopt) {
  if (!directory) {
    directory = default_plugins_directory;
  }
  if (env.empty()) {
    env = default_plugins_env_name;
  }
  for (const auto& path : get_plugins(plugins, env, directory)) {
    load_library(path);
  }
}

}  // namespace navground::core

#endif  // NAVGROUND_CORE_PLUGINS_H
