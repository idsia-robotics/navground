#include "navground/core/plugins.h"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>

#if defined(WIN32) || defined(_WIN32) || \
    defined(__WIN32) && !defined(__CYGWIN__)
#define NOMINMAX
#include <Windows.h>
#include <libloaderapi.h>
#else
#include <dlfcn.h>
#endif

#define STRINGIZE(x) #x

namespace fs = std::filesystem;

namespace navground::core {

static void load_library(const std::string& path) {
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

const static std::string default_plugins_env_name = "NAVGROUND_PLUGINS";

#ifdef NAVGROUND_PLUGINS_PATH
const static std::optional<fs::path> default_plugins_directory =
    fs::path(NAVGROUND_PLUGINS_PATH);
#else
const static std::optional<fs::path> default_plugins_directory = std::nullopt;
#endif

static std::set<std::string> get_plugins(
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
        std::string line;
        while (std::getline(file, line)) {
          all_plugins << line << ";";
        }
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

void load_plugins(const std::string& plugins, std::string env,
                  std::optional<fs::path> directory) {
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
