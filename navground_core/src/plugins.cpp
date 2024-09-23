#include "navground/core/plugins.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <set>
#include <sstream>
#include <vector>

#if defined(WIN32) || defined(_WIN32) ||                                       \
    defined(__WIN32) && !defined(__CYGWIN__)
#define NOMINMAX
#include <Windows.h>
#include <libloaderapi.h>
#else
#include <dlfcn.h>
#endif

#ifdef NAVGROUND_PLUGINS_IN_AMENT_INDEX
#include <ament_index_cpp/get_resource.hpp>
#include <ament_index_cpp/get_resources.hpp>
#endif

#define STRINGIZE(x) #x

namespace fs = std::filesystem;

namespace navground::core {

static void load_library(const fs::path &path) {
#if defined(WIN32) || defined(_WIN32) ||                                       \
    defined(__WIN32) && !defined(__CYGWIN__)
  const auto lib = LoadLibraryA(path.string().c_str());
#else
  const void *lib = dlopen(path.c_str(), RTLD_LAZY);
#endif
  // if (lib) {
  //   std::cerr << "Loaded plugin " << path << std::endl;
  // }
}

// static std::set<fs::path>
// get_plugins(const std::string &plugins, const std::string &env,
//             const std::optional<fs::path> &directory = std::nullopt) {
//   std::stringstream all_plugins(plugins);
//   all_plugins << ";";
//   char *env_value = getenv(env.c_str());
//   if (env_value) {
//     all_plugins << std::string(env_value) << ";";
//   }
//   if (directory && fs::exists(*directory) && fs::is_directory(*directory)) {
//     for (const auto &entry : fs::directory_iterator(*directory)) {
//       std::ifstream file(entry.path());
//       if (file) {
//         std::string line;
//         while (std::getline(file, line)) {
//           all_plugins << line << ";";
//         }
//       }
//     }
//   }
//   std::set<fs::path> paths;
//   std::string s;
//   while (std::getline(all_plugins, s, ';')) {
//     paths.insert(s);
//   }
//   return paths;
// }

static std::vector<std::string> split(const std::string &value,
                                      const std::string &delimiter) {
  std::vector<std::string> tokens;
  size_t start = 0;
  size_t end = value.find(delimiter);
  while (end != std::string::npos) {
    tokens.push_back(value.substr(start, end - start));
    start = end + delimiter.length();
    end = value.find(delimiter, start);
  }
  tokens.push_back(value.substr(start, end));
  return tokens;
}

// path
// path
// ...
static std::set<fs::path> read_plugins(const std::string &plugins,
                                       const fs::path &root = fs::path("/")) {
  std::set<fs::path> paths;
  for (const auto &s : split(plugins, "\n")) {
    fs::path path(s);
    if (path.is_relative()) {
      path = root / path;
    }
    paths.insert(path);
  }
  return paths;
}

static std::set<fs::path>
read_plugins_in_file(const fs::path &path,
                     const fs::path &root = fs::path("/")) {
  std::ifstream file(path);
  if (file) {
    std::stringstream ss;
    ss << file.rdbuf();
    return read_plugins(ss.str(), root);
  }
  return {};
}

static std::set<fs::path> read_plugins_in_directory(const fs::path &directory,
                                                    const fs::path &root) {
  std::set<fs::path> plugins;
  if (fs::exists(directory) && fs::is_directory(directory)) {
    for (const auto &entry : fs::directory_iterator(directory)) {
      plugins.merge(read_plugins_in_file(entry.path(), root));
    }
  }
  return plugins;
}

void load_plugins(const std::set<std::filesystem::path> &plugins,
                  const std::map<std::filesystem::path,
                                 std::set<std::filesystem::path>> &directories,
                  bool include_default) {
  std::set<fs::path> ps = plugins;
  for (const auto &[root, paths] : directories) {
    for (const auto &path : paths) {
      ps.merge(read_plugins_in_directory(path, root));
    }
  }
  if (include_default) {
#if NAVGROUND_PLUGINS_IN_AMENT_INDEX
    // printf("NAVGROUND_PLUGINS_IN_AMENT_INDEX\n");
    const char *value = std::getenv("AMENT_PREFIX_PATH");
    if (!value) {
      std::cerr << "Navground plugins are loaded from the ament resource index";
      std::cerr << "but the enviroment variabled AMENT_PREFIX_PATH is not set";
      std::cerr << std::endl;
    } else {
      for (const auto &[name, install] :
           ament_index_cpp::get_resources("navground_plugins")) {
        // printf("resource %s %s: ", name.c_str(), install.c_str());
        std::string content;
        ament_index_cpp::get_resource("navground_plugins", name, content);
        // printf("%s\n", content.c_str());
        if (!content.empty()) {
          ps.merge(read_plugins(content, install));
        }
      }
    }
#else
    // printf("NOT NAVGROUND_PLUGINS_IN_AMENT_INDEX\n");
    const char *value = std::getenv("NAVGROUND_PLUGINS_PREFIX");
    if (value) {
      for (const auto &s : split(std::string(value), ":")) {
        const fs::path prefix(s);

        ps.merge(read_plugins_in_directory(prefix / NAVGROUND_PLUGINS_INDEX,
                                           prefix));
      }
    }
#endif
  }
  for (const auto &path : ps) {
    if (fs::exists(fs::path(path))) {
      load_library(path);
    }
  }
}

} // namespace navground::core
