#include "navground/core/plugins.h"
#include "navground/core/behavior.h"
#include "navground/core/behavior_modulation.h"
#include "navground/core/build_info.h"
#include "navground/core/kinematics.h"

#include <filesystem>
#include <fstream>
#include <functional>
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

RegisterMap &get_registers() {
  static RegisterMap _registers{{"behaviors", &Behavior::types},
                                {"modulations", &BehaviorModulation::types},
                                {"kinematics", &Kinematics::types}};
  return _registers;
}

static Plugins loaded_plugins{};

static PkgDependencies pkg_deps{};

// static void print_build_info(const BuildInfo &bi) {
//   std::cout << "version:             " << bi.get_version_string() <<
//   std::endl; std::cout << "git commit:          " << bi.git_commit <<
//   std::endl; std::cout << "build date:          " << bi.date << std::endl;
//   std::cout << "floating-point type: " << bi.floating_point_type <<
//   std::endl;
// }

static const char *pg = "plugin_build_dependencies";
typedef void (*BuildDependenciesGetterPtr)(BuildDependencies &);

static BuildDependencies load_library(const fs::path &path) {
  BuildDependencies bd;
  BuildDependenciesGetterPtr fn = nullptr;
#if defined(WIN32) || defined(_WIN32) ||                                       \
    defined(__WIN32) && !defined(__CYGWIN__)
  // const auto lib =
  auto handle = LoadLibraryA(path.string().c_str());
  auto sym = GetProcAddress(handle, pg);
#else
  // const void *lib =
  void *handle = dlopen(path.c_str(), RTLD_LAZY);
  void *sym = dlsym(handle, pg);
#endif
  if (sym) {
    fn = reinterpret_cast<BuildDependenciesGetterPtr>(sym);
    fn(bd);
  }
  return bd;
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

// (pkg -> {shared lib path})
using PkgPaths = std::map<std::string, std::set<fs::path>>;

static PkgPaths read_plugins_in_directory(const fs::path &directory,
                                          const fs::path &root) {
  PkgPaths ps;
  if (fs::exists(directory) && fs::is_directory(directory)) {
    for (const auto &entry : fs::directory_iterator(directory)) {
      const auto pkg = entry.path().stem().string();
      ps[pkg] = read_plugins_in_file(entry.path(), root);
    }
  }
  return ps;
}

static bool has_ament_index() {
#if NAVGROUND_PLUGINS_IN_AMENT_INDEX
  return bool(std::getenv("AMENT_PREFIX_PATH"));
#else
  return false;
#endif
}

#if NAVGROUND_PLUGINS_IN_AMENT_INDEX
static PkgPaths read_plugins_from_ament_index() {
  PkgPaths ps;
  const char *value = std::getenv("AMENT_PREFIX_PATH");
  if (!value) {
    std::cerr << "Navground plugins are loaded from the ament resource index";
    std::cerr << "but the environment variabled AMENT_PREFIX_PATH is not set";
    std::cerr << std::endl;
  } else {
    for (const auto &[name, install] :
         ament_index_cpp::get_resources("navground_plugins")) {
      // printf("resource %s %s: ", name.c_str(), install.c_str());
      std::string content;
      ament_index_cpp::get_resource("navground_plugins", name, content);
      // printf("%s\n", content.c_str());
      if (!content.empty()) {
        ps[name] = read_plugins(content, install);
      }
    }
  }
  return ps;
}

#endif

void load_plugins(const PathSet &plugins, const PathSetMap &directories,
                  bool include_default) {
  PkgPaths ps{{"", plugins}};
  for (const auto &[root, paths] : directories) {
    for (const auto &path : paths) {
      ps.merge(read_plugins_in_directory(path, root));
    }
  }
  if (include_default) {
    if (has_ament_index()) {
#if NAVGROUND_PLUGINS_IN_AMENT_INDEX
      ps.merge(read_plugins_from_ament_index());
#endif
    } else {
      const char *prefixes = std::getenv("NAVGROUND_PLUGINS_PREFIX");
      const char *index_c = std::getenv("NAVGROUND_PLUGINS_INDEX");
      std::string index = NAVGROUND_PLUGINS_INDEX;
      if (index_c) {
        index = std::string(index_c);
      }
      if (prefixes) {
        for (const auto &s : split(std::string(prefixes), ":")) {
          const fs::path prefix(s);
          ps.merge(read_plugins_in_directory(prefix / index, prefix));
        }
      }
    }
  }
  for (const auto &[pkg_name, paths] : ps) {
    for (const auto &path : paths) {
      if (fs::exists(fs::path(path))) {
        std::map<std::string, std::vector<std::string>> old_types;
        for (const auto &[name, getter] : get_registers()) {
          old_types[name] = getter();
        }
        auto bd = load_library(path);
        pkg_deps[pkg_name][path] = bd;
        for (const auto &[name, getter] : get_registers()) {
          const auto new_types = getter();
          auto &added_types = loaded_plugins[pkg_name][name];
          std::set_difference(new_types.begin(), new_types.end(),
                              old_types[name].begin(), old_types[name].end(),
                              std::back_inserter(added_types));
        }
      }
    }
  }
}

const Plugins &get_loaded_plugins() { return loaded_plugins; }

const PkgDependencies &get_plugins_dependencies() { return pkg_deps; }

} // namespace navground::core
