#ifndef NAVGROUND_CORE_PLUGINS_H
#define NAVGROUND_CORE_PLUGINS_H

#include <filesystem>
#include <map>
#include <set>
#include <vector>

#include "navground/core/export.h"

namespace navground::core {

using PkgPlugins = std::map<std::string, std::vector<std::string>>;

using Plugins = std::map<std::string, PkgPlugins>;

using PathSet = std::set<std::filesystem::path>;

using PathSetMap = std::map<std::filesystem::path, PathSet>;

using RegisterMap =
    std::map<std::string, std::function<std::vector<std::string>()>>;

NAVGROUND_CORE_EXPORT RegisterMap &get_registers();

template <typename T> void add_register(const std::string &name) {
  get_registers()[name] = &T::types;
}

/**
 * @brief      Loads plugins
 *
 * Plugins are shared libraries that extend one or more registered classes.
 *
 * @param[in]  plugins         Paths to the shared libraries to import.
 * @param[in]  directories     A map of directories with files containing
 * paths to the shared libraries to import, one per line, relative to the
 * map keys.
 * @param[in]  include_default Whether to load the plugin from the
 *                             ament resources index, or from the
 *                             "NAVGROUND_PLUGINS_INDEX_PATH" env
 *                             (a list of directories separated by ":")
 */
void NAVGROUND_CORE_EXPORT load_plugins(const PathSet &plugins = {},
                                        const PathSetMap &directories = {},
                                        bool include_default = true);

/**
 * @brief      Returns all plugins implemented in C++
 *
 */
const Plugins &NAVGROUND_CORE_EXPORT get_loaded_plugins();

} // namespace navground::core

#endif // NAVGROUND_CORE_PLUGINS_H
