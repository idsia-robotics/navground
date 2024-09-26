#ifndef NAVGROUND_CORE_PLUGINS_H
#define NAVGROUND_CORE_PLUGINS_H

#include <filesystem>
#include <map>
#include <set>

#include "navground/core/export.h"

namespace navground::core {

/**
 * @brief      Loads plugins
 *
 * Plugins are shared libraries that extend one or more registered classes.
 *
 * @param[in]  plugins         Paths to the shared libraries to import.
 * @param[in]  directories     A map of directories with files containing paths
 *                             to the shared libraries to import,
 *                             one per line, relative to the map keys.
 * @param[in]  include_default Whether to load the plugin from the
 *                             ament resources index, or from the
 *                             "NAVGROUND_PLUGINS_INDEX_PATH" env
 *                             (a list of directories separated by ":")
 */
void NAVGROUND_CORE_EXPORT load_plugins(
    const std::set<std::filesystem::path> &plugins = {},
    const std::map<std::filesystem::path, std::set<std::filesystem::path>>
        &directories = {},
    bool include_default = true);
} // namespace navground::core

#endif // NAVGROUND_CORE_PLUGINS_H
